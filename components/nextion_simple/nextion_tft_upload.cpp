#include "nextion_simple.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"
#include "esphome/components/network/util.h"

#if defined(USE_ESP_IDF)
#include <esp_http_client.h>
#include <esp_heap_caps.h>
#endif

#if !defined(USE_ESP_IDF)
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#endif

#include <cinttypes>

namespace esphome {
namespace nextion_simple {

static const char *const TAG = "nextion_simple.upload";

// ========= Arduino framework implementation =========
#if !defined(USE_ESP_IDF)

static constexpr size_t NX_STREAM_CHUNK = 1024;

bool NextionSimple::upload_tft_arduino_() {
  ESP_LOGD(TAG, "Nextion TFT upload requested (Arduino)");
  ESP_LOGD(TAG, "URL: %s", this->tft_url_.c_str());

  if (this->is_updating_) {
    ESP_LOGW(TAG, "Currently uploading");
    this->upload_in_progress_ = false;
    return false;
  }
  if (!network::is_connected()) {
    ESP_LOGE(TAG, "Network is not connected");
    this->upload_in_progress_ = false;
    return false;
  }

  this->is_updating_ = true;
  this->original_baud_rate_ = this->uart_parent_->get_baud_rate();

  // ---- helpers ----
  auto wait_for_ack = [&](uint32_t timeout_ms, std::string &out) -> bool {
    out.clear();
    const uint32_t deadline = millis() + timeout_ms;
    uint32_t last_data = millis();
    while (millis() < deadline) {
      while (this->uart_parent_->available()) {
        uint8_t b;
        if (!this->uart_parent_->read_byte(&b))
          break;
        out.push_back(static_cast<char>(b));
        last_data = millis();
      }
      if (!out.empty() && (millis() - last_data) > 10)
        break;
      delay(2);
      App.feed_wdt();
    }
    return !out.empty();
  };

  auto prepare_nextion_for_upload = [&](uint32_t baud_rate) -> bool {
    // Wake & bright
    this->send_command_printf("sleep=0");
    this->send_command_printf("dim=100");
    delay(250);
    // purge RX
    while (this->uart_parent_->available()) {
      uint8_t d;
      if (!this->uart_parent_->read_byte(&d))
        break;
    }

    // whmi-wri <length>,<baud>,1
    char cmd[64];
    int n = snprintf(cmd, sizeof(cmd), "whmi-wri %" PRIu32 ",%" PRIu32 ",1", this->content_length_, baud_rate);
    if (n <= 0 || (size_t) n >= sizeof(cmd)) {
      ESP_LOGE(TAG, "Failed to format whmi-wri");
      return false;
    }
    this->send_command(cmd, (size_t) n);

    // switch ESP baud if needed
    if (baud_rate != this->original_baud_rate_) {
      ESP_LOGD(TAG, "Changing baud rate from %" PRIu32 " to %" PRIu32, this->original_baud_rate_, baud_rate);
      this->uart_parent_->set_baud_rate(baud_rate);
      this->uart_parent_->load_settings();
    }

    // wait for 0x05
    std::string resp;
    if (!wait_for_ack(5000, resp)) {
      ESP_LOGE(TAG, "Timeout waiting upload ACK");
      return false;
    }
    bool ok = resp.find(static_cast<char>(0x05)) != std::string::npos;
    ESP_LOGD(TAG, "Upload prep resp [%s] len=%u",
             format_hex_pretty(reinterpret_cast<const uint8_t *>(resp.data()), resp.size()).c_str(),
             (unsigned) resp.size());
    return ok;
  };

  auto begin_http = [&](HTTPClient &http, Client &client, const char *url) -> bool {
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    http.setReuse(false);  // nový TCP pro jistotu na každé range
    if (!http.begin(client, url))
      return false;
    http.addHeader("User-Agent", "esphome-nextion-uploader");
    http.addHeader("Accept-Encoding", "identity");
    return true;
  };

  auto is_https = [&](const String &url) -> bool { return url.startsWith("https://"); };

  // ---- probe file size: HEAD -> GET Range 0-0 fallback ----
  uint32_t total_size = 0;
  {
    HTTPClient http;
    std::unique_ptr<Client> client;
    if (is_https(this->tft_url_.c_str())) {
      auto *sec = new WiFiClientSecure();
      sec->setInsecure();  // bez CA
      client.reset(sec);
    } else {
      client.reset(new WiFiClient());
    }

    if (begin_http(http, *client, this->tft_url_.c_str())) {
      int code = http.sendRequest("HEAD");
      if (code > 0 && (code == 200 || code == 206)) {
        int len = http.getSize();  // z Content-Length (HEAD to většinou vrací)
        if (len > 0)
          total_size = (uint32_t) len;
      } else {
        ESP_LOGW(TAG, "HEAD failed/status=%d, falling back to GET Range 0-0", code);
      }
      http.end();
    } else {
      ESP_LOGW(TAG, "HTTP begin(HEAD) failed, fallback to GET Range 0-0");
    }

    if (total_size == 0) {
      if (begin_http(http, *client, this->tft_url_.c_str())) {
        http.addHeader("Range", "bytes=0-0");
        int code = http.GET();
        if (code > 0 && (code == 206 || code == 200)) {
          String cr = http.header("Content-Range");  // "bytes 0-0/NNN"
          if (cr.length() > 0) {
            int slash = cr.lastIndexOf('/');
            if (slash > 0 && slash + 1 < cr.length()) {
              total_size = (uint32_t) strtoul(cr.c_str() + slash + 1, nullptr, 10);
            }
          }
          if (total_size == 0) {
            int len = http.getSize();  // fallback
            if (len > 0)
              total_size = (uint32_t) len;
          }
        } else {
          ESP_LOGE(TAG, "GET Range 0-0 failed/status=%d", code);
        }
        http.end();
      } else {
        ESP_LOGE(TAG, "HTTP begin(GET Range) failed");
      }
    }
  }

  if (total_size < 4096 || total_size > 134217728) {
    ESP_LOGE(TAG, "File size out of range or unknown (size=%" PRIu32 ")", total_size);
    // restore baud
    uint32_t cur = this->uart_parent_->get_baud_rate();
    if (cur != this->original_baud_rate_) {
      this->uart_parent_->set_baud_rate(this->original_baud_rate_);
      this->uart_parent_->load_settings();
    }
    this->is_updating_ = false;
    this->upload_in_progress_ = false;
    return false;
  }
  this->tft_size_ = total_size;
  this->content_length_ = total_size;
  ESP_LOGD(TAG, "TFT file size: %" PRIu32 " B", this->tft_size_);

  // ---- prepare Nextion (baud) ----
  static const uint32_t SUPPORTED[] = {2400,   4800,   9600,   19200,  31250,  38400, 57600,
                                       115200, 230400, 250000, 256000, 512000, 921600};
  uint32_t desired_baud = 921600;
  bool ok_baud = false;
  for (auto b : SUPPORTED)
    if (b == desired_baud) {
      ok_baud = true;
      break;
    }
  if (!ok_baud)
    desired_baud = this->original_baud_rate_;
  if (!prepare_nextion_for_upload(desired_baud)) {
    ESP_LOGE(TAG, "Nextion not ready for upload");
    // restore baud
    uint32_t cur = this->uart_parent_->get_baud_rate();
    if (cur != this->original_baud_rate_) {
      this->uart_parent_->set_baud_rate(this->original_baud_rate_);
      this->uart_parent_->load_settings();
    }
    this->is_updating_ = false;
    this->upload_in_progress_ = false;
    return false;
  }

  // ---- ranged download & stream to UART ----
  uint8_t *buffer = (uint8_t *) malloc(4096);
  if (!buffer) {
    ESP_LOGE(TAG, "Failed to allocate upload buffer");
    // restore baud
    uint32_t cur = this->uart_parent_->get_baud_rate();
    if (cur != this->original_baud_rate_) {
      this->uart_parent_->set_baud_rate(this->original_baud_rate_);
      this->uart_parent_->load_settings();
    }
    this->is_updating_ = false;
    this->upload_in_progress_ = false;
    return false;
  }

  uint32_t range_start = 0;
  this->upload_first_chunk_sent_ = false;

  while (this->content_length_ > 0 && range_start < this->tft_size_) {
    const uint32_t block = 4096u;
    uint32_t range_end = range_start + block - 1;
    if (range_end >= this->tft_size_)
      range_end = this->tft_size_ - 1;

    // nový HTTP request pro daný Range
    HTTPClient http;
    std::unique_ptr<Client> client;
    if (is_https(this->tft_url_.c_str())) {
      auto *sec = new WiFiClientSecure();
      sec->setInsecure();
      client.reset(sec);
    } else {
      client.reset(new WiFiClient());
    }

    if (!begin_http(http, *client, this->tft_url_.c_str())) {
      ESP_LOGE(TAG, "HTTP begin(download) failed");
      free(buffer);
      // restore baud
      uint32_t cur = this->uart_parent_->get_baud_rate();
      if (cur != this->original_baud_rate_) {
        this->uart_parent_->set_baud_rate(this->original_baud_rate_);
        this->uart_parent_->load_settings();
      }
      this->is_updating_ = false;
      this->upload_in_progress_ = false;
      return false;
    }

    char range_header[48];
    snprintf(range_header, sizeof(range_header), "bytes=%" PRIu32 "-%" PRIu32, range_start, range_end);
    http.addHeader("Range", range_header);

    int code = http.GET();
    if (code <= 0 || (code != 206 && code != 200)) {
      ESP_LOGE(TAG, "HTTP GET range failed/status=%d", code);
      http.end();
      free(buffer);
      // restore baud
      uint32_t cur = this->uart_parent_->get_baud_rate();
      if (cur != this->original_baud_rate_) {
        this->uart_parent_->set_baud_rate(this->original_baud_rate_);
        this->uart_parent_->load_settings();
      }
      this->is_updating_ = false;
      this->upload_in_progress_ = false;
      return false;
    }

    WiFiClient *stream = http.getStreamPtr();
    uint32_t remain = (code == 206) ? (range_end - range_start + 1) : this->tft_size_;
    while (remain > 0) {
      App.feed_wdt();
      uint16_t want = (remain > 4096u) ? 4096u : (uint16_t) remain;
      size_t read_len = stream->readBytes((char *) buffer, want);
      if (read_len != want) {
        ESP_LOGE(TAG, "Short read: %u of %u", (unsigned) read_len, (unsigned) want);
        http.end();
        free(buffer);
        // restore baud
        uint32_t cur = this->uart_parent_->get_baud_rate();
        if (cur != this->original_baud_rate_) {
          this->uart_parent_->set_baud_rate(this->original_baud_rate_);
          this->uart_parent_->load_settings();
        }
        this->is_updating_ = false;
        this->upload_in_progress_ = false;
        return false;
      }

      // stream do Nextionu
      size_t sent = 0;
      while (sent < read_len) {
        size_t n = read_len - sent;
        if (n > NX_STREAM_CHUNK)
          n = NX_STREAM_CHUNK;
        this->uart_parent_->write_array(buffer + sent, n);
        sent += n;
        yield();
      }

      remain -= read_len;
      this->content_length_ -= read_len;
    }

    http.end();

    // ACK (0x05 OK / 0x08 resume)
    std::string ack;
    wait_for_ack(this->upload_first_chunk_sent_ ? 500 : 5000, ack);
    this->upload_first_chunk_sent_ = true;

    if (!ack.empty()) {
      const uint8_t *ab = reinterpret_cast<const uint8_t *>(ack.data());
      if (ab[0] == 0x08 && ack.size() >= 5) {
        uint32_t resume = 0;
        for (int j = 0; j < 4; ++j)
          resume |= ((uint32_t) ab[j + 1]) << (8 * j);
        ESP_LOGI(TAG, "Nextion requested resume at %" PRIu32, resume);
        if (resume > this->tft_size_) {
          ESP_LOGE(TAG, "Resume offset beyond EOF");
          free(buffer);
          // restore baud
          uint32_t cur = this->uart_parent_->get_baud_rate();
          if (cur != this->original_baud_rate_) {
            this->uart_parent_->set_baud_rate(this->original_baud_rate_);
            this->uart_parent_->load_settings();
          }
          this->is_updating_ = false;
          this->upload_in_progress_ = false;
          return false;
        }
        this->content_length_ = this->tft_size_ - resume;
        range_start = resume;
        continue;  // začne další Range od požadovaného offsetu
      } else if (ab[0] != 0x05 && ab[0] != 0x08) {
        ESP_LOGE(TAG, "Invalid ACK: [%s]",
                 format_hex_pretty(reinterpret_cast<const uint8_t *>(ack.data()), ack.size()).c_str());
        free(buffer);
        // restore baud
        uint32_t cur = this->uart_parent_->get_baud_rate();
        if (cur != this->original_baud_rate_) {
          this->uart_parent_->set_baud_rate(this->original_baud_rate_);
          this->uart_parent_->load_settings();
        }
        this->is_updating_ = false;
        this->upload_in_progress_ = false;
        return false;
      }
    }

    // další blok
    range_start = range_end + 1;

    // progress log
#if defined(BOARD_HAS_PSRAM) || defined(USE_PSRAM)
    ESP_LOGD(TAG, "Uploaded %0.2f%%, remaining %" PRIu32 " B, free heap: %" PRIu32 " (PSRAM unknown here)",
             100.0f * (this->tft_size_ - this->content_length_) / this->tft_size_, this->content_length_,
             (uint32_t) ESP.getFreeHeap());
#else
    ESP_LOGD(TAG, "Uploaded %0.2f%%, remaining %" PRIu32 " B, free heap: %" PRIu32,
             100.0f * (this->tft_size_ - this->content_length_) / this->tft_size_, this->content_length_,
             (uint32_t) ESP.getFreeHeap());
#endif
  }

  free(buffer);

  ESP_LOGI(TAG, "TFT upload successful (Arduino)");

  // restore baud
  {
    uint32_t cur = this->uart_parent_->get_baud_rate();
    if (cur != this->original_baud_rate_) {
      ESP_LOGD(TAG, "Restoring baud %" PRIu32 " -> %" PRIu32, cur, this->original_baud_rate_);
      this->uart_parent_->set_baud_rate(this->original_baud_rate_);
      this->uart_parent_->load_settings();
    }
  }

  // disable replies & return to write-only
  this->send_command_printf("bkcmd=0");
  this->bkcmd_ = 0;
  this->enter_writeonly_mode_();

  this->is_updating_ = false;
  this->upload_in_progress_ = false;

  delay(1500);
  arch_restart();
  return true;
}

#endif  // !USE_ESP_IDF

// ========= ESP-IDF implementation =========
#if defined(USE_ESP_IDF)

static constexpr size_t NX_STREAM_CHUNK = 1024;

bool NextionSimple::wait_for_ack_idf_(uint32_t timeout_ms, std::string &out) {
  out.clear();
  const uint32_t deadline = millis() + timeout_ms;
  uint32_t last_data = millis();
  while (millis() < deadline) {
    while (this->uart_parent_->available()) {
      uint8_t b;
      if (!this->uart_parent_->read_byte(&b))
        break;
      out.push_back(static_cast<char>(b));
      last_data = millis();
    }
    // malé timeout okna, ať se to nezasekne když už něco přišlo
    if (!out.empty() && (millis() - last_data) > 10)
      break;
    delay(2);
    App.feed_wdt();
  }
  return !out.empty();
}

bool NextionSimple::prepare_nextion_for_upload_idf_(uint32_t baud_rate) {
  // Display nesmí spát
  this->send_command_printf("sleep=0");
  this->send_command_printf("dim=100");
  delay(250);

  // Vyčisti RX
  while (this->uart_parent_->available()) {
    uint8_t d;
    if (!this->uart_parent_->read_byte(&d))
      break;
  }

  // whmi-wri <length>,<baud>,1
  char cmd[64];
  int n = snprintf(cmd, sizeof(cmd), "whmi-wri %" PRIu32 ",%" PRIu32 ",1", this->content_length_, baud_rate);
  if (n <= 0 || (size_t) n >= sizeof(cmd)) {
    ESP_LOGE(TAG, "Failed to format whmi-wri");
    return false;
  }
  this->send_command(cmd, (size_t) n);

  // Přepnout UART baud (ESP strana) pokud je třeba
  if (baud_rate != this->original_baud_rate_) {
    ESP_LOGD(TAG, "Changing baud rate from %" PRIu32 " to %" PRIu32, this->original_baud_rate_, baud_rate);
    this->uart_parent_->set_baud_rate(baud_rate);
    this->uart_parent_->load_settings();
  }

  // Čekej na 0x05 ("ready")
  std::string resp;
  if (!this->wait_for_ack_idf_(5000, resp)) {
    ESP_LOGE(TAG, "Timeout waiting upload ACK");
    return false;
  }

  bool ok = resp.find(static_cast<char>(0x05)) != std::string::npos;
  ESP_LOGD(TAG, "Upload prep resp [%s] len=%u",
           format_hex_pretty(reinterpret_cast<const uint8_t *>(resp.data()), resp.size()).c_str(),
           (unsigned) resp.size());
  return ok;
}

// Range upload (HEAD → GET s Range) – převod z originálu na naši třídu
int NextionSimple::upload_by_chunks_idf_(void *http_client_v, uint32_t &range_start) {
  auto http = reinterpret_cast<esp_http_client_handle_t>(http_client_v);

  if (range_start >= this->tft_size_) {
    ESP_LOGW(TAG, "Range start beyond EOF");
    return -1;
  }

  const uint32_t block = 4096u;
  uint32_t range_end = range_start + block - 1;
  if (range_end >= this->tft_size_)
    range_end = this->tft_size_ - 1;

  char range_header[48];
  // bytes=FROM-TO
  snprintf(range_header, sizeof(range_header), "bytes=%" PRIu32 "-%" PRIu32, range_start, range_end);
  esp_http_client_set_header(http, "Range", range_header);

  esp_err_t err = esp_http_client_open(http, 0);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "HTTP open failed: %s", esp_err_to_name(err));
    return -1;
  }

  esp_err_t fh = esp_http_client_fetch_headers(http);
  if (fh != ESP_OK) {
    ESP_LOGE(TAG, "fetch_headers failed: %s", esp_err_to_name(fh));
    esp_http_client_close(http);
    return -1;
  }

  int status = esp_http_client_get_status_code(http);
  if (status != 206 && status != 200) {  // některé servery ignorují Range a pošlou 200 + celé tělo
    ESP_LOGE(TAG, "Unexpected HTTP status: %d", status);
    esp_http_client_close(http);
    return -1;
  }

  const uint32_t want32 = (status == 206) ? (range_end - range_start + 1) : this->tft_size_;
  if (want32 == 0) {
    esp_http_client_close(http);
    return -1;
  }

  uint8_t *buffer = (uint8_t *) heap_caps_malloc(4096, MALLOC_CAP_DEFAULT);
  if (!buffer) {
    ESP_LOGE(TAG, "Failed to allocate upload buffer");
    esp_http_client_close(http);
    return -1;
  }

  uint32_t remain = want32;
  while (remain > 0) {
    App.feed_wdt();
    const uint16_t chunk = (remain > 4096u) ? 4096u : (uint16_t) remain;

    uint16_t read_len = 0;
    uint8_t retries = 0;
    while (retries < 5 && read_len < chunk) {
      int r = esp_http_client_read(http, reinterpret_cast<char *>(buffer) + read_len, chunk - read_len);
      if (r > 0) {
        read_len += (uint16_t) r;
        retries = 0;
      } else {
        retries++;
        vTaskDelay(pdMS_TO_TICKS(2));
      }
      App.feed_wdt();
    }
    if (read_len != chunk) {
      ESP_LOGE(TAG, "Short read: %u of %u", (unsigned) read_len, (unsigned) chunk);
      free(buffer);
      esp_http_client_close(http);
      return -1;
    }

    // stream do Nextionu
    size_t sent = 0;
    while (sent < read_len) {
      size_t n = read_len - sent;
      if (n > NX_STREAM_CHUNK)
        n = NX_STREAM_CHUNK;
      this->uart_parent_->write_array(buffer + sent, n);
      sent += n;
#if defined(USE_ESP_IDF)
      vTaskDelay(pdMS_TO_TICKS(0));
#endif
    }

    remain -= read_len;
    this->content_length_ -= read_len;
  }

  // po bloku čekáme na ACK (0x05 OK / 0x08 resume)
  std::string ack;
  this->wait_for_ack_idf_(upload_first_chunk_sent_ ? 500 : 5000, ack);
  upload_first_chunk_sent_ = true;

  free(buffer);
  esp_http_client_close(http);

  if (!ack.empty()) {
    const uint8_t *ab = reinterpret_cast<const uint8_t *>(ack.data());
    if (ab[0] == 0x08 && ack.size() >= 5) {
      uint32_t result = 0;
      for (int j = 0; j < 4; ++j)
        result |= ((uint32_t) ab[j + 1]) << (8 * j);
      ESP_LOGI(TAG, "Nextion requested resume at %" PRIu32, result);
      if (result > this->tft_size_) {
        ESP_LOGE(TAG, "Resume offset beyond EOF");
        return -1;
      }
      this->content_length_ = this->tft_size_ - result;
      range_start = result;
      return (int) range_start;
    } else if (ab[0] != 0x05 && ab[0] != 0x08) {
      ESP_LOGE(TAG, "Invalid ACK: [%s]",
               format_hex_pretty(reinterpret_cast<const uint8_t *>(ack.data()), ack.size()).c_str());
      return -1;
    }
  }

  // posun na další blok
  range_start = range_end + 1;

#ifdef USE_PSRAM
  ESP_LOGD(TAG, "Uploaded %0.2f%%, remaining %" PRIu32 " B, free: %" PRIu32 " (DRAM) + %" PRIu32 " (PSRAM)",
           100.0f * (this->tft_size_ - this->content_length_) / this->tft_size_, this->content_length_,
           (uint32_t) heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
           (uint32_t) heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
#else
  ESP_LOGD(TAG, "Uploaded %0.2f%%, remaining %" PRIu32 " B, free: %" PRIu32,
           100.0f * (this->tft_size_ - this->content_length_) / this->tft_size_, this->content_length_,
           (uint32_t) esp_get_free_heap_size());
#endif

  return (int) range_start;
}

bool NextionSimple::upload_tft_esp_idf_() {
  ESP_LOGD(TAG, "Nextion TFT upload requested");
  ESP_LOGD(TAG, "URL: %s", this->tft_url_.c_str());

  if (this->is_updating_) {
    ESP_LOGW(TAG, "Currently uploading");
    this->upload_in_progress_ = false;
    return false;
  }
  if (!network::is_connected()) {
    ESP_LOGE(TAG, "Network is not connected");
    this->upload_in_progress_ = false;
    return false;
  }

  this->is_updating_ = true;
  this->original_baud_rate_ = this->uart_parent_->get_baud_rate();

  // ========== PROBE SIZE (HEAD -> open+fetch_headers, fallback GET Range 0-0) ==========
  uint32_t total_size = 0;
  {
    esp_http_client_config_t probe_cfg = {};
    probe_cfg.url = this->tft_url_.c_str();
    probe_cfg.method = HTTP_METHOD_HEAD;
    probe_cfg.timeout_ms = 15000;
    probe_cfg.disable_auto_redirect = false;
    probe_cfg.max_redirection_count = 10;

    auto probe = esp_http_client_init(&probe_cfg);
    if (!probe) {
      ESP_LOGE(TAG, "esp_http_client_init (probe) failed");
      this->is_updating_ = false;
      this->upload_in_progress_ = false;
      return false;
    }
    esp_http_client_set_header(probe, "User-Agent", "esphome-nextion-uploader");
    esp_http_client_set_header(probe, "Accept-Encoding", "identity");
    esp_http_client_set_header(probe, "Connection", "close");

    esp_err_t err = esp_http_client_open(probe, 0);
    if (err == ESP_OK) {
      int hdrs = esp_http_client_fetch_headers(probe);
      if (hdrs >= 0) {
        int status = esp_http_client_get_status_code(probe);
        long long cl = esp_http_client_get_content_length(probe);
        ESP_LOGD(TAG, "HEAD status=%d, content-length=%lld", status, cl);
        if ((status == 200 || status == 206) && cl > 0 && cl < (1LL << 31)) {
          total_size = (uint32_t) cl;
        }
      } else {
        ESP_LOGW(TAG, "HEAD fetch_headers failed: %s", esp_err_to_name((esp_err_t) hdrs));
      }
      esp_http_client_close(probe);
    } else {
      ESP_LOGW(TAG, "HTTP open (HEAD) failed: %s", esp_err_to_name(err));
    }
    esp_http_client_cleanup(probe);
  }

  // fallback: GET Range 0-0 pro zisk Content-Range/Content-Length
  if (total_size == 0) {
    esp_http_client_config_t probe_cfg = {};
    probe_cfg.url = this->tft_url_.c_str();
    probe_cfg.method = HTTP_METHOD_GET;
    probe_cfg.timeout_ms = 15000;
    probe_cfg.disable_auto_redirect = false;
    probe_cfg.max_redirection_count = 10;

    auto probe = esp_http_client_init(&probe_cfg);
    if (!probe) {
      ESP_LOGE(TAG, "esp_http_client_init (probe2) failed");
      this->is_updating_ = false;
      this->upload_in_progress_ = false;
      return false;
    }
    esp_http_client_set_header(probe, "User-Agent", "esphome-nextion-uploader");
    esp_http_client_set_header(probe, "Accept-Encoding", "identity");
    esp_http_client_set_header(probe, "Connection", "keep-alive");
    esp_http_client_set_header(probe, "Range", "bytes=0-0");

    esp_err_t err = esp_http_client_perform(probe);
    if (err == ESP_OK) {
      int status = esp_http_client_get_status_code(probe);
      char cr_buf[96] = {0};
      char *cr = nullptr;
      if (esp_http_client_get_header(probe, "Content-Range", &cr) == ESP_OK && cr) {
        const char *slash = strrchr(cr, '/');
        if (slash && slash[1])
          total_size = (uint32_t) strtoul(slash + 1, nullptr, 10);
      }
      if (total_size == 0) {
        long long cl = esp_http_client_get_content_length(probe);
        if (cl > 0 && cl < (1LL << 31))
          total_size = (uint32_t) cl;
      }
    } else {
      ESP_LOGE(TAG, "HTTP GET (probe2) failed: %s", esp_err_to_name(err));
    }
    esp_http_client_cleanup(probe);
  }

  if (total_size < 4096 || total_size > 134217728) {
    ESP_LOGE(TAG, "File size out of range or unknown (size=%" PRIu32 ")", total_size);
    this->is_updating_ = false;
    this->upload_in_progress_ = false;
    return false;
  }
  this->tft_size_ = total_size;
  this->content_length_ = total_size;
  ESP_LOGD(TAG, "TFT file size: %" PRIu32 " B", this->tft_size_);

  // ========== Připrav Nextion ==========
  static const uint32_t SUPPORTED[] = {2400,   4800,   9600,   19200,  31250,  38400, 57600,
                                       115200, 230400, 250000, 256000, 512000, 921600};
  uint32_t desired_baud = 921600;
  bool ok_baud = false;
  for (auto b : SUPPORTED)
    if (b == desired_baud) {
      ok_baud = true;
      break;
    }
  if (!ok_baud)
    desired_baud = this->original_baud_rate_;

  if (!this->prepare_nextion_for_upload_idf_(desired_baud)) {
    ESP_LOGE(TAG, "Nextion not ready for upload");
    // restore baud just in case
    uint32_t cur = this->uart_parent_->get_baud_rate();
    if (cur != this->original_baud_rate_) {
      this->uart_parent_->set_baud_rate(this->original_baud_rate_);
      this->uart_parent_->load_settings();
    }
    this->is_updating_ = false;
    this->upload_in_progress_ = false;
    return false;
  }

  // ========== Fresh klient pro download ==========
  esp_http_client_config_t cfg_dl = {};
  cfg_dl.url = this->tft_url_.c_str();
  cfg_dl.method = HTTP_METHOD_GET;
  cfg_dl.timeout_ms = 15000;
  cfg_dl.disable_auto_redirect = false;
  cfg_dl.max_redirection_count = 10;

  auto http = esp_http_client_init(&cfg_dl);
  if (!http) {
    ESP_LOGE(TAG, "esp_http_client_init (download) failed");
    // restore baud
    uint32_t cur = this->uart_parent_->get_baud_rate();
    if (cur != this->original_baud_rate_) {
      this->uart_parent_->set_baud_rate(this->original_baud_rate_);
      this->uart_parent_->load_settings();
    }
    this->is_updating_ = false;
    this->upload_in_progress_ = false;
    return false;
  }
  esp_http_client_set_header(http, "User-Agent", "esphome-nextion-uploader");
  esp_http_client_set_header(http, "Accept-Encoding", "identity");
  esp_http_client_set_header(http, "Connection", "keep-alive");

  ESP_LOGD(TAG, "Uploading TFT to Nextion...");
  uint32_t position = 0;
  this->upload_first_chunk_sent_ = false;

  while (this->content_length_ > 0) {
    int r = this->upload_by_chunks_idf_(http, position);
    if (r < 0) {
      ESP_LOGE(TAG, "Upload failed");

      esp_http_client_close(http);
      esp_http_client_cleanup(http);

      uint32_t cur = this->uart_parent_->get_baud_rate();
      if (cur != this->original_baud_rate_) {
        this->uart_parent_->set_baud_rate(this->original_baud_rate_);
        this->uart_parent_->load_settings();
      }

      this->is_updating_ = false;
      this->upload_in_progress_ = false;

      this->send_command_printf("bkcmd=0");
      this->bkcmd_ = 0;
      this->enter_writeonly_mode_();
      return false;
    }
    App.feed_wdt();
  }

  ESP_LOGI(TAG, "TFT upload successful");

  esp_http_client_close(http);
  esp_http_client_cleanup(http);

  uint32_t cur = this->uart_parent_->get_baud_rate();
  if (cur != this->original_baud_rate_) {
    ESP_LOGD(TAG, "Restoring baud %" PRIu32 " -> %" PRIu32, cur, this->original_baud_rate_);
    this->uart_parent_->set_baud_rate(this->original_baud_rate_);
    this->uart_parent_->load_settings();
  }

  this->send_command_printf("bkcmd=0");
  this->bkcmd_ = 0;
  this->enter_writeonly_mode_();

  this->is_updating_ = false;
  this->upload_in_progress_ = false;

  delay(1500);
  arch_restart();
  return true;
}

#endif  // USE_ESP_IDF

}  // namespace nextion_simple
}  // namespace esphome

#include "nextion_simple.h"
#include "esphome/core/log.h"
#include "esphome/core/util.h"

namespace esphome {
namespace nextion_simple {

const char *NextionSimple::TAG = "nextion_simple";

NextionSimple::NextionSimple() {}

void NextionSimple::dump_config() {
  ESP_LOGCONFIG(TAG, "Nextion Simple:");
  ESP_LOGCONFIG(TAG, "  TFT URL: %s", this->tft_url_.c_str());
}

void NextionSimple::setup() {
  ESP_LOGCONFIG(TAG, "Initializing Nextion Simple...");
  if (this->uart_parent_ == nullptr) {
    ESP_LOGE(TAG, "UART parent not set!");
    mark_failed();
    return;
  }

  this->start_init_handshake_();
  this->on_setup_callback_.call();
}

void NextionSimple::loop() {
  if (this->upload_in_progress_ || this->uart_parent_ == nullptr)
    return;

  switch (this->mode_) {
    case NxMode::INIT: {
      this->drain_uart_into_ring_();
      this->parse_from_ring_init_only_();

      if (this->handshake_done_) {
        this->enter_writeonly_mode_();
      } else if (millis() >= this->init_deadline_ms_) {
        ESP_LOGW(TAG, "Init handshake timeout, proceeding to write-only.");
        this->enter_writeonly_mode_();
      }
      break;
    }
    case NxMode::RUN_WRITEONLY:
      // nic – write-only režim
      break;

    case NxMode::DIAG_CHECK:
      this->diagnostic_tick_();
      break;
  }
}

// ====== INIT / MODE mgmt ======

void NextionSimple::start_init_handshake_() {
  this->mode_ = NxMode::INIT;
  this->rx_enabled_ = true;
  this->handshake_done_ = false;
  this->saw_expected_reply_ = false;
  this->rb_head_ = this->rb_tail_ = 0;
  this->init_deadline_ms_ = millis() + 3000;

  if (bkcmd_ != 3) {
    this->send_command_printf("bkcmd=3");
    bkcmd_ = 3;
  }
  this->send_command_printf("sendme");
}

void NextionSimple::enter_writeonly_mode_() {
  if (bkcmd_ != 0) {
    this->send_command_printf("bkcmd=0");
    bkcmd_ = 0;
  }
  this->rx_enabled_ = false;
  this->rb_head_ = this->rb_tail_ = 0;
  this->mode_ = NxMode::RUN_WRITEONLY;

  uint32_t now = millis();
  if (now - this->last_ready_ms_ >= this->nextion_ready_cooldown_) {
    this->last_ready_ms_ = now;
    this->on_nextion_ready_callback_.call();
  }
}

void NextionSimple::request_health_check_() {
  if (this->mode_ != NxMode::RUN_WRITEONLY)
    return;
  this->send_command_printf("bkcmd=1");
  bkcmd_ = 1;
  this->rx_enabled_ = true;
  this->saw_expected_reply_ = false;
  this->rb_head_ = this->rb_tail_ = 0;
  this->diag_deadline_ms_ = millis() + 100;
  this->mode_ = NxMode::DIAG_CHECK;
  this->send_command_printf("get dim");
}

void NextionSimple::diagnostic_tick_() {
  if (!rx_enabled_) {
    this->mode_ = NxMode::RUN_WRITEONLY;
    return;
  }
  this->drain_uart_into_ring_();

  uint8_t b, frame[64];
  size_t flen = 0, ffterm = 0;
  bool in_frame = false;

  while (this->rb_pop_(b)) {
    if (!in_frame) {
      if (b == 0x70 || b == 0x71) {
        in_frame = true;
        frame[0] = b;
        flen = 1;
        ffterm = 0;
      }
    } else {
      if (flen < sizeof(frame))
        frame[flen++] = b;
      if (b == 0xFF) {
        if (++ffterm == 3) {
          this->saw_expected_reply_ = true;
          break;
        }
      } else
        ffterm = 0;
    }
  }

  if (this->saw_expected_reply_ || millis() >= this->diag_deadline_ms_) {
    this->send_command_printf("bkcmd=0");
    bkcmd_ = 0;
    this->rx_enabled_ = false;
    this->rb_head_ = this->rb_tail_ = 0;
    this->mode_ = NxMode::RUN_WRITEONLY;
  }
}

// ====== INIT-only RX ======

void NextionSimple::drain_uart_into_ring_() {
  if (!rx_enabled_)
    return;
  size_t avail = this->uart_parent_->available();
  while (avail--) {
    uint8_t b;
    if (!this->uart_parent_->read_byte(&b))
      break;
    this->rb_push_(b);
  }
}

void NextionSimple::parse_from_ring_init_only_() {
  uint8_t b;
  uint8_t frame[64];
  size_t flen = 0, ffterm = 0;
  bool in_frame = false;

  while (this->rb_pop_(b)) {
    if (!in_frame) {
      if (b == 0x66 || b == 0x88 || b == 0x00) {
        in_frame = true;
        frame[0] = b;
        flen = 1;
        ffterm = 0;
      }
    } else {
      if (flen < sizeof(frame))
        frame[flen++] = b;
      if (b == 0xFF) {
        if (++ffterm == 3) {
          handle_frame_init_only_(frame, flen - 3);
          in_frame = false;
          flen = 0;
          ffterm = 0;
          if (this->handshake_done_)
            return;
        }
      } else {
        ffterm = 0;
      }
    }
  }
}

void NextionSimple::handle_frame_init_only_(const uint8_t *frame, size_t len) {
  if (len == 0)
    return;
  uint8_t code = frame[0];

  switch (code) {
    case 0x66: {  // sendme response: 0x66, page_id
      if (len >= 2) {
        this->current_page_ = static_cast<int>(frame[1]);
        this->on_page_callback_.call(this->current_page_);
      }
      this->handshake_done_ = true;
      break;
    }
    case 0x88: {  // OK
      break;
    }
    case 0x00: {
      ESP_LOGW(TAG, "Nextion returned invalid instruction during INIT");
      break;
    }
    default:
      break;
  }
}

// ====== High-level API ======

void NextionSimple::set_component_value(const std::string &component_name, float value) {
  int iv = static_cast<int>(value);
  this->send_command_printf("%s.val=%d", component_name.c_str(), iv);
}

void NextionSimple::set_component_text(const std::string &component_name, const std::string &text,
                                       const std::vector<std::string> &args) {
  if (args.empty()) {
    this->send_command_printf("%s.txt=\"%s\"", component_name.c_str(), text.c_str());
  } else {
    std::string result;
    result.reserve(text.size() + args.size() * 8);
    size_t last = 0;
    for (const auto &arg : args) {
      size_t pos = text.find("%s", last);
      if (pos == std::string::npos)
        break;
      result.append(text.data() + last, pos - last);
      result.append(arg);
      last = pos + 2;
    }
    result.append(text.data() + last, text.size() - last);
    this->send_command_printf("%s.txt=\"%s\"", component_name.c_str(), result.c_str());
  }
}

void NextionSimple::set_component_text_printf(const std::string &component_name, const char *format, ...) {
  static char text[160];
  static char cmd[224];
  va_list ap;
  va_start(ap, format);
  int tn = vsnprintf(text, sizeof(text), format, ap);
  va_end(ap);
  if (tn <= 0)
    return;
  int cn = snprintf(cmd, sizeof(cmd), "%s.txt=\"%s\"", component_name.c_str(), text);
  if (cn <= 0)
    return;
  this->send_command(cmd, static_cast<size_t>(cn));
}

void NextionSimple::set_component_picc(const std::string &component_name, int value) {
  this->send_command_printf("%s.picc=%d", component_name.c_str(), value);
}

void NextionSimple::set_component_picc1(const std::string &component_name, int value) {
  this->send_command_printf("%s.picc1=%d", component_name.c_str(), value);
}

void NextionSimple::set_component_background_color(const std::string &component_name, int color) {
  this->send_command_printf("%s.bco=%d", component_name.c_str(), color);
}

void NextionSimple::set_component_background_color(const std::string &component_name, Color color) {
  this->set_component_background_color(component_name, this->color_to_integer_(color));
}

void NextionSimple::set_component_font_color(const std::string &component_name, int color) {
  this->send_command_printf("%s.pco=%d", component_name.c_str(), color);
}

void NextionSimple::set_component_font_color(const std::string &component_name, Color color) {
  this->set_component_font_color(component_name, this->color_to_integer_(color));
}

void NextionSimple::set_component_visibility(const std::string &component_name, bool state) {
  this->set_component_visibility(component_name, state ? 1 : 0);
}

void NextionSimple::set_component_visibility(const std::string &component_name, int state) {
  this->send_command_printf("vis %s,%d", component_name.c_str(), state);
}

void NextionSimple::set_page(int page) {
  if (page < 0)
    return;
  this->send_command_printf("page %d", page);
  this->current_page_ = page;
  this->on_page_callback_.call(page);
}

void NextionSimple::set_page(const std::string &page_name) {
  if (page_name.empty())
    return;
  this->send_command_printf("page %s", page_name.c_str());
  // V write-only režimu neznáme ID -> označíme -1
  this->current_page_ = -1;
  this->on_page_callback_.call(-1);
}

// ====== Low-level send ======

void NextionSimple::send_command_printf(const char *fmt, ...) {
  if (this->upload_in_progress_ || this->uart_parent_ == nullptr)
    return;
  static char buf[kMaxCmd + 3];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf) - 3, fmt, ap);
  va_end(ap);
  if (n <= 0)
    return;
  size_t len = (size_t) (n > (int) sizeof(buf) - 3 ? sizeof(buf) - 3 : n);
  buf[len] = 0xFF;
  buf[len + 1] = 0xFF;
  buf[len + 2] = 0xFF;
  this->uart_parent_->write_array(reinterpret_cast<const uint8_t *>(buf), len + 3);
}

// ====== Maintenance ======

void NextionSimple::reset_nextion() { this->send_command_printf("rest"); }

void NextionSimple::upload_tft() {
  if (this->tft_url_.empty()) {
    ESP_LOGW(TAG, "TFT URL not configured; skipping upload");
    return;
  }
  if (this->upload_in_progress_)
    return;

  this->upload_in_progress_ = true;
#if defined(USE_ESP_IDF)
  (void) this->upload_tft_esp_idf_();
#else
  (void) this->upload_tft_arduino_();  // not implemented here
#endif
}

uint32_t NextionSimple::get_free_heap_() {
#if defined(USE_ESP32)
#ifdef USE_ESP_IDF
  return esp_get_free_heap_size();
#else
  return ESP.getFreeHeap();
#endif
#elif defined(USE_ESP8266)
  return ESP.getFreeHeap();
#else
  return 0;
#endif
}

}  // namespace nextion_simple
}  // namespace esphome

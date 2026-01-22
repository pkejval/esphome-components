// nextion_simple.cpp

#include "nextion_simple.h"

#include "esphome/core/log.h"
#include "esphome/core/util.h"

#if defined(USE_ESP32) && defined(USE_ESP_IDF)
#include "esp_heap_caps.h"
#endif

namespace esphome {
namespace nextion_simple {

const char *NextionSimple::TAG = "nextion_simple";

NextionSimple::NextionSimple() = default;

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
      if (this->handshake_done_ || millis() >= this->init_deadline_ms_) {
        if (!this->handshake_done_) {
          ESP_LOGW(TAG, "Init handshake timeout, proceeding to write-only.");
        }
        this->enter_writeonly_mode_();
        break;
      }

      this->drain_uart_into_ring_();
      this->parse_from_ring_(RxFilter::INIT_ONLY);

      if (this->handshake_done_ || millis() >= this->init_deadline_ms_) {
        if (!this->handshake_done_) {
          ESP_LOGW(TAG, "Init handshake timeout, proceeding to write-only.");
        }
        this->enter_writeonly_mode_();
      }
      break;
    }

    case NxMode::RUN_WRITEONLY:
      // Write-only režim – nic nečteme
      break;

    case NxMode::DIAG_CHECK:
      this->diagnostic_tick_();
      break;
  }
}

// ================= RX =================

void NextionSimple::reset_rx_state_() {
  this->rb_head_ = 0;
  this->rb_tail_ = 0;
  this->rb_overflow_ = false;

  this->in_frame_ = false;
  this->frame_len_ = 0;
  this->ff_term_ = 0;
  this->frame_overflow_ = false;
}

void NextionSimple::drain_uart_into_ring_() {
  if (!this->rx_enabled_ || this->uart_parent_ == nullptr)
    return;

  uint32_t drained = 0;
  while (this->uart_parent_->available() && drained < this->rx_drain_max_per_loop_) {
    uint8_t b;
    if (!this->uart_parent_->read_byte(&b))
      break;
    this->rb_push_(b);
    drained++;
  }

  if (this->rb_overflow_) {
    this->rb_overflow_ = false;
    this->rx_rb_overflow_count_++;

    const uint32_t now = millis();
    if (now - this->rx_rb_overflow_last_log_ms_ >= 1000) {
      ESP_LOGW(TAG, "RX ring overflow (%" PRIu32 "x in last ~1s)", this->rx_rb_overflow_count_);
      this->rx_rb_overflow_count_ = 0;
      this->rx_rb_overflow_last_log_ms_ = now;
    }
  }
}

static inline bool is_init_start(uint8_t b) { return b == 0x66 || b == 0x88 || b == 0x00; }
static inline bool is_diag_start(uint8_t b) { return b == 0x70 || b == 0x71; }

void NextionSimple::parse_from_ring_(RxFilter filter) {
  uint8_t b;

  while (this->rb_pop_(b)) {
    if (!this->in_frame_) {
      const bool start = (filter == RxFilter::INIT_ONLY) ? is_init_start(b) : is_diag_start(b);
      if (!start)
        continue;

      this->in_frame_ = true;
      this->frame_buf_[0] = b;
      this->frame_len_ = 1;
      this->ff_term_ = 0;
      this->frame_overflow_ = false;
      continue;
    }

    if (!this->frame_overflow_) {
      if (this->frame_len_ < FRAME_BUF_SIZE) {
        this->frame_buf_[this->frame_len_++] = b;
      } else {
        // Frame je delší než náš buffer: zahodíme payload a budeme jen čekat na terminátor
        this->frame_overflow_ = true;
        this->rx_frame_drop_count_++;
      }
    }

    if (b == 0xFF) {
      if (++this->ff_term_ == 3) {
        if (!this->frame_overflow_) {
          const size_t len_no_term = (this->frame_len_ >= 3) ? (this->frame_len_ - 3) : 0;
          this->handle_frame_(filter, this->frame_buf_, len_no_term);
        } else {
          const uint32_t now = millis();
          if (now - this->rx_frame_drop_last_log_ms_ >= 1000 && this->rx_frame_drop_count_ > 0) {
            ESP_LOGW(TAG, "Dropped RX frame(s) due to overflow (%" PRIu32 "x in last ~1s)", this->rx_frame_drop_count_);
            this->rx_frame_drop_count_ = 0;
            this->rx_frame_drop_last_log_ms_ = now;
          }
        }

        this->in_frame_ = false;
        this->frame_len_ = 0;
        this->ff_term_ = 0;
        this->frame_overflow_ = false;

        if ((filter == RxFilter::INIT_ONLY && this->handshake_done_) ||
            (filter == RxFilter::DIAG_ONLY && this->saw_expected_reply_))
          return;
      }
    } else {
      this->ff_term_ = 0;
    }
  }
}

void NextionSimple::handle_frame_(RxFilter filter, const uint8_t *frame, size_t len_no_term) {
  if (len_no_term == 0 || frame == nullptr)
    return;

  const uint8_t code = frame[0];

  if (filter == RxFilter::DIAG_ONLY) {
    if (code == 0x70 || code == 0x71)
      this->saw_expected_reply_ = true;
    return;
  }

  switch (code) {
    case 0x66: {  // sendme response: 0x66, page_id
      if (len_no_term >= 2) {
        this->current_page_ = static_cast<int>(frame[1]);
        this->on_page_callback_.call(this->current_page_);
      }
      this->handshake_done_ = true;
      break;
    }
    case 0x88:  // OK
      break;
    case 0x00:
      ESP_LOGW(TAG, "Nextion returned invalid instruction during INIT");
      break;
    default:
      break;
  }
}

// ================= Modes =================

void NextionSimple::start_init_handshake_() {
  this->mode_ = NxMode::INIT;
  this->rx_enabled_ = true;
  this->handshake_done_ = false;
  this->saw_expected_reply_ = false;
  this->reset_rx_state_();
  this->init_deadline_ms_ = millis() + 3000;

  if (this->bkcmd_ != 3) {
    this->send_command_printf("bkcmd=3");
    this->bkcmd_ = 3;
  }
  this->send_command_printf("sendme");
}

void NextionSimple::enter_writeonly_mode_() {
  if (this->bkcmd_ != 0) {
    this->send_command_printf("bkcmd=0");
    this->bkcmd_ = 0;
  }

  this->rx_enabled_ = false;
  this->reset_rx_state_();
  this->mode_ = NxMode::RUN_WRITEONLY;

  const uint32_t now = millis();
  if (now - this->last_ready_ms_ >= this->nextion_ready_cooldown_) {
    this->last_ready_ms_ = now;
    this->on_nextion_ready_callback_.call();
  }
}

void NextionSimple::request_health_check_() {
  if (this->mode_ != NxMode::RUN_WRITEONLY)
    return;
  if (this->upload_in_progress_ || this->uart_parent_ == nullptr)
    return;

  if (this->bkcmd_ != 1) {
    this->send_command_printf("bkcmd=1");
    this->bkcmd_ = 1;
  }

  this->rx_enabled_ = true;
  this->saw_expected_reply_ = false;
  this->reset_rx_state_();
  this->diag_deadline_ms_ = millis() + 100;
  this->mode_ = NxMode::DIAG_CHECK;

  this->send_command_printf("get dim");
}

void NextionSimple::diagnostic_tick_() {
  if (!this->rx_enabled_) {
    this->mode_ = NxMode::RUN_WRITEONLY;
    return;
  }

  this->drain_uart_into_ring_();
  this->parse_from_ring_(RxFilter::DIAG_ONLY);

  if (this->saw_expected_reply_ || millis() >= this->diag_deadline_ms_) {
    if (this->bkcmd_ != 0) {
      this->send_command_printf("bkcmd=0");
      this->bkcmd_ = 0;
    }
    this->rx_enabled_ = false;
    this->reset_rx_state_();
    this->mode_ = NxMode::RUN_WRITEONLY;
  }
}

// ================= TX helpers =================

bool NextionSimple::can_tx_() const { return !this->upload_in_progress_ && this->uart_parent_ != nullptr; }

bool NextionSimple::tx_append_escaped_nextion_string_(const char *s) {
  if (s == nullptr)
    return false;

  for (; *s; s++) {
    const char c = *s;
    switch (c) {
      case '\\':
        if (!this->tx_append_char_('\\') || !this->tx_append_char_('\\'))
          return false;
        break;
      case '"':
        if (!this->tx_append_char_('\\') || !this->tx_append_char_('"'))
          return false;
        break;
      case '\r':
      case '\n':
      case '\t':
        // Nextion stringy jsou citlivé; raději nahradit mezerou
        if (!this->tx_append_char_(' '))
          return false;
        break;
      default:
        if (!this->tx_append_char_(c))
          return false;
        break;
    }
  }
  return true;
}

bool NextionSimple::tx_append_escaped_nextion_string_(const std::string &s) {
  return this->tx_append_escaped_nextion_string_(s.c_str());
}

void NextionSimple::send_prop_int_(const std::string &component_name, const char *prop, int value) {
  if (!this->can_tx_())
    return;

  this->tx_begin_();

  bool ok = true;
  ok &= this->tx_append_str_(component_name);
  ok &= this->tx_append_char_('.');
  ok &= this->tx_append_cstr_(prop);
  ok &= this->tx_append_char_('=');
  ok &= this->tx_append_int_(value);

  if (ok)
    this->tx_send_();
}

void NextionSimple::send_vis_(const std::string &component_name, int state) {
  if (!this->can_tx_())
    return;

  this->tx_begin_();

  bool ok = true;
  ok &= this->tx_append_cstr_("vis ");
  ok &= this->tx_append_str_(component_name);
  ok &= this->tx_append_char_(',');
  ok &= this->tx_append_int_(state);

  if (ok)
    this->tx_send_();
}

void NextionSimple::send_set_text_(const std::string &component_name, const char *text) {
  if (!this->can_tx_())
    return;

  this->tx_begin_();

  bool ok = true;
  ok &= this->tx_append_str_(component_name);
  ok &= this->tx_append_cstr_(".txt=\"");
  ok &= this->tx_append_escaped_nextion_string_(text);
  ok &= this->tx_append_char_('"');

  if (ok)
    this->tx_send_();
}

void NextionSimple::send_set_text_formatted_(const std::string &component_name, const std::string &fmt,
                                            const std::vector<std::string> &args) {
  if (!this->can_tx_())
    return;

  this->tx_begin_();

  bool ok = true;
  ok &= this->tx_append_str_(component_name);
  ok &= this->tx_append_cstr_(".txt=\"");

  size_t arg_i = 0;
  const char *p = fmt.c_str();

  while (*p) {
    if (p[0] == '%' && p[1] == 's') {
      if (arg_i < args.size()) {
        ok &= this->tx_append_escaped_nextion_string_(args[arg_i++]);
        p += 2;
        continue;
      }
      // Došly args – zbytek necháme jako literal (vč. "%s")
    }

    const char c = *p++;
    switch (c) {
      case '\\':
        ok &= this->tx_append_char_('\\');
        ok &= this->tx_append_char_('\\');
        break;
      case '"':
        ok &= this->tx_append_char_('\\');
        ok &= this->tx_append_char_('"');
        break;
      case '\r':
      case '\n':
      case '\t':
        ok &= this->tx_append_char_(' ');
        break;
      default:
        ok &= this->tx_append_char_(c);
        break;
    }

    if (!ok)
      break;
  }

  ok &= this->tx_append_char_('"');

  if (ok)
    this->tx_send_();
}

// ================= High-level API =================

void NextionSimple::set_component_value(const std::string &component_name, float value) {
  this->send_prop_int_(component_name, "val", static_cast<int>(value));
}

void NextionSimple::set_component_text(const std::string &component_name, const std::string &text,
                                       const std::vector<std::string> &args) {
  if (!this->can_tx_())
    return;

  if (args.empty()) {
    this->send_set_text_(component_name, text.c_str());
    return;
  }

  this->send_set_text_formatted_(component_name, text, args);
}

void NextionSimple::set_component_text_printf(const std::string &component_name, const char *format, ...) {
  if (!this->can_tx_() || format == nullptr)
    return;

  char text[160];

  va_list ap;
  va_start(ap, format);
  const int tn = vsnprintf(text, sizeof(text), format, ap);
  va_end(ap);

  if (tn <= 0)
    return;

  this->send_set_text_(component_name, text);
}

void NextionSimple::set_component_picc(const std::string &component_name, int value) {
  this->send_prop_int_(component_name, "picc", value);
}

void NextionSimple::set_component_picc1(const std::string &component_name, int value) {
  this->send_prop_int_(component_name, "picc1", value);
}

void NextionSimple::set_component_background_color(const std::string &component_name, int color) {
  this->send_prop_int_(component_name, "bco", color);
}

void NextionSimple::set_component_background_color(const std::string &component_name, Color color) {
  this->set_component_background_color(component_name, this->color_to_integer_(color));
}

void NextionSimple::set_component_font_color(const std::string &component_name, int color) {
  this->send_prop_int_(component_name, "pco", color);
}

void NextionSimple::set_component_font_color(const std::string &component_name, Color color) {
  this->set_component_font_color(component_name, this->color_to_integer_(color));
}

void NextionSimple::set_component_visibility(const std::string &component_name, bool state) {
  this->set_component_visibility(component_name, state ? 1 : 0);
}

void NextionSimple::set_component_visibility(const std::string &component_name, int state) {
  this->send_vis_(component_name, state);
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
  this->current_page_ = -1;
  this->on_page_callback_.call(-1);
}

// ================= Low-level =================

void NextionSimple::send_command(const char *cmd, size_t len) {
  if (!this->can_tx_() || cmd == nullptr)
    return;

  if (len > kMaxCmd)
    len = kMaxCmd;

  memcpy(this->tx_buf_, cmd, len);
  this->tx_buf_[len] = 0xFF;
  this->tx_buf_[len + 1] = 0xFF;
  this->tx_buf_[len + 2] = 0xFF;

  this->uart_parent_->write_array(this->tx_buf_, len + 3);
}

void NextionSimple::send_command_printf(const char *fmt, ...) {
  if (!this->can_tx_() || fmt == nullptr)
    return;

  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(reinterpret_cast<char *>(this->tx_buf_), kMaxCmd, fmt, ap);
  va_end(ap);

  if (n <= 0)
    return;

  if (n >= static_cast<int>(kMaxCmd)) {
    const uint32_t now = millis();
    if (now - this->tx_trunc_last_log_ms_ >= 1000) {
      ESP_LOGW(TAG, "TX command truncated to %u bytes", static_cast<unsigned>(kMaxCmd));
      this->tx_trunc_last_log_ms_ = now;
    }
  }

  size_t len = static_cast<size_t>(n);
  if (len > kMaxCmd)
    len = kMaxCmd;

  this->tx_buf_[len] = 0xFF;
  this->tx_buf_[len + 1] = 0xFF;
  this->tx_buf_[len + 2] = 0xFF;

  this->uart_parent_->write_array(this->tx_buf_, len + 3);
}

// ================= Maintenance =================

void NextionSimple::reset_nextion() {
  this->send_command_printf("rest");
}

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
  (void) this->upload_tft_arduino_();
#endif
}

uint32_t NextionSimple::get_free_heap_() {
#if defined(USE_ESP32) && defined(USE_ESP_IDF)
  return esp_get_free_heap_size();
#elif defined(USE_ESP32)
  return ESP.getFreeHeap();
#elif defined(USE_ESP8266)
  return ESP.getFreeHeap();
#else
  return 0;
#endif
}

}  // namespace nextion_simple
}  // namespace esphome

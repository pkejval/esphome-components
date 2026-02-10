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

uint32_t NextionSimple::fnv1a32_(const char *s) {
  if (s == nullptr)
    return 0;
  uint32_t h = 2166136261u;
  for (; *s; s++) {
    h ^= static_cast<uint8_t>(*s);
    h *= 16777619u;
  }
  return h;
}

uint32_t NextionSimple::make_coalesce_key_(uint32_t comp_hash, TxCoalesceKind kind) {
  // key layout: [comp_hash (upper 24-ish bits mixed)] + [kind in low 8 bits]
  return (comp_hash ^ (comp_hash >> 16) ^ (comp_hash >> 8)) << 8 | static_cast<uint32_t>(kind);
}

void NextionSimple::dump_config() {
  ESP_LOGCONFIG(TAG, "Nextion Simple:");
  ESP_LOGCONFIG(TAG, "  TFT URL: %s", this->tft_url_.c_str());
  ESP_LOGCONFIG(TAG, "  TX batching: queue=%u, max_per_loop=%u, budget=%" PRIu32 "us",
                static_cast<unsigned>(TXQ_SIZE), static_cast<unsigned>(this->tx_max_per_loop_),
                static_cast<uint32_t>(this->tx_time_budget_us_));
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
  if (this->uart_parent_ == nullptr)
    return;

  if (!this->upload_in_progress_) {
    this->tx_flush_();
  }

  if (this->upload_in_progress_)
    return;

  switch (this->mode_) {
    case NxMode::INIT:
      this->init_tick_();
      break;

    case NxMode::RUN_WRITEONLY:
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
  this->frame_start_ms_ = 0;
  this->frame_bytes_seen_ = 0;
}

static inline bool is_init_start(uint8_t b) { return b == 0x66 || b == 0x88 || b == 0x00; }
static inline bool is_diag_start(uint8_t b) { return b == 0x70 || b == 0x71; }

void NextionSimple::log_rx_drops_if_needed_() {
  const uint32_t now = millis();

  if (this->rx_rb_drop_count_ != 0 && (now - this->rx_rb_drop_last_log_ms_ >= 1000)) {
    ESP_LOGW(TAG, "RX ring dropped %" PRIu32 " byte(s) in last ~1s", this->rx_rb_drop_count_);
    this->rx_rb_drop_count_ = 0;
    this->rx_rb_drop_last_log_ms_ = now;
  }

  if (this->rx_frame_drop_count_ != 0 && (now - this->rx_frame_drop_last_log_ms_ >= 1000)) {
    ESP_LOGW(TAG, "Dropped %" PRIu32 " RX frame(s) due to overflow in last ~1s", this->rx_frame_drop_count_);
    this->rx_frame_drop_count_ = 0;
    this->rx_frame_drop_last_log_ms_ = now;
  }
}

void NextionSimple::drain_uart_into_ring_() {
  if (!this->rx_enabled_ || this->uart_parent_ == nullptr)
    return;

  const uint32_t start_us = micros();
  uint32_t drained = 0;

  while (this->uart_parent_->available() && drained < this->rx_drain_max_per_loop_) {
    if ((micros() - start_us) >= this->rx_time_budget_us_)
      break;

    size_t avail = static_cast<size_t>(this->uart_parent_->available());
    if (avail == 0)
      break;

    const size_t remaining = static_cast<size_t>(this->rx_drain_max_per_loop_ - drained);
    size_t to_read = avail;
    if (to_read > remaining)
      to_read = remaining;
    if (to_read > sizeof(this->rx_read_buf_))
      to_read = sizeof(this->rx_read_buf_);

    if (to_read <= 1) {
      uint8_t b;
      if (!this->uart_parent_->read_byte(&b))
        break;
      (void) this->rb_push_(b);
      drained++;
      continue;
    }

    if (!this->uart_parent_->read_array(this->rx_read_buf_, to_read)) {
      uint8_t b;
      if (!this->uart_parent_->read_byte(&b))
        break;
      (void) this->rb_push_(b);
      drained++;
      continue;
    }

    for (size_t i = 0; i < to_read; i++) {
      (void) this->rb_push_(this->rx_read_buf_[i]);
    }
    drained += static_cast<uint32_t>(to_read);
  }

  this->log_rx_drops_if_needed_();
}

bool NextionSimple::parse_from_ring_(RxFilter filter) {
  const uint32_t start_us = micros();
  uint8_t b;

  while (this->rb_pop_(b)) {
    if ((micros() - start_us) >= this->rx_time_budget_us_) {
      return false;
    }

    if (!this->in_frame_) {
      const bool start = (filter == RxFilter::INIT_ONLY) ? is_init_start(b) : is_diag_start(b);
      if (!start)
        continue;

      this->in_frame_ = true;
      this->frame_buf_[0] = b;
      this->frame_len_ = 1;
      this->ff_term_ = 0;
      this->frame_overflow_ = false;
      this->frame_start_ms_ = millis();
      this->frame_bytes_seen_ = 1;
      continue;
    }

    this->frame_bytes_seen_++;
    const uint32_t now_ms = millis();
    if ((now_ms - this->frame_start_ms_) > this->frame_timeout_ms_ ||
        this->frame_bytes_seen_ > this->frame_max_bytes_seen_) {
      this->in_frame_ = false;
      this->frame_len_ = 0;
      this->ff_term_ = 0;
      this->frame_overflow_ = false;
      this->frame_start_ms_ = 0;
      this->frame_bytes_seen_ = 0;
      continue;
    }

    if (!this->frame_overflow_) {
      if (this->frame_len_ < FRAME_BUF_SIZE) {
        this->frame_buf_[this->frame_len_++] = b;
      } else {
        this->frame_overflow_ = true;
        this->rx_frame_drop_count_++;
      }
    }

    if (b == 0xFF) {
      if (++this->ff_term_ == 3) {
        if (!this->frame_overflow_) {
          const size_t len_no_term = (this->frame_len_ >= 3) ? (this->frame_len_ - 3) : 0;
          this->handle_frame_(filter, this->frame_buf_, len_no_term);
        }

        this->in_frame_ = false;
        this->frame_len_ = 0;
        this->ff_term_ = 0;
        this->frame_overflow_ = false;
        this->frame_start_ms_ = 0;
        this->frame_bytes_seen_ = 0;

        if ((filter == RxFilter::INIT_ONLY && this->handshake_done_) ||
            (filter == RxFilter::DIAG_ONLY && this->saw_expected_reply_)) {
          return true;
        }
      }
    } else {
      this->ff_term_ = 0;
    }
  }

  return false;
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
    case 0x66: {
      if (len_no_term >= 2) {
        this->current_page_ = static_cast<int>(frame[1]);
        this->on_page_callback_.call(this->current_page_);
      }
      this->handshake_done_ = true;
      break;
    }
    case 0x88:
      break;
    case 0x00:
      ESP_LOGW(TAG, "Nextion returned invalid instruction during INIT");
      break;
    default:
      break;
  }
}

// ================= Modes =================

void NextionSimple::init_tick_() {
  const uint32_t now = millis();
  if (this->handshake_done_ || now >= this->init_deadline_ms_) {
    if (!this->handshake_done_) {
      ESP_LOGW(TAG, "Init handshake timeout, proceeding to write-only.");
    }
    this->enter_writeonly_mode_();
    return;
  }

  this->drain_uart_into_ring_();
  (void) this->parse_from_ring_(RxFilter::INIT_ONLY);

  if (this->handshake_done_ || millis() >= this->init_deadline_ms_) {
    if (!this->handshake_done_) {
      ESP_LOGW(TAG, "Init handshake timeout, proceeding to write-only.");
    }
    this->enter_writeonly_mode_();
  }
}

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
  this->diag_deadline_ms_ = millis() + this->diag_timeout_ms_;
  this->mode_ = NxMode::DIAG_CHECK;

  this->send_command_printf("get dim");
}

void NextionSimple::diagnostic_tick_() {
  if (!this->rx_enabled_) {
    this->mode_ = NxMode::RUN_WRITEONLY;
    return;
  }

  this->drain_uart_into_ring_();
  (void) this->parse_from_ring_(RxFilter::DIAG_ONLY);

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

// ================= TX queue (raw + coalesce) =================

bool NextionSimple::txq_replace_existing_(uint32_t key, const uint8_t *data, size_t len) {
  if (key == 0 || data == nullptr || len == 0)
    return false;

  // Linear scan (TXQ_SIZE is small). Replace first match (oldest in queue).
  size_t i = this->txq_tail_;
  while (i != this->txq_head_) {
    auto &slot = this->txq_[i];
    if (slot.coalesce == 1 && slot.key == key) {
      if (len > sizeof(slot.data))
        len = sizeof(slot.data);
      memcpy(slot.data, data, len);
      slot.len = static_cast<uint16_t>(len);
      return true;
    }
    i = (i + 1) & (TXQ_SIZE - 1);
  }
  return false;
}

bool NextionSimple::txq_push_raw_(const uint8_t *data, size_t len) {
  if (data == nullptr || len == 0)
    return false;

  size_t nh = (this->txq_head_ + 1) & (TXQ_SIZE - 1);
  if (nh == this->txq_tail_) {
    this->txq_tail_ = (this->txq_tail_ + 1) & (TXQ_SIZE - 1);
    this->txq_drop_count_++;
    this->txq_overflow_ = true;
    nh = (this->txq_head_ + 1) & (TXQ_SIZE - 1);
    if (nh == this->txq_tail_)
      return false;
  }

  auto &slot = this->txq_[this->txq_head_];
  if (len > sizeof(slot.data))
    len = sizeof(slot.data);
  memcpy(slot.data, data, len);
  slot.len = static_cast<uint16_t>(len);
  slot.key = 0;
  slot.coalesce = 0;

  this->txq_head_ = nh;
  return true;
}

bool NextionSimple::txq_push_coalesce_(uint32_t key, const uint8_t *data, size_t len) {
  if (key == 0)
    return this->txq_push_raw_(data, len);

  if (this->txq_replace_existing_(key, data, len)) {
    return true;
  }

  size_t nh = (this->txq_head_ + 1) & (TXQ_SIZE - 1);
  if (nh == this->txq_tail_) {
    // Prefer dropping oldest; coalescing already keeps queue short.
    this->txq_tail_ = (this->txq_tail_ + 1) & (TXQ_SIZE - 1);
    this->txq_drop_count_++;
    this->txq_overflow_ = true;
    nh = (this->txq_head_ + 1) & (TXQ_SIZE - 1);
    if (nh == this->txq_tail_)
      return false;
  }

  auto &slot = this->txq_[this->txq_head_];
  if (len > sizeof(slot.data))
    len = sizeof(slot.data);
  memcpy(slot.data, data, len);
  slot.len = static_cast<uint16_t>(len);
  slot.key = key;
  slot.coalesce = 1;

  this->txq_head_ = nh;
  return true;
}

bool NextionSimple::txq_pop_(TxEntry &out) {
  if (this->txq_head_ == this->txq_tail_)
    return false;
  out = this->txq_[this->txq_tail_];
  this->txq_tail_ = (this->txq_tail_ + 1) & (TXQ_SIZE - 1);
  return true;
}

void NextionSimple::txq_log_overflow_if_needed_() {
  if (!this->txq_overflow_)
    return;

  const uint32_t now = millis();
  if (now - this->txq_overflow_last_log_ms_ >= 1000) {
    ESP_LOGW(TAG, "TX queue overflow: dropped %" PRIu32 " cmd(s) in last ~1s", this->txq_drop_count_);
    this->txq_drop_count_ = 0;
    this->txq_overflow_last_log_ms_ = now;
  }
  this->txq_overflow_ = false;
}

void NextionSimple::tx_flush_() {
  if (this->uart_parent_ == nullptr)
    return;

  const uint32_t start_us = micros();
  uint8_t sent = 0;

  TxEntry e{};
  while (sent < this->tx_max_per_loop_ && this->txq_pop_(e)) {
    if ((micros() - start_us) >= this->tx_time_budget_us_) {
      // Budget vyčerpán – pro jednoduchost pošleme ještě tento jeden frame (už je vyndán z queue).
    }
    this->uart_parent_->write_array(e.data, e.len);
    sent++;
  }

  this->txq_log_overflow_if_needed_();
}

// ================= TX builders =================

void NextionSimple::tx_begin_() { this->tx_len_ = 0; }

bool NextionSimple::tx_append_char_(char c) {
  if (this->tx_len_ >= kMaxCmd)
    return false;
  this->tx_buf_[this->tx_len_++] = static_cast<uint8_t>(c);
  return true;
}

bool NextionSimple::tx_append_cstr_(const char *s) {
  if (s == nullptr)
    return false;
  while (*s) {
    if (!tx_append_char_(*s++))
      return false;
  }
  return true;
}

bool NextionSimple::tx_append_str_(const std::string &s) {
  const size_t n = s.size();
  if (this->tx_len_ + n > kMaxCmd)
    return false;
  memcpy(this->tx_buf_ + this->tx_len_, s.data(), n);
  this->tx_len_ += n;
  return true;
}

bool NextionSimple::tx_append_int_(int v) {
  if (this->tx_len_ >= kMaxCmd)
    return false;

  uint32_t x;
  if (v < 0) {
    if (!tx_append_char_('-'))
      return false;
    x = static_cast<uint32_t>(-(int64_t) v);
  } else {
    x = static_cast<uint32_t>(v);
  }

  char tmp[12];
  int i = 0;
  do {
    tmp[i++] = static_cast<char>('0' + (x % 10));
    x /= 10;
  } while (x != 0 && i < static_cast<int>(sizeof(tmp)));

  while (i--) {
    if (!tx_append_char_(tmp[i]))
      return false;
  }
  return true;
}

void NextionSimple::tx_send_raw_() {
  if (this->uart_parent_ == nullptr)
    return;

  this->tx_buf_[this->tx_len_] = 0xFF;
  this->tx_buf_[this->tx_len_ + 1] = 0xFF;
  this->tx_buf_[this->tx_len_ + 2] = 0xFF;

  const size_t n = this->tx_len_ + 3;
  (void) this->txq_push_raw_(this->tx_buf_, n);
}

void NextionSimple::tx_send_coalesce_(uint32_t key) {
  if (this->uart_parent_ == nullptr)
    return;

  this->tx_buf_[this->tx_len_] = 0xFF;
  this->tx_buf_[this->tx_len_ + 1] = 0xFF;
  this->tx_buf_[this->tx_len_ + 2] = 0xFF;

  const size_t n = this->tx_len_ + 3;
  (void) this->txq_push_coalesce_(key, this->tx_buf_, n);
}

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

void NextionSimple::send_prop_int_(const std::string &component_name, const char *prop, TxCoalesceKind kind, int value) {
  if (this->uart_parent_ == nullptr)
    return;

  this->tx_begin_();

  bool ok = true;
  ok &= this->tx_append_str_(component_name);
  ok &= this->tx_append_char_('.');
  ok &= this->tx_append_cstr_(prop);
  ok &= this->tx_append_char_('=');
  ok &= this->tx_append_int_(value);

  if (!ok)
    return;

  const uint32_t key = make_coalesce_key_(fnv1a32_(component_name.c_str()), kind);
  this->tx_send_coalesce_(key);
}

void NextionSimple::send_vis_(const std::string &component_name, int state) {
  if (this->uart_parent_ == nullptr)
    return;

  this->tx_begin_();

  bool ok = true;
  ok &= this->tx_append_cstr_("vis ");
  ok &= this->tx_append_str_(component_name);
  ok &= this->tx_append_char_(',');
  ok &= this->tx_append_int_(state);

  if (!ok)
    return;

  const uint32_t key = make_coalesce_key_(fnv1a32_(component_name.c_str()), TxCoalesceKind::VIS);
  this->tx_send_coalesce_(key);
}

void NextionSimple::send_set_text_(const std::string &component_name, const char *text) {
  if (this->uart_parent_ == nullptr)
    return;

  this->tx_begin_();

  bool ok = true;
  ok &= this->tx_append_str_(component_name);
  ok &= this->tx_append_cstr_(".txt=\"");
  ok &= this->tx_append_escaped_nextion_string_(text);
  ok &= this->tx_append_char_('"');

  if (!ok)
    return;

  const uint32_t key = make_coalesce_key_(fnv1a32_(component_name.c_str()), TxCoalesceKind::TXT);
  this->tx_send_coalesce_(key);
}

void NextionSimple::send_set_text_formatted_(const std::string &component_name, const std::string &fmt,
                                            const std::vector<std::string> &args) {
  if (this->uart_parent_ == nullptr)
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
        if (!ok)
          break;
        continue;
      }
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

  if (!ok)
    return;

  const uint32_t key = make_coalesce_key_(fnv1a32_(component_name.c_str()), TxCoalesceKind::TXT);
  this->tx_send_coalesce_(key);
}

// ================= High-level API =================

void NextionSimple::set_component_value(const std::string &component_name, float value) {
  this->send_prop_int_(component_name, "val", TxCoalesceKind::VAL, static_cast<int>(value));
}

void NextionSimple::set_component_text(const std::string &component_name, const std::string &text,
                                       const std::vector<std::string> &args) {
  if (args.empty()) {
    this->send_set_text_(component_name, text.c_str());
    return;
  }
  this->send_set_text_formatted_(component_name, text, args);
}

void NextionSimple::set_component_text_printf(const std::string &component_name, const char *format, ...) {
  if (format == nullptr)
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
  this->send_prop_int_(component_name, "picc", TxCoalesceKind::PICC, value);
}

void NextionSimple::set_component_picc1(const std::string &component_name, int value) {
  this->send_prop_int_(component_name, "picc1", TxCoalesceKind::PICC1, value);
}

void NextionSimple::set_component_background_color(const std::string &component_name, int color) {
  this->send_prop_int_(component_name, "bco", TxCoalesceKind::BCO, color);
}

void NextionSimple::set_component_background_color(const std::string &component_name, Color color) {
  this->set_component_background_color(component_name, this->color_to_integer_(color));
}

void NextionSimple::set_component_font_color(const std::string &component_name, int color) {
  this->send_prop_int_(component_name, "pco", TxCoalesceKind::PCO, color);
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
  // Not coalesced: ordering matters
  this->send_command_printf("page %d", page);
  this->current_page_ = page;
  this->on_page_callback_.call(page);
}

void NextionSimple::set_page(const std::string &page_name) {
  if (page_name.empty())
    return;
  // Not coalesced
  this->send_command_printf("page %s", page_name.c_str());
  this->current_page_ = -1;
  this->on_page_callback_.call(-1);
}

// ================= Low-level (enqueue raw) =================

void NextionSimple::send_command(const char *cmd, size_t len) {
  if (this->uart_parent_ == nullptr || cmd == nullptr)
    return;

  if (len > kMaxCmd)
    len = kMaxCmd;

  memcpy(this->tx_buf_, cmd, len);
  this->tx_buf_[len] = 0xFF;
  this->tx_buf_[len + 1] = 0xFF;
  this->tx_buf_[len + 2] = 0xFF;

  (void) this->txq_push_raw_(this->tx_buf_, len + 3);
}

void NextionSimple::send_command_printf(const char *fmt, ...) {
  if (this->uart_parent_ == nullptr || fmt == nullptr)
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

  (void) this->txq_push_raw_(this->tx_buf_, len + 3);
}

// ================= Maintenance =================

void NextionSimple::reset_nextion() {
  // Not coalesced
  this->send_command_printf("rest");
}

void NextionSimple::upload_tft() {
  if (this->tft_url_.empty()) {
    ESP_LOGW(TAG, "TFT URL not configured; skipping upload");
    return;
  }
  if (this->upload_in_progress_)
    return;

  // Flush queue before upload
  this->tx_flush_();

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

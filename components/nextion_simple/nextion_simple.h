// nextion_simple.h

#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/automation.h"
#include "esphome/core/color.h"

#include <cstdarg>
#include <cstdint>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

namespace esphome {
namespace nextion_simple {

class NextionSimple : public Component {
 public:
  NextionSimple();

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::PROCESSOR; }

  void set_uart_parent(uart::UARTComponent *parent) { this->uart_parent_ = parent; }
  void set_tft_url(const std::string &tft_url) { this->tft_url_ = tft_url; }
  void set_nextion_ready_cooldown(uint32_t cooldown) { nextion_ready_cooldown_ = cooldown; }

  // TX batching tunables (optional)
  void set_tx_max_per_loop(uint8_t v) { this->tx_max_per_loop_ = v; }
  void set_tx_time_budget_us(uint32_t v) { this->tx_time_budget_us_ = v; }

  // High-level API
  void set_component_value(const std::string &component_name, float value);
  void set_component_text(const std::string &component_name, const std::string &text,
                          const std::vector<std::string> &args = {});
  void set_component_text_printf(const std::string &component_name, const char *format, ...);
  void set_component_picc(const std::string &component_name, int value);
  void set_component_picc1(const std::string &component_name, int value);
  void set_component_background_color(const std::string &component_name, int color);
  void set_component_background_color(const std::string &component_name, Color color);
  void set_component_font_color(const std::string &component_name, int color);
  void set_component_font_color(const std::string &component_name, Color color);
  void set_component_visibility(const std::string &component_name, bool state);
  void set_component_visibility(const std::string &component_name, int state);
  void set_page(int page);
  void set_page(const std::string &page_name);

  // Low-level send (now enqueued)
  void send_command(const char *cmd, size_t len);
  void send_command_printf(const char *fmt, ...);

  // Maintenance
  void reset_nextion();
  void upload_tft();
  bool is_uploading() const { return this->upload_in_progress_; }
  int get_current_page() const { return this->current_page_; }

  // Callbacks / triggers
  void add_on_setup_callback(std::function<void()> &&callback) { this->on_setup_callback_.add(std::move(callback)); }
  void add_on_page_callback(std::function<void(int)> &&callback) { this->on_page_callback_.add(std::move(callback)); }
  void add_on_nextion_ready_callback(std::function<void()> &&callback) {
    this->on_nextion_ready_callback_.add(std::move(callback));
  }

  static const char *TAG;

 protected:
  // ===== Upload internals =====
  bool upload_tft_arduino_();
  bool upload_tft_esp_idf_();

  bool prepare_nextion_for_upload_idf_(uint32_t baud_rate);
  bool wait_for_ack_idf_(uint32_t timeout_ms, std::string &out);
  int upload_by_chunks_idf_(void *http_client, uint32_t &range_start);

  // ===== Modes =====
  enum class NxMode : uint8_t { INIT, RUN_WRITEONLY, DIAG_CHECK };
  void start_init_handshake_();
  void enter_writeonly_mode_();
  void request_health_check_();
  void init_tick_();
  void diagnostic_tick_();

  // ===== RX =====
  void drain_uart_into_ring_();
  void reset_rx_state_();

  enum class RxFilter : uint8_t { INIT_ONLY, DIAG_ONLY };
  bool parse_from_ring_(RxFilter filter);
  void handle_frame_(RxFilter filter, const uint8_t *frame, size_t len_no_term);

  void log_rx_drops_if_needed_();

  // ===== Utils =====
  inline int color_to_integer_(Color color) {
    uint16_t r = (color.r >> 3) & 0x1F;
    uint16_t g = (color.g >> 2) & 0x3F;
    uint16_t b = (color.b >> 3) & 0x1F;
    return (r << 11) | (g << 5) | (b);
  }

  uint32_t get_free_heap_();

  // ===== TX batching + coalescing =====
  enum class TxCoalesceKind : uint8_t {
    NONE = 0,
    VAL = 1,
    TXT = 2,
    PCO = 3,
    BCO = 4,
    PICC = 5,
    PICC1 = 6,
    VIS = 7,
  };

  static uint32_t fnv1a32_(const char *s);
  static uint32_t make_coalesce_key_(uint32_t comp_hash, TxCoalesceKind kind);

  struct TxEntry {
    uint16_t len{0};
    uint8_t data[256 + 3]{};
    uint32_t key{0};       // 0 => not coalescable
    uint8_t coalesce{0};   // 1 => coalescable
  };

  static constexpr size_t TXQ_SIZE = 32;
  static_assert((TXQ_SIZE & (TXQ_SIZE - 1)) == 0, "TXQ_SIZE must be power of two");

  bool txq_push_raw_(const uint8_t *data, size_t len);
  bool txq_push_coalesce_(uint32_t key, const uint8_t *data, size_t len);
  bool txq_replace_existing_(uint32_t key, const uint8_t *data, size_t len);
  bool txq_pop_(TxEntry &out);
  void txq_log_overflow_if_needed_();
  void tx_flush_();

  uint8_t tx_max_per_loop_{3};       // “ne realtime” default
  uint32_t tx_time_budget_us_{1500}; // max time spent sending per loop

  TxEntry txq_[TXQ_SIZE]{};
  size_t txq_head_{0};
  size_t txq_tail_{0};
  bool txq_overflow_{false};
  uint32_t txq_overflow_last_log_ms_{0};
  uint32_t txq_drop_count_{0};

  // ===== TX (builders) =====
  void tx_begin_();
  bool tx_append_char_(char c);
  bool tx_append_cstr_(const char *s);
  bool tx_append_str_(const std::string &s);
  bool tx_append_int_(int v);

  void tx_send_raw_();
  void tx_send_coalesce_(uint32_t key);

  bool tx_append_escaped_nextion_string_(const char *s);
  bool tx_append_escaped_nextion_string_(const std::string &s);

  void send_prop_int_(const std::string &component_name, const char *prop, TxCoalesceKind kind, int value);
  void send_vis_(const std::string &component_name, int state);
  void send_set_text_(const std::string &component_name, const char *text);
  void send_set_text_formatted_(const std::string &component_name, const std::string &fmt,
                                const std::vector<std::string> &args);

  // ===== State =====
  uart::UARTComponent *uart_parent_{nullptr};
  std::string tft_url_;

  static constexpr size_t kMaxCmd = 256;
  uint8_t bkcmd_{3};
  uint8_t tx_buf_[kMaxCmd + 3]{};
  size_t tx_len_{0};

  uint32_t tx_trunc_last_log_ms_{0};

  static constexpr size_t RB_SIZE = 1024;
  static_assert((RB_SIZE & (RB_SIZE - 1)) == 0, "RB_SIZE must be power of two");
  uint8_t rx_rb_[RB_SIZE]{};
  size_t rb_head_{0};
  size_t rb_tail_{0};
  bool rb_overflow_{false};

  uint32_t rx_rb_drop_last_log_ms_{0};
  uint32_t rx_rb_drop_count_{0};

  uint32_t rx_drain_max_per_loop_{256};

  static constexpr size_t RX_READ_BUF_SIZE = 128;
  uint8_t rx_read_buf_[RX_READ_BUF_SIZE]{};

  uint32_t rx_time_budget_us_{1500};

  uint32_t frame_timeout_ms_{80};
  uint32_t frame_max_bytes_seen_{256};
  uint32_t frame_start_ms_{0};
  uint32_t frame_bytes_seen_{0};

  inline void rb_drop_oldest_() { rb_tail_ = (rb_tail_ + 1) & (RB_SIZE - 1); }

  inline bool rb_push_(uint8_t b) {
    size_t nh = (rb_head_ + 1) & (RB_SIZE - 1);
    if (nh == rb_tail_) {
      rb_overflow_ = true;
      rb_drop_oldest_();
      rx_rb_drop_count_++;
      nh = (rb_head_ + 1) & (RB_SIZE - 1);
      if (nh == rb_tail_)
        return false;
    }
    rx_rb_[rb_head_] = b;
    rb_head_ = nh;
    return true;
  }

  inline bool rb_pop_(uint8_t &b) {
    if (rb_head_ == rb_tail_)
      return false;
    b = rx_rb_[rb_tail_];
    rb_tail_ = (rb_tail_ + 1) & (RB_SIZE - 1);
    return true;
  }

  static constexpr size_t FRAME_BUF_SIZE = 64;
  bool in_frame_{false};
  size_t frame_len_{0};
  uint8_t ff_term_{0};
  bool frame_overflow_{false};
  uint8_t frame_buf_[FRAME_BUF_SIZE]{};

  uint32_t rx_frame_drop_last_log_ms_{0};
  uint32_t rx_frame_drop_count_{0};

  bool rx_enabled_{true};
  bool handshake_done_{false};
  int current_page_{-1};
  uint32_t init_deadline_ms_{0};

  uint32_t diag_deadline_ms_{0};
  bool saw_expected_reply_{false};

  uint32_t diag_timeout_ms_{200};

  uint32_t nextion_ready_cooldown_{500};
  uint32_t last_ready_ms_{0};

  bool upload_in_progress_{false};
  bool is_updating_{false};
  bool upload_first_chunk_sent_{false};
  bool ignore_is_setup_{false};
  uint32_t original_baud_rate_{0};
  uint32_t tft_size_{0};
  uint32_t content_length_{0};

  NxMode mode_{NxMode::INIT};

  CallbackManager<void()> on_setup_callback_;
  CallbackManager<void(int)> on_page_callback_;
  CallbackManager<void()> on_nextion_ready_callback_;
};

// ===================== Actions =====================

template<typename... Ts> class SetComponentValueAction : public Action<Ts...> {
 public:
  explicit SetComponentValueAction(NextionSimple *parent) : parent_(parent) {}
  void set_component_name(const std::string &nm) { this->component_name_ = nm; }
  void set_value(float v) { this->value_ = v; }
  void play(Ts... /*x*/) override { this->parent_->set_component_value(this->component_name_, this->value_); }

 protected:
  NextionSimple *parent_;
  std::string component_name_;
  float value_{0};
};

template<typename... Ts> class SetComponentTextAction : public Action<Ts...> {
 public:
  explicit SetComponentTextAction(NextionSimple *parent) : parent_(parent) {}
  void set_component_name(const std::string &nm) { this->component_name_ = nm; }
  void set_text(const std::string &t) { this->text_ = t; }
  void play(Ts... /*x*/) override { this->parent_->set_component_text(this->component_name_, this->text_); }

 protected:
  NextionSimple *parent_;
  std::string component_name_;
  std::string text_;
};

template<typename... Ts> class SetComponentTextPrintfAction : public Action<Ts...> {
 public:
  explicit SetComponentTextPrintfAction(NextionSimple *parent) : parent_(parent) {}
  void set_component_name(const std::string &nm) { this->component_name_ = nm; }
  void set_format(const char *f) { this->format_ = f; }
  void play(Ts... /*x*/) override { this->parent_->set_component_text_printf(this->component_name_, this->format_); }

 protected:
  NextionSimple *parent_;
  std::string component_name_;
  const char *format_{nullptr};
};

template<typename... Ts> class SetComponentPiccAction : public Action<Ts...> {
 public:
  explicit SetComponentPiccAction(NextionSimple *parent) : parent_(parent) {}
  void set_component_name(const std::string &nm) { this->component_name_ = nm; }
  void set_value(int v) { this->value_ = v; }
  void play(Ts... /*x*/) override { this->parent_->set_component_picc(this->component_name_, this->value_); }

 protected:
  NextionSimple *parent_;
  std::string component_name_;
  int value_{0};
};

template<typename... Ts> class SetComponentPicc1Action : public Action<Ts...> {
 public:
  explicit SetComponentPicc1Action(NextionSimple *parent) : parent_(parent) {}
  void set_component_name(const std::string &nm) { this->component_name_ = nm; }
  void set_value(int v) { this->value_ = v; }
  void play(Ts... /*x*/) override { this->parent_->set_component_picc1(this->component_name_, this->value_); }

 protected:
  NextionSimple *parent_;
  std::string component_name_;
  int value_{0};
};

template<typename... Ts> class SetComponentBackgroundColorAction : public Action<Ts...> {
 public:
  explicit SetComponentBackgroundColorAction(NextionSimple *parent) : parent_(parent) {}
  void set_component_name(const std::string &nm) { this->component_name_ = nm; }
  void set_color(int c) { this->color_ = c; }
  void play(Ts... /*x*/) override { this->parent_->set_component_background_color(this->component_name_, this->color_); }

 protected:
  NextionSimple *parent_;
  std::string component_name_;
  int color_{0};
};

template<typename... Ts> class SetComponentFontColorAction : public Action<Ts...> {
 public:
  explicit SetComponentFontColorAction(NextionSimple *parent) : parent_(parent) {}
  void set_component_name(const std::string &nm) { this->component_name_ = nm; }
  void set_color(int c) { this->color_ = c; }
  void play(Ts... /*x*/) override { this->parent_->set_component_font_color(this->component_name_, this->color_); }

 protected:
  NextionSimple *parent_;
  std::string component_name_;
  int color_{0};
};

template<typename... Ts> class SetComponentVisibilityAction : public Action<Ts...> {
 public:
  explicit SetComponentVisibilityAction(NextionSimple *parent) : parent_(parent) {}
  void set_component_name(const std::string &nm) { this->component_name_ = nm; }
  void set_state(bool s) { this->state_ = s ? 1 : 0; }
  void play(Ts... /*x*/) override { this->parent_->set_component_visibility(this->component_name_, this->state_); }

 protected:
  NextionSimple *parent_;
  std::string component_name_;
  int state_{1};
};

template<typename... Ts> class SetPageAction : public Action<Ts...> {
 public:
  explicit SetPageAction(NextionSimple *parent) : parent_(parent) {}
  void set_page(int p) { this->page_ = p; }
  void play(Ts... /*x*/) override { this->parent_->set_page(this->page_); }

 protected:
  NextionSimple *parent_;
  int page_{0};
};

// ===================== Triggers =====================

class NextionSetupTrigger : public Trigger<> {
 public:
  explicit NextionSetupTrigger(NextionSimple *parent) {
    parent->add_on_setup_callback([this]() { this->trigger(); });
  }
};

class NextionPageTrigger : public Trigger<int> {
 public:
  explicit NextionPageTrigger(NextionSimple *parent) {
    parent->add_on_page_callback([this](int page) { this->trigger(page); });
  }
};

class NextionReadyTrigger : public Trigger<> {
 public:
  explicit NextionReadyTrigger(NextionSimple *parent) {
    parent->add_on_nextion_ready_callback([this]() { this->trigger(); });
  }
};

}  // namespace nextion_simple
}  // namespace esphome

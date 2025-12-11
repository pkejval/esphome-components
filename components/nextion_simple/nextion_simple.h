#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/automation.h"
#include "esphome/core/color.h"

#include <cstdarg>
#include <cstring>
#include <functional>

namespace esphome {
namespace nextion_simple {

class NextionSimple : public Component {
 public:
  NextionSimple();

  // esphome::Component
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::PROCESSOR; }

  // Wiring
  void set_uart_parent(uart::UARTComponent *parent) { this->uart_parent_ = parent; }
  void set_tft_url(const std::string &tft_url) { this->tft_url_ = tft_url; }

  // High-level API (hot paths bez alokací)
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
  void set_nextion_ready_cooldown(uint32_t cooldown) { nextion_ready_cooldown_ = cooldown; }
  void set_page(int page);
  void set_page(const std::string &page_name);

  // Low-level fast send: one buffered write + 0xFF 0xFF 0xFF
  inline void send_command(const char *cmd, size_t len) {
    if (this->upload_in_progress_ || this->uart_parent_ == nullptr)
      return;
    if (len > kMaxCmd)
      len = kMaxCmd;
    char buffer[kMaxCmd + 3];
    memcpy(buffer, cmd, len);
    buffer[len] = static_cast<char>(0xFF);
    buffer[len + 1] = static_cast<char>(0xFF);
    buffer[len + 2] = static_cast<char>(0xFF);
    this->uart_parent_->write_array(reinterpret_cast<const uint8_t *>(buffer), len + 3);
  }
  void send_command_printf(const char *fmt, ...);

  // Maintenance
  void reset_nextion();
  void upload_tft();
  bool is_uploading() const { return this->upload_in_progress_; }
  int get_current_page() const { return this->current_page_; }

  // Callbacks
  void add_on_setup_callback(std::function<void()> &&callback) { this->on_setup_callback_.add(std::move(callback)); }
  void add_on_page_callback(std::function<void(int)> &&callback) { this->on_page_callback_.add(std::move(callback)); }
  void add_on_nextion_ready_callback(std::function<void()> &&callback) {
    this->on_nextion_ready_callback_.add(std::move(callback));
  }

  static const char *TAG;

 protected:
  // ====== Upload internals ======
  bool upload_tft_arduino_();
  bool upload_tft_esp_idf_();

  // (ESP-IDF) helpers
  bool prepare_nextion_for_upload_idf_(uint32_t baud_rate);
  bool wait_for_ack_idf_(uint32_t timeout_ms, std::string &out);
  int upload_by_chunks_idf_(void *http_client, uint32_t &range_start);  // forward decl (esp_http_client_handle_t*)

  // ====== INIT-only RX parser ======
  void drain_uart_into_ring_();
  void parse_from_ring_init_only_();
  void handle_frame_init_only_(const uint8_t *frame, size_t len);

  // ====== Modes ======
  enum class NxMode : uint8_t { INIT, RUN_WRITEONLY, DIAG_CHECK };
  void start_init_handshake_();
  void enter_writeonly_mode_();
  void request_health_check_();
  void diagnostic_tick_();

  // ====== Utils ======
  inline int color_to_integer_(Color color) {
    uint16_t r = (color.r >> 3) & 0x1F;
    uint16_t g = (color.g >> 2) & 0x3F;
    uint16_t b = (color.b >> 3) & 0x1F;
    return (r << 11) | (g << 5) | (b);
  }

  uint32_t get_free_heap_();

  // ====== State ======
  uart::UARTComponent *uart_parent_{nullptr};
  std::string tft_url_;

  // TX
  static constexpr size_t kMaxCmd = 256;
  uint8_t bkcmd_{3};  // INIT chce odpovědi, runtime 0

  // RX ring
  static constexpr size_t RB_SIZE = 1024;  // 2^N
  uint8_t rx_rb_[RB_SIZE]{};
  size_t rb_head_{0};
  size_t rb_tail_{0};
  inline bool rb_push_(uint8_t b) {
    size_t nh = (rb_head_ + 1) & (RB_SIZE - 1);
    if (nh == rb_tail_)
      return false;
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

  // INIT parser state
  bool rx_enabled_{true};
  bool handshake_done_{false};
  int current_page_{-1};
  uint32_t init_deadline_ms_{0};

  // Optional diag
  uint32_t diag_deadline_ms_{0};
  bool saw_expected_reply_{false};

  // Ready cooldown
  uint32_t nextion_ready_cooldown_{500};
  uint32_t last_ready_ms_{0};

  // Upload flags & params
  bool upload_in_progress_{false};
  bool is_updating_{false};
  bool upload_first_chunk_sent_{false};
  bool ignore_is_setup_{false};
  uint32_t original_baud_rate_{0};
  uint32_t tft_size_{0};
  uint32_t content_length_{0};

  // Mode
  NxMode mode_{NxMode::INIT};

  // Callbacks
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
  void play(Ts... /*x*/) override {
    this->parent_->set_component_background_color(this->component_name_, this->color_);
  }

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
  void set_page(int p) { page_ = p; }
  void play(Ts... /*x*/) override { this->parent_->set_page(this->page_); }

 private:
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

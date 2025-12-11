#pragma once

#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"

#include <cinttypes>
#include <stdint.h>

#if __has_include("esp_idf_version.h")
#include "esp_idf_version.h"
#endif

#if __has_include("driver/gpio_filter.h")
#include "driver/gpio_filter.h"
#endif

#if __has_include("driver/rmt_rx.h")
#include "driver/rmt_rx.h"
#endif

#if __has_include("freertos/FreeRTOS.h")
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

#ifndef LIKELY
#define LIKELY(x) (__builtin_expect(!!(x), 1))
#endif
#ifndef UNLIKELY
#define UNLIKELY(x) (__builtin_expect(!!(x), 0))
#endif

namespace esphome {
namespace pulse_meter {

class PulseMeterSensor : public sensor::Sensor, public Component {
 public:
  enum InternalFilterMode {
    FILTER_EDGE = 0,
    FILTER_PULSE,
  };

  void set_pin(InternalGPIOPin *pin) { this->pin_ = pin; }

  void set_filter_us(uint32_t filter) {
    this->filter_us_ = filter;
    this->update_hysteresis_defaults_();
  }

  void set_timeout_us(uint32_t timeout) { this->timeout_us_ = timeout; }
  void set_total_sensor(sensor::Sensor *sensor) { this->total_sensor_ = sensor; }
  void set_filter_mode(InternalFilterMode mode) { this->filter_mode_ = mode; }

  void set_pulse_hysteresis_us(uint32_t min_low_us, uint32_t min_high_us) {
    this->min_low_us_ = min_low_us;
    this->min_high_us_ = min_high_us;
    this->coalesce_min_us_ = (this->min_low_us_ < this->min_high_us_) ? this->min_low_us_ : this->min_high_us_;
  }

  void set_total_pulses(uint32_t pulses);

  void setup() override;
  void loop() override;
  float get_setup_priority() const override;
  void dump_config() override;

 protected:
  static void IRAM_ATTR edge_intr(PulseMeterSensor *sensor);
  static void IRAM_ATTR pulse_intr(PulseMeterSensor *sensor);

#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5) && __has_include("driver/rmt_rx.h")
  static bool IRAM_ATTR rmt_rx_done_cb_(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata,
                                        void *user_ctx);
#endif

#if (defined(portNUM_PROCESSORS) && (portNUM_PROCESSORS > 1))
  static void attach_isr_task_(void *arg);
#endif

  // wrap-safe: true, pokud now < deadline i přes wrap micros()
  static inline bool IRAM_ATTR before_deadline_(uint32_t now, uint32_t deadline) {
    return (int32_t) (now - deadline) < 0;
  }

  void update_hysteresis_defaults_() {
    this->min_low_us_ = (this->filter_us_ * 4U) / 5U;   // 0.8x
    this->min_high_us_ = (this->filter_us_ * 6U) / 5U;  // 1.2x
    this->coalesce_min_us_ = (this->min_low_us_ < this->min_high_us_) ? this->min_low_us_ : this->min_high_us_;
  }

  inline void update_period_estimate_(uint32_t delta_us, uint32_t count) {
    const float w = 0.25f;
    const float p = float(delta_us) / float(count);
    if (this->period_estimate_us_ <= 0.0f)
      this->period_estimate_us_ = p;
    else
      this->period_estimate_us_ = (1.0f - w) * this->period_estimate_us_ + w * p;
  }

  inline void plan_next_check_(uint32_t now) {
    uint32_t soft_gap = this->timeout_us_ / 4U;
    if (this->period_estimate_us_ > 0.0f) {
      const uint32_t two_periods = (uint32_t) (2.0f * this->period_estimate_us_);
      if (two_periods > soft_gap)
        soft_gap = two_periods;
    }
    const uint32_t hard_deadline = this->last_processed_edge_us_ + this->timeout_us_;
    const uint32_t soft_deadline = now + soft_gap;
    this->next_timeout_check_us_ = (soft_deadline < hard_deadline) ? soft_deadline : hard_deadline;
  }

  InternalGPIOPin *pin_{nullptr};
  uint32_t filter_us_ = 0;
  uint32_t timeout_us_ = 1000000UL * 60UL * 5UL;
  sensor::Sensor *total_sensor_{nullptr};
  InternalFilterMode filter_mode_{FILTER_EDGE};

  enum class MeterState { INITIAL, RUNNING, TIMED_OUT };
  MeterState meter_state_ = MeterState::INITIAL;
  bool peeked_edge_ = false;
  uint32_t total_pulses_ = 0;
  uint32_t last_processed_edge_us_ = 0;

  struct State {
    uint32_t last_detected_edge_us_ = 0;
    uint32_t last_rising_edge_us_ = 0;
    uint32_t count_ = 0;
  } __attribute__((packed, aligned(4)));

  State state_[2];
  volatile State *set_ = state_;
  volatile State *get_ = state_ + 1;

  ISRInternalGPIOPin isr_pin_;

  struct EdgeState {
    uint32_t last_sent_edge_us_ = 0;
  };
  EdgeState edge_state_{};

  struct PulseState {
    uint32_t last_intr_ = 0;
    bool latched_ = false;
    bool last_pin_val_ = false;
  };
  PulseState pulse_state_{};

  volatile bool new_event_ = false;
  uint32_t next_timeout_check_us_ = 0;

  uint32_t min_low_us_ = 0;
  uint32_t min_high_us_ = 0;

  float period_estimate_us_ = 0.0f;

#if defined(ESP_IDF_VERSION) && __has_include("driver/gpio_filter.h")
  gpio_glitch_filter_handle_t glitch_filter_{nullptr};
#endif

#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5) && __has_include("driver/rmt_rx.h")
  bool use_rmt_ = false;
  rmt_channel_handle_t rmt_rx_channel_{nullptr};
  rmt_receive_config_t rmt_rx_cfg_{};
  volatile const rmt_symbol_word_t *rmt_recv_symbols_ = nullptr;
  volatile size_t rmt_recv_count_ = 0;
  uint32_t rmt_resolution_hz_ = 1000000UL;  // 1 us
#endif

  // Soft coalescing (pouze EDGE); wrap-safe deadline
  bool coalesce_enabled_edge_ = true;
  volatile uint32_t coalesce_until_us_ = 0;
  uint32_t coalesce_min_us_ = 0;
};

}  // namespace pulse_meter
}  // namespace esphome

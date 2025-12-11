#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/sensor/sensor.h"

#include <cinttypes>
#include <memory>
#include <atomic>

#if defined(USE_ESP32)
#include <driver/pulse_cnt.h>
#include <esp_timer.h>
#define HAS_PCNT
#endif

namespace esphome {
namespace pulse_counter {

enum PulseCounterCountMode {
  PULSE_COUNTER_DISABLE = 0,
  PULSE_COUNTER_INCREMENT,
  PULSE_COUNTER_DECREMENT,
};

using pulse_counter_t = int32_t;

struct PulseCounterStorageBase {
  virtual ~PulseCounterStorageBase() = default;
  virtual bool pulse_counter_setup(InternalGPIOPin *pin) = 0;
  virtual pulse_counter_t read_raw_value() = 0;

  InternalGPIOPin *pin{nullptr};
  PulseCounterCountMode rising_edge_mode{PULSE_COUNTER_INCREMENT};
  PulseCounterCountMode falling_edge_mode{PULSE_COUNTER_DISABLE};
  uint32_t filter_us{0};
  pulse_counter_t last_value{0};
};

struct BasicPulseCounterStorage : public PulseCounterStorageBase {
  bool pulse_counter_setup(InternalGPIOPin *pin) override;
  pulse_counter_t read_raw_value() override;
  ~BasicPulseCounterStorage() override;

  volatile pulse_counter_t counter{0};
  ISRInternalGPIOPin isr_pin;
  bool last_level_{false};
  uint32_t last_edge_us_{0};
  bool initialized_{false};
};

#ifdef HAS_PCNT
struct HwPulseCounterStorage : public PulseCounterStorageBase {
  bool pulse_counter_setup(InternalGPIOPin *pin) override;
  pulse_counter_t read_raw_value() override;
  ~HwPulseCounterStorage() override;

  pcnt_unit_handle_t unit{nullptr};
  pcnt_channel_handle_t channel{nullptr};
};
#endif

std::unique_ptr<PulseCounterStorageBase> get_storage(bool hw_pcnt = false);

class PulseCounterSensor : public sensor::Sensor, public PollingComponent {
 public:
  explicit PulseCounterSensor(bool hw_pcnt = false) : storage_(get_storage(hw_pcnt)) {}
  ~PulseCounterSensor();

  void set_pin(InternalGPIOPin *pin) { pin_ = pin; }
  void set_rising_edge_mode(PulseCounterCountMode mode) { storage_->rising_edge_mode = mode; }
  void set_falling_edge_mode(PulseCounterCountMode mode) { storage_->falling_edge_mode = mode; }
  void set_filter_us(uint32_t filter) { storage_->filter_us = filter; }
  void set_total_sensor(sensor::Sensor *total_sensor) { total_sensor_ = total_sensor; }

  void set_min_pulses_per_calc(uint32_t n) {
    min_pulses_for_calc_ = (n == 0 ? 1u : n);
    custom_tuning_ = true;
  }
  void set_max_accumulation_ms(uint32_t ms) {
    max_accumulation_us_ = static_cast<uint64_t>(ms) * 1000ULL;
    custom_tuning_ = true;
  }

  void set_total_pulses(uint32_t pulses);

  void setup() override;
  void update() override;
  void dump_config() override;
  void set_update_interval(uint32_t update_interval) override;

 protected:
  static void timer_callback(void *arg);

  InternalGPIOPin *pin_{nullptr};
  std::unique_ptr<PulseCounterStorageBase> storage_;
  uint64_t current_total_{0};
  sensor::Sensor *total_sensor_{nullptr};
  bool total_ever_published_{false};

  uint64_t accum_start_us_{0};
  uint64_t accum_dt_us_{0};
  uint32_t accum_pulses_{0};
  uint32_t min_pulses_for_calc_{3};
  uint64_t max_accumulation_us_{0};
  bool custom_tuning_{false};

#if defined(USE_ESP32)
  esp_timer_handle_t timer_handle_{nullptr};
  std::atomic<float> last_calculated_ppm_{NAN};
  std::atomic<bool> new_value_ready_{false};
  std::atomic<int64_t> pending_total_delta_{0};
  uint64_t last_tick_us_{0};
#else
  uint64_t last_time_us_{0};
#endif
};

}  // namespace pulse_counter
}  // namespace esphome

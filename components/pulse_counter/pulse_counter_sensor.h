#pragma once

#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"

#include <atomic>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <memory>

#if defined(USE_ESP32)
#include <esp_timer.h>
#include <soc/soc_caps.h>
#include <esp_cpu.h>

#if __has_include("esp_clk_tree.h")
#include "esp_clk_tree.h"
#include "esp_err.h"
#define ESPHOME_PULSE_COUNTER_HAS_CLK_TREE 1
#else
#define ESPHOME_PULSE_COUNTER_HAS_CLK_TREE 0
#endif

#if __has_include("soc/clk_tree_defs.h")
#include "soc/clk_tree_defs.h"
#define ESPHOME_PULSE_COUNTER_HAS_CLK_TREE_DEFS 1
#else
#define ESPHOME_PULSE_COUNTER_HAS_CLK_TREE_DEFS 0
#endif

#if __has_include(<esp_clk.h>)
#include <esp_clk.h>
#define ESPHOME_PULSE_COUNTER_HAS_ESP_CLK 1
#else
#define ESPHOME_PULSE_COUNTER_HAS_ESP_CLK 0
#endif

#if __has_include(<Arduino.h>)
#include <Arduino.h>
#define ESPHOME_PULSE_COUNTER_HAS_ARDUINO 1
#else
#define ESPHOME_PULSE_COUNTER_HAS_ARDUINO 0
#endif

#if SOC_PCNT_SUPPORTED
#include <driver/pulse_cnt.h>
#define HAS_PCNT
#endif
#endif  // USE_ESP32

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
  virtual pulse_counter_t read_delta() = 0;
  virtual int64_t read_total() = 0;

  InternalGPIOPin *pin{nullptr};
  PulseCounterCountMode rising_edge_mode{PULSE_COUNTER_INCREMENT};
  PulseCounterCountMode falling_edge_mode{PULSE_COUNTER_DISABLE};
  uint32_t filter_us{0};
};

struct BasicPulseCounterStorage : public PulseCounterStorageBase {
  bool pulse_counter_setup(InternalGPIOPin *pin) override;
  pulse_counter_t read_delta() override;
  int64_t read_total() override;
  ~BasicPulseCounterStorage() override;

 protected:
  static void IRAM_ATTR gpio_intr(BasicPulseCounterStorage *arg);

#if defined(USE_ESP32)
  inline uint32_t now_ticks_() const { return esp_cpu_get_cycle_count(); }

  static inline uint32_t cpu_freq_hz_() {
#if ESPHOME_PULSE_COUNTER_HAS_CLK_TREE && ESPHOME_PULSE_COUNTER_HAS_CLK_TREE_DEFS
    uint32_t hz = 0;
    const esp_err_t err =
        esp_clk_tree_src_get_freq_hz(SOC_MOD_CLK_CPU, ESP_CLK_TREE_SRC_FREQ_PRECISION_APPROX, &hz);
    if (err == ESP_OK && hz != 0) {
      return hz;
    }
#endif

#if ESPHOME_PULSE_COUNTER_HAS_ESP_CLK
    const uint32_t hz = esp_clk_cpu_freq();
    if (hz != 0) {
      return hz;
    }
#endif

#if ESPHOME_PULSE_COUNTER_HAS_ARDUINO
    const uint32_t mhz = static_cast<uint32_t>(ESP.getCpuFreqMHz());
    if (mhz != 0) {
      return mhz * 1000000UL;
    }
#endif

    return 240000000UL;
  }

#else
  inline uint32_t now_ticks_() const { return micros(); }
#endif

  ISRInternalGPIOPin isr_pin_;
  std::atomic<int32_t> counter_{0};
  std::atomic<int32_t> last_read_{0};

  volatile uint32_t last_edge_ticks_{0};
  uint32_t filter_ticks_{0};
};

#ifdef HAS_PCNT
struct HwPulseCounterStorage : public PulseCounterStorageBase {
  bool pulse_counter_setup(InternalGPIOPin *pin) override;
  pulse_counter_t read_delta() override;
  int64_t read_total() override;
  ~HwPulseCounterStorage() override;

 protected:
  static pcnt_channel_edge_action_t map_edge_(PulseCounterCountMode m);
  static bool on_pcnt_reach_(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx);

  bool configure_glitch_filter_();
  bool configure_watchpoints_();

  pcnt_unit_handle_t unit_{nullptr};
  pcnt_channel_handle_t channel_{nullptr};

  int16_t low_limit_{-30000};
  int16_t high_limit_{30000};

  std::atomic<int64_t> base_total_{0};
  std::atomic<uint32_t> seq_{0};

  int64_t last_abs_total_{0};
};
#endif

std::unique_ptr<PulseCounterStorageBase> get_storage(bool hw_pcnt = false);

class PulseCounterSensor : public sensor::Sensor, public PollingComponent {
 public:
  explicit PulseCounterSensor(bool hw_pcnt = false) : storage_(get_storage(hw_pcnt)) {}
  ~PulseCounterSensor() override;

  void set_pin(InternalGPIOPin *pin) { pin_ = pin; }
  void set_rising_edge_mode(PulseCounterCountMode mode) { storage_->rising_edge_mode = mode; }
  void set_falling_edge_mode(PulseCounterCountMode mode) { storage_->falling_edge_mode = mode; }
  void set_filter_us(uint32_t filter) { storage_->filter_us = filter; }
  void set_total_sensor(sensor::Sensor *total_sensor) { total_sensor_ = total_sensor; }

  void set_min_pulses_per_calc(uint32_t n);
  void set_max_accumulation_ms(uint32_t ms);

  void set_total_pulses(uint32_t pulses);

  void setup() override;
  void update() override;
  void dump_config() override;
  void set_update_interval(uint32_t update_interval) override;

 protected:
#if defined(USE_ESP32)
  static void timer_callback(void *arg);
#endif

  void reset_accumulation_();

  InternalGPIOPin *pin_{nullptr};
  std::unique_ptr<PulseCounterStorageBase> storage_;

  sensor::Sensor *total_sensor_{nullptr};
  bool total_ever_published_{false};

  int64_t published_total_{0};

  uint64_t accum_dt_us_{0};
  int64_t accum_pulses_{0};
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

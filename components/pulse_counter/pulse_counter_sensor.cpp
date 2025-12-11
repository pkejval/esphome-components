#include "pulse_counter_sensor.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

#include <limits>
#include <cmath>
#include <algorithm>

#if !defined(USE_ESP32)
#include <Arduino.h>
#endif

namespace esphome {
namespace pulse_counter {

static const char *const TAG = "pulse_counter";
static const char *const EDGE_MODE_TO_STRING[] = {"DISABLE", "INCREMENT", "DECREMENT"};

#ifdef HAS_PCNT
std::unique_ptr<PulseCounterStorageBase> get_storage(bool hw_pcnt) {
  if (hw_pcnt)
    return std::make_unique<HwPulseCounterStorage>();
  return std::make_unique<BasicPulseCounterStorage>();
}
#else
std::unique_ptr<PulseCounterStorageBase> get_storage(bool) { return std::make_unique<BasicPulseCounterStorage>(); }
#endif

// -------------------- Basic (bez HW PCNT; ponecháno pro kompatibilitu) --------------------

bool BasicPulseCounterStorage::pulse_counter_setup(InternalGPIOPin *pin) {
  this->pin = pin;
  this->pin->setup();
  this->isr_pin = this->pin->to_isr();
  this->last_value = 0;
  this->counter = 0;
  this->initialized_ = false;
  this->last_edge_us_ = 0;
  this->last_level_ = false;
  return true;
}

pulse_counter_t BasicPulseCounterStorage::read_raw_value() {
  // Minimalistická soft vzorkovací varianta: v update() se volá periodicky,
  // hrany detekujeme porovnáním úrovně a měkkým dead-timem.
  const uint32_t now = micros();
  const bool level = this->isr_pin.digital_read();

  if (!this->initialized_) {
    this->initialized_ = true;
    this->last_level_ = level;
    this->last_edge_us_ = now;
  } else if (level != this->last_level_) {
    const uint32_t dt = now - this->last_edge_us_;
    if (dt >= this->filter_us) {
      const auto mode = level ? this->rising_edge_mode : this->falling_edge_mode;
      switch (mode) {
        case PULSE_COUNTER_DISABLE:
          break;
        case PULSE_COUNTER_INCREMENT:
          this->counter = this->counter + 1;
          break;
        case PULSE_COUNTER_DECREMENT:
          this->counter = this->counter - 1;
          break;
      }
      this->last_edge_us_ = now;
    }
    this->last_level_ = level;
  }

  pulse_counter_t current;
  pulse_counter_t delta;
  {
    InterruptLock lk;
    current = this->counter;
    delta = current - this->last_value;
    this->last_value = current;
  }
  return delta;
}

BasicPulseCounterStorage::~BasicPulseCounterStorage() {
  // nic
}

// -------------------- Hardware PCNT (read & clear) --------------------

#ifdef HAS_PCNT
static pcnt_channel_edge_action_t map_edge(PulseCounterCountMode m) {
  switch (m) {
    case PULSE_COUNTER_INCREMENT:
      return PCNT_CHANNEL_EDGE_ACTION_INCREASE;
    case PULSE_COUNTER_DECREMENT:
      return PCNT_CHANNEL_EDGE_ACTION_DECREASE;
    default:
      return PCNT_CHANNEL_EDGE_ACTION_HOLD;
  }
}

bool HwPulseCounterStorage::pulse_counter_setup(InternalGPIOPin *pin) {
  this->pin = pin;
  this->pin->setup();

  pcnt_unit_config_t unit_cfg = {};
  unit_cfg.low_limit = std::numeric_limits<int16_t>::min();
  unit_cfg.high_limit = std::numeric_limits<int16_t>::max();

  esp_err_t err = pcnt_new_unit(&unit_cfg, &this->unit);
  if (err != ESP_OK || this->unit == nullptr) {
    ESP_LOGE(TAG, "Creating PCNT unit failed: %s", esp_err_to_name(err));
    return false;
  }

  pcnt_chan_config_t chan_cfg = {};
  chan_cfg.edge_gpio_num = this->pin->get_pin();
  chan_cfg.level_gpio_num = -1;

  err = pcnt_new_channel(this->unit, &chan_cfg, &this->channel);
  if (err != ESP_OK || this->channel == nullptr) {
    ESP_LOGE(TAG, "Creating PCNT channel failed: %s", esp_err_to_name(err));
    return false;
  }

  err =
      pcnt_channel_set_edge_action(this->channel, map_edge(this->rising_edge_mode), map_edge(this->falling_edge_mode));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Setting edge action failed: %s", esp_err_to_name(err));
  }

  err = pcnt_channel_set_level_action(this->channel, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Setting level action failed: %s", esp_err_to_name(err));
  }

  if (this->filter_us != 0) {
    // Bezpečný clamp na rozumný rozsah ESP-IDF (typicky do jednotek ms).
    // Zároveň převod us -> ns.
    const uint64_t req_ns = static_cast<uint64_t>(this->filter_us) * 1000ULL;

    // Konzervativní horní limit (např. 5 ms), aby IDF nevracel chybu.
    constexpr uint64_t MAX_NS = 5ULL * 1000ULL * 1000ULL;

    pcnt_glitch_filter_config_t gf = {};
    gf.max_glitch_ns = std::min<uint64_t>(req_ns, MAX_NS);

    err = pcnt_unit_set_glitch_filter(this->unit, &gf);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Setting glitch filter failed: %s", esp_err_to_name(err));
    }
  }

  err = pcnt_unit_enable(this->unit);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Enabling PCNT unit failed: %s", esp_err_to_name(err));
    return false;
  }

  err = pcnt_unit_clear_count(this->unit);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Clearing PCNT count failed: %s", esp_err_to_name(err));
    return false;
  }

  err = pcnt_unit_start(this->unit);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Starting PCNT unit failed: %s", esp_err_to_name(err));
    return false;
  }

  return true;
}

pulse_counter_t HwPulseCounterStorage::read_raw_value() {
  int value = 0;
  esp_err_t err = pcnt_unit_get_count(this->unit, &value);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Getting PCNT count failed: %s", esp_err_to_name(err));
    return 0;
  }

  // Pozn.: nepausujeme unit, aby nevznikala slepá okna; pulzy během clear
  // prostě spadnou do dalšího intervalu (správné časové rozdělení).
  err = pcnt_unit_clear_count(this->unit);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Clearing PCNT count failed: %s", esp_err_to_name(err));
  }

  return static_cast<pulse_counter_t>(value);
}

HwPulseCounterStorage::~HwPulseCounterStorage() {
  if (this->unit != nullptr)
    pcnt_unit_stop(this->unit);
  if (this->channel != nullptr) {
    pcnt_del_channel(this->channel);
    this->channel = nullptr;
  }
  if (this->unit != nullptr) {
    pcnt_del_unit(this->unit);
    this->unit = nullptr;
  }
}
#endif

// -------------------- Sensor wrapper (adaptivní akumulace pro přesnost) --------------------

PulseCounterSensor::~PulseCounterSensor() {
#if defined(USE_ESP32)
  if (this->timer_handle_ != nullptr) {
    esp_timer_stop(this->timer_handle_);
    esp_timer_delete(this->timer_handle_);
    this->timer_handle_ = nullptr;
  }
#endif
}

#if defined(USE_ESP32)
void PulseCounterSensor::timer_callback(void *arg) {
  auto *self = static_cast<PulseCounterSensor *>(arg);

  const uint64_t t = static_cast<uint64_t>(esp_timer_get_time());
  const pulse_counter_t raw = self->storage_->read_raw_value();

  if (self->last_tick_us_ == 0) {
    self->last_tick_us_ = t;
    // Inicializace akumulace
    self->accum_start_us_ = t;
    self->accum_dt_us_ = 0;
    self->accum_pulses_ = 0;
    if (raw != 0)
      self->pending_total_delta_.fetch_add(raw, std::memory_order_relaxed);
    return;
  }

  const uint64_t dt_us = t - self->last_tick_us_;
  self->last_tick_us_ = t;

  if (raw != 0) {
    self->pending_total_delta_.fetch_add(static_cast<int64_t>(raw), std::memory_order_relaxed);
  }
  if (dt_us == 0)
    return;

  // Akumulace pro přesnější výpočet
  self->accum_dt_us_ += dt_us;
  if (raw != 0)
    self->accum_pulses_ += static_cast<uint32_t>((raw > 0) ? raw : 0);

  // Urči max. akumulační čas (výchozí: 2 × update_interval), pokud nebyl ručně přepsán
  uint64_t max_accum_us = self->max_accumulation_us_;
  if (!self->custom_tuning_) {
    const uint64_t ui_us = static_cast<uint64_t>(self->get_update_interval()) * 1000ULL;
    max_accum_us = ui_us * 2ULL;
  }

  const bool enough_pulses = (self->accum_pulses_ >= self->min_pulses_for_calc_);
  const bool time_up = (self->accum_dt_us_ >= max_accum_us);

  if (enough_pulses || time_up) {
    double ppm = NAN;
    if (self->accum_dt_us_ > 0) {
      ppm = (static_cast<double>(self->accum_pulses_) * 60000000.0) / static_cast<double>(self->accum_dt_us_);
    }

    if (std::isfinite(ppm) && ppm >= 0.0 && ppm < 1e9) {
      self->last_calculated_ppm_.store(static_cast<float>(ppm), std::memory_order_relaxed);
      self->new_value_ready_.store(true, std::memory_order_release);
    }

    // Reset akumulace pro další interval
    self->accum_start_us_ = t;
    self->accum_dt_us_ = 0;
    self->accum_pulses_ = 0;
  }
}
#endif

void PulseCounterSensor::setup() {
  if (!this->storage_->pulse_counter_setup(this->pin_)) {
    this->mark_failed();
    return;
  }

#if defined(USE_ESP32)
  esp_timer_create_args_t timer_args = {};
  timer_args.callback = &PulseCounterSensor::timer_callback;
  timer_args.arg = this;
  timer_args.dispatch_method = ESP_TIMER_TASK;
  timer_args.name = "pulse_counter";

  esp_err_t err = esp_timer_create(&timer_args, &this->timer_handle_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_timer_create failed: %s", esp_err_to_name(err));
    this->mark_failed();
    return;
  }

  uint64_t period_us = static_cast<uint64_t>(this->get_update_interval()) * 1000ULL;
  if (period_us == 0)
    period_us = 10000ULL;  // 10 ms

  err = esp_timer_start_periodic(this->timer_handle_, period_us);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_timer_start_periodic failed: %s", esp_err_to_name(err));
    this->mark_failed();
    return;
  }
#endif

  if (this->total_sensor_ != nullptr) {
    this->total_sensor_->publish_state(static_cast<float>(this->current_total_));
    this->total_ever_published_ = true;
  }

  // Inicializace akumulačních parametrů (pokud neuživatel nepřepsal)
  if (!this->custom_tuning_) {
    const uint64_t ui_us = static_cast<uint64_t>(this->get_update_interval()) * 1000ULL;
    this->max_accumulation_us_ = ui_us * 2ULL;
  }
  this->accum_start_us_ = 0;
  this->accum_dt_us_ = 0;
  this->accum_pulses_ = 0;
}

void PulseCounterSensor::set_update_interval(uint32_t update_interval) {
  PollingComponent::set_update_interval(update_interval);
#if defined(USE_ESP32)
  if (this->timer_handle_ != nullptr) {
    esp_timer_stop(this->timer_handle_);
    uint64_t period_us = static_cast<uint64_t>(this->get_update_interval()) * 1000ULL;
    if (period_us == 0)
      period_us = 10000ULL;
    esp_err_t err = esp_timer_start_periodic(this->timer_handle_, period_us);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "esp_timer_start_periodic (restart) failed: %s", esp_err_to_name(err));
      this->mark_failed();
    }
    this->last_tick_us_ = 0;
  }
#endif

  // Při změně intervalu aktualizuj implicitní max_accum, když není custom
  if (!this->custom_tuning_) {
    const uint64_t ui_us = static_cast<uint64_t>(this->get_update_interval()) * 1000ULL;
    this->max_accumulation_us_ = ui_us * 2ULL;
  }
}

void PulseCounterSensor::set_total_pulses(uint32_t pulses) {
  this->current_total_ = static_cast<uint64_t>(pulses);
  if (this->total_sensor_ != nullptr) {
    this->total_sensor_->publish_state(static_cast<float>(this->current_total_));
    this->total_ever_published_ = true;
  }
}

void PulseCounterSensor::dump_config() {
  LOG_SENSOR("", "Pulse Counter", this);
  LOG_PIN("  Pin: ", this->pin_);
  ESP_LOGCONFIG(TAG,
                "  Rising Edge: %s\n"
                "  Falling Edge: %s\n"
                "  Filtering pulses shorter than %" PRIu32 " us\n"
                "  Min pulses per calc: %" PRIu32 "\n"
                "  Max accumulation (ms): %" PRIu64,
                EDGE_MODE_TO_STRING[this->storage_->rising_edge_mode],
                EDGE_MODE_TO_STRING[this->storage_->falling_edge_mode], this->storage_->filter_us,
                this->min_pulses_for_calc_, this->max_accumulation_us_ / 1000ULL);
  LOG_UPDATE_INTERVAL(this);
}

void PulseCounterSensor::update() {
#if defined(USE_ESP32)
  if (this->new_value_ready_.load(std::memory_order_acquire)) {
    this->new_value_ready_.store(false, std::memory_order_release);
    const float ppm = this->last_calculated_ppm_.load(std::memory_order_relaxed);
    if (std::isfinite(ppm)) {
      this->publish_state(ppm);
    }
  }

  if (this->total_sensor_ != nullptr) {
    const int64_t delta = this->pending_total_delta_.exchange(0, std::memory_order_acq_rel);
    if (delta != 0 || !this->total_ever_published_) {
      if (delta != 0) {
        const int64_t new_total = static_cast<int64_t>(this->current_total_) + std::max<int64_t>(0, delta);
        this->current_total_ = static_cast<uint64_t>(std::max<int64_t>(0, new_total));
      }
      this->total_sensor_->publish_state(static_cast<float>(this->current_total_));
      this->total_ever_published_ = true;
    }
  }
#else
  const pulse_counter_t raw = this->storage_->read_raw_value();
  const uint64_t t = static_cast<uint64_t>(micros());

  if (this->last_time_us_ != 0) {
    const uint64_t dt = t - this->last_time_us_;
    if (dt > 0) {
      const double ppm = (static_cast<double>(raw) * 60000000.0) / static_cast<double>(dt);
      if (std::isfinite(ppm)) {
        this->publish_state(static_cast<float>(ppm));
      }
    }
  }
  this->last_time_us_ = t;

  if (this->total_sensor_ != nullptr) {
    if (raw > 0) {
      this->current_total_ += static_cast<uint64_t>(raw);
    }
    if (!this->total_ever_published_ || raw > 0) {
      this->total_sensor_->publish_state(static_cast<float>(this->current_total_));
      this->total_ever_published_ = true;
    }
  }
#endif
}

}  // namespace pulse_counter
}  // namespace esphome

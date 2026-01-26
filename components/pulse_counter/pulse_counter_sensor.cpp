// pulse_counter_sensor.cpp
#include "pulse_counter_sensor.h"

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace esphome {
namespace pulse_counter {

static const char *const TAG = "pulse_counter";
static const char *const EDGE_MODE_TO_STRING[] = {"DISABLE", "INCREMENT", "DECREMENT"};

std::unique_ptr<PulseCounterStorageBase> get_storage(bool hw_pcnt) {
#ifdef HAS_PCNT
  if (hw_pcnt)
    return std::make_unique<HwPulseCounterStorage>();
  return std::make_unique<BasicPulseCounterStorage>();
#else
  (void) hw_pcnt;
  return std::make_unique<BasicPulseCounterStorage>();
#endif
}

// ===================== Basic (ISR) =====================

void IRAM_ATTR BasicPulseCounterStorage::gpio_intr(BasicPulseCounterStorage *arg) {
  const uint32_t now = arg->now_ticks_();

  const uint32_t last = arg->last_edge_ticks_;
  if (last != 0) {
    const uint32_t dt = now - last;
    if (arg->filter_ticks_ != 0 && dt < arg->filter_ticks_) {
      // Important: Do NOT update last_edge_ticks_ on rejected edges,
      // otherwise high-frequency noise would "reset" the filter window forever.
      return;
    }
  }

  // Accept this edge: update baseline time/ticks.
  arg->last_edge_ticks_ = now;

  const bool level = arg->isr_pin_.digital_read();
  const PulseCounterCountMode mode = level ? arg->rising_edge_mode : arg->falling_edge_mode;

  switch (mode) {
    case PULSE_COUNTER_DISABLE:
      break;
    case PULSE_COUNTER_INCREMENT:
      arg->counter_.fetch_add(1, std::memory_order_relaxed);
      break;
    case PULSE_COUNTER_DECREMENT:
      arg->counter_.fetch_sub(1, std::memory_order_relaxed);
      break;
  }
}

bool BasicPulseCounterStorage::pulse_counter_setup(InternalGPIOPin *pin) {
  this->pin = pin;
  this->pin->setup();
  this->isr_pin_ = this->pin->to_isr();

  this->counter_.store(0, std::memory_order_relaxed);
  this->last_read_.store(0, std::memory_order_relaxed);

#if defined(USE_ESP32)
  const uint32_t cpu_mhz = static_cast<uint32_t>(esp_clk_cpu_freq() / 1000000UL);
  const uint64_t ft = static_cast<uint64_t>(this->filter_us) * static_cast<uint64_t>(cpu_mhz);
  this->filter_ticks_ = (ft > 0xFFFFFFFFULL) ? 0xFFFFFFFFUL : static_cast<uint32_t>(ft);
#else
  this->filter_ticks_ = this->filter_us;
#endif

  // Baseline set to 0 so first accepted edge always arms the filter correctly.
  this->last_edge_ticks_ = 0;

  this->pin->attach_interrupt(BasicPulseCounterStorage::gpio_intr, this, gpio::INTERRUPT_ANY_EDGE);
  return true;
}

pulse_counter_t BasicPulseCounterStorage::read_delta() {
  const int32_t cur = this->counter_.load(std::memory_order_relaxed);
  const int32_t prev = this->last_read_.exchange(cur, std::memory_order_acq_rel);
  return static_cast<pulse_counter_t>(cur - prev);
}

int64_t BasicPulseCounterStorage::read_total() {
  return static_cast<int64_t>(this->counter_.load(std::memory_order_relaxed));
}

BasicPulseCounterStorage::~BasicPulseCounterStorage() {}

// ===================== ESP32 PCNT (ESP-IDF new driver) =====================
#ifdef HAS_PCNT

pcnt_channel_edge_action_t HwPulseCounterStorage::map_edge_(PulseCounterCountMode m) {
  switch (m) {
    case PULSE_COUNTER_INCREMENT:
      return PCNT_CHANNEL_EDGE_ACTION_INCREASE;
    case PULSE_COUNTER_DECREMENT:
      return PCNT_CHANNEL_EDGE_ACTION_DECREASE;
    default:
      return PCNT_CHANNEL_EDGE_ACTION_HOLD;
  }
}

bool HwPulseCounterStorage::on_pcnt_reach_(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata,
                                         void *user_ctx) {
  auto *self = static_cast<HwPulseCounterStorage *>(user_ctx);
  if (self == nullptr || edata == nullptr)
    return false;

  const int16_t wp = static_cast<int16_t>(edata->watch_point_value);
  if (wp != self->high_limit_ && wp != self->low_limit_)
    return false;

  // Seqlock write section (odd -> write -> even)
  self->seq_.fetch_add(1, std::memory_order_acq_rel);

  self->base_total_.fetch_add(static_cast<int64_t>(wp), std::memory_order_relaxed);
  (void) pcnt_unit_clear_count(unit);

  self->seq_.fetch_add(1, std::memory_order_acq_rel);
  return true;
}

bool HwPulseCounterStorage::configure_glitch_filter_() {
  if (this->filter_us == 0)
    return true;

  const uint64_t req_ns = static_cast<uint64_t>(this->filter_us) * 1000ULL;
  constexpr uint64_t MAX_NS = 13000ULL;
  const uint64_t use_ns = std::min<uint64_t>(req_ns, MAX_NS);

  pcnt_glitch_filter_config_t gf{};
  gf.max_glitch_ns = use_ns;

  const esp_err_t err = pcnt_unit_set_glitch_filter(this->unit_, &gf);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "PCNT glitch filter set failed (requested=%" PRIu32 "us clamped=%" PRIu64 "ns): %s",
             this->filter_us, use_ns, esp_err_to_name(err));
    return true;
  }

  if (use_ns != req_ns) {
    ESP_LOGW(TAG, "PCNT glitch filter clamped: requested=%" PRIu32 "us -> applied=%" PRIu64 "ns", this->filter_us,
             use_ns);
  } else {
    ESP_LOGCONFIG(TAG, "PCNT glitch filter: applied=%" PRIu64 "ns", use_ns);
  }

  return true;
}

bool HwPulseCounterStorage::configure_watchpoints_() {
  pcnt_event_callbacks_t cbs{};
  cbs.on_reach = &HwPulseCounterStorage::on_pcnt_reach_;

  esp_err_t err = pcnt_unit_register_event_callbacks(this->unit_, &cbs, this);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Registering PCNT callbacks failed: %s", esp_err_to_name(err));
    return false;
  }

  err = pcnt_unit_add_watch_point(this->unit_, this->high_limit_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Adding PCNT watchpoint (high=%d) failed: %s", this->high_limit_, esp_err_to_name(err));
    return false;
  }

  err = pcnt_unit_add_watch_point(this->unit_, this->low_limit_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Adding PCNT watchpoint (low=%d) failed: %s", this->low_limit_, esp_err_to_name(err));
    return false;
  }

  return true;
}

bool HwPulseCounterStorage::pulse_counter_setup(InternalGPIOPin *pin) {
  this->pin = pin;
  this->pin->setup();

  pcnt_unit_config_t unit_cfg{};
  unit_cfg.low_limit = this->low_limit_;
  unit_cfg.high_limit = this->high_limit_;

  esp_err_t err = pcnt_new_unit(&unit_cfg, &this->unit_);
  if (err != ESP_OK || this->unit_ == nullptr) {
    ESP_LOGE(TAG, "Creating PCNT unit failed: %s", esp_err_to_name(err));
    return false;
  }

  pcnt_chan_config_t chan_cfg{};
  chan_cfg.edge_gpio_num = this->pin->get_pin();
  chan_cfg.level_gpio_num = -1;

  err = pcnt_new_channel(this->unit_, &chan_cfg, &this->channel_);
  if (err != ESP_OK || this->channel_ == nullptr) {
    ESP_LOGE(TAG, "Creating PCNT channel failed: %s", esp_err_to_name(err));
    return false;
  }

  err = pcnt_channel_set_edge_action(this->channel_, map_edge_(this->rising_edge_mode), map_edge_(this->falling_edge_mode));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Setting PCNT edge action failed: %s", esp_err_to_name(err));
    return false;
  }

  err = pcnt_channel_set_level_action(this->channel_, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Setting PCNT level action failed: %s", esp_err_to_name(err));
    return false;
  }

  if (!this->configure_glitch_filter_())
    return false;

  if (!this->configure_watchpoints_())
    return false;

  err = pcnt_unit_enable(this->unit_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Enabling PCNT unit failed: %s", esp_err_to_name(err));
    return false;
  }

  err = pcnt_unit_clear_count(this->unit_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Clearing PCNT count failed: %s", esp_err_to_name(err));
    return false;
  }

  this->base_total_.store(0, std::memory_order_relaxed);
  this->seq_.store(0, std::memory_order_relaxed);
  this->last_abs_total_ = 0;

  err = pcnt_unit_start(this->unit_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Starting PCNT unit failed: %s", esp_err_to_name(err));
    return false;
  }

  ESP_LOGCONFIG(TAG, "PCNT enabled (range [%d..%d])", this->low_limit_, this->high_limit_);
  return true;
}

int64_t HwPulseCounterStorage::read_total() {
  for (uint32_t i = 0; i < 8; i++) {
    const uint32_t s1 = this->seq_.load(std::memory_order_acquire);
    if ((s1 & 1U) != 0)
      continue;

    const int64_t base = this->base_total_.load(std::memory_order_relaxed);

    int value = 0;
    const esp_err_t err = pcnt_unit_get_count(this->unit_, &value);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Getting PCNT count failed: %s", esp_err_to_name(err));
      return base;
    }

    const uint32_t s2 = this->seq_.load(std::memory_order_acquire);
    if (s1 == s2 && (s2 & 1U) == 0) {
      return base + static_cast<int64_t>(value);
    }
  }

  const int64_t base = this->base_total_.load(std::memory_order_relaxed);
  int value = 0;
  if (pcnt_unit_get_count(this->unit_, &value) != ESP_OK)
    value = 0;
  return base + static_cast<int64_t>(value);
}

pulse_counter_t HwPulseCounterStorage::read_delta() {
  const int64_t abs_total = this->read_total();
  const int64_t delta64 = abs_total - this->last_abs_total_;
  this->last_abs_total_ = abs_total;

  if (delta64 > static_cast<int64_t>(std::numeric_limits<pulse_counter_t>::max()))
    return std::numeric_limits<pulse_counter_t>::max();
  if (delta64 < static_cast<int64_t>(std::numeric_limits<pulse_counter_t>::min()))
    return std::numeric_limits<pulse_counter_t>::min();
  return static_cast<pulse_counter_t>(delta64);
}

HwPulseCounterStorage::~HwPulseCounterStorage() {
  if (this->unit_ != nullptr) {
    (void) pcnt_unit_stop(this->unit_);
  }
  if (this->channel_ != nullptr) {
    pcnt_del_channel(this->channel_);
    this->channel_ = nullptr;
  }
  if (this->unit_ != nullptr) {
    pcnt_del_unit(this->unit_);
    this->unit_ = nullptr;
  }
}

#endif  // HAS_PCNT

// ===================== Sensor wrapper =====================

PulseCounterSensor::~PulseCounterSensor() {
#if defined(USE_ESP32)
  if (this->timer_handle_ != nullptr) {
    (void) esp_timer_stop(this->timer_handle_);
    (void) esp_timer_delete(this->timer_handle_);
    this->timer_handle_ = nullptr;
  }
#endif
}

void PulseCounterSensor::set_min_pulses_per_calc(uint32_t n) {
  this->min_pulses_for_calc_ = (n == 0 ? 1u : n);
  this->custom_tuning_ = true;
}

void PulseCounterSensor::set_max_accumulation_ms(uint32_t ms) {
  this->max_accumulation_us_ = static_cast<uint64_t>(ms) * 1000ULL;
  this->custom_tuning_ = true;
}

void PulseCounterSensor::reset_accumulation_() {
  this->accum_dt_us_ = 0;
  this->accum_pulses_ = 0;
}

void PulseCounterSensor::setup() {
  if (!this->storage_->pulse_counter_setup(this->pin_)) {
    this->mark_failed();
    return;
  }

  if (!this->custom_tuning_) {
    const uint64_t ui_us = static_cast<uint64_t>(this->get_update_interval()) * 1000ULL;
    this->max_accumulation_us_ = std::max<uint64_t>(ui_us * 2ULL, 20000ULL);
  }

#if defined(USE_ESP32)
  esp_timer_create_args_t timer_args{};
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
    period_us = 10000ULL;

  err = esp_timer_start_periodic(this->timer_handle_, period_us);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_timer_start_periodic failed: %s", esp_err_to_name(err));
    this->mark_failed();
    return;
  }

  this->last_tick_us_ = 0;
  this->reset_accumulation_();
#endif

  if (this->total_sensor_ != nullptr) {
    this->published_total_ = this->storage_->read_total();
    if (this->published_total_ < 0)
      this->published_total_ = 0;
    this->total_sensor_->publish_state(static_cast<float>(this->published_total_));
    this->total_ever_published_ = true;
  }
}

void PulseCounterSensor::set_update_interval(uint32_t update_interval) {
  PollingComponent::set_update_interval(update_interval);

  if (!this->custom_tuning_) {
    const uint64_t ui_us = static_cast<uint64_t>(this->get_update_interval()) * 1000ULL;
    this->max_accumulation_us_ = std::max<uint64_t>(ui_us * 2ULL, 20000ULL);
  }

#if defined(USE_ESP32)
  if (this->timer_handle_ != nullptr) {
    (void) esp_timer_stop(this->timer_handle_);

    uint64_t period_us = static_cast<uint64_t>(this->get_update_interval()) * 1000ULL;
    if (period_us == 0)
      period_us = 10000ULL;

    const esp_err_t err = esp_timer_start_periodic(this->timer_handle_, period_us);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "esp_timer_start_periodic (restart) failed: %s", esp_err_to_name(err));
      this->mark_failed();
      return;
    }

    this->last_tick_us_ = 0;
    this->reset_accumulation_();
  }
#endif
}

void PulseCounterSensor::set_total_pulses(uint32_t pulses) {
  this->published_total_ = static_cast<int64_t>(pulses);
  if (this->total_sensor_ != nullptr) {
    this->total_sensor_->publish_state(static_cast<float>(this->published_total_));
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

#if defined(USE_ESP32)
void PulseCounterSensor::timer_callback(void *arg) {
  auto *self = static_cast<PulseCounterSensor *>(arg);
  const uint64_t t = static_cast<uint64_t>(esp_timer_get_time());

  const pulse_counter_t delta = self->storage_->read_delta();

  if (self->last_tick_us_ == 0) {
    self->last_tick_us_ = t;
    if (delta != 0)
      self->pending_total_delta_.fetch_add(static_cast<int64_t>(delta), std::memory_order_relaxed);
    return;
  }

  const uint64_t dt_us = t - self->last_tick_us_;
  self->last_tick_us_ = t;

  if (delta != 0)
    self->pending_total_delta_.fetch_add(static_cast<int64_t>(delta), std::memory_order_relaxed);

  if (dt_us == 0)
    return;

  self->accum_dt_us_ += dt_us;
  if (delta != 0)
    self->accum_pulses_ += static_cast<int64_t>(delta);

  const bool enough_pulses = (std::llabs(self->accum_pulses_) >= static_cast<int64_t>(self->min_pulses_for_calc_));
  const bool time_up = (self->accum_dt_us_ >= self->max_accumulation_us_);

  if (enough_pulses || time_up) {
    if (self->accum_dt_us_ > 0) {
      const double ppm = (static_cast<double>(self->accum_pulses_) * 60000000.0) /
                         static_cast<double>(self->accum_dt_us_);
      if (std::isfinite(ppm) && std::fabs(ppm) < 1e9) {
        self->last_calculated_ppm_.store(static_cast<float>(ppm), std::memory_order_relaxed);
        self->new_value_ready_.store(true, std::memory_order_release);
      }
    }
    self->reset_accumulation_();
  }
}
#endif

void PulseCounterSensor::update() {
#if defined(USE_ESP32)
  if (this->new_value_ready_.load(std::memory_order_acquire)) {
    this->new_value_ready_.store(false, std::memory_order_release);
    const float ppm = this->last_calculated_ppm_.load(std::memory_order_relaxed);
    if (std::isfinite(ppm))
      this->publish_state(ppm);
  }

  if (this->total_sensor_ != nullptr) {
    const int64_t delta = this->pending_total_delta_.exchange(0, std::memory_order_acq_rel);

    if (delta != 0 || !this->total_ever_published_) {
      int64_t abs_total = this->storage_->read_total();
      if (abs_total < 0)
        abs_total = 0;

      if (abs_total == 0 && (this->published_total_ != 0 || delta != 0)) {
        this->published_total_ = std::max<int64_t>(0, this->published_total_ + delta);
      } else {
        this->published_total_ = abs_total;
      }

      this->total_sensor_->publish_state(static_cast<float>(this->published_total_));
      this->total_ever_published_ = true;
    }
  }
#else
  const uint64_t t = static_cast<uint64_t>(micros());
  const pulse_counter_t delta = this->storage_->read_delta();

  if (this->last_time_us_ != 0) {
    const uint64_t dt_us = t - this->last_time_us_;
    if (dt_us > 0) {
      const double ppm = (static_cast<double>(delta) * 60000000.0) / static_cast<double>(dt_us);
      if (std::isfinite(ppm) && std::fabs(ppm) < 1e9)
        this->publish_state(static_cast<float>(ppm));
    }
  }
  this->last_time_us_ = t;

  if (this->total_sensor_ != nullptr) {
    int64_t abs_total = this->storage_->read_total();
    if (abs_total < 0)
      abs_total = 0;
    this->published_total_ = abs_total;
    this->total_sensor_->publish_state(static_cast<float>(this->published_total_));
    this->total_ever_published_ = true;
  }
#endif
}

}  // namespace pulse_counter
}  // namespace esphome

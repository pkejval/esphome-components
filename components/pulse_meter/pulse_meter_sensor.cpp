#include "pulse_meter_sensor.h"

#include <utility>

#include "esphome/core/log.h"

namespace esphome {
namespace pulse_meter {

static const char *const TAG = "pulse_meter";

#if ESPHOME_PULSE_METER_HAS_PCNT
bool IRAM_ATTR PulseMeterSensor::pcnt_on_reach_(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata,
                                                void *user_ctx) {
  auto *self = static_cast<PulseMeterSensor *>(user_ctx);
  if (self == nullptr || edata == nullptr)
    return false;

  const int16_t watch_point = static_cast<int16_t>(edata->watch_point_value);
  if (watch_point != PCNT_EDGE_WATCH_POINT)
    return false;

  // PCNT has already filtered the input in hardware. Reset its one-edge
  // watchpoint immediately, then hand the event to the normal pulse_meter
  // double buffer so loop() can publish with the same cadence as GPIO ISR.
  if (pcnt_unit_clear_count(unit) != ESP_OK)
    return false;
  const uint32_t now = micros();
  auto &set = *self->set_;
  set.last_detected_edge_us_ = now;
  set.last_rising_edge_us_ = now;
  set.count_ = set.count_ + 1;
  self->new_event_ = true;
  return false;
}

void PulseMeterSensor::teardown_pcnt_() {
  if (this->pcnt_unit_ != nullptr)
    (void) pcnt_unit_stop(this->pcnt_unit_);
  if (this->pcnt_channel_ != nullptr) {
    (void) pcnt_del_channel(this->pcnt_channel_);
    this->pcnt_channel_ = nullptr;
  }
  if (this->pcnt_unit_ != nullptr) {
    (void) pcnt_unit_disable(this->pcnt_unit_);
    (void) pcnt_del_unit(this->pcnt_unit_);
    this->pcnt_unit_ = nullptr;
  }
  this->use_pcnt_ = false;
}

bool PulseMeterSensor::setup_pcnt_() {
  if (this->filter_mode_ != FILTER_EDGE)
    return false;

  pcnt_unit_config_t unit_cfg{};
  unit_cfg.low_limit = PCNT_LOW_LIMIT;
  unit_cfg.high_limit = PCNT_HIGH_LIMIT;
  if (pcnt_new_unit(&unit_cfg, &this->pcnt_unit_) != ESP_OK || this->pcnt_unit_ == nullptr) {
    this->teardown_pcnt_();
    return false;
  }

  pcnt_chan_config_t channel_cfg{};
  channel_cfg.edge_gpio_num = this->pin_->get_pin();
  channel_cfg.level_gpio_num = -1;
  if (pcnt_new_channel(this->pcnt_unit_, &channel_cfg, &this->pcnt_channel_) != ESP_OK ||
      this->pcnt_channel_ == nullptr) {
    this->teardown_pcnt_();
    return false;
  }

  if (pcnt_channel_set_edge_action(this->pcnt_channel_, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                   PCNT_CHANNEL_EDGE_ACTION_HOLD) != ESP_OK ||
      pcnt_channel_set_level_action(this->pcnt_channel_, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                    PCNT_CHANNEL_LEVEL_ACTION_KEEP) != ESP_OK) {
    this->teardown_pcnt_();
    return false;
  }

  if (this->filter_us_ > 0) {
    const uint32_t applied_filter_us = this->filter_us_ > PCNT_MAX_GLITCH_FILTER_US
                                           ? PCNT_MAX_GLITCH_FILTER_US
                                           : this->filter_us_;
    pcnt_glitch_filter_config_t filter_cfg{};
    filter_cfg.max_glitch_ns = applied_filter_us * 1000U;
    if (pcnt_unit_set_glitch_filter(this->pcnt_unit_, &filter_cfg) != ESP_OK) {
      this->teardown_pcnt_();
      return false;
    }
    if (applied_filter_us != this->filter_us_) {
      ESP_LOGW(TAG, "PCNT internal filter clamped from %" PRIu32 "us to %" PRIu32 "us", this->filter_us_,
               applied_filter_us);
    }
  }

  pcnt_event_callbacks_t callbacks{};
  callbacks.on_reach = &PulseMeterSensor::pcnt_on_reach_;
  if (pcnt_unit_register_event_callbacks(this->pcnt_unit_, &callbacks, this) != ESP_OK ||
      pcnt_unit_add_watch_point(this->pcnt_unit_, PCNT_EDGE_WATCH_POINT) != ESP_OK ||
      pcnt_unit_enable(this->pcnt_unit_) != ESP_OK || pcnt_unit_clear_count(this->pcnt_unit_) != ESP_OK ||
      pcnt_unit_start(this->pcnt_unit_) != ESP_OK) {
    this->teardown_pcnt_();
    return false;
  }

  this->use_pcnt_ = true;
  return true;
}
#endif

void PulseMeterSensor::set_total_pulses(uint32_t pulses) {
  this->total_pulses_ = pulses;
  if (this->total_sensor_ != nullptr) {
    this->total_sensor_->publish_state(this->total_pulses_);
  }
}

#if (defined(portNUM_PROCESSORS) && (portNUM_PROCESSORS > 1))
void PulseMeterSensor::attach_isr_task_(void *arg) {
  auto *self = static_cast<PulseMeterSensor *>(arg);
  if (self->pin_ == nullptr) {
    vTaskDelete(nullptr);
    return;
  }

  if (self->filter_mode_ == FILTER_EDGE) {
    self->pin_->attach_interrupt(PulseMeterSensor::edge_intr, self, gpio::INTERRUPT_RISING_EDGE);
  } else {
    self->pulse_state_.last_pin_val_ = self->isr_pin_.digital_read();
    self->pulse_state_.latched_ = self->pulse_state_.last_pin_val_;
    self->pin_->attach_interrupt(PulseMeterSensor::pulse_intr, self, gpio::INTERRUPT_ANY_EDGE);
  }
  vTaskDelete(nullptr);
}
#endif

void PulseMeterSensor::setup() {
  if (this->pin_ == nullptr) {
    ESP_LOGE(TAG, "Pin not set!");
    this->mark_failed();
    return;
  }

  this->pin_->setup();
  this->isr_pin_ = this->pin_->to_isr();

  const uint32_t now = micros();

  this->last_processed_edge_us_ = now;
  this->next_timeout_check_us_ = now + this->timeout_us_;
  this->next_zero_publish_us_ = 0;
  this->new_event_ = false;
  this->peeked_edge_ = false;

  this->reset_period_estimate_();

  this->last_polled_pin_val_ = this->isr_pin_.digital_read();
  this->next_poll_check_us_ = now + 2000U;

  this->pulse_state_.last_intr_ = now;
  this->pulse_state_.last_pin_val_ = this->last_polled_pin_val_;
  this->pulse_state_.latched_ = this->last_polled_pin_val_;

  this->coalesce_enabled_edge_ = (this->filter_mode_ == FILTER_EDGE);

  if (this->min_low_us_ == 0 && this->min_high_us_ == 0) {
    this->update_hysteresis_defaults_();
  }

#if ESPHOME_PULSE_METER_HAS_PCNT
  if (this->filter_mode_ == FILTER_EDGE) {
    if (!this->setup_pcnt_()) {
      ESP_LOGW(TAG, "PCNT backend unavailable; falling back to GPIO interrupts");
    }
  }
#endif

#if defined(ESP_IDF_VERSION) && __has_include("driver/gpio_filter.h")
#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5)
#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3) || \
    defined(CONFIG_IDF_TARGET_ESP32C6)
  if (!this->use_pcnt_) {
    // ESP32-S3: "pin glitch filter" nemá konfigurovatelný časový práh.
    if (this->filter_us_ > 0) {
      gpio_pin_glitch_filter_config_t cfg{};
      cfg.gpio_num = static_cast<gpio_num_t>(this->pin_->get_pin());
      cfg.clk_src = GLITCH_FILTER_CLK_SRC_DEFAULT;

      if (gpio_new_pin_glitch_filter(&cfg, &this->glitch_filter_) == ESP_OK && this->glitch_filter_ != nullptr) {
        gpio_glitch_filter_enable(this->glitch_filter_);
      } else {
        this->glitch_filter_ = nullptr;
      }
    } else {
      this->glitch_filter_ = nullptr;
    }
  }
#endif
#endif
#endif

#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5) && __has_include("driver/rmt_rx.h")
  if (this->filter_mode_ == FILTER_PULSE) {
    this->rmt_pending_ = false;
    this->rmt_recv_count_ = 0;
    this->rmt_done_us_ = 0;

    rmt_rx_channel_config_t ch_cfg{};
    ch_cfg.gpio_num = (gpio_num_t) this->pin_->get_pin();
    ch_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
    ch_cfg.resolution_hz = this->rmt_resolution_hz_;
    ch_cfg.mem_block_symbols = RMT_RX_BUFFER_SYMBOLS;

    if (rmt_new_rx_channel(&ch_cfg, &this->rmt_rx_channel_) == ESP_OK && this->rmt_rx_channel_ != nullptr) {
      rmt_rx_event_callbacks_t cbs{};
      cbs.on_recv_done = &PulseMeterSensor::rmt_rx_done_cb_;

      if (rmt_rx_register_event_callbacks(this->rmt_rx_channel_, &cbs, this) == ESP_OK) {
        rmt_receive_config_t rx_cfg{};
        const uint32_t filter_us = this->filter_us_ > RMT_MAX_SIGNAL_RANGE_US
                                       ? RMT_MAX_SIGNAL_RANGE_US
                                       : this->filter_us_;
        const uint32_t frame_timeout_us = this->timeout_us_ > RMT_MAX_SIGNAL_RANGE_US
                                              ? RMT_MAX_SIGNAL_RANGE_US
                                              : this->timeout_us_;
        rx_cfg.signal_range_min_ns = filter_us * 1000U;
        rx_cfg.signal_range_max_ns = frame_timeout_us * 1000U;

        this->rmt_rx_cfg_ = rx_cfg;

        if (rmt_enable(this->rmt_rx_channel_) == ESP_OK &&
            rmt_receive(this->rmt_rx_channel_, this->rmt_rx_buffer_, sizeof(this->rmt_rx_buffer_),
                        &this->rmt_rx_cfg_) == ESP_OK) {
          this->use_rmt_ = true;
        }
      }
    }

    if (!this->use_rmt_) {
      if (this->rmt_rx_channel_ != nullptr) {
        (void) rmt_disable(this->rmt_rx_channel_);
        (void) rmt_del_channel(this->rmt_rx_channel_);
      }
      this->rmt_rx_channel_ = nullptr;
      this->rmt_pending_ = false;
      this->rmt_recv_count_ = 0;
      this->rmt_done_us_ = 0;
    }
  }
#endif

  if (!this->use_rmt_ && !this->use_pcnt_) {
#if (defined(portNUM_PROCESSORS) && (portNUM_PROCESSORS > 1))
    xTaskCreatePinnedToCore(PulseMeterSensor::attach_isr_task_, "pm_attach_isr", 2048, this, 20, nullptr, 1);
#else
    if (this->filter_mode_ == FILTER_EDGE) {
      this->pin_->attach_interrupt(PulseMeterSensor::edge_intr, this, gpio::INTERRUPT_RISING_EDGE);
    } else {
      this->pulse_state_.last_pin_val_ = this->isr_pin_.digital_read();
      this->pulse_state_.latched_ = this->pulse_state_.last_pin_val_;
      this->pin_->attach_interrupt(PulseMeterSensor::pulse_intr, this, gpio::INTERRUPT_ANY_EDGE);
    }
#endif
  }

  if (this->total_sensor_ != nullptr) {
    this->total_sensor_->publish_state(this->total_pulses_);
  }
}

void PulseMeterSensor::loop() {
  const uint32_t now = micros();
  bool do_poll = false;
  bool poll_current = false;

#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5) && __has_include("driver/rmt_rx.h")
  bool use_rmt = this->use_rmt_;
#endif
  if (LIKELY(!this->new_event_) && LIKELY(time_before_(now, this->next_timeout_check_us_))) {
#if ESPHOME_PULSE_METER_HAS_PCNT
    if (this->use_pcnt_) {
      return;
    } else
#endif
#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5) && __has_include("driver/rmt_rx.h")
    if (use_rmt) {
      if (!__atomic_load_n(&this->rmt_pending_, __ATOMIC_ACQUIRE) && this->rmt_recv_count_ == 0) {
        return;
      }
    } else
#endif
    {
      do_poll = this->should_poll_fallback_(now);
      if (!do_poll) {
        return;
      }

      poll_current = this->isr_pin_.digital_read();

      if (poll_current == this->last_polled_pin_val_) {
        this->last_polled_pin_val_ = poll_current;
        this->schedule_next_poll_(now);
        return;
      }
    }
  }

  uint32_t cnt = 0;
  uint32_t tdet = 0;
  uint32_t trise = 0;
  bool had_event = false;

#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5) && __has_include("driver/rmt_rx.h")
  size_t local_sym_count = 0;
  uint32_t local_rmt_done_us = 0;
  bool local_rmt_consumed = false;
#endif

  {
    InterruptLock lock;

#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5) && __has_include("driver/rmt_rx.h")
    if (use_rmt) {
      if (__atomic_load_n(&this->rmt_pending_, __ATOMIC_ACQUIRE) || this->rmt_recv_count_ != 0) {
        local_sym_count = (size_t) this->rmt_recv_count_;
        local_rmt_done_us = this->rmt_done_us_;
        __atomic_store_n(&this->rmt_pending_, false, __ATOMIC_RELEASE);
        this->rmt_recv_count_ = 0;
        this->rmt_done_us_ = 0;
        local_rmt_consumed = true;
      }
    }
#endif

#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5) && __has_include("driver/rmt_rx.h")
    if (!use_rmt)
#endif
    {
      if (do_poll) {
        const bool current = this->isr_pin_.digital_read();

        if (this->filter_mode_ == FILTER_EDGE) {
          if (current && !this->last_polled_pin_val_) {
            PulseMeterSensor::edge_intr(this);
          }
        } else {
          if (current != this->last_polled_pin_val_) {
            PulseMeterSensor::pulse_intr_sample_(this, micros(), current);
          }
        }

        this->last_polled_pin_val_ = current;
        this->schedule_next_poll_(now);
      }
    }

    this->get_->count_ = 0;
    std::swap(this->set_, this->get_);

    cnt = this->get_->count_;
    tdet = this->get_->last_detected_edge_us_;
    trise = this->get_->last_rising_edge_us_;

    had_event = this->new_event_;
    this->new_event_ = false;
  }

#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5) && __has_include("driver/rmt_rx.h")
  if (use_rmt && local_rmt_consumed) {
    if (local_sym_count > 0) {
      bool latched = this->pulse_state_.latched_;
      bool last_pin = this->pulse_state_.last_pin_val_;
      uint32_t last_edge_us = tdet;
      uint32_t last_rise_us = trise;
      uint32_t elapsed_us = 0;
      uint32_t last_edge_offset_us = 0;

      uint32_t add_cnt = 0;

      const uint32_t frame_done_us = (local_rmt_done_us != 0) ? local_rmt_done_us : now;

      for (size_t i = 0; i < local_sym_count; ++i) {
        const auto &w = this->rmt_rx_buffer_[i];
        const bool levels[] = {w.level0 != 0, w.level1 != 0};
        const uint32_t durations[] = {w.duration0, w.duration1};

        for (uint8_t part = 0; part < 2; part++) {
          const uint32_t duration_us = durations[part];
          if (duration_us == 0)
            continue;

          const bool level = levels[part];
          const uint32_t segment_start_offset_us = elapsed_us;
          elapsed_us += duration_us;

          // Every RMT part describes the duration of its own level. Qualify that
          // completed level directly, rather than applying its duration to the
          // preceding level on a transition.
          if (level) {
            if (!latched && duration_us >= this->min_high_us_) {
              latched = true;
              last_edge_offset_us = segment_start_offset_us;
              add_cnt++;
            }
          } else if (latched && duration_us >= this->min_low_us_) {
            latched = false;
          }
          last_pin = level;
        }
      }

      this->pulse_state_.latched_ = latched;
      this->pulse_state_.last_pin_val_ = last_pin;

      if (add_cnt > 0) {
        const uint32_t tail_us = elapsed_us - last_edge_offset_us;
        last_edge_us = frame_done_us - tail_us;
        last_rise_us = last_edge_us;
        cnt += add_cnt;
        tdet = last_edge_us;
        trise = last_rise_us;
        had_event = true;
      }
    }

    // The RMT driver writes into rmt_rx_buffer_. Do not re-arm it until every
    // symbol above has been consumed.
    if (this->rmt_rx_channel_ != nullptr &&
        rmt_receive(this->rmt_rx_channel_, this->rmt_rx_buffer_, sizeof(this->rmt_rx_buffer_),
                    &this->rmt_rx_cfg_) != ESP_OK) {
      ESP_LOGE(TAG, "Failed to restart RMT receive; falling back to GPIO interrupts");
      (void) rmt_disable(this->rmt_rx_channel_);
      (void) rmt_del_channel(this->rmt_rx_channel_);
      this->rmt_rx_channel_ = nullptr;
      this->use_rmt_ = false;

      const bool pin_val = this->isr_pin_.digital_read();
      this->pulse_state_.last_intr_ = now;
      this->pulse_state_.last_pin_val_ = pin_val;
      this->pulse_state_.latched_ = pin_val;
#if (defined(portNUM_PROCESSORS) && (portNUM_PROCESSORS > 1))
      xTaskCreatePinnedToCore(PulseMeterSensor::attach_isr_task_, "pm_attach_isr", 2048, this, 20, nullptr, 1);
#else
      this->pin_->attach_interrupt(PulseMeterSensor::pulse_intr, this, gpio::INTERRUPT_ANY_EDGE);
#endif
    }
  }
#endif

  if (this->peeked_edge_ && cnt > 0) {
    this->peeked_edge_ = false;
    cnt -= 1;
  }

  const uint32_t edge_timeout_us = (this->filter_mode_ == FILTER_PULSE && this->min_high_us_ > 0) ? this->min_high_us_ : this->filter_us_;
  if (trise != tdet && time_reached_(now, trise + edge_timeout_us)) {
    this->peeked_edge_ = true;
    tdet = trise;
    cnt += 1;
    had_event = true;
  }

  if (cnt > 0) {
    if (this->total_sensor_ != nullptr) {
      this->total_pulses_ += cnt;
      const uint32_t total = this->total_pulses_;
      this->total_sensor_->publish_state(total);
    }

    switch (this->meter_state_) {
      case MeterState::INITIAL:
      case MeterState::TIMED_OUT:
        this->meter_state_ = MeterState::RUNNING;
        break;

      case MeterState::RUNNING: {
        const uint32_t delta_us = tdet - this->last_processed_edge_us_;
        if (delta_us > 0) {
          const float rpm = (60000000.0f * float(cnt)) / float(delta_us);
          this->publish_state(rpm);
          this->update_period_estimate_(delta_us, cnt);
        }
      } break;
    }

    this->last_processed_edge_us_ = tdet;
    this->next_zero_publish_us_ = 0;
    this->plan_next_check_(now);
    return;
  }

  if (!had_event) {
    const uint32_t time_since_valid_edge_us = now - this->last_processed_edge_us_;
    switch (this->meter_state_) {
      case MeterState::INITIAL:
      case MeterState::RUNNING:
        if (time_since_valid_edge_us > this->timeout_us_) {
          this->meter_state_ = MeterState::TIMED_OUT;
          ESP_LOGD(TAG, "No pulse detected for %" PRIu32 "s, assuming 0 pulses/min",
                   time_since_valid_edge_us / 1000000);
          this->publish_state(0.0f);

          this->reset_period_estimate_();

          if (this->timeout_zero_publish_interval_us_ > 0) {
            this->next_zero_publish_us_ = now + this->timeout_zero_publish_interval_us_;
            this->next_timeout_check_us_ = this->next_zero_publish_us_;
          } else {
            this->next_zero_publish_us_ = 0;
            this->next_timeout_check_us_ = now + this->timeout_us_;
          }
        } else {
          this->plan_next_check_(now);
        }
        break;
      case MeterState::TIMED_OUT:
        if (this->timeout_zero_publish_interval_us_ > 0 && this->next_zero_publish_us_ != 0) {
          if (time_reached_(now, this->next_zero_publish_us_)) {
            this->publish_state(0.0f);
            this->next_zero_publish_us_ = now + this->timeout_zero_publish_interval_us_;
          }
          this->next_timeout_check_us_ = this->next_zero_publish_us_;
        }
        break;
      default:
        break;
    }
  } else {
    if (this->meter_state_ == MeterState::TIMED_OUT && this->timeout_zero_publish_interval_us_ > 0 &&
        this->next_zero_publish_us_ != 0) {
      this->next_timeout_check_us_ = this->next_zero_publish_us_;
    } else {
      this->plan_next_check_(now);
    }
  }
}

float PulseMeterSensor::get_setup_priority() const { return setup_priority::DATA; }

void PulseMeterSensor::dump_config() {
  LOG_SENSOR("", "Pulse Meter", this);
  LOG_PIN("  Pin: ", this->pin_);
  if (this->filter_mode_ == FILTER_EDGE) {
    ESP_LOGCONFIG(TAG, "  Filtering rising edges less than %" PRIu32 " us apart", this->filter_us_);
    if (this->use_pcnt_) {
      ESP_LOGCONFIG(TAG, "  PCNT backend: enabled (hardware glitch filter)");
    } else {
      ESP_LOGCONFIG(TAG, "  PCNT backend: not available; using GPIO ISR");
      ESP_LOGCONFIG(TAG, "  ISR coalescing (EDGE): %s, window >= %" PRIu32 " us",
                    this->coalesce_enabled_edge_ ? "on" : "off", this->coalesce_min_us_);
    }
  } else {
    ESP_LOGCONFIG(TAG, "  Filtering pulses shorter than %" PRIu32 " us (low>=%" PRIu32 " us, high>=%" PRIu32 " us)",
                  this->filter_us_, this->min_low_us_, this->min_high_us_);
    ESP_LOGCONFIG(TAG, "  ISR coalescing (PULSE): off");
  }
  ESP_LOGCONFIG(TAG, "  Assuming 0 pulses/min after not receiving a pulse for %" PRIu32 " s",
                this->timeout_us_ / 1000000);
  if (this->timeout_zero_publish_interval_us_ > 0) {
    ESP_LOGCONFIG(TAG, "  Timeout zero re-publish interval: %" PRIu32 " ms",
                  this->timeout_zero_publish_interval_us_ / 1000U);
  } else {
    ESP_LOGCONFIG(TAG, "  Timeout zero re-publish interval: disabled");
  }
  ESP_LOGCONFIG(TAG, "  Poll fallback: %s", this->poll_fallback_enabled_ ? "enabled" : "disabled");

#if defined(ESP_IDF_VERSION) && __has_include("driver/gpio_filter.h")
#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5)
#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3) || \
    defined(CONFIG_IDF_TARGET_ESP32C6)
  ESP_LOGCONFIG(TAG, "  GPIO glitch filter: %s", this->glitch_filter_ ? "enabled" : "not available");
#endif
#endif
#endif

#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5) && __has_include("driver/rmt_rx.h")
  if (this->filter_mode_ == FILTER_PULSE) {
    ESP_LOGCONFIG(TAG, "  RMT backend: %s", this->use_rmt_ ? "enabled" : "not available");
  }
#endif
}

void IRAM_ATTR PulseMeterSensor::edge_intr(PulseMeterSensor *sensor) {
  const uint32_t now = micros();
  const bool coalesce = sensor->coalesce_enabled_edge_ && sensor->coalesce_min_us_ > 0;

  if (coalesce && time_before_(now, sensor->coalesce_until_us_)) {
    return;
  }

  auto &state = sensor->edge_state_;
  auto &set = *sensor->set_;

  if ((now - state.last_sent_edge_us_) >= sensor->filter_us_) {
    state.last_sent_edge_us_ = now;
    set.last_detected_edge_us_ = now;
    set.last_rising_edge_us_ = now;
    set.count_ = set.count_ + 1;
    sensor->new_event_ = true;

    if (coalesce) {
      sensor->coalesce_until_us_ = now + sensor->coalesce_min_us_;
    }
  }
}

void IRAM_ATTR PulseMeterSensor::pulse_intr(PulseMeterSensor *sensor) {
  const uint32_t now = micros();
  const bool pin_val = sensor->isr_pin_.digital_read();
  PulseMeterSensor::pulse_intr_sample_(sensor, now, pin_val);
}

void IRAM_ATTR PulseMeterSensor::pulse_intr_sample_(PulseMeterSensor *sensor, uint32_t now, bool pin_val) {
  auto &st = sensor->pulse_state_;
  auto &set = *sensor->set_;

  const uint32_t last_intr = st.last_intr_;
  const bool prev_pin = st.last_pin_val_;
  bool latched = st.latched_;

  const uint32_t dt_us = now - last_intr;

  if (latched && !prev_pin) {
    if (dt_us >= sensor->min_low_us_) {
      latched = false;
    }
  } else if (!latched && prev_pin) {
    if (dt_us >= sensor->min_high_us_) {
      latched = true;
      set.last_detected_edge_us_ = last_intr;
      set.count_ = set.count_ + 1;
      sensor->new_event_ = true;
    }
  }

  set.last_rising_edge_us_ = (!latched && pin_val) ? now : set.last_detected_edge_us_;

  st.latched_ = latched;
  st.last_intr_ = now;
  st.last_pin_val_ = pin_val;
}

#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5) && __has_include("driver/rmt_rx.h")
bool IRAM_ATTR PulseMeterSensor::rmt_rx_done_cb_(rmt_channel_handle_t, const rmt_rx_done_event_data_t *edata,
                                                 void *user_ctx) {
  auto *self = static_cast<PulseMeterSensor *>(user_ctx);

  if (edata == nullptr || edata->received_symbols != self->rmt_rx_buffer_ || edata->num_symbols == 0) {
    self->rmt_recv_count_ = 0;
  } else {
    // received_symbols points to the application-owned buffer passed to
    // rmt_receive(). Keep only its length; retaining a driver-owned pointer past
    // this callback would be unsafe.
    self->rmt_recv_count_ =
        edata->num_symbols > RMT_RX_BUFFER_SYMBOLS ? RMT_RX_BUFFER_SYMBOLS : edata->num_symbols;
  }

  self->rmt_done_us_ = micros();
  self->new_event_ = true;
  // Publish this last so the loop sees a complete RX frame on dual-core chips.
  __atomic_store_n(&self->rmt_pending_, true, __ATOMIC_RELEASE);
  return false;
}
#endif

}  // namespace pulse_meter
}  // namespace esphome

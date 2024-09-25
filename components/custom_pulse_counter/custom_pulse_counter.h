#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace custom_pulse_counter {

static const char *const TAG = "custom_pulse_counter";

class CustomPulseCounter : public sensor::Sensor, public PollingComponent {
  public:
    // constructor
    explicit CustomPulseCounter() {}

    void set_pin(InternalGPIOPin *pin) { this->pin = pin; }
    void set_filter_us(uint32_t filter) { this->pin_filter_us = filter; }
    void set_hfloop(bool use_hfloop) { this->use_hfloop = use_hfloop; }

    void setup() override {
      this->pin->setup();

      if (this->use_hfloop) {
        this->high_freq_req.start();
      }
    }

    void loop() override {
      bool pin_state = this->pin->digital_read();
      if (pin_state == pin_last_state) { return; }

      uint32_t now_us = micros();
      if (now_us - pin_last_change_time < pin_filter_us) { return; }

      if (pin_state) {
        pulse_counter++;
      }

      pin_last_state = pin_state;
      pin_last_change_time = now_us;
    }

    uint32_t read_raw_value() {
      uint32_t counter = this->pulse_counter;
      uint32_t ret = counter - this->last_read_value;
      this->last_read_value = counter;
      return ret;
    }

    void update() {
      uint32_t now = millis();

      if (this->last_update_time > 0) {
        uint32_t raw = read_raw_value();
        uint32_t interval = now - this->last_update_time;
        float value = (60000.0f * raw) / interval;  // per minute
        this->publish_state(value);
      }

      this->last_update_time = now;
    }

    float get_setup_priority() const override { return setup_priority::DATA; }
    
    void dump_config() override {
        LOG_SENSOR("", "Custom Pulse Counter", this);
        LOG_PIN("  Pin: ", this->pin);
        ESP_LOGCONFIG(TAG, "  Using HighFrequencyLoop: %s", this->use_hfloop ? "true" : "false");
        ESP_LOGCONFIG(TAG, "  Filtering pulses shorter than %d µs", this->pin_filter_us);

        LOG_UPDATE_INTERVAL(this);
    }

  protected:
    HighFrequencyLoopRequester high_freq_req;
    InternalGPIOPin *pin;

    bool use_hfloop;
    uint32_t last_update_time;
    uint32_t pulse_counter;
    uint32_t last_read_value;
    uint32_t pin_filter_us;
    
    uint32_t pin_last_change_time;
    bool pin_last_state;
};

}
}
#pragma once
#include "esphome/core/automation.h"
#include "nextion.h"

namespace esphome {
namespace nextion {

class SetupTrigger : public Trigger<> {
 public:
  explicit SetupTrigger(Nextion *nextion) {
    nextion->add_setup_state_callback([this]() { this->trigger(); });
  }
};

class SleepTrigger : public Trigger<> {
 public:
  explicit SleepTrigger(Nextion *nextion) {
    nextion->add_sleep_state_callback([this]() { this->trigger(); });
  }
};

class WakeTrigger : public Trigger<> {
 public:
  explicit WakeTrigger(Nextion *nextion) {
    nextion->add_wake_state_callback([this]() { this->trigger(); });
  }
};

class PageTrigger : public Trigger<uint8_t> {
 public:
  explicit PageTrigger(Nextion *nextion) {
    nextion->add_new_page_callback([this](const uint8_t page_id) { this->trigger(page_id); });
  }
};

class TouchTrigger : public Trigger<uint8_t, uint8_t, bool> {
 public:
  explicit TouchTrigger(Nextion *nextion) {
    nextion->add_touch_event_callback([this](uint8_t page_id, uint8_t component_id, bool touch_event) {
      this->trigger(page_id, component_id, touch_event);
    });
  }
};

template<typename... Ts> class SensorPublishAction : public Action<Ts...> {
 public:
  SensorPublishAction(NextionSensor *sensor) : sensor_(sensor) {}
  TEMPLATABLE_VALUE(float, state)
  TEMPLATABLE_VALUE(bool, publish)
  TEMPLATABLE_VALUE(bool, send_to_nextion)

  void play(Ts... x) override { this->sensor_->set_state(this->state_.value(x...), this->publish_.optional_value(x...), this->send_to_nextion_.optional_value(x...)); }

 protected:
  NextionSensor *sensor_;
};

}  // namespace nextion
}  // namespace esphome

#pragma once

#include <Arduino.h>
#include "HardwareConfig.h"

// Edge-only, active-low button. A button held during boot/STOP must be
// released before it can issue a new command; holding never repeats a press.
class LocalButton {
 public:
  void begin(uint8_t pin) {
    pin_ = pin;
    pinMode(pin_, INPUT_PULLUP);
    rawPressed_ = stablePressed_ = digitalRead(pin_) == LOW;
    ready_ = !stablePressed_;
    changedAtMs_ = millis();
  }

  bool takePress() {
    const bool pressed = digitalRead(pin_) == LOW;
    const uint32_t now = millis();
    if (pressed != rawPressed_) {
      rawPressed_ = pressed;
      changedAtMs_ = now;
    }
    if (rawPressed_ == stablePressed_ ||
        now - changedAtMs_ < HardwareConfig::BUTTON_DEBOUNCE_MS)
      return false;
    stablePressed_ = rawPressed_;
    if (!stablePressed_) {
      ready_ = true;
      return false;
    }
    const bool event = ready_;
    ready_ = false;
    return event;
  }

 private:
  uint8_t pin_ = 0;
  bool rawPressed_ = false;
  bool stablePressed_ = false;
  bool ready_ = false;
  uint32_t changedAtMs_ = 0;
};

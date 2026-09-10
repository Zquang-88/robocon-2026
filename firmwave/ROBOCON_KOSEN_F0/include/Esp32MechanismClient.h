#pragma once

#include <Arduino.h>

class Esp32MechanismClient {
 public:
  using LineHandler = void (*)(const char *line, void *context);

  explicit Esp32MechanismClient(HardwareSerial &serial) : serial_(serial) {}
  void begin(uint8_t rxPin, uint8_t txPin, uint32_t baud);
  void update(LineHandler handler, void *context = nullptr);
  void send(const char *line);

 private:
  HardwareSerial &serial_;
  char buffer_[256] = {};
  size_t length_ = 0;
};

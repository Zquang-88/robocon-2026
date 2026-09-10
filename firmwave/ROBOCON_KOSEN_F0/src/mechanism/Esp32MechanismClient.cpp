#include "Esp32MechanismClient.h"

void Esp32MechanismClient::begin(uint8_t rxPin, uint8_t txPin, uint32_t baud) {
  // Pin mux is configured on the concrete Teensy Serial6 object in main.cpp;
  // HardwareSerial's portable base class does not expose setRX()/setTX().
  (void)rxPin;
  (void)txPin;
  serial_.begin(baud);
}

void Esp32MechanismClient::send(const char *line) {
  if (line && *line) serial_.println(line);
}

void Esp32MechanismClient::update(LineHandler handler, void *context) {
  while (serial_.available()) {
    const char c = static_cast<char>(serial_.read());
    if (c == '\r' || c == '\n') {
      if (length_) {
        buffer_[length_] = '\0';
        if (handler) handler(buffer_, context);
        length_ = 0;
      }
    } else if (length_ < sizeof(buffer_) - 1) {
      buffer_[length_++] = c;
    } else {
      // Drop only the overlong record; the next delimiter resynchronizes the
      // parser without blocking the 100 Hz robot control loop.
      length_ = 0;
    }
  }
}

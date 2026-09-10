#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
using std::min;
using std::max;
using std::isfinite;
using boolean = bool;
constexpr int HIGH = 1, LOW = 0, OUTPUT = 1, INPUT_PULLUP = 2;
extern uint64_t mockMicros;
inline uint32_t millis() { return static_cast<uint32_t>(mockMicros / 1000); }
inline unsigned long micros() { return static_cast<unsigned long>(mockMicros); }
inline void delayMicroseconds(unsigned int us) { mockMicros += us; }
inline int mockDigitalPinState[64] = {};
inline void digitalWrite(int, int) {}
inline int digitalRead(int pin) {
  return pin >= 0 && pin < 64 ? mockDigitalPinState[pin] : LOW;
}
inline void setMockDigitalPin(int pin, int value) {
  if (pin >= 0 && pin < 64) mockDigitalPinState[pin] = value;
}
inline void resetMockDigitalPins() {
  for (int &value : mockDigitalPinState) value = LOW;
}
inline void pinMode(int, int) {}
inline void yield() {}
template <typename T> T constrain(T value, T lo, T hi) { return min(hi, max(lo, value)); }
class Stream {
 public:
  virtual ~Stream() = default;
  virtual int available() = 0;
  virtual int read() = 0;
  virtual size_t println(const char *) = 0;
};

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

class TwoWire {
 public:
  bool begin(int sda, int scl) {
    sdaPin = sda;
    sclPin = scl;
    return true;
  }
  void setClock(uint32_t hz) { clockHz = hz; }
  void beginTransmission(uint8_t address) {
    currentAddress = address;
    hasPendingByte = false;
  }
  size_t write(uint8_t value) {
    pendingByte = value;
    hasPendingByte = true;
    return 1;
  }
  uint8_t endTransmission() {
    if (failTransactions) return 4;
    if (hasPendingByte) {
      lastWritten = pendingByte;
      writes.push_back(pendingByte);
    }
    return 0;
  }
  void reset() {
    sdaPin = -1;
    sclPin = -1;
    clockHz = 0;
    currentAddress = 0;
    pendingByte = 0xFF;
    lastWritten = 0xFF;
    hasPendingByte = false;
    failTransactions = false;
    writes.clear();
  }

  int sdaPin = -1;
  int sclPin = -1;
  uint32_t clockHz = 0;
  uint8_t currentAddress = 0;
  uint8_t pendingByte = 0xFF;
  uint8_t lastWritten = 0xFF;
  bool hasPendingByte = false;
  bool failTransactions = false;
  std::vector<uint8_t> writes;
};

inline TwoWire Wire;
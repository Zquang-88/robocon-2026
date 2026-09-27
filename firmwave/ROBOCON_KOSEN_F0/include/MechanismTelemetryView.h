#pragma once
#include <Arduino.h>
struct MechanismTelemetrySnapshot {
  char state[20] = "OFFLINE";
  char activeProfile[16] = "NONE";
  bool online = false;
  bool busy = false;
  bool zeroed = false;
  uint8_t fault = 0;
  uint8_t valveMask = 0;
  uint8_t sequenceStep = 0;
  float positionAmm = 0.0f;
  float positionBmm = 0.0f;
  float jogSpeedA = 0.0f;
  float jogSpeedB = 0.0f;
  uint32_t lastUpdateMs = 0;
};

class MechanismTelemetryView {
 public:
  bool consume(const char *line);
  void updateTimeout(uint32_t timeoutMs);
  const MechanismTelemetrySnapshot &snapshot() const { return snapshot_; }

 private:
  MechanismTelemetrySnapshot snapshot_;
};

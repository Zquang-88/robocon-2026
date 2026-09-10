#include "MechanismTelemetryView.h"

#include <stdio.h>
#include <string.h>

bool MechanismTelemetryView::consume(const char *line) {
  if (!line || strncmp(line, "STATUS,", 7) != 0) return false;
  char state[20] = {};
  char profile[16] = {};
  unsigned busy = 0, fault = 0, valves = 0, step = 0, zeroed = 0;
  float positionA = 0.0f, positionB = 0.0f;
  float jogSpeedA = 0.0f, jogSpeedB = 0.0f;
  const int parsed = sscanf(line + 7, "%19[^,],%u,%u,%f,%f,%u,%15[^,],%u,%u,%f,%f",
      state, &busy, &fault, &positionA, &positionB, &valves,
      profile, &step, &zeroed, &jogSpeedA, &jogSpeedB);
  if (parsed < 9) return false;
  snprintf(snapshot_.state, sizeof(snapshot_.state), "%s", state);
  snprintf(snapshot_.activeProfile, sizeof(snapshot_.activeProfile), "%s", profile);
  snapshot_.busy = busy != 0;
  snapshot_.fault = static_cast<uint8_t>(fault);
  snapshot_.positionAmm = positionA;
  snapshot_.positionBmm = positionB;
  snapshot_.jogSpeedA = parsed >= 11 ? jogSpeedA : 0.0f;
  snapshot_.jogSpeedB = parsed >= 11 ? jogSpeedB : 0.0f;
  snapshot_.valveMask = static_cast<uint8_t>(valves);
  snapshot_.sequenceStep = static_cast<uint8_t>(step);
  snapshot_.zeroed = zeroed != 0;
  snapshot_.online = true;
  snapshot_.lastUpdateMs = millis();
  return true;
}

void MechanismTelemetryView::updateTimeout(uint32_t timeoutMs) {
  if (snapshot_.online && millis() - snapshot_.lastUpdateMs > timeoutMs)
    snapshot_.online = false;
}

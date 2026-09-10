#pragma once

#include <Arduino.h>

enum class ProfileId : uint8_t {
  RobotStart = 0,
  PickA,
  PickB,
  Retract,
  Home,
  Count
};

enum class SequenceStepType : uint8_t {
  MoveProfile = 0,
  ValveSet,
  WaitTime,
  WaitStepper,
  WaitSensor,
  End
};

enum class SensorId : uint8_t { E18 = 0, Line = 1 };

constexpr uint8_t PROFILE_COUNT = static_cast<uint8_t>(ProfileId::Count);
constexpr uint8_t MAX_PROFILE_STEPS = 8;

struct SequenceStep {
  SequenceStepType type = SequenceStepType::End;
  int8_t index = 0;              // Valve or Sensor index.
  int8_t value = 0;              // Valve state or expected sensor level.
  uint32_t durationMs = 0;       // Wait or sensor timeout.
};

struct StepperProfile {
  char name[16] = {};
  float distanceAmm = 0.0f;
  float distanceBmm = 0.0f;
  int8_t directionA = 1;
  int8_t directionB = 1;
  float maxSpeedA = 3000.0f;
  float maxSpeedB = 3000.0f;
  float accelerationA = 2500.0f;
  float accelerationB = 2500.0f;
  float stepsPerRevA = 200.0f;
  float stepsPerRevB = 200.0f;
  float screwPitchAmm = 12.0f;
  float screwPitchBmm = 10.0f;
  float correctionA = 1.0f;
  float correctionB = 1.0f;
  uint32_t postWaitMs = 0;
  uint32_t timeoutMs = 15000;
  bool absoluteMode = false;
  uint8_t stepCount = 0;
  SequenceStep steps[MAX_PROFILE_STEPS] = {};
};

struct MechanismConfig {
  uint32_t magic = 0;
  uint16_t version = 0;
  uint16_t size = 0;
  float minimumPositionAmm = -3000.0f;
  float maximumPositionAmm = 3000.0f;
  float minimumPositionBmm = -3000.0f;
  float maximumPositionBmm = 3000.0f;
  StepperProfile profiles[PROFILE_COUNT] = {};
  uint32_t crc32 = 0;
};

const char *profileName(ProfileId id);
bool parseProfileId(const char *text, ProfileId &id);
void loadDefaultMechanismConfig(MechanismConfig &config);
bool validateMechanismConfig(const MechanismConfig &config,
                             char *reason, size_t reasonCapacity);
uint32_t mechanismConfigCrc(const MechanismConfig &config);

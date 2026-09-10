#include "StepperProfiles.h"

#include <math.h>
#include <string.h>

namespace {
constexpr uint32_t CONFIG_MAGIC = 0x4D454348UL;  // "MECH"
constexpr uint16_t CONFIG_VERSION = 1;

void initProfile(StepperProfile &profile, const char *name,
                 float distanceA, float distanceB,
                 int8_t directionA, int8_t directionB,
                 uint32_t postWaitMs = 300) {
  memset(&profile, 0, sizeof(profile));
  strncpy(profile.name, name, sizeof(profile.name) - 1);
  profile.distanceAmm = fabsf(distanceA);
  profile.distanceBmm = fabsf(distanceB);
  profile.directionA = directionA;
  profile.directionB = directionB;
  profile.maxSpeedA = 5000.0f;
  profile.maxSpeedB = 5000.0f;
  profile.accelerationA = 3500.0f;
  profile.accelerationB = 3500.0f;
  profile.stepsPerRevA = 200.0f;
  profile.stepsPerRevB = 200.0f;
  profile.screwPitchAmm = 12.0f;
  profile.screwPitchBmm = 10.0f;
  profile.correctionA = 1.0f;
  profile.correctionB = 1.0f;
  profile.postWaitMs = postWaitMs;
  profile.timeoutMs = 30000;
  profile.absoluteMode = false;
  profile.stepCount = 3;
  profile.steps[0].type = SequenceStepType::MoveProfile;
  profile.steps[1].type = SequenceStepType::WaitStepper;
  profile.steps[2].type = SequenceStepType::WaitTime;
  profile.steps[2].durationMs = postWaitMs;
}

bool finiteInRange(float value, float minimum, float maximum) {
  return isfinite(value) && value >= minimum && value <= maximum;
}

void setReason(char *reason, size_t capacity, const char *text) {
  if (!reason || capacity == 0) return;
  strncpy(reason, text, capacity - 1);
  reason[capacity - 1] = '\0';
}
}  // namespace

const char *profileName(ProfileId id) {
  static const char *names[PROFILE_COUNT] = {
      "ROBOT_START", "PICK_A", "PICK_B", "RETRACT", "HOME"};
  const uint8_t index = static_cast<uint8_t>(id);
  return index < PROFILE_COUNT ? names[index] : "UNKNOWN";
}

bool parseProfileId(const char *text, ProfileId &id) {
  if (!text) return false;
  for (uint8_t i = 0; i < PROFILE_COUNT; ++i) {
    if (strcmp(text, profileName(static_cast<ProfileId>(i))) == 0) {
      id = static_cast<ProfileId>(i);
      return true;
    }
  }
  return false;
}

void loadDefaultMechanismConfig(MechanismConfig &config) {
  memset(&config, 0, sizeof(config));
  config.magic = CONFIG_MAGIC;
  config.version = CONFIG_VERSION;
  config.size = sizeof(MechanismConfig);
  config.minimumPositionAmm = -3000.0f;
  config.maximumPositionAmm = 3000.0f;
  config.minimumPositionBmm = -3000.0f;
  config.maximumPositionBmm = 3000.0f;

  // Defaults reproduce the intent of the old hard-coded program, but are now
  // editable and written to NVS only after an explicit SAVE command.
  initProfile(config.profiles[0], "ROBOT_START", 10.0f, 0.0f, -1, 1, 300);
  initProfile(config.profiles[1], "PICK_A", 500.0f, 500.0f, -1, -1, 3000);
  initProfile(config.profiles[2], "PICK_B", 615.0f, 500.0f, -1, 1, 3000);
  initProfile(config.profiles[3], "RETRACT", 2200.0f, 2200.0f, 1, -1, 3000);
  initProfile(config.profiles[4], "HOME", 0.0f, 0.0f, 1, 1, 0);
  config.profiles[4].absoluteMode = true;
  config.crc32 = mechanismConfigCrc(config);
}

uint32_t mechanismConfigCrc(const MechanismConfig &config) {
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&config);
  const size_t length = offsetof(MechanismConfig, crc32);
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < length; ++i) {
    crc ^= bytes[i];
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc >> 1) ^ (0xEDB88320UL & (0UL - (crc & 1UL)));
  }
  return ~crc;
}

bool validateMechanismConfig(const MechanismConfig &config,
                             char *reason, size_t reasonCapacity) {
  if (config.magic != CONFIG_MAGIC || config.version != CONFIG_VERSION ||
      config.size != sizeof(MechanismConfig)) {
    setReason(reason, reasonCapacity, "CONFIG_HEADER");
    return false;
  }
  if (!finiteInRange(config.minimumPositionAmm, -10000, 10000) ||
      !finiteInRange(config.maximumPositionAmm, -10000, 10000) ||
      !finiteInRange(config.minimumPositionBmm, -10000, 10000) ||
      !finiteInRange(config.maximumPositionBmm, -10000, 10000) ||
      config.minimumPositionAmm >= config.maximumPositionAmm ||
      config.minimumPositionBmm >= config.maximumPositionBmm) {
    setReason(reason, reasonCapacity, "POSITION_LIMITS");
    return false;
  }
  for (uint8_t i = 0; i < PROFILE_COUNT; ++i) {
    const StepperProfile &p = config.profiles[i];
    if (!finiteInRange(p.distanceAmm, 0, 10000) ||
        !finiteInRange(p.distanceBmm, 0, 10000) ||
        (p.directionA != -1 && p.directionA != 1) ||
        (p.directionB != -1 && p.directionB != 1) ||
        !finiteInRange(p.maxSpeedA, 10, 30000) ||
        !finiteInRange(p.maxSpeedB, 10, 30000) ||
        !finiteInRange(p.accelerationA, 10, 100000) ||
        !finiteInRange(p.accelerationB, 10, 100000) ||
        !finiteInRange(p.stepsPerRevA, 1, 100000) ||
        !finiteInRange(p.stepsPerRevB, 1, 100000) ||
        !finiteInRange(p.screwPitchAmm, 0.01f, 1000) ||
        !finiteInRange(p.screwPitchBmm, 0.01f, 1000) ||
        !finiteInRange(p.correctionA, 0.1f, 10) ||
        !finiteInRange(p.correctionB, 0.1f, 10) ||
        p.postWaitMs > 60000UL || p.timeoutMs < 100 ||
        p.timeoutMs > 180000UL || p.stepCount > MAX_PROFILE_STEPS) {
      setReason(reason, reasonCapacity, "PROFILE_RANGE");
      return false;
    }
  }
  setReason(reason, reasonCapacity, "OK");
  return true;
}

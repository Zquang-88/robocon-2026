#pragma once

#include <Arduino.h>
#include <Preferences.h>

#include "StepperProfiles.h"

class PersistentConfig {
 public:
  bool begin();
  bool load(MechanismConfig &config, char *reason, size_t reasonCapacity);
  bool save(MechanismConfig &config, char *reason, size_t reasonCapacity);
  bool erase(char *reason, size_t reasonCapacity);

 private:
  Preferences preferences_;
  bool ready_ = false;
};

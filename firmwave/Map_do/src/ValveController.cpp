#include "ValveController.h"

#include <Wire.h>

#include "HardwareConfig.h"

bool ValveController::probe(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

bool ValveController::stateIsSafe(uint8_t physicalState) const {
  for (uint8_t pin = 0; pin < 4; ++pin) {
    const uint8_t opposite = static_cast<uint8_t>(7U - pin);
    const uint8_t pairMask = static_cast<uint8_t>((1U << pin) |
                                                  (1U << opposite));
    // Both physical bits LOW would energize both coils of one valve.
    if ((physicalState & pairMask) == 0) return false;
  }
  return true;
}

bool ValveController::writePCF() {
  if (!available_ || !stateIsSafe(pcfState)) {
    lastWriteOk_ = false;
    return false;
  }
  Wire.beginTransmission(address_);
  Wire.write(pcfState);
  if (Wire.endTransmission() != 0) {
    available_ = false;
    lastWriteOk_ = false;
    return false;
  }
  lastWriteOk_ = true;
  return true;
}

void ValveController::begin() {
  pcfState = 0xFF;
  pendingState_ = 0xFF;
  switchPending_ = false;
  lastCommandAccepted_ = false;
  invalidPin_ = false;
  lastWriteOk_ = false;

  Wire.begin(HardwareConfig::PCF8574_SDA_PIN,
             HardwareConfig::PCF8574_SCL_PIN);
  Wire.setClock(HardwareConfig::PCF8574_I2C_HZ);
  address_ = HardwareConfig::PCF8574_ADDRESS;
  available_ = probe(address_);

  // The competition wiring is fixed at 0x20. Do not drive an unrelated I2C
  // device if the configured expander is absent.
  if (available_) writePCF();
}

bool ValveController::beginTransitionTo(uint8_t targetPhysicalState) {
  lastCommandAccepted_ = false;
  invalidPin_ = false;
  if (!healthy() || switchPending_ || !stateIsSafe(targetPhysicalState))
    return false;
  if (targetPhysicalState == pcfState) {
    lastCommandAccepted_ = true;
    return true;
  }

  // Break-before-make: only pairs that change are first driven HIGH/HIGH.
  uint8_t breakState = pcfState;
  for (uint8_t pin = 0; pin < 4; ++pin) {
    const uint8_t opposite = static_cast<uint8_t>(7U - pin);
    const uint8_t pairMask = static_cast<uint8_t>((1U << pin) |
                                                  (1U << opposite));
    if ((pcfState & pairMask) != (targetPhysicalState & pairMask))
      breakState = static_cast<uint8_t>(breakState | pairMask);
  }

  pcfState = breakState;
  if (!writePCF()) return false;
  pendingState_ = targetPhysicalState;
  if (pendingState_ == pcfState) {
    lastCommandAccepted_ = true;
    return true;
  }

  switchStartedMs_ = millis();
  switchPending_ = true;
  lastCommandAccepted_ = true;
  return true;
}

void ValveController::update() {
  if (!switchPending_ ||
      millis() - switchStartedMs_ < HardwareConfig::VALVE_SWITCH_DEADTIME_MS)
    return;
  pcfState = pendingState_;
  switchPending_ = false;
  if (!writePCF()) lastCommandAccepted_ = false;
}

void ValveController::setOutput(uint8_t pin, bool enabled) {
  // Public output commands always operate a complete double-solenoid pair.
  // enabled=false therefore releases this coil and selects its opposite coil.
  switchValveOutput(pin, enabled);
}

void ValveController::switchValveOutput(uint8_t pin, bool enabled) {
  invalidPin_ = pin >= HardwareConfig::VALVE_COUNT;
  if (invalidPin_) {
    lastCommandAccepted_ = false;
    return;
  }
  const uint8_t opposite = static_cast<uint8_t>(7U - pin);
  const uint8_t pairMask = static_cast<uint8_t>((1U << pin) |
                                                (1U << opposite));
  const uint8_t targetPin = enabled ? pin : opposite;
  uint8_t target = static_cast<uint8_t>(pcfState | pairMask);
  target = static_cast<uint8_t>(target & ~(1U << targetPin));
  beginTransitionTo(target);
}

void ValveController::allValveCoilsOff() {
  switchPending_ = false;
  pendingState_ = 0xFF;
  pcfState = 0xFF;
  invalidPin_ = false;
  lastCommandAccepted_ = writePCF();
}

bool ValveController::set(uint8_t index, bool enabled) {
  switchValveOutput(index, enabled);
  return lastCommandAccepted_;
}

bool ValveController::setMask(uint8_t enabledMask) {
  const uint8_t target = static_cast<uint8_t>(~enabledMask);
  return beginTransitionTo(target);
}

void ValveController::safeOff() {
  allValveCoilsOff();
}
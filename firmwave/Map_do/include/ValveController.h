#pragma once

#include <Arduino.h>

class ValveController {
 public:
  void begin();
  void update();

  // Physical PCF8574 access. pcfState bit 1 = HIGH/coil OFF,
  // bit 0 = LOW/coil ON because the optocoupler board is active-low.
  bool writePCF();
  void setOutput(uint8_t pin, bool enabled);
  void switchValveOutput(uint8_t pin, bool enabled);
  void allValveCoilsOff();

  // Compatibility API used by the existing mechanism/UI code.
  bool set(uint8_t index, bool enabled);
  bool setMask(uint8_t enabledMask);
  void safeOff();

  uint8_t mask() const { return static_cast<uint8_t>(~pcfState); }
  uint8_t rawState() const { return pcfState; }
  bool available() const { return available_; }
  bool healthy() const { return available_ && lastWriteOk_; }
  bool switchBusy() const { return switchPending_; }
  bool lastCommandAccepted() const { return lastCommandAccepted_; }
  bool invalidPin() const { return invalidPin_; }
  uint8_t address() const { return address_; }

 private:
  // Required complete physical state for P0..P7. Safe startup is all HIGH.
  uint8_t pcfState = 0xFF;
  uint8_t pendingState_ = 0xFF;
  uint8_t address_ = 0;
  bool available_ = false;
  bool lastWriteOk_ = false;
  bool lastCommandAccepted_ = false;
  bool invalidPin_ = false;
  bool switchPending_ = false;
  uint32_t switchStartedMs_ = 0;

  bool probe(uint8_t address);
  bool stateIsSafe(uint8_t physicalState) const;
  bool beginTransitionTo(uint8_t targetPhysicalState);
};
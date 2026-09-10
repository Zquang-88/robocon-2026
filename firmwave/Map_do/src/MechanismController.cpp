#include "MechanismController.h"

#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "HardwareConfig.h"

MechanismController::MechanismController(AccelStepper &motorA,
                                         AccelStepper &motorB,
                                         ValveController &valves)
    : motorA_(motorA), motorB_(motorB), valves_(valves) {}

void MechanismController::setOperationName(const char *name) {
  strncpy(operationName_, name ? name : "NONE", sizeof(operationName_) - 1);
  operationName_[sizeof(operationName_) - 1] = '\0';
}

void MechanismController::begin(MechanismConfig &config) {
  config_ = &config;
  motorA_.setMinPulseWidth(HardwareConfig::STEPPER_MIN_PULSE_US);
  motorB_.setMinPulseWidth(HardwareConfig::STEPPER_MIN_PULSE_US);
  pinMode(HardwareConfig::HOME_SWITCH_A_PIN, INPUT_PULLUP);
  pinMode(HardwareConfig::HOME_SWITCH_B_PIN, INPUT_PULLUP);
  // main.cpp performs an early safe-off before slower startup work. Keep this
  // retry for tests and for recovery when that first I2C probe did not succeed.
  if (!valves_.healthy()) valves_.begin();
  if (!valves_.healthy()) {
    setFault(MechanismFault::Pcf8574Unavailable, "PCF8574_NOT_FOUND");
    return;
  }
  emergencyStop("BOOT_SAFE");
  clearFault();
  startHome();
}

const StepperProfile &MechanismController::profile() const {
  return config_->profiles[static_cast<uint8_t>(activeProfile_)];
}

float MechanismController::stepsPerMmA(const StepperProfile &p) const {
  return p.stepsPerRevA * p.correctionA / p.screwPitchAmm;
}

float MechanismController::stepsPerMmB(const StepperProfile &p) const {
  return p.stepsPerRevB * p.correctionB / p.screwPitchBmm;
}

float MechanismController::positionAmm() const {
  if (!config_) return 0.0f;
  const float scale = stepsPerMmA(profile());
  return scale > 0.0f ? motorA_.currentPosition() / scale : 0.0f;
}

float MechanismController::positionBmm() const {
  if (!config_) return 0.0f;
  const float scale = stepsPerMmB(profile());
  return scale > 0.0f ? motorB_.currentPosition() / scale : 0.0f;
}

bool MechanismController::homeSwitchAActive() const {
  return digitalRead(HardwareConfig::HOME_SWITCH_A_PIN) ==
         HardwareConfig::HOME_SWITCH_ACTIVE_LEVEL;
}

bool MechanismController::homeSwitchBActive() const {
  return digitalRead(HardwareConfig::HOME_SWITCH_B_PIN) ==
         HardwareConfig::HOME_SWITCH_ACTIVE_LEVEL;
}

void MechanismController::selectProfileScale(ProfileId id) {
  const float positionA = positionAmm();
  const float positionB = positionBmm();
  activeProfile_ = id;
  const StepperProfile &next = profile();
  motorA_.setCurrentPosition(lroundf(positionA * stepsPerMmA(next)));
  motorB_.setCurrentPosition(lroundf(positionB * stepsPerMmB(next)));
}

bool MechanismController::targetsWithinLimits(float targetAmm,
                                               float targetBmm) const {
  return config_ && isfinite(targetAmm) && isfinite(targetBmm) &&
         targetAmm >= config_->minimumPositionAmm &&
         targetAmm <= config_->maximumPositionAmm &&
         targetBmm >= config_->minimumPositionBmm &&
         targetBmm <= config_->maximumPositionBmm;
}

bool MechanismController::issueAbsoluteMotion(
    float targetAmm, float targetBmm, float maxSpeedA, float maxSpeedB,
    float accelerationA, float accelerationB) {
  if (!targetsWithinLimits(targetAmm, targetBmm)) {
    setFault(MechanismFault::PositionLimit, "POSITION_LIMIT");
    return false;
  }

  const StepperProfile &scaleProfile = profile();
  const float scaleA = stepsPerMmA(scaleProfile);
  const float scaleB = stepsPerMmB(scaleProfile);
  const float currentA = positionAmm();
  const float currentB = positionBmm();
  const float travelA = fabsf(targetAmm - currentA) * scaleA;
  const float travelB = fabsf(targetBmm - currentB) * scaleB;
  float speedA = maxSpeedA;
  float speedB = maxSpeedB;
  const float timeA = speedA > 0.0f ? travelA / speedA : 0.0f;
  const float timeB = speedB > 0.0f ? travelB / speedB : 0.0f;
  const float longest = max(timeA, timeB);
  if (longest > 0.0f) {
    if (travelA > 0.0f) speedA = min(speedA, travelA / longest);
    if (travelB > 0.0f) speedB = min(speedB, travelB / longest);
  }

  motorA_.setMaxSpeed(max(10.0f, speedA));
  motorB_.setMaxSpeed(max(10.0f, speedB));
  motorA_.setAcceleration(accelerationA);
  motorB_.setAcceleration(accelerationB);
  motorA_.moveTo(lroundf(targetAmm * scaleA));
  motorB_.moveTo(lroundf(targetBmm * scaleB));
  motionIssued_ = true;
  return true;
}

bool MechanismController::issueProfileMotion(const StepperProfile &p) {
  const float currentA = positionAmm();
  const float currentB = positionBmm();
  const float targetA = p.absoluteMode
      ? p.distanceAmm * p.directionA
      : currentA + p.distanceAmm * p.directionA;
  const float targetB = p.absoluteMode
      ? p.distanceBmm * p.directionB
      : currentB + p.distanceBmm * p.directionB;
  return issueAbsoluteMotion(targetA, targetB, p.maxSpeedA, p.maxSpeedB,
                             p.accelerationA, p.accelerationB);
}

bool MechanismController::startProfile(ProfileId id) {
  if (!config_ || busy_ || fault_ != MechanismFault::None || !zeroed_)
    return false;
  selectProfileScale(id);
  setOperationName(profileName(id));
  sequenceIndex_ = 0;
  motionIssued_ = false;
  busy_ = true;
  state_ = State::Sequence;
  operationStartedMs_ = millis();
  stepStartedMs_ = operationStartedMs_;
  operationDeadlineMs_ = operationStartedMs_ + profile().timeoutMs;
  doneEvent_ = false;
  return true;
}

bool MechanismController::startCompetitionAction(CompetitionAction action) {
  if (!config_ || busy_ || fault_ != MechanismFault::None || !zeroed_)
    return false;
  if (action != CompetitionAction::ReadyHomeA && !valves_.healthy()) {
    setFault(MechanismFault::Pcf8574Unavailable, "PCF8574_NOT_FOUND");
    return false;
  }
  selectProfileScale(ProfileId::RobotStart);
  competitionAction_ = action;
  actionStep_ = 0;
  motionIssued_ = false;
  busy_ = true;
  state_ = State::Competition;
  operationStartedMs_ = millis();
  stepStartedMs_ = operationStartedMs_;
  operationDeadlineMs_ =
      operationStartedMs_ + HardwareConfig::COMPETITION_ACTION_TIMEOUT_MS;
  doneEvent_ = false;
  switch (action) {
    case CompetitionAction::ReadyHomeA: setOperationName("ROBOT_START"); break;
    case CompetitionAction::PointA: setOperationName("PICK_A"); break;
    case CompetitionAction::PointB: setOperationName("PICK_B"); break;
    case CompetitionAction::Drop2A: setOperationName("THA2A"); break;
    case CompetitionAction::Drop2B: setOperationName("THA2B"); break;
  }
  return true;
}

bool MechanismController::startDirectMove(float signedAmm, float signedBmm,
                                          float speedStepsPerSecond) {
  if (!config_ || busy_ || fault_ != MechanismFault::None || !zeroed_ ||
      !isfinite(signedAmm) || !isfinite(signedBmm) ||
      speedStepsPerSecond < 10.0f || speedStepsPerSecond > 30000.0f)
    return false;
  selectProfileScale(ProfileId::RobotStart);
  setOperationName("MANUAL");
  if (!issueAbsoluteMotion(positionAmm() + signedAmm,
                           positionBmm() + signedBmm,
                           speedStepsPerSecond, speedStepsPerSecond,
                           profile().accelerationA, profile().accelerationB))
    return false;
  busy_ = true;
  state_ = State::DirectMove;
  operationStartedMs_ = millis();
  operationDeadlineMs_ = operationStartedMs_ + 60000UL;
  doneEvent_ = false;
  return true;
}

void MechanismController::applyJogAxis(AccelStepper &motor, uint8_t axis,
                                        float speed, long target,
                                        float acceleration) {
  if (speed == jogSpeed_[axis]) return;
  if (speed == 0.0f) {
    motor.setCurrentPosition(motor.currentPosition());
  } else {
    if (jogSpeed_[axis] * speed < 0.0f)
      motor.setCurrentPosition(motor.currentPosition());
    motor.setMaxSpeed(fabsf(speed));
    motor.setAcceleration(acceleration);
    motor.moveTo(target);
  }
  jogSpeed_[axis] = speed;
}

bool MechanismController::setJog(float signedSpeedA, float signedSpeedB) {
  const float speeds[2] = {signedSpeedA, signedSpeedB};
  for (float speed : speeds) {
    if (!isfinite(speed) || fabsf(speed) > HardwareConfig::JOG_MAX_SPEED ||
        (speed != 0.0f && fabsf(speed) < HardwareConfig::JOG_MIN_SPEED))
      return false;
  }
  if (signedSpeedA == 0.0f && signedSpeedB == 0.0f) {
    stopJog();
    return true;
  }
  if (!config_ || !zeroed_ || fault_ != MechanismFault::None ||
      (busy_ && !jogging()))
    return false;
  if (!jogging()) {
    selectProfileScale(ProfileId::RobotStart);
    setOperationName("MANUAL");
  }
  const StepperProfile &p = profile();
  const float scaleA = stepsPerMmA(p);
  const float scaleB = stepsPerMmB(p);
  const double targetA = signedSpeedA < 0
      ? ceil(double(config_->minimumPositionAmm) * scaleA)
      : floor(double(config_->maximumPositionAmm) * scaleA);
  const double targetB = signedSpeedB < 0
      ? ceil(double(config_->minimumPositionBmm) * scaleB)
      : floor(double(config_->maximumPositionBmm) * scaleB);
  if (!isfinite(targetA) || !isfinite(targetB) ||
      targetA < LONG_MIN || targetA > LONG_MAX ||
      targetB < LONG_MIN || targetB > LONG_MAX)
    return false;
  applyJogAxis(motorA_, 0, signedSpeedA, static_cast<long>(targetA),
               p.accelerationA);
  applyJogAxis(motorB_, 1, signedSpeedB, static_cast<long>(targetB),
               p.accelerationB);
  lastJogCommandMs_ = millis();
  state_ = State::ManualJog;
  busy_ = true;
  doneEvent_ = false;
  return true;
}

void MechanismController::stopJog() {
  if (!jogging()) return;
  motorA_.setCurrentPosition(motorA_.currentPosition());
  motorB_.setCurrentPosition(motorB_.currentPosition());
  jogSpeed_[0] = jogSpeed_[1] = 0.0f;
  busy_ = false;
  motionIssued_ = false;
  state_ = State::Idle;
  doneEvent_ = false;
}

bool MechanismController::motorsAtTarget() const {
  return motorA_.distanceToGo() == 0 && motorB_.distanceToGo() == 0;
}

void MechanismController::queueValveLog(const char *text) {
  strncpy(valveLogText_, text ? text : "", sizeof(valveLogText_) - 1);
  valveLogText_[sizeof(valveLogText_) - 1] = '\0';
  valveLogEvent_ = true;
}

void MechanismController::queueValveSwitchLog(uint8_t pin, bool enabled) {
  const uint8_t opposite = static_cast<uint8_t>(7U - pin);
  char line[80];
  snprintf(line, sizeof(line), "[%s] P%u %s, P%u %s",
           valveSequenceName_, pin, enabled ? "ON" : "OFF",
           opposite, enabled ? "OFF" : "ON");
  queueValveLog(line);
}

bool MechanismController::startValveSequence(
    const char *name, uint8_t firstPin, bool firstEnabled,
    uint8_t secondPin, bool secondEnabled, uint32_t firstOutputWaitMs) {
  if (!valves_.healthy() || valves_.switchBusy() ||
      valveSequenceState_ != ValveSequenceState::Idle ||
      firstPin >= HardwareConfig::VALVE_COUNT ||
      secondPin >= HardwareConfig::VALVE_COUNT) {
    setFault(valves_.healthy() ? MechanismFault::ValveInterlock
                               : MechanismFault::Pcf8574Unavailable,
             valves_.healthy() ? "VALVE_SEQUENCE_BUSY_OR_PIN"
                               : "PCF8574_NOT_FOUND");
    return false;
  }
  strncpy(valveSequenceName_, name ? name : "VALVE",
          sizeof(valveSequenceName_) - 1);
  valveSequenceName_[sizeof(valveSequenceName_) - 1] = '\0';
  valveFirstPin_ = firstPin;
  valveSecondPin_ = secondPin;
  valveFirstEnabled_ = firstEnabled;
  valveSecondEnabled_ = secondEnabled;
  valveFirstOutputWaitMs_ = firstOutputWaitMs;
  valveSequenceStartedMs_ = millis();
  valveSequenceComplete_ = false;
  valveSequenceState_ = ValveSequenceState::SwitchFirstOutput;
  return true;
}

void MechanismController::updateValveSequence() {
  switch (valveSequenceState_) {
    case ValveSequenceState::Idle:
      return;
    case ValveSequenceState::SwitchFirstOutput:
      valves_.switchValveOutput(valveFirstPin_, valveFirstEnabled_);
      if (!valves_.lastCommandAccepted()) {
        setFault(valves_.invalidPin() ? MechanismFault::InvalidProfile
                                     : MechanismFault::Pcf8574Unavailable,
                 valves_.invalidPin() ? "VALVE_PIN_INVALID"
                                      : "PCF8574_WRITE");
        return;
      }
      queueValveSwitchLog(valveFirstPin_, valveFirstEnabled_);
      valveSequenceState_ = ValveSequenceState::WaitFirstSwitch;
      return;
    case ValveSequenceState::WaitFirstSwitch:
      if (valves_.switchBusy()) return;
      if (!valves_.healthy()) {
        setFault(MechanismFault::Pcf8574Unavailable, "PCF8574_WRITE");
        return;
      }
      valveSequenceStartedMs_ = millis();
      valveSequenceState_ = ValveSequenceState::WaitFirstOutput;
      {
        char line[48];
        snprintf(line, sizeof(line), "[%s] Wait %lu ms", valveSequenceName_,
                 static_cast<unsigned long>(valveFirstOutputWaitMs_));
        queueValveLog(line);
      }
      return;
    case ValveSequenceState::WaitFirstOutput:
      if (millis() - valveSequenceStartedMs_ >=
          valveFirstOutputWaitMs_)
        valveSequenceState_ = ValveSequenceState::SwitchSecondOutput;
      return;
    case ValveSequenceState::SwitchSecondOutput:
      valves_.switchValveOutput(valveSecondPin_, valveSecondEnabled_);
      if (!valves_.lastCommandAccepted()) {
        setFault(valves_.invalidPin() ? MechanismFault::InvalidProfile
                                     : MechanismFault::Pcf8574Unavailable,
                 valves_.invalidPin() ? "VALVE_PIN_INVALID"
                                      : "PCF8574_WRITE");
        return;
      }
      queueValveSwitchLog(valveSecondPin_, valveSecondEnabled_);
      valveSequenceState_ = ValveSequenceState::WaitSecondSwitch;
      return;
    case ValveSequenceState::WaitSecondSwitch:
      if (valves_.switchBusy()) return;
      if (!valves_.healthy()) {
        setFault(MechanismFault::Pcf8574Unavailable, "PCF8574_WRITE");
        return;
      }
      valveSequenceStartedMs_ = millis();
      valveSequenceState_ = ValveSequenceState::WaitSecondOutput;
      {
        char line[48];
        snprintf(line, sizeof(line), "[%s] Wait %lu ms", valveSequenceName_,
                 static_cast<unsigned long>(HardwareConfig::VALVE_SEQUENCE_WAIT_MS));
        queueValveLog(line);
      }
      return;
    case ValveSequenceState::WaitSecondOutput:
      if (millis() - valveSequenceStartedMs_ >=
          HardwareConfig::VALVE_SEQUENCE_WAIT_MS)
        valveSequenceState_ = ValveSequenceState::SequenceDone;
      return;
    case ValveSequenceState::SequenceDone: {
      char line[40];
      snprintf(line, sizeof(line), "[%s] DONE", valveSequenceName_);
      queueValveLog(line);
      valveSequenceState_ = ValveSequenceState::Idle;
      valveSequenceComplete_ = true;
      return;
    }
  }
}

void MechanismController::abortValveSequence() {
  valveSequenceState_ = ValveSequenceState::Idle;
  valveSequenceComplete_ = false;
  valveSequenceStartedMs_ = 0;
}

bool MechanismController::consumeValveSequenceComplete() {
  if (!valveSequenceComplete_) return false;
  valveSequenceComplete_ = false;
  return true;
}
void MechanismController::advanceActionStep() {
  ++actionStep_;
  stepStartedMs_ = millis();
  motionIssued_ = false;
}

void MechanismController::startPostDrop2BHome() {
  const StepperProfile &p = profile();
  motorA_.setMaxSpeed(HardwareConfig::POST_THA2B_SPEED_STEPS_S);
  motorB_.setMaxSpeed(HardwareConfig::POST_THA2B_SPEED_STEPS_S);
  motorA_.setAcceleration(HardwareConfig::POST_THA2B_ACCEL_STEPS_S2);
  motorB_.setAcceleration(HardwareConfig::POST_THA2B_ACCEL_STEPS_S2);

  zeroed_ = false;
  homeADone_ = false;
  homeBDone_ = false;
  homeAActiveSinceMs_ = 0;
  homeBActiveSinceMs_ = 0;
  // Phase 1: hold A and lower only B by the configured 400 mm.
  motorA_.setCurrentPosition(motorA_.currentPosition());
  const long bPrelowerSteps = lroundf(
      HardwareConfig::POST_THA2B_B_PRELOWER_MM * stepsPerMmB(p));
  motorB_.moveTo(motorB_.currentPosition() +
                 HardwareConfig::POST_THA2B_LOWER_DIRECTION_B *
                     bPrelowerSteps);
  operationDeadlineMs_ =
      millis() + HardwareConfig::POST_THA2B_HOME_TIMEOUT_MS;
}

void MechanismController::startPostDrop2BSynchronizedLowering() {
  motorA_.moveTo(motorA_.currentPosition() +
                 HardwareConfig::POST_THA2B_LOWER_DIRECTION_A *
                     HardwareConfig::HOME_SEARCH_STEPS);
  motorB_.moveTo(motorB_.currentPosition() +
                 HardwareConfig::POST_THA2B_LOWER_DIRECTION_B *
                     HardwareConfig::HOME_SEARCH_STEPS);
}

void MechanismController::startPostDrop2BReturnAToHome() {
  motorA_.setCurrentPosition(motorA_.currentPosition());
  motorB_.setCurrentPosition(0);
  homeBDone_ = true;
  homeAActiveSinceMs_ = 0;
  motorA_.moveTo(motorA_.currentPosition() +
                 HardwareConfig::HOME_DIRECTION_A *
                     HardwareConfig::HOME_SEARCH_STEPS);
}

void MechanismController::updatePostDrop2BPrelowerB() {
  const uint32_t now = millis();
  if (homeSwitchBActive()) {
    // HOME37 has priority even during the 400 mm pre-lower phase.
    motorB_.setCurrentPosition(motorB_.currentPosition());
    if (!homeBActiveSinceMs_) homeBActiveSinceMs_ = now;
    if (now - homeBActiveSinceMs_ >=
        HardwareConfig::HOME_SWITCH_CONFIRM_MS) {
      startPostDrop2BReturnAToHome();
      // Skip synchronized lowering because B is already at HOME37.
      advanceActionStep();
      advanceActionStep();
    }
    return;
  }

  homeBActiveSinceMs_ = 0;
  if (motorB_.distanceToGo() == 0) {
    startPostDrop2BSynchronizedLowering();
    advanceActionStep();
  }
}

void MechanismController::updatePostDrop2BLowerToBHome() {
  const uint32_t now = millis();
  if (homeSwitchBActive()) {
    // Stop both immediately while HOME37 is being debounced.
    motorA_.setCurrentPosition(motorA_.currentPosition());
    motorB_.setCurrentPosition(motorB_.currentPosition());
    if (!homeBActiveSinceMs_) homeBActiveSinceMs_ = now;
    if (now - homeBActiveSinceMs_ >=
        HardwareConfig::HOME_SWITCH_CONFIRM_MS) {
      startPostDrop2BReturnAToHome();
      advanceActionStep();
    }
    return;
  }

  homeBActiveSinceMs_ = 0;
  if (motorA_.distanceToGo() == 0)
    motorA_.moveTo(motorA_.currentPosition() +
                   HardwareConfig::POST_THA2B_LOWER_DIRECTION_A *
                       HardwareConfig::HOME_SEARCH_STEPS);
  if (motorB_.distanceToGo() == 0)
    motorB_.moveTo(motorB_.currentPosition() +
                   HardwareConfig::POST_THA2B_LOWER_DIRECTION_B *
                       HardwareConfig::HOME_SEARCH_STEPS);
}
void MechanismController::updatePostDrop2BReturnAToHome() {
  const uint32_t now = millis();
  if (homeSwitchAActive()) {
    motorA_.setCurrentPosition(motorA_.currentPosition());
    if (!homeAActiveSinceMs_) homeAActiveSinceMs_ = now;
    if (now - homeAActiveSinceMs_ >=
        HardwareConfig::HOME_SWITCH_CONFIRM_MS) {
      motorA_.setCurrentPosition(0);
      homeADone_ = true;
      zeroed_ = true;
      completeOperation();
    }
    return;
  }

  homeAActiveSinceMs_ = 0;
  if (motorA_.distanceToGo() == 0)
    motorA_.moveTo(motorA_.currentPosition() +
                   HardwareConfig::HOME_DIRECTION_A *
                       HardwareConfig::HOME_SEARCH_STEPS);
}
void MechanismController::updateCompetitionAction() {
  const float readyA = HardwareConfig::READY_HOME_A_POSITION_A_MM;
  const float readyB = HardwareConfig::READY_HOME_A_POSITION_B_MM;
  const float pickupA = readyA + HardwareConfig::PICK_LOWER_DIRECTION_A *
                                   HardwareConfig::PICK_LOWER_DISTANCE_MM;
  const float pickupB = readyB + HardwareConfig::PICK_LOWER_DIRECTION_B *
                                   HardwareConfig::PICK_LOWER_DISTANCE_MM;
  const float highA = readyA + HardwareConfig::POST_POINT_B_RAISE_DIRECTION_A *
                                  HardwareConfig::POST_POINT_B_RAISE_DISTANCE_MM;
  const float highB = readyB + HardwareConfig::POST_POINT_B_RAISE_DIRECTION_B *
                                  HardwareConfig::POST_POINT_B_RAISE_DISTANCE_MM;
  const float speed = HardwareConfig::COMPETITION_SPEED_STEPS_S;
  const float accel = HardwareConfig::COMPETITION_ACCEL_STEPS_S2;

  auto issueMove = [&](float a, float b) {
    if (!motionIssued_ &&
        issueAbsoluteMotion(a, b, speed, speed, accel, accel))
      advanceActionStep();
  };

  switch (competitionAction_) {
    case CompetitionAction::ReadyHomeA:
      if (actionStep_ == 0) issueMove(readyA, readyB);
      else if (actionStep_ == 1 && motorsAtTarget()) completeOperation();
      break;

    case CompetitionAction::PointA:
      if (actionStep_ == 0) issueMove(pickupA, pickupB);
      else if (actionStep_ == 1 && motorsAtTarget()) {
        motionIssued_ = false;
        if (startValveSequence("POINT_A", 2, true, 3, true,
                               HardwareConfig::VALVE_SEQUENCE_WAIT_MS))
          advanceActionStep();
      } else if (actionStep_ == 2 && consumeValveSequenceComplete()) {
        advanceActionStep();
      } else if (actionStep_ == 3) issueMove(readyA, readyB);
      else if (actionStep_ == 4 && motorsAtTarget()) completeOperation();
      break;

    case CompetitionAction::PointB:
      if (actionStep_ == 0) issueMove(pickupA, pickupB);
      else if (actionStep_ == 1 && motorsAtTarget()) {
        motionIssued_ = false;
        if (startValveSequence("POINT_B", 0, true, 1, true,
                               HardwareConfig::VALVE_SEQUENCE_WAIT_MS))
          advanceActionStep();
      } else if (actionStep_ == 2 && consumeValveSequenceComplete()) {
        advanceActionStep();
      } else if (actionStep_ == 3) issueMove(readyA, readyB);
      else if (actionStep_ == 4 && motorsAtTarget()) {
        motionIssued_ = false;
        advanceActionStep();
      } else if (actionStep_ == 5) issueMove(highA, highB);
      else if (actionStep_ == 6 && motorsAtTarget()) completeOperation();
      break;

    case CompetitionAction::Drop2A:
      if (actionStep_ == 0) {
        if (startValveSequence("THA_2A", 1, false, 3, false,
                               HardwareConfig::VALVE_SEQUENCE_WAIT_MS))
          advanceActionStep();
      } else if (actionStep_ == 1 && consumeValveSequenceComplete()) {
        completeOperation();
      }
      break;

    case CompetitionAction::Drop2B:
      if (actionStep_ == 0) {
        if (startValveSequence("THA_2B", 2, false, 0, false,
                               HardwareConfig::THA2B_FIRST_TO_SECOND_OUTPUT_MS))
          advanceActionStep();
      } else if (actionStep_ == 1 && consumeValveSequenceComplete()) {
        // Release the chassis after the pneumatic sequence. The following
        // mechanism HOME motion continues concurrently with robot travel.
        startPostDrop2BHome();
        advanceActionStep();
      } else if (actionStep_ == 2) {
        updatePostDrop2BPrelowerB();
      } else if (actionStep_ == 3) {
        updatePostDrop2BLowerToBHome();
      } else if (actionStep_ == 4) {
        updatePostDrop2BReturnAToHome();
      }
      break;
  }
}
void MechanismController::updateHome() {
  const uint32_t now = millis();

  if (!homeADone_) {
    if (homeSwitchAActive()) {
      motorA_.setCurrentPosition(motorA_.currentPosition());
      if (!homeAActiveSinceMs_) homeAActiveSinceMs_ = now;
      if (now - homeAActiveSinceMs_ >=
          HardwareConfig::HOME_SWITCH_CONFIRM_MS) {
        motorA_.setCurrentPosition(0);
        homeADone_ = true;
      }
    } else {
      homeAActiveSinceMs_ = 0;
      if (motorA_.distanceToGo() == 0)
        motorA_.moveTo(motorA_.currentPosition() +
                       HardwareConfig::HOME_DIRECTION_A *
                       HardwareConfig::HOME_SEARCH_STEPS);
      motorA_.run();
    }
  }

  if (!homeBDone_) {
    if (homeSwitchBActive()) {
      motorB_.setCurrentPosition(motorB_.currentPosition());
      if (!homeBActiveSinceMs_) homeBActiveSinceMs_ = now;
      if (now - homeBActiveSinceMs_ >=
          HardwareConfig::HOME_SWITCH_CONFIRM_MS) {
        motorB_.setCurrentPosition(0);
        homeBDone_ = true;
      }
    } else {
      homeBActiveSinceMs_ = 0;
      if (motorB_.distanceToGo() == 0)
        motorB_.moveTo(motorB_.currentPosition() +
                       HardwareConfig::HOME_DIRECTION_B *
                       HardwareConfig::HOME_SEARCH_STEPS);
      motorB_.run();
    }
  }

  if (homeADone_ && homeBDone_) {
    zeroed_ = true;
    completeOperation();
  }
}

void MechanismController::update() {
  valves_.update();
  if (!valves_.healthy()) {
    if (state_ != State::FaultStop)
      setFault(MechanismFault::Pcf8574Unavailable, "PCF8574_WRITE");
    return;
  }
  if (valveSequenceState_ != ValveSequenceState::Idle) updateValveSequence();
  if (state_ == State::FaultStop || state_ == State::Idle) return;

  if (jogging()) {
    if (millis() - lastJogCommandMs_ >=
        HardwareConfig::JOG_COMMAND_TIMEOUT_MS) {
      setFault(MechanismFault::JogLinkTimeout, "JOG_LINK_TIMEOUT");
      return;
    }
    motorA_.run();
    motorB_.run();
    return;
  }

  if (static_cast<int32_t>(millis() - operationDeadlineMs_) >= 0) {
    setFault(state_ == State::Homing ? MechanismFault::HomeTimeout
                                     : MechanismFault::MotionTimeout,
             state_ == State::Homing ? "HOME_TIMEOUT" : "MOTION_TIMEOUT");
    return;
  }

  if (state_ == State::Homing) {
    updateHome();
    return;
  }

  motorA_.run();
  motorB_.run();

  if (state_ == State::DirectMove) {
    if (motorsAtTarget()) completeOperation();
    return;
  }

  if (state_ == State::Competition) {
    updateCompetitionAction();
    return;
  }

  const StepperProfile &p = profile();
  if (sequenceIndex_ >= p.stepCount) {
    completeOperation();
    return;
  }
  const SequenceStep &step = p.steps[sequenceIndex_];
  switch (step.type) {
    case SequenceStepType::MoveProfile:
      if (!motionIssued_ && issueProfileMotion(p)) advanceStep();
      break;
    case SequenceStepType::WaitStepper:
      if (motorsAtTarget()) {
        motionIssued_ = false;
        advanceStep();
      }
      break;
    case SequenceStepType::ValveSet:
      if (!valves_.set(static_cast<uint8_t>(step.index), step.value != 0))
        setFault(valves_.available() ? MechanismFault::ValveInterlock
                                     : MechanismFault::Pcf8574Unavailable,
                 valves_.available() ? "VALVE_INTERLOCK"
                                      : "PCF8574_NOT_FOUND");
      else
        advanceStep();
      break;
    case SequenceStepType::WaitTime:
      if (millis() - stepStartedMs_ >= step.durationMs) advanceStep();
      break;
    case SequenceStepType::WaitSensor: {
      bool available = false;
      const bool value =
          sensorValue(static_cast<uint8_t>(step.index), available);
      if (!available)
        setFault(MechanismFault::SensorUnavailable, "SENSOR_UNAVAILABLE");
      else if (value == (step.value != 0))
        advanceStep();
      else if (step.durationMs &&
               millis() - stepStartedMs_ >= step.durationMs)
        setFault(MechanismFault::SensorTimeout, "SENSOR_TIMEOUT");
      break;
    }
    case SequenceStepType::End:
      completeOperation();
      break;
  }
}

bool MechanismController::setValve(uint8_t index, bool enabled) {
  if (busy_ || fault_ != MechanismFault::None || valves_.switchBusy())
    return false;
  if (index >= HardwareConfig::VALVE_COUNT) return false;
  if (!valves_.set(index, enabled)) {
    if (!valves_.healthy())
      setFault(MechanismFault::Pcf8574Unavailable, "PCF8574_WRITE");
    return false;
  }
  return true;
}

bool MechanismController::allValveCoilsOff() {
  valves_.allValveCoilsOff();
  if (valves_.healthy()) return true;
  setFault(MechanismFault::Pcf8574Unavailable, "PCF8574_WRITE");
  return false;
}
void MechanismController::setSoftwareZero() {
  if (busy_) return;
  motorA_.setCurrentPosition(0);
  motorB_.setCurrentPosition(0);
  zeroed_ = true;
}

bool MechanismController::startHome() {
  if (!config_ || busy_ || fault_ != MechanismFault::None ||
      !HardwareConfig::HAS_HOME_SWITCH)
    return false;
  selectProfileScale(ProfileId::RobotStart);
  setOperationName("HOME");
  zeroed_ = false;
  homeADone_ = false;
  homeBDone_ = false;
  homeAActiveSinceMs_ = 0;
  homeBActiveSinceMs_ = 0;
  motorA_.setMaxSpeed(HardwareConfig::HOME_SPEED_STEPS_S);
  motorB_.setMaxSpeed(HardwareConfig::HOME_SPEED_STEPS_S);
  motorA_.setAcceleration(HardwareConfig::HOME_ACCEL_STEPS_S2);
  motorB_.setAcceleration(HardwareConfig::HOME_ACCEL_STEPS_S2);
  motorA_.moveTo(motorA_.currentPosition() +
                 HardwareConfig::HOME_DIRECTION_A *
                 HardwareConfig::HOME_SEARCH_STEPS);
  motorB_.moveTo(motorB_.currentPosition() +
                 HardwareConfig::HOME_DIRECTION_B *
                 HardwareConfig::HOME_SEARCH_STEPS);
  busy_ = true;
  state_ = State::Homing;
  operationStartedMs_ = millis();
  operationDeadlineMs_ =
      operationStartedMs_ + HardwareConfig::HOME_TIMEOUT_MS;
  doneEvent_ = false;
  return true;
}

void MechanismController::emergencyStop(const char *reason) {
  abortValveSequence();
  motorA_.setCurrentPosition(motorA_.currentPosition());
  motorB_.setCurrentPosition(motorB_.currentPosition());
  jogSpeed_[0] = jogSpeed_[1] = 0.0f;
  doneEvent_ = false;
  busy_ = false;
  motionIssued_ = false;
  state_ = State::Idle;
  valves_.safeOff();
  if (reason && strcmp(reason, "STOP") != 0 &&
      strcmp(reason, "BOOT_SAFE") != 0)
    setFault(MechanismFault::Busy, reason);
}

void MechanismController::clearFault() {
  if (busy_ || !valves_.healthy()) return;
  fault_ = MechanismFault::None;
  strcpy(faultText_, "NONE");
  faultEvent_ = false;
  state_ = State::Idle;
}

void MechanismController::advanceStep() {
  ++sequenceIndex_;
  stepStartedMs_ = millis();
}

void MechanismController::completeOperation() {
  strncpy(doneText_, operationName_, sizeof(doneText_) - 1);
  doneText_[sizeof(doneText_) - 1] = '\0';
  busy_ = false;
  motionIssued_ = false;
  state_ = State::Idle;
  doneEvent_ = true;
}

void MechanismController::setFault(MechanismFault fault, const char *reason) {
  abortValveSequence();
  motorA_.setCurrentPosition(motorA_.currentPosition());
  motorB_.setCurrentPosition(motorB_.currentPosition());
  jogSpeed_[0] = jogSpeed_[1] = 0.0f;
  doneEvent_ = false;
  valves_.safeOff();
  busy_ = false;
  state_ = State::FaultStop;
  fault_ = fault;
  strncpy(faultText_, reason ? reason : "FAULT", sizeof(faultText_) - 1);
  faultText_[sizeof(faultText_) - 1] = '\0';
  faultEvent_ = true;
}

bool MechanismController::takeDoneEvent(char *profileText, size_t capacity) {
  if (!doneEvent_) return false;
  doneEvent_ = false;
  if (profileText && capacity) {
    strncpy(profileText, doneText_, capacity - 1);
    profileText[capacity - 1] = '\0';
  }
  return true;
}

bool MechanismController::takeFaultEvent(char *reason, size_t capacity) {
  if (!faultEvent_) return false;
  faultEvent_ = false;
  if (reason && capacity) {
    strncpy(reason, faultText_, capacity - 1);
    reason[capacity - 1] = '\0';
  }
  return true;
}

bool MechanismController::takeValveLogEvent(char *text, size_t capacity) {
  if (!valveLogEvent_) return false;
  valveLogEvent_ = false;
  if (text && capacity) {
    strncpy(text, valveLogText_, capacity - 1);
    text[capacity - 1] = '\0';
  }
  return true;
}
bool MechanismController::sensorValue(uint8_t sensorIndex,
                                      bool &available) const {
  int8_t pin = -1;
  if (sensorIndex == static_cast<uint8_t>(SensorId::E18))
    pin = HardwareConfig::E18_SENSOR_PIN;
  else if (sensorIndex == static_cast<uint8_t>(SensorId::Line))
    pin = HardwareConfig::LINE_SENSOR_PIN;
  available = pin >= 0;
  return available ? digitalRead(pin) == HIGH : false;
}

const char *MechanismController::stateName() const {
  switch (state_) {
    case State::Idle: return "IDLE";
    case State::Sequence: return "SEQUENCE";
    case State::DirectMove: return "DIRECT_MOVE";
    case State::ManualJog: return "MANUAL_JOG";
    case State::Homing: return "HOMING";
    case State::Competition: return "COMPETITION";
    case State::FaultStop: return "FAULT_STOP";
  }
  return "UNKNOWN";
}
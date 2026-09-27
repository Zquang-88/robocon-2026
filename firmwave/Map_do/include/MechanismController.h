#pragma once

#include <AccelStepper.h>
#include <Arduino.h>

#include "StepperProfiles.h"
#include "ValveController.h"

enum class MechanismFault : uint8_t {
  None = 0,
  Busy,
  InvalidProfile,
  PositionLimit,
  MotionTimeout,
  SensorUnavailable,
  SensorTimeout,
  ValveInterlock,
  NotZeroed,
  JogLinkTimeout,
  HomeTimeout,
  Pcf8574Unavailable
};

enum class CompetitionAction : uint8_t {
  ReadyHomeA = 0,
  PointA,
  PointB,
  PointC,
  BridgeBFinish,
  Drop2A,
  Drop2B
};

enum class MechanismField : uint8_t {
  Red = 0,
  Blue = 1
};

class MechanismController {
 public:
  MechanismController(AccelStepper &motorA, AccelStepper &motorB,
                      ValveController &valves);

  void begin(MechanismConfig &config);
  void update();
  bool startProfile(ProfileId id);
  bool startCompetitionAction(CompetitionAction action);
  bool setCompetitionField(MechanismField field);
  MechanismField competitionField() const { return competitionField_; }
  const char *competitionFieldName() const {
    return competitionField_ == MechanismField::Blue ? "BLUE" : "RED";
  }
  bool startDirectMove(float signedAmm, float signedBmm,
                       float speedStepsPerSecond);
  bool setJog(float signedSpeedA, float signedSpeedB);
  void stopJog();
  bool setValve(uint8_t index, bool enabled);
  bool setValveMask(uint8_t enabledMask);
  bool allValveCoilsOff();
  void setSoftwareZero();
  bool startHome(bool sequentialBFirst = false);
  void emergencyStop(const char *reason = "STOP");
  void clearFault();

  bool busy() const { return busy_; }
  bool homing() const { return state_ == State::Homing; }
  bool competitionActionRunning() const {
    return state_ == State::Competition;
  }
  bool jogging() const { return state_ == State::ManualJog; }
  float jogSpeed(uint8_t axis) const {
    return axis < 2 ? jogSpeed_[axis] : 0.0f;
  }
  bool zeroed() const { return zeroed_; }
  bool homeSwitchAActive() const;
  bool homeSwitchBActive() const;
  bool valvesAvailable() const { return valves_.healthy(); }
  uint8_t pcf8574Address() const { return valves_.address(); }
  MechanismFault fault() const { return fault_; }
  const char *faultText() const { return faultText_; }
  const char *stateName() const;
  const char *activeProfileName() const { return operationName_; }
  uint8_t activeStep() const {
    return state_ == State::Competition ? actionStep_ : sequenceIndex_;
  }
  float positionAmm() const;
  float positionBmm() const;
  uint8_t valveMask() const { return valves_.mask(); }
  bool takeDoneEvent(char *profile, size_t capacity);
  bool takeFaultEvent(char *reason, size_t capacity);
  bool takeValveLogEvent(char *text, size_t capacity);

 private:
  enum class State : uint8_t {
    Idle,
    Sequence,
    DirectMove,
    ManualJog,
    Homing,
    Competition,
    FaultStop
  };

  // Non-blocking two-output pneumatic sequence. ValveController independently
  // performs the 50 ms break-before-make transition for every switch.
  enum class ValveSequenceState : uint8_t {
    Idle,
    SwitchFirstOutput,
    WaitFirstSwitch,
    WaitFirstOutput,
    SwitchSecondOutput,
    WaitSecondSwitch,
    WaitSecondOutput,
    SequenceDone
  };

  AccelStepper &motorA_;
  AccelStepper &motorB_;
  ValveController &valves_;
  MechanismConfig *config_ = nullptr;
  State state_ = State::Idle;
  MechanismFault fault_ = MechanismFault::None;
  char faultText_[32] = "NONE";
  char operationName_[24] = "NONE";
  char doneText_[24] = "NONE";
  bool busy_ = false;
  bool zeroed_ = false;
  bool motionIssued_ = false;
  bool doneEvent_ = false;
  bool faultEvent_ = false;
  ProfileId activeProfile_ = ProfileId::RobotStart;
  uint8_t sequenceIndex_ = 0;
  uint32_t operationStartedMs_ = 0;
  uint32_t stepStartedMs_ = 0;
  uint32_t operationDeadlineMs_ = 0;
  uint32_t lastJogCommandMs_ = 0;
  float jogSpeed_[2] = {};

  bool homeADone_ = false;
  bool homeBDone_ = false;
  bool homeBFirst_ = false;
  uint32_t homeAActiveSinceMs_ = 0;
  uint32_t homeBActiveSinceMs_ = 0;

  CompetitionAction competitionAction_ = CompetitionAction::ReadyHomeA;
  MechanismField competitionField_ = MechanismField::Red;
  uint8_t actionStep_ = 0;

  ValveSequenceState valveSequenceState_ = ValveSequenceState::Idle;
  char valveSequenceName_[16] = "NONE";
  uint8_t valveFirstPin_ = 0;
  uint8_t valveSecondPin_ = 0;
  bool valveFirstEnabled_ = false;
  bool valveSecondEnabled_ = false;
  uint32_t valveSequenceStartedMs_ = 0;
  uint32_t valveFirstOutputWaitMs_ = 0;
  bool valveSequenceComplete_ = false;
  char valveLogText_[80] = {};
  bool valveLogEvent_ = false;

  const StepperProfile &profile() const;
  float stepsPerMmA(const StepperProfile &p) const;
  float stepsPerMmB(const StepperProfile &p) const;
  void selectProfileScale(ProfileId id);
  void applyJogAxis(AccelStepper &motor, uint8_t axis, float speed,
                    long target, float acceleration);
  bool issueProfileMotion(const StepperProfile &p);
  bool issueAbsoluteMotion(float targetAmm, float targetBmm,
                           float maxSpeedA, float maxSpeedB,
                           float accelerationA, float accelerationB);
  bool targetsWithinLimits(float targetAmm, float targetBmm) const;
  bool sensorValue(uint8_t sensorIndex, bool &available) const;
  bool motorsAtTarget() const;
  bool startSimultaneousValveOutputs(const char *name,
                                     uint8_t firstPin, uint8_t secondPin);
  bool startValveSequence(const char *name,
                          uint8_t firstPin, bool firstEnabled,
                          uint8_t secondPin, bool secondEnabled,
                          uint32_t firstOutputWaitMs);
  void updateValveSequence();
  void abortValveSequence();
  bool consumeValveSequenceComplete();
  void queueValveLog(const char *text);
  void queueValveSwitchLog(uint8_t pin, bool enabled);
  void updateHome();
  void startPostDrop2BHome();
  void startPostDrop2BSynchronizedLowering();
  void startPostDrop2BReturnAToHome();
  void updatePostDrop2BPrelowerB();
  void updatePostDrop2BLowerToBHome();
  void updatePostDrop2BReturnAToHome();
  void updateCompetitionAction();
  void updateFieldLed();
  void advanceActionStep();
  void advanceStep();
  void completeOperation();
  void setFault(MechanismFault fault, const char *reason);
  void setOperationName(const char *name);
};

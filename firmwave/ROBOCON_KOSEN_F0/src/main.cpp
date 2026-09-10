#include <Arduino.h>
#include <Encoder.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <PWFusion_VL53L3C.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "RobotConfig.h"
#include "Esp32MechanismClient.h"
#include "MechanismTelemetryView.h"
using namespace RobotConfig;

#ifndef ROBOCON_AFTER_BRIDGE_TEST_BUILD
#define ROBOCON_AFTER_BRIDGE_TEST_BUILD 0
#endif

// Keep a direct USB Serial command path for supervised tests. These functions
// are declared before the Serial macro below, so they always refer to the
// Teensy native USB Serial object rather than Serial5.
static Stream &usbCommandStream() { return Serial; }
static void beginUsbCommandStream() { Serial.begin(115200); }

// Route RBT/1 replies and telemetry prints to Serial5. USB Serial (COM14) is a
// receive-only backup command path for supervised service tests.
#define Serial TELEMETRY_UART

// ============================================================
// Types, faults and shared state
// ============================================================

enum FaultCode : uint32_t {
  FAULT_NONE          = 0,
  FAULT_IMU_TIMEOUT   = 1UL << 0,
  FAULT_FLOW_TIMEOUT  = 1UL << 1,
  FAULT_LINE_LOST     = 1UL << 2,
  FAULT_TOF_TIMEOUT   = 1UL << 3,
  FAULT_ENCODER       = 1UL << 4,
  FAULT_ESP_TIMEOUT   = 1UL << 5,
  FAULT_STATE_TIMEOUT = 1UL << 6,
  FAULT_CONFIG        = 1UL << 7
};

enum LineState : uint8_t {
  LINE_VALID,
  LINE_LOST_LEFT,
  LINE_LOST_RIGHT,
  LINE_LOST_UNKNOWN,
  CROSS_LINE
};

enum AutoState : uint8_t {
  WAIT_START,
  DOI_HOME_A_READY,
  CAN_START,
  CAN_START_FAST,
  CAN_START_FINE,
  DI_SANG_TRAI_A,
  BU_TAM_A,
  CAN_YAW_A,
  CAN_A,
  DOI_GAP_A,
  DI_SANG_TRAI_B,
  BU_TAM_B,
  CAN_YAW_B,
  CAN_B,
  DOI_GAP_B,
  DI_SANG_C,
  BU_TAM_C,
  CAN_YAW_C,
  CHAY_LEN_XANH,
  CAN_GIUA_LINE_C,
  DI_LEN_DEM_LINE,
  HOME_CENTER,
  HOME_DROP,
  HOME_TO_B,
  TIEN_34CM_B,
  DOI_THA_2B,
  CHO_SAU_B,
  LUI_33CM_A,
  DOI_THA_2A,
  CHO_SAU_THA_A,
  LUI_3M,
  SANG_TRAI_1M5,
  CAN_PHAI_LINE_B_E18,
  TIEN_E18_70,
  DOI_DONE_THA2A,
  DI_CHEO_PHAI_THA2B,
  CAN_LINE_DOC_PHAI_THA2B,
  LUI_BAM_LINE_DOC_THA2B,
  LUI_RA_LINE_NGANG_THA2B,
  DOI_DONE_THA2B,
  SAU_THA2B_DI_CHEO_PHAI_2M_LUI_3M,

  FINISH,
  FAULT_STOP
};

struct RobotPose {
  float x_mm = 0;
  float y_mm = 0;
  float yaw_deg = 0;
};

struct OpticalFlowData {
  float deltaX = 0;
  float deltaY = 0;
  float velocityX = 0;
  float velocityY = 0;
  float quality = 0;
  float groundDistance = 0;
  bool valid = false;
  bool fresh = false;
  uint32_t lastUpdateMs = 0;
};

struct LineArrayData {
  uint16_t raw[8] = {};
  float filtered[8] = {};
  uint16_t normalized[8] = {};
  float position = 0;
  float lastPosition = 0;
  uint8_t activeCount = 0;
  bool valid = false;
  bool cross = false;
  LineState state = LINE_LOST_UNKNOWN;
  uint32_t lastValidMs = 0;
};

struct StopArrayData {
  uint16_t raw[6] = {};
  float filtered[6] = {};
  uint16_t normalized[6] = {};
  float leftPosition = 0;
  float rightPosition = 0;
  bool leftValid = false;
  bool rightValid = false;
  uint16_t holdRaw[2] = {};
  float holdFiltered[2] = {};
  uint16_t holdNormalized[2] = {};
};

class PID {
 public:
  float kp = 0, ki = 0, kd = 0;
  float integral = 0, previousError = 0;
  float integralLimit = 0;

  PID() = default;
  PID(float p, float i, float d, float limit)
      : kp(p), ki(i), kd(d), integralLimit(limit) {}

  void reset() { integral = 0; previousError = 0; }

  float updateError(float error, float dt, float low, float high) {
    if (dt <= 0 || !isfinite(error)) return 0;
    const float derivative = (error - previousError) / dt;
    const float candidateIntegral = constrain(integral + error * dt,
                                               -integralLimit, integralLimit);
    const float candidate = kp * error + ki * candidateIntegral + kd * derivative;
    const bool saturatingHigh = candidate > high && error > 0;
    const bool saturatingLow = candidate < low && error < 0;
    if (!saturatingHigh && !saturatingLow) integral = candidateIntegral;
    previousError = error;
    return constrain(kp * error + ki * integral + kd * derivative, low, high);
  }
};

enum WheelId : uint8_t { FL, FR, RL, RR, WHEEL_COUNT };

struct WheelControl {
  Encoder *encoder;
  uint8_t rpwm, lpwm;
  int8_t motorSign, encoderSign;
  long count = 0;
  long previousCount = 0;
  float deltaMm = 0;
  float targetMmS = 0;
  float measuredMmS = 0;
  int pwm = 0;
  float kff = 0;
  int deadzone = 0;
  PID pid;
  uint32_t lastEncoderMotionMs = 0;
};

Encoder encFL(FL_ENC_A, FL_ENC_B);
Encoder encFR(FR_ENC_A, FR_ENC_B);
Encoder encRL(RL_ENC_A, RL_ENC_B);
Encoder encRR(RR_ENC_A, RR_ENC_B);

WheelControl wheels[WHEEL_COUNT] = {
  {&encFL, FL_RPWM, FL_LPWM, MOTOR_SIGN_FL, ENCODER_SIGN_FL},
  {&encFR, FR_RPWM, FR_LPWM, MOTOR_SIGN_FR, ENCODER_SIGN_FR},
  {&encRL, RL_RPWM, RL_LPWM, MOTOR_SIGN_RL, ENCODER_SIGN_RL},
  {&encRR, RR_RPWM, RR_LPWM, MOTOR_SIGN_RR, ENCODER_SIGN_RR}
};

RobotPose pose;
OpticalFlowData flow;
LineArrayData centerLine;
StopArrayData stopLine;

Adafruit_BNO08x bno085(-1);
sh2_SensorValue_t bnoEvent;
VL53L3C tof;

PID headingPid(HEADING_KP, HEADING_KI, HEADING_KD, HEADING_INTEGRAL_LIMIT);
PID fineHeadingPid(FINE_HEADING_KP, 0.0f, FINE_HEADING_KD, 0.0f);
PID linePid(LINE_KP, LINE_KI, LINE_KD, 2500.0f);
PID positionXPid(POSITION_X_KP, POSITION_X_KI, POSITION_X_KD, POSITION_X_INTEGRAL_LIMIT);
PID positionYPid(POSITION_Y_KP, POSITION_Y_KI, POSITION_Y_KD, POSITION_Y_INTEGRAL_LIMIT);
PID stopForwardPid(STOP_FORWARD_KP, STOP_FORWARD_KI, STOP_FORWARD_KD, STOP_FORWARD_INTEGRAL_LIMIT);
PID stopYawPid(STOP_YAW_KP, STOP_YAW_KI, STOP_YAW_KD, STOP_YAW_INTEGRAL_LIMIT);
PID tofDistancePid(TOF_DISTANCE_KP, TOF_DISTANCE_KI, TOF_DISTANCE_KD, TOF_DISTANCE_INTEGRAL_LIMIT);
PID rightLinePid(RIGHT_LINE_KP, RIGHT_LINE_KI, RIGHT_LINE_KD, RIGHT_LINE_INTEGRAL_LIMIT);
PID leftLinePid(LEFT_LINE_KP, LEFT_LINE_KI, LEFT_LINE_KD, LEFT_LINE_INTEGRAL_LIMIT);

float maxWheelSpeedMmS = MAX_WHEEL_SPEED_MM_S;
float maxManualHeadingWzRadS = MAX_MANUAL_HEADING_WZ_RAD_S;
float maxAutoHeadLockWzRadS = MAX_AUTO_HEAD_LOCK_WZ_RAD_S;
float headingOutputDeadbandDeg = HEADING_OUTPUT_DEADBAND_DEG;
float headingMinWzRadS = HEADING_MIN_WZ_RAD_S;

uint32_t faultFlags = FAULT_NONE;
bool armed = false;
enum ControlMode : uint8_t { CONTROL_MANUAL = 0, CONTROL_AUTO = 1 };
enum FieldSide : uint8_t { FIELD_RED = 0, FIELD_BLUE = 1 };
enum AutoMechanismMode : uint8_t { AUTO_MECHANISM_REAL = 0 };
enum AutoTelemetryMode : uint8_t {
  AUTO_TELEMETRY_ON_REQUEST = 0,
  AUTO_TELEMETRY_SILENT = 1
};
ControlMode controlMode = CONTROL_MANUAL;
FieldSide selectedField = FIELD_RED;
// AUTO always uses the real ESP32 mechanism.
AutoMechanismMode autoMechanismMode = AUTO_MECHANISM_REAL;
// Competition AUTO never streams the large periodic telemetry burst. In
// ON_REQUEST mode the dashboard may request one compact, staged snapshot.
// SILENT rejects snapshots, while safety/state/fault records remain enabled.
AutoTelemetryMode autoTelemetryMode = AUTO_TELEMETRY_ON_REQUEST;
uint8_t autoTelemetrySnapshotStage = 0;
uint32_t autoTelemetrySnapshotSequence = 0;

int8_t selectedFieldLateralSign() {
  return selectedField == FIELD_RED ? RED_FIELD_LATERAL_SIGN
                                    : BLUE_FIELD_LATERAL_SIGN;
}

bool selectedFieldUsesRightStopGroup() {
  // Confirmed physical sensor selection: RED counts with the right-side
  // three-eye array. BLUE mirrors this and counts with the left-side array.
  return selectedField == FIELD_RED;
}

bool secondPickUsesRightStopGroup() {
  // Point B is reached by the opposite side of the chassis.  Do not keep
  // counting with the point-A array or RED will stop on line 3 at the right.
  return !selectedFieldUsesRightStopGroup();
}

bool softwareEStopLatched = false;
bool bnoInitialized = false;
bool bnoValid = false;
bool tofInitialized = false;
int16_t tofInitModeStatus = 32767, tofInitBudgetStatus = 32767, tofInitStartStatus = 32767;
bool tofValid = false;
// Accept valid VL53L3CX samples continuously so telemetry is available in
// MANUAL as well as AUTO. State transitions may clear the filter history,
// but must not gate every subsequent sample forever.
bool tofAcceptanceEnabled = true;
bool calibrationActive = false;
bool calibrationCenterOnly = false;
bool calibrationStopOnly = false;
bool espDone = false;
bool espDoneTha2A = false;
bool espDoneTha2B = false;
bool espE18BothDetected = false;
bool autoMechanismWaiting = false;
uint32_t autoMechanismStartedMs = 0;
char autoMechanismAction[24] = "NONE";
bool xAligned = false, yAligned = false;
bool supervisedAutoRun = false;
char lastEspCommand[64] = "NONE";
char lastEspResponse[96] = "NONE";
Esp32MechanismClient mechanismClient(ESP32_UART);
MechanismTelemetryView mechanismTelemetry;
float robotVx = 0, robotVy = 0, robotWz = 0;
float positionErrorX = 0, positionErrorY = 0;
float lateralHoldXMm = 0.0f;
float lateralHoldTofMm = 0.0f;
bool lateralHoldTofCaptured = false;
float lateralHeadingCommandWz = 0.0f;
float stationHeadingCommandWz = 0.0f;
float lateralLineHoldCommandVx = 0.0f;
int8_t lateralLineHoldLastSign = 0;
uint32_t lateralLineHoldLastSeenMs = 0;
uint32_t lateralLineHoldStartedMs = 0;
float firstLateralStartedYmm = 0.0f;
bool firstLateralGuidanceArmed = false;
float currentYawDeg = 0, targetYawDeg = 0, yawReferenceDeg = 0;
float yawRateDegS = 0.0f;
float currentRollDeg = 0.0f, currentPitchDeg = 0.0f;
bool bnoTiltInitialized = false;
float previousBnoYawDeg = 0.0f;
uint32_t previousBnoYawMs = 0;
enum BridgeSlopePhase : uint8_t {
  BRIDGE_PHASE_APPROACH = 0,
  BRIDGE_PHASE_ASCENDING,
  BRIDGE_PHASE_CREST,
  BRIDGE_PHASE_DESCENDING
};
enum BridgeEndMarkerSource : uint8_t {
  BRIDGE_MARKER_NONE = 0,
  BRIDGE_MARKER_CENTER_8,
  BRIDGE_MARKER_SIDE_CLUSTERS
};
BridgeSlopePhase bridgeSlopePhase = BRIDGE_PHASE_APPROACH;
BridgeEndMarkerSource bridgeEndMarkerSource = BRIDGE_MARKER_NONE;
RobotPose bridgeEndMarkerPose;
float bridgeEndMarkerAdvanceTargetMm = 0.0f;
float bridgeLevelRollDeg = 0.0f, bridgeLevelPitchDeg = 0.0f;
float bridgeRelativeTiltDeg = 0.0f;
uint32_t bridgeInclineSinceMs = 0, bridgeLevelSinceMs = 0;
uint32_t bridgeDescentSinceMs = 0;
uint32_t bridgeDescentLevelSinceMs = 0;
bool bridgeEndMarkerArmed = false;
uint32_t bridgeEndMarkerSinceMs = 0, bridgeSideMarkerSinceMs = 0;
uint32_t postBridgeE18LineSinceMs = 0;
bool postBridgeCoarseMoveDone = false;
uint32_t lastPostBridgeE18PollMs = 0;
uint8_t doneTha2AAttempts = 0;
uint32_t lastDoneTha2ASendMs = 0;
uint8_t doneTha2BAttempts = 0;
uint32_t lastDoneTha2BSendMs = 0;
uint32_t bridgeAscentDetectedMs = 0;
uint8_t bridgeSlopeAxis = 0; // 1=roll X, 2=pitch Y
float competitionHeadingDeg = 0.0f; // Captured once at START and held through A/B
float tofDistanceMm = 0;
float lateralSlipMmS = 0;
uint32_t lastBnoMs = 0, lastTofMs = 0;
uint32_t lastControlUs = 0, lastTelemetryMs = 0;
AutoState state = WAIT_START;

class DebouncedActiveLowButton {
 public:
  void begin(uint8_t assignedPin) {
    pin = assignedPin;
    pinMode(pin, INPUT_PULLUP);
    rawHigh = digitalRead(pin) == HIGH;
    stableHigh = rawHigh;
    rawChangedMs = millis();
  }

  bool pressed() {
    if (pin == 0xFF) return false;
    const bool sampleHigh = digitalRead(pin) == HIGH;
    const uint32_t now = millis();
    if (sampleHigh != rawHigh) {
      rawHigh = sampleHigh;
      rawChangedMs = now;
    }
    if (sampleHigh != stableHigh &&
        now - rawChangedMs >= PHYSICAL_BUTTON_DEBOUNCE_MS) {
      stableHigh = sampleHigh;
      return !stableHigh;
    }
    return false;
  }

 private:
  uint8_t pin = 0xFF;
  bool rawHigh = true;
  bool stableHigh = true;
  uint32_t rawChangedMs = 0;
};

class NonBlockingBuzzer {
 public:
  void begin() {
    pinMode(BUZZER_PIN, OUTPUT);
    setOutput(false);
  }

  void beep(uint8_t count) {
    enqueue(count, false);
  }

  void beepFast(uint8_t count) {
    enqueue(count, true);
  }

  void alignmentAlarm() {
    head = tail = 0;
    patternActive = false;
    alarmActive = true;
    alarmEndsMs = millis() + BUZZER_ALIGNMENT_ALARM_MS;
    deadlineMs = millis() + BUZZER_ALARM_TOGGLE_MS;
    setOutput(true);
  }

  void update() {
    const uint32_t now = millis();
    if (alarmActive) {
      if (static_cast<int32_t>(now - alarmEndsMs) >= 0) {
        alarmActive = false;
        setOutput(false);
        deadlineMs = now + BUZZER_PATTERN_GAP_MS;
      } else if (static_cast<int32_t>(now - deadlineMs) >= 0) {
        setOutput(!outputOn);
        deadlineMs = now + BUZZER_ALARM_TOGGLE_MS;
      }
      return;
    }

    if (!patternActive) {
      if (head == tail || static_cast<int32_t>(now - deadlineMs) < 0) return;
      const uint8_t encodedPattern = queue[head];
      currentFast = (encodedPattern & FAST_PATTERN_FLAG) != 0;
      pulsesRemaining = encodedPattern & COUNT_MASK;
      head = static_cast<uint8_t>((head + 1) % QUEUE_SIZE);
      patternActive = true;
      setOutput(true);
      deadlineMs = now + (currentFast ? BUZZER_UART_FAST_ON_MS
                                       : BUZZER_BEEP_ON_MS);
      return;
    }

    if (static_cast<int32_t>(now - deadlineMs) < 0) return;
    if (outputOn) {
      setOutput(false);
      if (pulsesRemaining > 0) --pulsesRemaining;
      if (pulsesRemaining == 0) {
        patternActive = false;
        deadlineMs = now + (currentFast ? BUZZER_UART_FAST_GAP_MS
                                        : BUZZER_PATTERN_GAP_MS);
      } else {
        deadlineMs = now + (currentFast ? BUZZER_UART_FAST_OFF_MS
                                        : BUZZER_BEEP_OFF_MS);
      }
    } else {
      setOutput(true);
      deadlineMs = now + (currentFast ? BUZZER_UART_FAST_ON_MS
                                     : BUZZER_BEEP_ON_MS);
    }
  }

 private:
  static constexpr uint8_t QUEUE_SIZE = 8;
  static constexpr uint8_t FAST_PATTERN_FLAG = 0x80;
  static constexpr uint8_t COUNT_MASK = 0x7F;
  uint8_t queue[QUEUE_SIZE] = {};
  uint8_t head = 0, tail = 0, pulsesRemaining = 0;
  bool patternActive = false, alarmActive = false, outputOn = false;
  bool currentFast = false;
  uint32_t deadlineMs = 0, alarmEndsMs = 0;

  void enqueue(uint8_t count, bool fast) {
    count &= COUNT_MASK;
    if (count == 0) return;
    const uint8_t nextTail = static_cast<uint8_t>((tail + 1) % QUEUE_SIZE);
    if (nextTail == head) head = static_cast<uint8_t>((head + 1) % QUEUE_SIZE);
    queue[tail] = count | (fast ? FAST_PATTERN_FLAG : 0);
    tail = nextTail;
  }

  void setOutput(bool enabled) {
    outputOn = enabled;
    digitalWrite(BUZZER_PIN,
                 enabled == BUZZER_ACTIVE_HIGH ? HIGH : LOW);
  }
};

DebouncedActiveLowButton physicalStartButton;
DebouncedActiveLowButton physicalFieldButton;
NonBlockingBuzzer buzzer;
uint8_t pendingPhysicalFieldClicks = 0;
uint32_t lastPhysicalFieldClickMs = 0;

uint16_t centerCalMin[8];
uint16_t centerCalMax[8];
uint16_t stopCalMin[6];
uint16_t stopCalMax[6];

void applyFactoryPidConfig() {
  for(uint8_t i=0;i<4;i++){
    wheels[i].pid.kp=WHEEL_KP[i];wheels[i].pid.ki=WHEEL_KI[i];
    wheels[i].pid.kd=WHEEL_KD[i];wheels[i].kff=WHEEL_KFF[i];
    wheels[i].deadzone=WHEEL_DEADZONE_PWM[i];wheels[i].pid.reset();
  }
  headingPid.kp=HEADING_KP;headingPid.ki=HEADING_KI;headingPid.kd=HEADING_KD;
  linePid.kp=LINE_KP;linePid.ki=LINE_KI;linePid.kd=LINE_KD;
  maxWheelSpeedMmS=MAX_WHEEL_SPEED_MM_S;
  maxManualHeadingWzRadS=MAX_MANUAL_HEADING_WZ_RAD_S;
  maxAutoHeadLockWzRadS=MAX_AUTO_HEAD_LOCK_WZ_RAD_S;
  headingOutputDeadbandDeg=HEADING_OUTPUT_DEADBAND_DEG;
  headingMinWzRadS=HEADING_MIN_WZ_RAD_S;
  headingPid.reset();linePid.reset();
}

// ============================================================
// Utility and safety
// ============================================================

float wrap180(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

float shortestAngleError(float target, float current) {
  return wrap180(target - current);
}

float readFloatLE(const uint8_t *p) {
  float value;
  memcpy(&value, p, sizeof(value));
  return value;
}

uint32_t readU32LE(const uint8_t *p) {
  uint32_t value;
  memcpy(&value, p, sizeof(value));
  return value;
}

void resetAllControllers();
void stopRobot();
void transitionTo(AutoState next);

bool stationAlignmentInProgress() {
  return state == DI_SANG_TRAI_A || state == BU_TAM_A ||
         state == CAN_YAW_A || state == CAN_A ||
         state == DI_SANG_TRAI_B || state == BU_TAM_B ||
         state == CAN_YAW_B || state == CAN_B;
}

void setFault(FaultCode fault, const char *reason) {
  const bool newFault = (faultFlags & fault) == 0;
  faultFlags |= fault;
  if (newFault) {
    Serial.print("FAULT,");
    Serial.println(reason);
    Serial.print("SAFETY,FAULT,");
    Serial.print(faultFlags);
    Serial.print(',');
    Serial.println(reason);
    if (stationAlignmentInProgress()) buzzer.alignmentAlarm();
  }
  armed = false;
  stopRobot();
}

bool stopInputActive() {
  return softwareEStopLatched;
}

void clearFaults() {
  if (stopInputActive()) return;
  faultFlags = FAULT_NONE;
  armed = false;
  resetAllControllers();
  Serial.println("ACK,RESET");
}

bool hardwareConfigurationValid() {
  return HARDWARE_CONFIG_VERIFIED && dimensionsValid() &&
         MECANUM_X_CONFIGURATION;
}

bool competitionConfigurationValid() {
  return hardwareConfigurationValid() && ESP32_UART_ENABLED &&
         (LEFT_STOP_CONTROL_SIGN == -1 || LEFT_STOP_CONTROL_SIGN == 1) &&
         (RIGHT_STOP_CONTROL_SIGN == -1 || RIGHT_STOP_CONTROL_SIGN == 1) &&
         CENTER_LINE_CALIBRATION_VERIFIED &&
         STOP_LINE_CALIBRATION_VERIFIED && STOP_SENSOR_ORDER_VERIFIED;
}

bool wheelTuneConfigurationValid() {
  return HARDWARE_CONFIG_VERIFIED&&WHEEL_DIAMETER_MM>0.0f&&
         ENCODER_COUNTS_PER_WHEEL_REV>0.0f;
}

bool configurationAllowsMotion() {
  return hardwareConfigurationValid() && faultFlags == FAULT_NONE &&
         !stopInputActive();
}

// ============================================================
// Motor output, encoder measurement and wheel PID
// ============================================================

void writeMotor(WheelControl &wheel, int pwm) {
  pwm = constrain(pwm * wheel.motorSign, -PWM_MAX, PWM_MAX);
  analogWrite(wheel.rpwm, pwm > 0 ? pwm : 0);
  analogWrite(wheel.lpwm, pwm < 0 ? -pwm : 0);
}

void resetWheelPid(WheelControl &wheel) {
  wheel.targetMmS = 0;
  wheel.pwm = 0;
  wheel.pid.reset();
  wheel.lastEncoderMotionMs = millis();
  writeMotor(wheel, 0);
}

void resetAllControllers() {
  for (WheelControl &wheel : wheels) resetWheelPid(wheel);
  headingPid.reset();
  fineHeadingPid.reset();
  linePid.reset();
  positionXPid.reset();
  positionYPid.reset();
  stopForwardPid.reset();
  stopYawPid.reset();
  tofDistancePid.reset();
  rightLinePid.reset();
  leftLinePid.reset();
  lateralHeadingCommandWz = 0.0f;
  stationHeadingCommandWz = 0.0f;
  lateralLineHoldCommandVx = 0.0f;
  robotVx = robotVy = robotWz = 0;
}

void stopRobot() {
  robotVx = robotVy = robotWz = 0;
  lateralHeadingCommandWz = 0.0f;
  stationHeadingCommandWz = 0.0f;
  lateralLineHoldCommandVx = 0.0f;
  positionErrorX = positionErrorY = 0;
  for (WheelControl &wheel : wheels) resetWheelPid(wheel);
}

void setRobotVelocity(float vx, float vy, float wz) {
  robotVx = vx;
  robotVy = vy;
  robotWz = wz;

  if (!armed || !configurationAllowsMotion()) {
    for (WheelControl &wheel : wheels) wheel.targetMmS = 0;
    return;
  }

  const float k = mecanumKmm();
  float targets[4] = {
    vx - vy - k * wz,
    vx + vy + k * wz,
    vx + vy - k * wz,
    vx - vy + k * wz
  };

  float scale = 1.0f;
  for (float target : targets)
    scale = max(scale, fabsf(target) / maxWheelSpeedMmS);

  // Slew all wheel targets by one common factor. Independent clipping here
  // would alter the requested Mecanum direction during acceleration.
  float desired[WHEEL_COUNT] = {};
  float maxDelta = 0.0f;
  float currentMagnitude = 0.0f;
  float desiredMagnitude = 0.0f;
  for (uint8_t i = 0; i < WHEEL_COUNT; ++i) {
    desired[i] = targets[i] / scale;
    maxDelta = max(maxDelta, fabsf(desired[i] - wheels[i].targetMmS));
    currentMagnitude = max(currentMagnitude, fabsf(wheels[i].targetMmS));
    desiredMagnitude = max(desiredMagnitude, fabsf(desired[i]));
  }

  const bool accelerating = desiredMagnitude > currentMagnitude;
  const float rate = accelerating ? WHEEL_TARGET_ACCEL_MM_S2
                                  : WHEEL_TARGET_DECEL_MM_S2;
  const float allowedDelta = rate * (CONTROL_PERIOD_US * 1.0e-6f);
  const float slewScale = maxDelta > allowedDelta ? allowedDelta / maxDelta
                                                   : 1.0f;
  for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    wheels[i].targetMmS += (desired[i] - wheels[i].targetMmS) * slewScale;
}

void updateEncoderMeasurements(float dt) {
  const float scale = mmPerCount();
  for (WheelControl &wheel : wheels) {
    wheel.count = wheel.encoder->read() * wheel.encoderSign;
    const long delta = wheel.count - wheel.previousCount;
    wheel.previousCount = wheel.count;
    wheel.deltaMm = delta * scale;
    const float speed = dt > 0 ? wheel.deltaMm / dt : 0;
    wheel.measuredMmS += WHEEL_SPEED_FILTER_ALPHA * (speed - wheel.measuredMmS);
    if (delta != 0) wheel.lastEncoderMotionMs = millis();
  }
}

void updateWheelSpeedPid(float dt) {
  for (uint8_t i = 0; i < WHEEL_COUNT; ++i) {
    WheelControl &wheel = wheels[i];
    if (!armed || faultFlags != FAULT_NONE || fabsf(wheel.targetMmS) < 0.5f) {
      resetWheelPid(wheel);
      continue;
    }

    if (fabsf(wheel.targetMmS) > 100.0f &&
        millis() - wheel.lastEncoderMotionMs > 600) {
      setFault(FAULT_ENCODER, "ENCODER_NO_MOTION");
      return;
    }

    const float sign = wheel.targetMmS > 0 ? 1.0f : -1.0f;
    const float feedForward = wheel.kff * wheel.targetMmS + sign * wheel.deadzone;
    const float correction = wheel.pid.updateError(
        wheel.targetMmS - wheel.measuredMmS, dt,
        -PWM_MAX - feedForward, PWM_MAX - feedForward);
    wheel.pwm = constrain(lroundf(feedForward + correction), -PWM_MAX, PWM_MAX);
    writeMotor(wheel, wheel.pwm);
  }
}

// ============================================================
// BNO085 heading controller
// ============================================================

bool enableBnoReport() {
  const sh2_SensorId_t report = BNO_USE_GAME_ROTATION_VECTOR
      ? SH2_GAME_ROTATION_VECTOR : SH2_ROTATION_VECTOR;
  return bno085.enableReport(report, BNO_REPORT_INTERVAL_US);
}

uint32_t lastBnoInitAttemptMs = 0;

bool initializeBno085() {
  lastBnoInitAttemptMs = millis();
  bnoTiltInitialized = false;
  if (!bno085.begin_I2C(BNO085_I2C_ADDRESS, &Wire1)) {
    bnoInitialized = false;
    bnoValid = false;
    return false;
  }
  bnoInitialized = enableBnoReport();
  bnoValid = false;
  lastBnoMs = millis();
  return bnoInitialized;
}

float quaternionYaw(float real, float i, float j, float k) {
  return wrap180(atan2f(2.0f * (real * k + i * j),
                        1.0f - 2.0f * (j * j + k * k)) * 180.0f / PI);
}

float quaternionRoll(float real, float i, float j, float k) {
  return wrap180(atan2f(2.0f * (real * i + j * k),
                        1.0f - 2.0f * (i * i + j * j)) * 180.0f / PI);
}

float quaternionPitch(float real, float i, float j, float k) {
  const float sine = constrain(2.0f * (real * j - k * i), -1.0f, 1.0f);
  return asinf(sine) * 180.0f / PI;
}

void updateBno085() {
  if (!bnoInitialized) {
    bnoValid = false;
    if (millis() - lastBnoInitAttemptMs >= BNO_RETRY_INTERVAL_MS)
      initializeBno085();
    return;
  }
  if (bno085.wasReset()) {
    bnoValid = false;
    bnoTiltInitialized = false;
    if (!enableBnoReport()) {
      bnoInitialized = false;
      return;
    }
    lastBnoMs = millis();
  }

  const sh2_SensorId_t wanted = BNO_USE_GAME_ROTATION_VECTOR
      ? SH2_GAME_ROTATION_VECTOR : SH2_ROTATION_VECTOR;
  while (bno085.getSensorEvent(&bnoEvent)) {
    if (bnoEvent.sensorId != wanted) continue;
    float qReal = 1.0f;
    float qI = 0.0f;
    float qJ = 0.0f;
    float qK = 0.0f;
    if (BNO_USE_GAME_ROTATION_VECTOR) {
      const sh2_RotationVector_t &q = bnoEvent.un.gameRotationVector;
      qReal = q.real;
      qI = q.i;
      qJ = q.j;
      qK = q.k;
    } else {
      const sh2_RotationVectorWAcc_t &q = bnoEvent.un.rotationVector;
      qReal = q.real;
      qI = q.i;
      qJ = q.j;
      qK = q.k;
    }
    const float rawYaw = wrap180(BNO_YAW_SIGN * quaternionYaw(qReal, qI, qJ, qK));
    const float rawRoll = quaternionRoll(qReal, qI, qJ, qK);
    const float rawPitch = quaternionPitch(qReal, qI, qJ, qK);
    if (!bnoTiltInitialized) {
      currentRollDeg = rawRoll;
      currentPitchDeg = rawPitch;
      bnoTiltInitialized = true;
    } else {
      currentRollDeg = wrap180(currentRollDeg + BNO_TILT_FILTER_ALPHA *
          shortestAngleError(rawRoll, currentRollDeg));
      currentPitchDeg += BNO_TILT_FILTER_ALPHA * (rawPitch - currentPitchDeg);
    }
    const float newYawDeg = wrap180(rawYaw - yawReferenceDeg);
    const uint32_t nowMs = millis();
    if (previousBnoYawMs != 0 && nowMs != previousBnoYawMs) {
      const float sampleDt = (nowMs - previousBnoYawMs) * 0.001f;
      const float measuredRate = shortestAngleError(newYawDeg, previousBnoYawDeg) / sampleDt;
      yawRateDegS += 0.25f * (measuredRate - yawRateDegS);
    } else {
      yawRateDegS = 0.0f;
    }
    previousBnoYawDeg = newYawDeg;
    previousBnoYawMs = nowMs;
    currentYawDeg = newYawDeg;
    lastBnoMs = nowMs;
    bnoValid = true;
  }
  if (millis() - lastBnoMs > BNO_TIMEOUT_MS) {
    bnoValid = false;
    if (millis() - lastBnoInitAttemptMs >= BNO_RETRY_INTERVAL_MS)
      initializeBno085();
  }
}

float headingControl(float dt, float limit = MAX_HEADING_WZ_RAD_S) {
  if (!bnoValid) return 0;
  const float error = shortestAngleError(targetYawDeg, currentYawDeg);
  // Stop completely close to the target. Without this deadband, sensor noise
  // keeps commanding a very small wheel speed indefinitely.
  if (fabsf(error) <= headingOutputDeadbandDeg) {
    headingPid.reset();
    return 0.0f;
  }
  float output = headingPid.updateError(error, dt, -limit, limit);
  // Outside the deadband, ensure the correction can overcome drivetrain
  // stiction. Never exceed the limit requested by the active motion mode.
  const float minimum = min(headingMinWzRadS, limit);
  if (output * error > 0.0f && fabsf(output) < minimum)
    output = copysignf(minimum, error);
  return constrain(output, -limit, limit);
}

float headingControlFine(float dt, float limit = FINE_HEADING_MAX_WZ_RAD_S) {
  if (!bnoValid) return 0.0f;
  const float error = shortestAngleError(targetYawDeg, currentYawDeg);
  if (fabsf(error) <= FINE_HEADING_DEADBAND_DEG) {
    fineHeadingPid.reset();
    return 0.0f;
  }
  float output = fineHeadingPid.updateError(error, dt, -limit, limit);
  const float minimum = min(FINE_HEADING_MIN_WZ_RAD_S, limit);
  if (output * error > 0.0f && fabsf(output) < minimum)
    output = copysignf(minimum, error);
  return constrain(output, -limit, limit);
}

float headingControlLateral(float dt) {
  if (!bnoValid || dt <= 0.0f) {
    lateralHeadingCommandWz = 0.0f;
    return 0.0f;
  }
  const float error = shortestAngleError(targetYawDeg, currentYawDeg);
  float requestedWz = 0.0f;
  if (fabsf(error) <= LATERAL_HEADING_DEADBAND_DEG) {
    fineHeadingPid.reset();
  } else {
    requestedWz = fineHeadingPid.updateError(
        error, dt, -LATERAL_HEADING_MAX_WZ_RAD_S,
        LATERAL_HEADING_MAX_WZ_RAD_S);
    if (requestedWz * error > 0.0f &&
        fabsf(requestedWz) < LATERAL_HEADING_MIN_WZ_RAD_S) {
      requestedWz = copysignf(LATERAL_HEADING_MIN_WZ_RAD_S, error);
    }
  }
  const float maxStep = LATERAL_HEADING_SLEW_RAD_S2 * dt;
  lateralHeadingCommandWz += constrain(
      requestedWz - lateralHeadingCommandWz, -maxStep, maxStep);
  if (requestedWz == 0.0f && fabsf(lateralHeadingCommandWz) <= maxStep)
    lateralHeadingCommandWz = 0.0f;
  return constrain(lateralHeadingCommandWz,
                   -LATERAL_HEADING_MAX_WZ_RAD_S,
                    LATERAL_HEADING_MAX_WZ_RAD_S);
}

float headingControlStationary(float dt) {
  if (!bnoValid || dt <= 0.0f) {
    stationHeadingCommandWz = 0.0f;
    return 0.0f;
  }
  const float error = shortestAngleError(targetYawDeg, currentYawDeg);
  float requestedWz = 0.0f;
  if (fabsf(error) <= STATION_HEADING_DEADBAND_DEG) {
    fineHeadingPid.reset();
  } else {
    requestedWz = fineHeadingPid.updateError(
        error, dt, -STATION_HEADING_MAX_WZ_RAD_S,
        STATION_HEADING_MAX_WZ_RAD_S);
    if (requestedWz * error > 0.0f &&
        fabsf(requestedWz) < STATION_HEADING_MIN_WZ_RAD_S) {
      requestedWz = copysignf(STATION_HEADING_MIN_WZ_RAD_S, error);
    }
  }
  const float maxStep = STATION_HEADING_SLEW_RAD_S2 * dt;
  stationHeadingCommandWz += constrain(
      requestedWz - stationHeadingCommandWz, -maxStep, maxStep);
  if (requestedWz == 0.0f && fabsf(stationHeadingCommandWz) <= maxStep)
    stationHeadingCommandWz = 0.0f;
  return constrain(stationHeadingCommandWz,
                   -STATION_HEADING_MAX_WZ_RAD_S,
                    STATION_HEADING_MAX_WZ_RAD_S);
}

// ============================================================
// MCP3008 and calibrated line arrays
// ============================================================

uint16_t readMCP3008Raw(uint8_t csPin, uint8_t channel) {
  digitalWrite(csPin, LOW);
  SPI.transfer(0x01);
  const uint8_t high = SPI.transfer((0x08 | (channel & 0x07)) << 4);
  const uint8_t low = SPI.transfer(0);
  digitalWrite(csPin, HIGH);
  return ((high & 0x03) << 8) | low;
}

uint16_t readMCP3008(uint8_t csPin, uint8_t channel) {
  SPI.beginTransaction(SPISettings(MCP_SPI_HZ, MSBFIRST, SPI_MODE0));
  const uint16_t result = readMCP3008Raw(csPin, channel);
  SPI.endTransaction();
  return result;
}

uint16_t normalizeLine(uint16_t raw, uint16_t minimum, uint16_t maximum) {
  if (maximum <= minimum + 5) return 0;
  long value = constrain(raw, minimum, maximum);
  value = (value - minimum) * 1000L / (maximum - minimum);
  if (!LINE_IS_HIGHER_THAN_FLOOR) value = 1000 - value;
  return static_cast<uint16_t>(constrain(value, 0L, 1000L));
}

void updateLineArrays() {
  SPI.beginTransaction(SPISettings(MCP_SPI_HZ, MSBFIRST, SPI_MODE0));
  for (uint8_t i = 0; i < 8; ++i)
    centerLine.raw[i] = readMCP3008Raw(MCP_CENTER_CS, i);
  // Convert crossed physical wiring to logical [left0..2, right0..2].
  for (uint8_t i = 0; i < 3; ++i) {
    stopLine.raw[i] = readMCP3008Raw(MCP_STOP_CS, STOP_LEFT_CHANNELS[i]);
    stopLine.raw[3 + i] = readMCP3008Raw(MCP_STOP_CS, STOP_RIGHT_CHANNELS[i]);
  }
  stopLine.holdRaw[0] = readMCP3008Raw(MCP_STOP_CS, LATERAL_HOLD_TAIL_CHANNEL);
  stopLine.holdRaw[1] = readMCP3008Raw(MCP_STOP_CS, LATERAL_HOLD_FRONT_CHANNEL);
  SPI.endTransaction();

  // H1/H2 must be filtered and normalized on every sensor update. Keeping
  // this in a motion helper left holdNormalized[] stuck at its initial zero
  // until that helper happened to run, so AUTO could never see the black line.
  for (uint8_t i = 0; i < 2; ++i) {
    stopLine.holdFiltered[i] += LINE_FILTER_ALPHA *
        (stopLine.holdRaw[i] - stopLine.holdFiltered[i]);
    stopLine.holdNormalized[i] = normalizeLine(
        lroundf(stopLine.holdFiltered[i]),
        LATERAL_HOLD_SENSOR_MIN[i], LATERAL_HOLD_SENSOR_MAX[i]);
  }

  static const int16_t weights[8] = {-3500,-2500,-1500,-500,500,1500,2500,3500};
  int64_t weighted = 0;
  uint32_t total = 0;
  centerLine.activeCount = 0;
  for (uint8_t i = 0; i < 8; ++i) {
    if (calibrationActive && !calibrationStopOnly) {
      centerCalMin[i] = min(centerCalMin[i], centerLine.raw[i]);
      centerCalMax[i] = max(centerCalMax[i], centerLine.raw[i]);
    }
    centerLine.filtered[i] += LINE_FILTER_ALPHA *
        (centerLine.raw[i] - centerLine.filtered[i]);
    centerLine.normalized[i] = normalizeLine(
        lroundf(centerLine.filtered[i]), centerCalMin[i], centerCalMax[i]);
    weighted += static_cast<int64_t>(centerLine.normalized[i]) * weights[i];
    total += centerLine.normalized[i];
    if (centerLine.normalized[i] >= LINE_ACTIVE_NORMALIZED) centerLine.activeCount++;
  }

  centerLine.valid = total >= 500;
  if (centerLine.valid) {
    centerLine.position = static_cast<float>(weighted) / total;
    centerLine.lastPosition = centerLine.position;
    centerLine.lastValidMs = millis();
    centerLine.state = LINE_VALID;
  } else if (millis() - centerLine.lastValidMs <= LINE_SHORT_LOST_MS) {
    centerLine.state = centerLine.lastPosition < 0 ? LINE_LOST_LEFT : LINE_LOST_RIGHT;
  } else if (centerLine.lastPosition < -100) {
    centerLine.state = LINE_LOST_LEFT;
  } else if (centerLine.lastPosition > 100) {
    centerLine.state = LINE_LOST_RIGHT;
  } else {
    centerLine.state = LINE_LOST_UNKNOWN;
  }

  static uint32_t crossStarted = 0;
  if (centerLine.activeCount >= CROSS_MIN_ACTIVE_SENSORS) {
    if (crossStarted == 0) crossStarted = millis();
    centerLine.cross = millis() - crossStarted >= CROSS_CONFIRM_MS;
    if (centerLine.cross) centerLine.state = CROSS_LINE;
  } else {
    crossStarted = 0;
    centerLine.cross = false;
  }

  for (uint8_t i = 0; i < 6; ++i) {
    if (calibrationActive && !calibrationCenterOnly) {
      stopCalMin[i] = min(stopCalMin[i], stopLine.raw[i]);
      stopCalMax[i] = max(stopCalMax[i], stopLine.raw[i]);
    }
    stopLine.filtered[i] += LINE_FILTER_ALPHA * (stopLine.raw[i] - stopLine.filtered[i]);
    stopLine.normalized[i] = normalizeLine(
        lroundf(stopLine.filtered[i]), stopCalMin[i], stopCalMax[i]);
  }

  auto calculateStopPosition = [](const uint16_t *values, float &position, bool &valid) {
    int32_t weightedSum = 0;
    uint32_t sum = 0;
    for (uint8_t i = 0; i < 3; ++i) {
      weightedSum += static_cast<int32_t>(values[i]) * STOP_WEIGHTS[i];
      sum += values[i];
    }
    valid = sum >= 400;
    if (valid) position = static_cast<float>(weightedSum) / sum;
  };
  calculateStopPosition(&stopLine.normalized[0], stopLine.leftPosition, stopLine.leftValid);
  calculateStopPosition(&stopLine.normalized[3], stopLine.rightPosition, stopLine.rightValid);
}

float updateLineFollowing(float dt, float &vx) {
  if (centerLine.valid) {
    // Convert physical array order to the robot convention (+Vy right).
    const float error = CENTER_LINE_CONTROL_SIGN * centerLine.position;
    const float vy = linePid.updateError(error, dt, -MAX_LINE_VY_MM_S, MAX_LINE_VY_MM_S);
    const float severity = min(1.0f, fabsf(centerLine.position) / 3500.0f);
    // Retain only 20% forward speed at the edge so lateral correction has
    // enough time and wheel-speed headroom to bring the line back to center.
    vx *= 1.0f - 0.80f * severity;
    return vy;
  }
  linePid.reset();
  const uint32_t lostMs = millis() - centerLine.lastValidMs;
  if (lostMs > LINE_FAULT_TIMEOUT_MS) {
    setFault(FAULT_LINE_LOST, "CENTER_LINE_LOST");
    return 0;
  }
  vx *= 0.10f;
  if (centerLine.state == LINE_LOST_LEFT)
    return CENTER_LINE_CONTROL_SIGN * -LINE_LOST_SEARCH_VY_MM_S;
  if (centerLine.state == LINE_LOST_RIGHT)
    return CENTER_LINE_CONTROL_SIGN * LINE_LOST_SEARCH_VY_MM_S;
  return 0;
}

void printLineCalibration() {
  Serial.print("CENTER_MIN=");
  for (uint8_t i=0;i<8;i++){if(i)Serial.print(',');Serial.print(centerCalMin[i]);}
  Serial.print("\nCENTER_MAX=");
  for (uint8_t i=0;i<8;i++){if(i)Serial.print(',');Serial.print(centerCalMax[i]);}
  Serial.print("\nSTOP_MIN=");
  for (uint8_t i=0;i<6;i++){if(i)Serial.print(',');Serial.print(stopCalMin[i]);}
  Serial.print("\nSTOP_MAX=");
  for (uint8_t i=0;i<6;i++){if(i)Serial.print(',');Serial.print(stopCalMax[i]);}
  Serial.println();
}

void printCenterCalibrationTelemetry() {
  Serial.print("CAL,CENTER,MIN");
  for(uint8_t i=0;i<8;i++){Serial.print(',');Serial.print(centerCalMin[i]);}
  Serial.println();
  Serial.print("CAL,CENTER,MAX");
  for(uint8_t i=0;i<8;i++){Serial.print(',');Serial.print(centerCalMax[i]);}
  Serial.println();
}

void printStopCalibrationTelemetry() {
  Serial.print("CAL,STOP,MIN");
  for(uint8_t i=0;i<6;i++){Serial.print(',');Serial.print(stopCalMin[i]);}
  Serial.println();
  Serial.print("CAL,STOP,MAX");
  for(uint8_t i=0;i<6;i++){Serial.print(',');Serial.print(stopCalMax[i]);}
  Serial.println();
}

// ============================================================
// Optical flow MAVLink v1 parser (OPTICAL_FLOW_RAD, ID 106)
// ============================================================

class MavlinkFlowParser {
 public:
  void feed(uint8_t byte) {
    switch (state) {
      case WAIT_STX:
        if (byte == 0xFE) { state = LENGTH; crc = 0xFFFF; index = 0; }
        break;
      case LENGTH:
        payloadLength = byte; crcAccumulate(byte); state = SEQ;
        if (payloadLength > sizeof(payload)) reset();
        break;
      case SEQ: crcAccumulate(byte); state = SYS; break;
      case SYS: crcAccumulate(byte); state = COMP; break;
      case COMP: crcAccumulate(byte); state = MSG; break;
      case MSG:
        messageId = byte; crcAccumulate(byte);
        state = payloadLength ? PAYLOAD : CRC_LOW;
        break;
      case PAYLOAD:
        payload[index++] = byte; crcAccumulate(byte);
        if (index >= payloadLength) state = CRC_LOW;
        break;
      case CRC_LOW: receivedCrc = byte; state = CRC_HIGH; break;
      case CRC_HIGH:
        receivedCrc |= static_cast<uint16_t>(byte) << 8;
        if (messageId == MAVLINK_OPTICAL_FLOW_RAD_ID) crcAccumulate(MAVLINK_OPTICAL_FLOW_RAD_CRC_EXTRA);
        if (receivedCrc == crc) decode();
        reset();
        break;
    }
  }

 private:
  enum State : uint8_t { WAIT_STX,LENGTH,SEQ,SYS,COMP,MSG,PAYLOAD,CRC_LOW,CRC_HIGH } state = WAIT_STX;
  uint8_t payload[255] = {}, payloadLength = 0, index = 0, messageId = 0;
  uint16_t crc = 0xFFFF, receivedCrc = 0;

  void reset() { state = WAIT_STX; index = 0; }
  void crcAccumulate(uint8_t data) {
    uint8_t tmp = data ^ static_cast<uint8_t>(crc & 0xFF);
    tmp ^= tmp << 4;
    crc = (crc >> 8) ^ (static_cast<uint16_t>(tmp) << 8) ^
          (static_cast<uint16_t>(tmp) << 3) ^ (tmp >> 4);
  }
  void decode() {
    if (messageId != MAVLINK_OPTICAL_FLOW_RAD_ID ||
        payloadLength != MAVLINK_OPTICAL_FLOW_RAD_LEN) return;
    const uint32_t integrationUs = readU32LE(payload + 8);
    const float integratedX = readFloatLE(payload + 12);
    const float integratedY = readFloatLE(payload + 16);
    const float distanceM = readFloatLE(payload + 36);
    const uint8_t quality = payload[43];
    if (integrationUs == 0 || !isfinite(integratedX) || !isfinite(integratedY) ||
        !isfinite(distanceM) || distanceM <= 0 || quality < FLOW_MIN_QUALITY) return;

    const float sensorDelta[2] = {
      integratedX * distanceM * 1000.0f,
      integratedY * distanceM * 1000.0f
    };
    flow.deltaX = sensorDelta[FLOW_X_FROM_SENSOR_AXIS] * FLOW_X_SIGN * FLOW_SCALE_X_MM;
    flow.deltaY = sensorDelta[FLOW_Y_FROM_SENSOR_AXIS] * FLOW_Y_SIGN * FLOW_SCALE_Y_MM;
    const float dt = integrationUs * 1.0e-6f;
    flow.velocityX = flow.deltaX / dt;
    flow.velocityY = flow.deltaY / dt;
    flow.quality = quality;
    flow.groundDistance = distanceM * 1000.0f;
    flow.valid = true;
    flow.fresh = true;
    flow.lastUpdateMs = millis();
  }
};

MavlinkFlowParser flowParser;

void updateOpticalFlow() {
  if (!OPTICAL_FLOW_ENABLED) {
    flow.valid = false;
    flow.fresh = false;
    lateralSlipMmS = 0;
    return;
  }
  while (FLOW_UART.available()) flowParser.feed(static_cast<uint8_t>(FLOW_UART.read()));
  if (millis() - flow.lastUpdateMs > FLOW_TIMEOUT_MS) {
    flow.valid = false;
    flow.fresh = false;
  }
  lateralSlipMmS = flow.valid ? flow.velocityY - robotVy : 0;
}

// ============================================================
// VL53L3CX non-blocking, multi-target filtered distance
// ============================================================

uint16_t tofSamples[TOF_MEDIAN_SAMPLES] = {};
uint8_t tofSampleIndex = 0, tofSampleCount = 0;
uint32_t lastTofRestartMs = 0;

float medianTof() {
  uint16_t copy[TOF_MEDIAN_SAMPLES];
  for (uint8_t i=0;i<tofSampleCount;i++) copy[i]=tofSamples[i];
  for (uint8_t i=1;i<tofSampleCount;i++) {
    uint16_t key=copy[i]; int8_t j=i-1;
    while(j>=0 && copy[j]>key){copy[j+1]=copy[j];j--;}
    copy[j+1]=key;
  }
  return tofSampleCount ? copy[tofSampleCount/2] : 0;
}

void updateTof() {
  if (!TOF_ENABLED || !tofInitialized) { tofValid=false; return; }
  // BNO085 and VL53L3CX share Wire1; restore the ToF bus clock after IMU traffic.
  Wire1.setClock(SENSOR_I2C_HZ);
  if (tof.dataIsReady()) {
    MeasurmentResult measurement{};
    const VL53LX_Error readStatus = tof.getMeasurmentData(&measurement);
    uint16_t value = UINT16_MAX;
    if (readStatus == VL53LX_ERROR_NONE) {
      // Use the nearest valid return. This is the wall-facing target needed
      // by auto alignment when VL53L3CX reports more than one object.
      for (uint8_t i=0;i<measurement.numObjs;i++) {
        const int16_t candidate=measurement.rangeData[i].Range;
        if(candidate>=TOF_MIN_VALID_MM&&candidate<=TOF_MAX_VALID_MM&&
           static_cast<uint16_t>(candidate)<value)
          value=static_cast<uint16_t>(candidate);
      }
    }
    const bool plausible = tofAcceptanceEnabled && value != UINT16_MAX &&
      (tofSampleCount == 0 ||
       abs(static_cast<int32_t>(value) - static_cast<int32_t>(tofDistanceMm)) <=
         TOF_MAX_SAMPLE_JUMP_MM);
    if (plausible) {
      tofSamples[tofSampleIndex] = value;
      tofSampleIndex = (tofSampleIndex + 1) % TOF_MEDIAN_SAMPLES;
      tofSampleCount = min<uint8_t>(tofSampleCount + 1, TOF_MEDIAN_SAMPLES);
      tofDistanceMm = medianTof();
      lastTofMs = millis();
      tofValid = true;
    }
    // A delayed restart must not invalidate the measurement just accepted.
    // The timeout below is the single authority for declaring ToF stale.
    tof.startNextMeasurement();
  }
  if (millis() - lastTofMs > TOF_TIMEOUT_MS) tofValid = false;
}

// ============================================================
// Odometry: Mecanum encoder + BNO085 yaw + optional flow fusion
// ============================================================

void resetPose(float x=0, float y=0, float yaw=0) {
  pose.x_mm=x; pose.y_mm=y; pose.yaw_deg=yaw;
  for (WheelControl &wheel : wheels) {
    wheel.previousCount = wheel.encoder->read() * wheel.encoderSign;
    wheel.deltaMm = 0;
  }
  flow.fresh = false;
}

void updateOdometry(float dt) {
  const float fl=wheels[FL].deltaMm, fr=wheels[FR].deltaMm;
  const float rl=wheels[RL].deltaMm, rr=wheels[RR].deltaMm;
  float dxBody=(fl+fr+rl+rr)*0.25f;
  float dyBody=(-fl+fr+rl-rr)*0.25f;
  const float k=mecanumKmm();
  const float encoderDeltaYawRad=k>0.0f?(-fl+fr-rl+rr)/(4.0f*k):0.0f;

  if (flow.valid && dt > 0.0f) {
    const float weightSum=ODOM_ENCODER_WEIGHT+ODOM_FLOW_WEIGHT;
    const float encoderWeight=weightSum>0.0f?ODOM_ENCODER_WEIGHT/weightSum:1.0f;
    const float flowWeight=weightSum>0.0f?ODOM_FLOW_WEIGHT/weightSum:0.0f;
    const float flowDx=flow.velocityX*dt;
    const float flowDy=flow.velocityY*dt;
    dxBody=encoderWeight*dxBody+flowWeight*flowDx;
    dyBody=encoderWeight*dyBody+flowWeight*flowDy;
  }

  pose.yaw_deg=bnoValid?currentYawDeg:
      wrap180(pose.yaw_deg+encoderDeltaYawRad*180.0f/PI);
  const float yawRad=pose.yaw_deg*PI/180.0f;
  pose.x_mm += cosf(yawRad)*dxBody - sinf(yawRad)*dyBody;
  pose.y_mm += sinf(yawRad)*dxBody + cosf(yawRad)*dyBody;
}

float lateralXHoldVelocity() {
  if (!LATERAL_LONGITUDINAL_HOLD_ENABLED) {
    positionErrorX = 0.0f;
    return 0.0f;
  }
  if (TOF_ENABLED) {
    if (!tofValid) return 0.0f;
    if (!lateralHoldTofCaptured) {
      lateralHoldTofMm = tofDistanceMm;
      lateralHoldTofCaptured = true;
    }
    const float error = lateralHoldTofMm - tofDistanceMm;
    positionErrorX = error;
    if (!isfinite(error) || fabsf(error) <= LATERAL_TOF_HOLD_DEADBAND_MM) return 0.0f;
    return TOF_CONTROL_SIGN * constrain(LATERAL_TOF_HOLD_KP * error,
                                        -LATERAL_TOF_HOLD_MAX_SPEED_MM_S,
                                         LATERAL_TOF_HOLD_MAX_SPEED_MM_S);
  }
  const float error = lateralHoldXMm - pose.x_mm;
  positionErrorX = error;
  if (!isfinite(error) || fabsf(error) <= LATERAL_X_HOLD_DEADBAND_MM) return 0.0f;
  return constrain(LATERAL_X_HOLD_KP * error,
                   -LATERAL_X_HOLD_MAX_SPEED_MM_S,
                    LATERAL_X_HOLD_MAX_SPEED_MM_S);
}

void resetLateralLineHold() {
  lateralLineHoldCommandVx = 0.0f;
  lateralLineHoldLastSign = 0;
  lateralLineHoldLastSeenMs = 0;
  lateralLineHoldStartedMs = millis();
}

float lateralLineHoldVelocity(float dt) {
  const bool tailOnLine =
      stopLine.holdNormalized[0] >= LATERAL_HOLD_LINE_THRESHOLD;
  const bool frontOnLine =
      stopLine.holdNormalized[1] >= LATERAL_HOLD_LINE_THRESHOLD;
  float requestedVx = 0.0f;

  if (tailOnLine && frontOnLine) {
    lateralLineHoldLastSeenMs = millis();
    lateralLineHoldLastSign = 0;
  } else if (!tailOnLine && frontOnLine) {
    // Tail left the black line: move forward to pull the tail back onto it.
    lateralLineHoldLastSeenMs = millis();
    lateralLineHoldLastSign = LATERAL_HOLD_CONTROL_SIGN;
    requestedVx = lateralLineHoldLastSign * LATERAL_HOLD_CORRECTION_SPEED_MM_S;
  } else if (tailOnLine && !frontOnLine) {
    // Front left the black line: move backward to pull the front back onto it.
    lateralLineHoldLastSeenMs = millis();
    lateralLineHoldLastSign = -LATERAL_HOLD_CONTROL_SIGN;
    requestedVx = lateralLineHoldLastSign * LATERAL_HOLD_CORRECTION_SPEED_MM_S;
  } else if (lateralLineHoldLastSeenMs != 0 &&
             millis() - lateralLineHoldLastSeenMs <= LATERAL_HOLD_LOST_TIMEOUT_MS) {
    requestedVx = lateralLineHoldLastSign * LATERAL_HOLD_SEARCH_SPEED_MM_S;
  } else if (millis() - lateralLineHoldStartedMs >
             LATERAL_HOLD_ACQUIRE_TIMEOUT_MS) {
    setFault(FAULT_LINE_LOST, "LATERAL_H1_H2_LOST");
    return 0.0f;
  }

  const float maxStep = LATERAL_HOLD_SLEW_MM_S2 * max(dt, 0.0f);
  lateralLineHoldCommandVx += constrain(
      requestedVx - lateralLineHoldCommandVx, -maxStep, maxStep);
  if (requestedVx == 0.0f && fabsf(lateralLineHoldCommandVx) <= maxStep)
    lateralLineHoldCommandVx = 0.0f;
  return lateralLineHoldCommandVx;
}

bool firstLateralGuidanceReady() {
  if (state != DI_SANG_TRAI_A || firstLateralGuidanceArmed) return true;
  if (fabsf(pose.y_mm - firstLateralStartedYmm) < FIRST_LATERAL_BLIND_DISTANCE_MM)
    return false;
  firstLateralGuidanceArmed = true;
  resetLateralLineHold();
  Serial.println("ACK,AUTO,H1_H2_ENABLED_AFTER_400MM");
  return true;
}

// ============================================================
// Position, stop-line and ToF controllers
// ============================================================

struct RelativeMove {
  bool active=false;
  bool absolute=false;
  RobotPose start;
  float dx=0, dy=0, maxSpeed=0;
  float targetX=0, targetY=0, targetYaw=0;
  uint32_t stableSince=0;
} moveCommand;

void beginRelativeMove(float dx, float dy, float maxSpeed=MOVE_SPEED_MM_S) {
  RobotPose startPose = pose;
  // updateOdometry() uses the BNO085 yaw as its world-frame heading. Capture
  // that same heading here even immediately after X/Y pose has been reset.
  if (bnoValid) startPose.yaw_deg = currentYawDeg;
  moveCommand.active=true;moveCommand.absolute=false;moveCommand.start=startPose;
  moveCommand.dx=dx;moveCommand.dy=dy;moveCommand.maxSpeed=maxSpeed;
  moveCommand.stableSince=0;
  positionErrorX=dx;positionErrorY=dy;
  positionXPid.reset(); positionYPid.reset(); headingPid.reset();
}

void beginAbsoluteMove(float targetX,float targetY,float targetYaw,float maxSpeed) {
  moveCommand.active=true;moveCommand.absolute=true;moveCommand.start=pose;
  moveCommand.targetX=targetX;moveCommand.targetY=targetY;
  moveCommand.targetYaw=wrap180(targetYaw);moveCommand.maxSpeed=maxSpeed;
  moveCommand.stableSince=0;targetYawDeg=moveCommand.targetYaw;
  positionErrorX=targetX-pose.x_mm;positionErrorY=targetY-pose.y_mm;
  positionXPid.reset();positionYPid.reset();headingPid.reset();
}

bool updateAbsoluteMove(float dt) {
  if(!moveCommand.active||!moveCommand.absolute)return true;
  const float exWorld=moveCommand.targetX-pose.x_mm;
  const float eyWorld=moveCommand.targetY-pose.y_mm;
  const float distance=hypotf(exWorld,eyWorld);
  const float yawError=shortestAngleError(moveCommand.targetYaw,currentYawDeg);
  positionErrorX=exWorld;positionErrorY=eyWorld;

  float vxWorld=0.0f,vyWorld=0.0f;
  if(distance>=18.0f){
    vxWorld=positionXPid.updateError(exWorld,dt,-moveCommand.maxSpeed,moveCommand.maxSpeed);
    vyWorld=positionYPid.updateError(eyWorld,dt,-moveCommand.maxSpeed,moveCommand.maxSpeed);
    const float magnitude=hypotf(vxWorld,vyWorld);
    if(magnitude>moveCommand.maxSpeed&&magnitude>0.0f){
      const float scale=moveCommand.maxSpeed/magnitude;vxWorld*=scale;vyWorld*=scale;
    }
  }

  // Convert the world-frame position command into the robot body frame used
  // by setRobotVelocity(). This allows X/Y and yaw to converge together.
  const float inverseYaw=-pose.yaw_deg*PI/180.0f;
  const float vxBody=cosf(inverseYaw)*vxWorld-sinf(inverseYaw)*vyWorld;
  const float vyBody=sinf(inverseYaw)*vxWorld+cosf(inverseYaw)*vyWorld;
  const float headingLimit=min(maxAutoHeadLockWzRadS,0.60f);
  setRobotVelocity(vxBody,vyBody,headingControl(dt,headingLimit));

  if(distance<18.0f&&fabsf(yawError)<0.70f){
    if(moveCommand.stableSince==0)moveCommand.stableSince=millis();
    if(millis()-moveCommand.stableSince>=200){
      moveCommand.active=false;stopRobot();return true;
    }
  }else moveCommand.stableSince=0;
  return false;
}

bool updateRelativeMove(float dt) {
  if (!moveCommand.active) return true;
  const float worldDx=pose.x_mm-moveCommand.start.x_mm;
  const float worldDy=pose.y_mm-moveCommand.start.y_mm;
  const float yaw=-moveCommand.start.yaw_deg*PI/180.0f;
  const float localX=cosf(yaw)*worldDx-sinf(yaw)*worldDy;
  const float localY=sinf(yaw)*worldDx+cosf(yaw)*worldDy;
  const float ex=moveCommand.dx-localX, ey=moveCommand.dy-localY;
  positionErrorX=ex;positionErrorY=ey;
  if (fabsf(ex)<18 && fabsf(ey)<18) {
    moveCommand.active=false; stopRobot(); return true;
  }
  float vx=positionXPid.updateError(ex,dt,-moveCommand.maxSpeed,moveCommand.maxSpeed);
  float vy=positionYPid.updateError(ey,dt,-moveCommand.maxSpeed,moveCommand.maxSpeed);
  const float translationMagnitude=hypotf(vx,vy);
  if(translationMagnitude>moveCommand.maxSpeed && translationMagnitude>0.0f){
    const float scale=moveCommand.maxSpeed/translationMagnitude;
    vx*=scale;vy*=scale;
  }
  setRobotVelocity(vx,vy,headingControlFine(dt));
  return false;
}

bool updateRelativeLateralMoveWithHold(float dt) {
  if (!moveCommand.active) return true;
  const float worldDx = pose.x_mm - moveCommand.start.x_mm;
  const float worldDy = pose.y_mm - moveCommand.start.y_mm;
  const float yaw = -moveCommand.start.yaw_deg * PI / 180.0f;
  const float localY = sinf(yaw) * worldDx + cosf(yaw) * worldDy;
  const float error = moveCommand.dy - localY;
  positionErrorY = error;
  if (fabsf(error) < 18.0f) {
    moveCommand.active = false;
    stopRobot();
    return true;
  }

  const float vy = positionYPid.updateError(
      error, dt, -moveCommand.maxSpeed, moveCommand.maxSpeed);
  const float vx = lateralLineHoldVelocity(dt);
  setRobotVelocity(vx, vy, headingControlLateral(dt));
  return false;
}

bool updateRelativeMoveWithLine(float dt) {
  if (!moveCommand.active) return true;
  const float worldDx=pose.x_mm-moveCommand.start.x_mm;
  const float worldDy=pose.y_mm-moveCommand.start.y_mm;
  const float yaw=-moveCommand.start.yaw_deg*PI/180.0f;
  const float localX=cosf(yaw)*worldDx-sinf(yaw)*worldDy;
  const float localY=sinf(yaw)*worldDx+cosf(yaw)*worldDy;
  const float ex=moveCommand.dx-localX;
  if(fabsf(ex)<18.0f){
    moveCommand.active=false;stopRobot();return true;
  }
  float vx=positionXPid.updateError(ex,dt,-moveCommand.maxSpeed,moveCommand.maxSpeed);
  const float vy=updateLineFollowing(dt,vx);
  setRobotVelocity(vx,vy,headingControl(dt));
  (void)localY;
  return false;
}

bool updateBridgeLineUntilEndMarker(float dt) {
  if (!bnoValid) {
    setFault(FAULT_IMU_TIMEOUT, "BNO085_REQUIRED_ON_BRIDGE");
    stopRobot();
    return false;
  }

  const float relativeRoll = fabsf(
      shortestAngleError(currentRollDeg, bridgeLevelRollDeg));
  const float relativePitch = fabsf(currentPitchDeg - bridgeLevelPitchDeg);
  bridgeRelativeTiltDeg = max(relativeRoll, relativePitch);
  const uint32_t nowMs = millis();

  if (bridgeSlopePhase == BRIDGE_PHASE_APPROACH) {
    if (bridgeRelativeTiltDeg >= BRIDGE_INCLINE_ENTER_DEG) {
      if (!bridgeInclineSinceMs) bridgeInclineSinceMs = nowMs;
      if (nowMs - bridgeInclineSinceMs >= BRIDGE_INCLINE_CONFIRM_MS) {
        bridgeSlopePhase = BRIDGE_PHASE_ASCENDING;
        bridgeSlopeAxis = relativeRoll >= relativePitch ? 1 : 2;
        bridgeAscentDetectedMs = nowMs;
        bridgeInclineSinceMs = 0;
        bridgeLevelSinceMs = 0;
        Serial.println("ACK,AUTO_BRIDGE_ASCENT_DETECTED_FULL_SPEED");
      }
    } else {
      bridgeInclineSinceMs = 0;
    }
  } else if (bridgeSlopePhase == BRIDGE_PHASE_ASCENDING) {
    const float slopeAxisTilt = bridgeSlopeAxis == 1 ? relativeRoll : relativePitch;
    const bool ascentOldEnough =
        nowMs - bridgeAscentDetectedMs >= BRIDGE_MIN_ASCENT_BEFORE_CREST_MS;
    if (ascentOldEnough && slopeAxisTilt <= BRIDGE_CREST_LEVEL_DEG) {
      if (!bridgeLevelSinceMs) bridgeLevelSinceMs = nowMs;
      if (nowMs - bridgeLevelSinceMs >= BRIDGE_CREST_CONFIRM_MS) {
        bridgeSlopePhase = BRIDGE_PHASE_CREST;
        bridgeLevelSinceMs = 0;
        bridgeDescentSinceMs = 0;
        Serial.println("ACK,AUTO_BRIDGE_CREST_DETECTED_REDUCE_SPEED");
      }
    } else {
      bridgeLevelSinceMs = 0;
    }
  } else if (bridgeSlopePhase == BRIDGE_PHASE_CREST) {
    const float slopeAxisTilt = bridgeSlopeAxis == 1 ? relativeRoll : relativePitch;
    if (slopeAxisTilt >= BRIDGE_DESCENT_ENTER_DEG) {
      if (!bridgeDescentSinceMs) bridgeDescentSinceMs = nowMs;
      if (nowMs - bridgeDescentSinceMs >= BRIDGE_DESCENT_CONFIRM_MS) {
        bridgeSlopePhase = BRIDGE_PHASE_DESCENDING;
        bridgeDescentSinceMs = 0;
        bridgeDescentLevelSinceMs = 0;
        bridgeEndMarkerArmed = false;
        bridgeEndMarkerSinceMs = 0;
        bridgeSideMarkerSinceMs = 0;
        Serial.println("ACK,AUTO_BRIDGE_DESCENT_DETECTED_WAIT_LEVEL_500MS");
      }
    } else {
      bridgeDescentSinceMs = 0;
    }
  } else if (bridgeSlopePhase == BRIDGE_PHASE_DESCENDING &&
             !bridgeEndMarkerArmed) {
    const float slopeAxisTilt = bridgeSlopeAxis == 1 ? relativeRoll : relativePitch;
    if (slopeAxisTilt <= BRIDGE_DESCENT_EXIT_LEVEL_DEG) {
      if (!bridgeDescentLevelSinceMs) bridgeDescentLevelSinceMs = nowMs;
      if (nowMs - bridgeDescentLevelSinceMs >=
          BRIDGE_DESCENT_EXIT_LEVEL_CONFIRM_MS) {
        bridgeEndMarkerArmed = true;
        bridgeEndMarkerSinceMs = 0;
        bridgeSideMarkerSinceMs = 0;
        Serial.println("ACK,AUTO_BRIDGE_LEVEL_STABLE_500MS_MARKERS_ARMED");
      }
    } else {
      // Any renewed slope or chassis bounce restarts the full 500 ms window.
      bridgeDescentLevelSinceMs = 0;
    }
  }

  float vx = BRIDGE_APPROACH_SPEED_MM_S;
  if (bridgeSlopePhase == BRIDGE_PHASE_ASCENDING)
    vx = BRIDGE_FORWARD_SPEED_MM_S;
  else if (bridgeSlopePhase == BRIDGE_PHASE_CREST)
    vx = BRIDGE_CREST_SPEED_MM_S;
  else if (bridgeSlopePhase == BRIDGE_PHASE_DESCENDING)
    vx = bridgeEndMarkerSource == BRIDGE_MARKER_NONE
             ? BRIDGE_DESCENT_SPEED_MM_S
             : BRIDGE_POST_MARKER_SPEED_MM_S;

  const float vy = updateLineFollowing(dt, vx);
  if (faultFlags != FAULT_NONE) {
    stopRobot();
    return false;
  }
  setRobotVelocity(vx, vy, headingControl(dt, MAX_BRIDGE_HEADING_WZ_RAD_S));

  if (bridgeEndMarkerSource != BRIDGE_MARKER_NONE) {
    const float worldDx = pose.x_mm - bridgeEndMarkerPose.x_mm;
    const float worldDy = pose.y_mm - bridgeEndMarkerPose.y_mm;
    const float yaw = -bridgeEndMarkerPose.yaw_deg * PI / 180.0f;
    const float localX = cosf(yaw) * worldDx - sinf(yaw) * worldDy;
    if (localX >= bridgeEndMarkerAdvanceTargetMm) {
      stopRobot();
      Serial.print("ACK,AUTO_BRIDGE_POST_MARKER_ENCODER_DONE,");
      Serial.print(bridgeEndMarkerSource == BRIDGE_MARKER_CENTER_8
                       ? "CENTER_8," : "SIDE_CLUSTERS,");
      Serial.println(localX, 1);
      return true;
    }
    return false;
  }

  // Only the real descent can arm the finish marker. The eight-eye center
  // array detects a wide transverse line with >=6 active sensors. The side
  // fallback requires both three-eye clusters, each with its middle eye or
  // at least two of three eyes on black, which rejects a single noisy sensor.
  if (bridgeSlopePhase == BRIDGE_PHASE_DESCENDING &&
      bridgeEndMarkerArmed) {
    auto groupSeesMarker = [](uint8_t offset) {
      uint8_t active = 0;
      for (uint8_t i = 0; i < 3; ++i)
        if (stopLine.normalized[offset + i] >= LINE_ACTIVE_NORMALIZED) ++active;
      const bool middle =
          stopLine.normalized[offset + 1] >= LINE_ACTIVE_NORMALIZED;
      return middle || active >= 2;
    };
    uint8_t centerMarkerActive = 0;
    for (uint8_t i = 0; i < 8; ++i) {
      if (centerLine.normalized[i] >=
          BRIDGE_CENTER_MARKER_ACTIVE_NORMALIZED) {
        ++centerMarkerActive;
      }
    }
    const bool centerMarker =
        centerMarkerActive >= BRIDGE_CENTER_MARKER_MIN_ACTIVE_SENSORS;
    const bool sideMarker = groupSeesMarker(0) && groupSeesMarker(3);

    if (centerMarker) {
      if (!bridgeEndMarkerSinceMs) bridgeEndMarkerSinceMs = nowMs;
    } else {
      bridgeEndMarkerSinceMs = 0;
    }
    if (sideMarker) {
      if (!bridgeSideMarkerSinceMs) bridgeSideMarkerSinceMs = nowMs;
    } else {
      bridgeSideMarkerSinceMs = 0;
    }

    if (bridgeEndMarkerSinceMs &&
        nowMs - bridgeEndMarkerSinceMs >= BRIDGE_END_MARKER_CONFIRM_MS) {
      bridgeEndMarkerSource = BRIDGE_MARKER_CENTER_8;
      bridgeEndMarkerAdvanceTargetMm = BRIDGE_CENTER_MARKER_ADVANCE_MM;
      bridgeEndMarkerPose = pose;
#if ROBOCON_AFTER_BRIDGE_TEST_BUILD
      buzzer.beep(1);
#endif
      Serial.println("ACK,AUTO_BRIDGE_MARKER_CENTER_8_ADVANCE_400MM");
    } else if (bridgeSideMarkerSinceMs &&
               nowMs - bridgeSideMarkerSinceMs >= BRIDGE_SIDE_MARKER_CONFIRM_MS) {
      bridgeEndMarkerSource = BRIDGE_MARKER_SIDE_CLUSTERS;
      bridgeEndMarkerAdvanceTargetMm = BRIDGE_SIDE_MARKER_ADVANCE_MM;
      bridgeEndMarkerPose = pose;
#if ROBOCON_AFTER_BRIDGE_TEST_BUILD
      buzzer.beep(2);
#endif
      Serial.println("ACK,AUTO_BRIDGE_MARKER_SIDE_CLUSTERS_ADVANCE_300MM");
    }
  } else {
    bridgeEndMarkerSinceMs = 0;
    bridgeSideMarkerSinceMs = 0;
  }
  return false;
}

bool updateYawAlignment(float dt, float tolerance=1.0f) {
  if (!bnoValid) { setFault(FAULT_IMU_TIMEOUT,"IMU_REQUIRED"); return false; }
  const float error=shortestAngleError(targetYawDeg,currentYawDeg);
  if (fabsf(error)<tolerance) { stopRobot(); return true; }
  setRobotVelocity(0,0,headingControl(dt,MAX_LINE_ALIGN_WZ_RAD_S));
  return false;
}

bool updateTofAlignment(float dt) {
  static uint32_t stableSince=0;
  static uint32_t invalidSince=0;
  if (!TOF_ENABLED) {
    stableSince=0;invalidSince=0;
    stopRobot();
    return true;
  }
  if (!tofValid) {
    stableSince=0;stopRobot();
    if(!invalidSince)invalidSince=millis();
    if(millis()-invalidSince>TOF_ALIGNMENT_RECOVERY_MS)
      setFault(FAULT_TOF_TIMEOUT,"TOF_REQUIRED");
    return false;
  }
  invalidSince=0;
  const float error=TOF_TARGET_MM-tofDistanceMm;
  if (fabsf(error)<=TOF_TOLERANCE_MM) {
    stopRobot();
    if(!stableSince)stableSince=millis();
    if(millis()-stableSince>=TOF_CONFIRM_MS){stableSince=0;return true;}
    return false;
  }
  stableSince=0;
  const float speed=min(TOF_ALIGN_MAX_SPEED_MM_S,
                        fabsf(error)>60?TOF_COARSE_SPEED_MM_S:TOF_FINE_SPEED_MM_S);
  const float vx=TOF_CONTROL_SIGN*tofDistancePid.updateError(error,dt,-speed,speed);
  setRobotVelocity(vx,0,headingControlStationary(dt));
  return false;
}

enum StopAlignStage:uint8_t { STOP_SEARCH_FAST,STOP_SEARCH_SLOW,STOP_FINE,STOP_ALIGNED };
StopAlignStage stopAlignStage=STOP_SEARCH_FAST;
float stopSearchVy=0;
uint8_t requestedCrossings=1,observedCrossings=0;
bool stopCountRightGroup=false;
bool stopLineLatched=false;
uint32_t stopStableSince=0,stopLinePresenceSince=0,stopLineAbsenceSince=0;

uint8_t stopGroupActiveCount(uint8_t offset);

void beginStopAlignment(float searchVy,uint8_t crossings,bool rightGroup) {
  stopSearchVy=searchVy;requestedCrossings=crossings;observedCrossings=0;
  stopCountRightGroup=rightGroup;
  stopLineLatched=false;stopStableSince=0;stopLinePresenceSince=0;
  stopLineAbsenceSince=0;stopAlignStage=STOP_SEARCH_FAST;
  stopForwardPid.reset();stopYawPid.reset();headingPid.reset();
}

bool updateStopAlignment(float dt) {
  const uint8_t countOffset=stopCountRightGroup?3:0;
  const bool selectedGroupOnLine=stopGroupActiveCount(countOffset)>=2;
  if(selectedGroupOnLine){
    stopLineAbsenceSince=0;
    if(!stopLineLatched){
      if(!stopLinePresenceSince)stopLinePresenceSince=millis();
      if(millis()-stopLinePresenceSince>=STOP_CROSS_CONFIRM_MS){
        observedCrossings++;
        stopLineLatched=true;
        stopLinePresenceSince=0;
      }
    }
  }else{
    stopLinePresenceSince=0;
    if(stopLineLatched){
      if(!stopLineAbsenceSince)stopLineAbsenceSince=millis();
      if(millis()-stopLineAbsenceSince>=STOP_CROSS_RELEASE_MS){
        stopLineLatched=false;
        stopLineAbsenceSince=0;
      }
    }
  }
  if(stopAlignStage==STOP_SEARCH_FAST){
    setRobotVelocity(0,stopSearchVy,headingControl(dt));
    if(observedCrossings>=requestedCrossings)stopAlignStage=STOP_SEARCH_SLOW;
  } else if(stopAlignStage==STOP_SEARCH_SLOW){
    setRobotVelocity(0,stopSearchVy*0.25f,headingControl(dt,MAX_LINE_ALIGN_WZ_RAD_S));
    if(stopLine.leftValid&&stopLine.rightValid)stopAlignStage=STOP_FINE;
  } else if(stopAlignStage==STOP_FINE){
    if(!stopLine.leftValid||!stopLine.rightValid){
      setRobotVelocity(0,stopSearchVy*0.12f,headingControl(dt,MAX_LINE_ALIGN_WZ_RAD_S));
      stopStableSince=0;return false;
    }
    const float forwardError=0.5f*(stopLine.leftPosition+stopLine.rightPosition);
    const float yawLineError=stopLine.leftPosition-stopLine.rightPosition;
    const float vx=stopForwardPid.updateError(-forwardError,dt,-90,90);
    const float lineWz=stopYawPid.updateError(-yawLineError,dt,
                                              -MAX_LINE_ALIGN_WZ_RAD_S,
                                               MAX_LINE_ALIGN_WZ_RAD_S);
    const float bnoAssist=constrain(headingControl(dt,0.18f),-0.18f,0.18f);
    setRobotVelocity(vx,0,constrain(lineWz+bnoAssist,
                                    -MAX_LINE_ALIGN_WZ_RAD_S,
                                     MAX_LINE_ALIGN_WZ_RAD_S));
    const bool centered=fabsf(stopLine.leftPosition)<STOP_POSITION_TOLERANCE&&
                        fabsf(stopLine.rightPosition)<STOP_POSITION_TOLERANCE&&
                        fabsf(yawLineError)<STOP_SIDE_DIFFERENCE_TOLERANCE;
    if(centered){if(!stopStableSince)stopStableSince=millis();}
    else stopStableSince=0;
    if(stopStableSince&&millis()-stopStableSince>=STOP_ALIGNMENT_CONFIRM_MS){
      stopAlignStage=STOP_ALIGNED;stopRobot();resetPose(0,0,currentYawDeg);return true;
    }
  }
  return stopAlignStage==STOP_ALIGNED;
}

// Debounced one-shot counter used by each autonomous line group. A new count
// is permitted only after the group has left the previous line.
struct AutoLineCounter {
  uint8_t count=0;
  bool latched=false;
  bool enabledAfterClear=false;
  uint32_t activeSince=0;
  uint32_t clearSince=0;

  void reset(){count=0;latched=false;enabledAfterClear=false;activeSince=0;clearSince=0;}
  void update(bool onLine,uint32_t confirmMs=LINE_DEBOUNCE_MS,
              uint32_t releaseMs=LINE_CLEAR_DEBOUNCE_MS){
    // Do not count the line already underneath the robot when a route segment
    // starts (notably the red starting line). Arm only after a confirmed clear
    // floor interval, then count subsequent complete lines normally.
    if(!enabledAfterClear){
      activeSince=0;
      if(onLine){clearSince=0;return;}
      if(!clearSince)clearSince=millis();
      if(millis()-clearSince>=releaseMs){
        enabledAfterClear=true;clearSince=0;
      }
      return;
    }
    if(onLine){
      clearSince=0;
      if(!latched){
        if(!activeSince)activeSince=millis();
        if(millis()-activeSince>=confirmMs){
          if(count<255)count++;
          latched=true;activeSince=0;
        }
      }
    }else{
      activeSince=0;
      if(latched){
        if(!clearSince)clearSince=millis();
        if(millis()-clearSince>=releaseMs){
          latched=false;clearSince=0;
        }
      }
    }
  }
};

AutoLineCounter rightAutoCounter,leftAutoCounter,bridgeAutoCounter;
uint32_t autoXStableSince=0,autoHeadingStableSince=0;
float pointBSearchCenterYmm=0.0f;
int8_t pointBSearchDirectionSign=1;
uint32_t pointBPairLostSinceMs=0;
bool pointARightMiddleSeen=false,pointARightInsideSeen=false;
uint32_t pointARightPairStartedMs=0;
// Point-A pass memory: arm near the measured coordinate, then remember the
// middle and inside eyes independently until both have crossed the target line.
bool pointATargetPairTracking=false;
bool pointAMiddlePassed=false,pointAInsidePassed=false;
bool pointASearchActive=false;
int8_t pointASearchDirectionSign=1;
int8_t pointAPassDirectionSign=1;
bool pointASidePairLocked=false;
float pointAHSearchCenterXmm=0.0f;
int8_t pointAHSearchDirectionSign=1;
// Locked after POINT_B: the two 3-eye pickup arrays remain visible in
// telemetry but can no longer count lines or drive an AUTO transition.
bool stopArrayCountingLocked=false;

uint8_t stopGroupActiveCount(uint8_t offset){
  uint8_t count=0;
  for(uint8_t i=0;i<3;i++)
    if(stopLine.normalized[offset+i]>=LINE_ACTIVE_NORMALIZED)count++;
  return count;
}

bool pointARequiredSensorsOnLine(bool rightGroup){
  const uint8_t offset=rightGroup?3:0;
  const bool middleOnLine=
      stopLine.normalized[offset+1]>=POINT_A_RIGHT_SENSOR_THRESHOLD;
  const bool insideOnLine=
      stopLine.normalized[offset+2]>=POINT_A_RIGHT_SENSOR_THRESHOLD;
  const bool tailOnLine=
      stopLine.holdNormalized[0]>=LATERAL_HOLD_LINE_THRESHOLD;
  const bool frontOnLine=
      stopLine.holdNormalized[1]>=LATERAL_HOLD_LINE_THRESHOLD;
  return middleOnLine&&insideOnLine&&tailOnLine&&frontOnLine;
}

void updatePointAPassMemory(bool rightGroup){
  const uint8_t offset=rightGroup?3:0;
  pointAMiddlePassed|=
      stopLine.normalized[offset+1]>=POINT_A_RIGHT_SENSOR_THRESHOLD;
  pointAInsidePassed|=
      stopLine.normalized[offset+2]>=POINT_A_RIGHT_SENSOR_THRESHOLD;
}

bool pointBRequiredSensorsOnLine(bool rightGroup){
  const uint8_t offset=rightGroup?3:0;
  return stopLine.normalized[offset+1]>=LINE_ACTIVE_NORMALIZED&&
         stopLine.normalized[offset+2]>=LINE_ACTIVE_NORMALIZED;
}

bool updateAutoPickTravel(float dt,bool rightGroup,uint8_t targetCount){
  if(stopArrayCountingLocked){
    stopRobot();
    return false;
  }
  if(!firstLateralGuidanceReady()){
    setRobotVelocity(0.0f,
                     selectedFieldLateralSign()*MOVE_LEFT_SPEED_MM_S,
                     headingControlLateral(dt));
    return false;
  }
  AutoLineCounter &counter=rightGroup?rightAutoCounter:leftAutoCounter;
  const uint8_t offset=rightGroup?3:0;
  uint8_t activeForCount=0;
  for(uint8_t i=0;i<3;i++)
    if(stopLine.normalized[offset+i]>=PICK_LINE_DETECT_NORMALIZED)
      activeForCount++;
  // At high lateral speed a narrow/skewed line may touch only one outer eye
  // for two control samples. Latch that event here; BU_TAM_A/B still requires
  // the middle eye or 2/3 eyes before accepting final alignment.
  const float yawError=fabsf(shortestAngleError(targetYawDeg,currentYawDeg));
  const bool headingSafe=bnoValid&&yawError<=LATERAL_LINE_COUNT_MAX_YAW_ERROR_DEG;
  const bool onLine=activeForCount>=1&&headingSafe;
  const bool approachingTarget=onLine&&!counter.latched&&
                               counter.count+1>=targetCount;
  counter.update(onLine,PICK_LINE_CONFIRM_MS,PICK_LINE_CLEAR_MS);

  const float pointATravelMm=fabsf(pose.y_mm-firstLateralStartedYmm);
  const float pointASearchMinMm=max(
      0.0f,POINT_A_EXPECTED_LATERAL_MM-POINT_A_SEARCH_HALF_RANGE_MM);
  const float pointASearchMaxMm=
      POINT_A_EXPECTED_LATERAL_MM+POINT_A_SEARCH_HALF_RANGE_MM;
  const bool pointAEncoderGate=
      pointATravelMm>=pointASearchMinMm&&pointATravelMm<=pointASearchMaxMm;
  if(state==DI_SANG_TRAI_A&&pointAEncoderGate&&
     (approachingTarget||counter.count>=targetCount))
    pointATargetPairTracking=true;
  if(state==DI_SANG_TRAI_A&&pointATargetPairTracking)
    updatePointAPassMemory(rightGroup);

  const bool pointAPairPassed=pointAMiddlePassed&&pointAInsidePassed;
  const bool pointALineCountReady=counter.count>=targetCount;
  if(state==DI_SANG_TRAI_A&&pointATargetPairTracking&&
     pointALineCountReady&&pointAEncoderGate&&pointAPairPassed){
    pointAPassDirectionSign=pointASearchActive?
        pointASearchDirectionSign:selectedFieldLateralSign();
    stopRobot();
    Serial.println(
        "ACK,AUTO_POINT_A_COUNT_ENCODER_PAIR_CONFIRMED_REVERSE_ALIGN");
    return true;
  }

  float travelSpeed=approachingTarget?MOVE_LEFT_SPEED_MM_S*0.30f:
                                       MOVE_LEFT_SPEED_MM_S;
  int8_t travelDirectionSign=selectedFieldLateralSign();
  if(state==DI_SANG_TRAI_A&&pointATravelMm>=pointASearchMinMm){
    travelSpeed=min(travelSpeed,POINT_A_SEARCH_SPEED_MM_S);
    if(!pointASearchActive&&pointATravelMm>=pointASearchMaxMm){
      pointASearchActive=true;
      pointASearchDirectionSign=-selectedFieldLateralSign();
      Serial.println("ACK,AUTO_POINT_A_LINE_MISSED_START_LEFT_RIGHT_SEARCH");
    }
    if(pointASearchActive){
      if(pointASearchDirectionSign==selectedFieldLateralSign()&&
         pointATravelMm>=pointASearchMaxMm)
        pointASearchDirectionSign=-selectedFieldLateralSign();
      else if(pointASearchDirectionSign==-selectedFieldLateralSign()&&
              pointATravelMm<=pointASearchMinMm)
        pointASearchDirectionSign=selectedFieldLateralSign();
      travelDirectionSign=pointASearchDirectionSign;
    }
  }
  if(!headingSafe)travelSpeed*=LATERAL_HEADING_RECOVERY_SPEED_FACTOR;
  const float holdVx = state==DI_SANG_TRAI_A ? 0.0f :
                       lateralLineHoldVelocity(dt);
  setRobotVelocity(holdVx,
                   travelDirectionSign*travelSpeed,
                   headingControlLateral(dt));
  // Point A is accepted only by the sensor/encoder condition above.
  if(state==DI_SANG_TRAI_A)return false;
  if(counter.count<targetCount)return false;
  stopRobot();return true;
}

bool updateAutoPickX(float dt,bool rightGroup){
  const uint8_t offset=rightGroup?3:0;
  const uint8_t active=stopGroupActiveCount(offset);
  const bool centerOnLine=stopLine.normalized[offset+1]>=LINE_ACTIVE_NORMALIZED;

  if(state==BU_TAM_A){
    const bool outsideOnLine=
        stopLine.normalized[offset]>=POINT_A_RIGHT_SENSOR_THRESHOLD;
    const bool insideOnLine=
        stopLine.normalized[offset+2]>=POINT_A_RIGHT_SENSOR_THRESHOLD;
    const bool sidePairOnLine=centerOnLine&&insideOnLine;
    const bool tailOnLine=
        stopLine.holdNormalized[0]>=LATERAL_HOLD_LINE_THRESHOLD;
    const bool frontOnLine=
        stopLine.holdNormalized[1]>=LATERAL_HOLD_LINE_THRESHOLD;

    if(sidePairOnLine&&!pointASidePairLocked){
      pointASidePairLocked=true;
      pointAHSearchCenterXmm=pose.x_mm;
      pointAHSearchDirectionSign=1;
      resetLateralLineHold();
      Serial.println("ACK,AUTO_POINT_A_SIDE_PAIR_LOCKED_START_H1_H2_SEARCH");
    }

    if(pointARequiredSensorsOnLine(rightGroup)){
      if(!autoXStableSince)autoXStableSince=millis();
      setRobotVelocity(0.0f,0.0f,headingControlFine(dt));
      if(millis()-autoXStableSince>=X_ALIGN_STABLE_TIME_MS){
        xAligned=true;
        stopRobot();
        Serial.println("ACK,AUTO_POINT_A_MIDDLE_INSIDE_H1_H2_ALIGNED");
        return true;
      }
      return false;
    }

    autoXStableSince=0;
    if(!sidePairOnLine){
      // After passing the line, reverse slowly until middle+inside are on black.
      float vy=-pointAPassDirectionSign*POINT_A_RIGHT_ALIGN_SPEED_MM_S;
      if(!centerOnLine&&insideOnLine)
        vy=pointAPassDirectionSign*POINT_A_RIGHT_ALIGN_SPEED_MM_S;
      else if(outsideOnLine||centerOnLine)
        vy=-pointAPassDirectionSign*POINT_A_RIGHT_ALIGN_SPEED_MM_S;
      setRobotVelocity(0.0f,vy,headingControlLateral(dt));
      return false;
    }

    // Keep the side pair fixed and search H1/H2 only along X within +/-100 mm.
    const float searchOffsetX=pose.x_mm-pointAHSearchCenterXmm;
    float holdVx=0.0f;
    if(tailOnLine||frontOnLine){
      holdVx=lateralLineHoldVelocity(dt);
    }else{
      if(searchOffsetX>=POINT_A_H_SEARCH_RADIUS_MM)
        pointAHSearchDirectionSign=-1;
      else if(searchOffsetX<=-POINT_A_H_SEARCH_RADIUS_MM)
        pointAHSearchDirectionSign=1;
      holdVx=pointAHSearchDirectionSign*POINT_A_H_SEARCH_SPEED_MM_S;
    }
    if(searchOffsetX>=POINT_A_H_SEARCH_RADIUS_MM&&holdVx>0.0f)
      holdVx=-POINT_A_H_SEARCH_SPEED_MM_S;
    else if(searchOffsetX<=-POINT_A_H_SEARCH_RADIUS_MM&&holdVx<0.0f)
      holdVx=POINT_A_H_SEARCH_SPEED_MM_S;
    setRobotVelocity(holdVx,0.0f,headingControlLateral(dt));
    return false;
  }

  // Point B requires the 0 and +1000 eyes simultaneously on black.
  const bool requiredPairOnLineB=pointBRequiredSensorsOnLine(rightGroup);
  if(requiredPairOnLineB){
    if(!autoXStableSince)autoXStableSince=millis();
    setRobotVelocity(0.0f,0.0f,headingControlStationary(dt));
    const uint32_t pairStableMs=state==CAN_LINE_DOC_PHAI_THA2B
        ?THA2B_PAIR_CONFIRM_MS:X_ALIGN_STABLE_TIME_MS;
    if(millis()-autoXStableSince>=pairStableMs){
      xAligned=true;stopRobot();return true;
    }
    return false;
  }
  autoXStableSince=0;

  const bool tha2BAlignment=state==CAN_LINE_DOC_PHAI_THA2B;
  const bool postBridgeAlignment=state==CAN_PHAI_LINE_B_E18||tha2BAlignment;
  const float searchSpeed=tha2BAlignment?THA2B_PAIR_SEARCH_SPEED_MM_S:
      (postBridgeAlignment?POST_BRIDGE_E18_ALIGN_MIN_SPEED_MM_S:
                           X_ALIGN_SEARCH_SPEED_MM_S);
  const float alignMinSpeed=tha2BAlignment?THA2B_PAIR_ALIGN_MIN_SPEED_MM_S:
      (postBridgeAlignment?POST_BRIDGE_E18_ALIGN_MIN_SPEED_MM_S:
                           X_ALIGN_MIN_SPEED_MM_S);
  const float alignMaxSpeed=tha2BAlignment?THA2B_PAIR_ALIGN_MAX_SPEED_MM_S:
      (postBridgeAlignment?POST_BRIDGE_E18_ALIGN_MAX_SPEED_MM_S:
                           X_ALIGN_MAX_SPEED_MM_S);
  float vy=pointBSearchDirectionSign*searchSpeed;
  if(active){
    const float position=rightGroup?stopLine.rightPosition:stopLine.leftPosition;
    PID &pid=rightGroup?rightLinePid:leftLinePid;
    const int8_t controlSign = rightGroup ? RIGHT_STOP_CONTROL_SIGN
                                          : LEFT_STOP_CONTROL_SIGN;
    const float signedError=controlSign*(POINT_B_PAIR_TARGET_POSITION-position);
    vy=controlSign*pid.updateError(POINT_B_PAIR_TARGET_POSITION-position,dt,
                                   -alignMaxSpeed,alignMaxSpeed);
    if(alignMinSpeed>0.0f&&fabsf(vy)<alignMinSpeed)
      vy=copysignf(alignMinSpeed,fabsf(vy)>0.01f?vy:signedError);
  }else{
    const float searchOffsetY=pose.y_mm-pointBSearchCenterYmm;
    const int8_t routeSign=state==CAN_LINE_DOC_PHAI_THA2B
        ?-1:selectedFieldLateralSign();
    if(searchOffsetY>=POINT_B_PAIR_SEARCH_RADIUS_MM)
      pointBSearchDirectionSign=-routeSign;
    else if(searchOffsetY<=-POINT_B_PAIR_SEARCH_RADIUS_MM)
      pointBSearchDirectionSign=routeSign;
    vy=pointBSearchDirectionSign*searchSpeed;
  }
  setRobotVelocity(0,vy,headingControlStationary(dt));
  return false;
}

bool pointBPairLostBeyondGrace(bool rightGroup){
  if(pointBRequiredSensorsOnLine(rightGroup)){
    pointBPairLostSinceMs=0;
    return false;
  }
  if(!pointBPairLostSinceMs)pointBPairLostSinceMs=millis();
  return millis()-pointBPairLostSinceMs>=POINT_B_PAIR_LOST_GRACE_MS;
}

bool tha2BHorizontalMarkerPresent(){
  const bool leftGroupOnHorizontal=
      stopGroupActiveCount(0)>=THA2B_LEFT_MARKER_MIN_ACTIVE;
  const bool rightPairOnVertical=pointBRequiredSensorsOnLine(true);
  const bool rightOutsideOnHorizontal=
      stopLine.normalized[3]>=LINE_ACTIVE_NORMALIZED;
  return leftGroupOnHorizontal&&rightPairOnVertical&&rightOutsideOnHorizontal;
}

bool tha2BHorizontalMarkerCleared(){
  const bool leftGroupClear=
      stopGroupActiveCount(0)<THA2B_LEFT_MARKER_MIN_ACTIVE;
  const bool rightOutsideClear=
      stopLine.normalized[3]<LINE_ACTIVE_NORMALIZED;
  return leftGroupClear&&rightOutsideClear;
}

bool updateReverseKeepingRightPair(float dt,float reverseSpeed){
  const uint8_t offset=3;
  const bool middleOnLine=
      stopLine.normalized[offset+1]>=LINE_ACTIVE_NORMALIZED;
  const bool insideOnLine=
      stopLine.normalized[offset+2]>=LINE_ACTIVE_NORMALIZED;
  if(middleOnLine||insideOnLine){
    pointBPairLostSinceMs=0;
  }else{
    if(!pointBPairLostSinceMs)pointBPairLostSinceMs=millis();
    if(millis()-pointBPairLostSinceMs>=
       POST_BRIDGE_FORWARD_PAIR_LOST_CONFIRM_MS){
      stopRobot();
      return false;
    }
  }

  const float middleStrength=stopLine.normalized[offset+1];
  const float insideStrength=stopLine.normalized[offset+2];
  const float pairStrength=middleStrength+insideStrength;
  const float linePosition=pairStrength>1.0f
      ?1000.0f*insideStrength/pairStrength
      :POINT_B_PAIR_TARGET_POSITION;
  const float vy=RIGHT_STOP_CONTROL_SIGN*rightLinePid.updateError(
      POINT_B_PAIR_TARGET_POSITION-linePosition,dt,
      -X_ALIGN_MAX_SPEED_MM_S,X_ALIGN_MAX_SPEED_MM_S);
  setRobotVelocity(-fabsf(reverseSpeed),vy,headingControlFine(dt));
  return true;
}

bool updateForwardKeepingPointBPair(float dt,bool rightGroup){
  if(!moveCommand.active)return true;

  const uint8_t offset=rightGroup?3:0;
  const bool middleOnLine=
      stopLine.normalized[offset+1]>=LINE_ACTIVE_NORMALIZED;
  const bool insideOnLine=
      stopLine.normalized[offset+2]>=LINE_ACTIVE_NORMALIZED;
  // One eye leaving black supplies the correction direction; do not cancel
  // forward travel for that normal tracking condition. Pause only if both
  // eyes are absent continuously, which rejects a single noisy ADC sample.
  if(middleOnLine||insideOnLine){
    pointBPairLostSinceMs=0;
  }else{
    if(!pointBPairLostSinceMs)pointBPairLostSinceMs=millis();
    if(millis()-pointBPairLostSinceMs>=
       POST_BRIDGE_FORWARD_PAIR_LOST_CONFIRM_MS)xAligned=false;
  }
  if(!xAligned){
    // Pause X immediately and reuse point B's search/PID until both the
    // middle (0) and inside (+1000) eyes are stable on black again.
    if(updateAutoPickX(dt,rightGroup)){
      xAligned=true;pointBPairLostSinceMs=0;
      rightLinePid.reset();leftLinePid.reset();
      Serial.println("ACK,AUTO_FORWARD_LINE_PAIR_REACQUIRED");
    }
    return false;
  }

  const float worldDx=pose.x_mm-moveCommand.start.x_mm;
  const float worldDy=pose.y_mm-moveCommand.start.y_mm;
  const float yaw=-moveCommand.start.yaw_deg*PI/180.0f;
  const float localX=cosf(yaw)*worldDx-sinf(yaw)*worldDy;
  const float errorX=moveCommand.dx-localX;
  positionErrorX=errorX;
  if(fabsf(errorX)<18.0f){
    moveCommand.active=false;stopRobot();return true;
  }

  float vx=positionXPid.updateError(
      errorX,dt,-moveCommand.maxSpeed,moveCommand.maxSpeed);
  // A 70 mm move starts below the requested speed from the position P term.
  // Keep enough command to overcome loaded-wheel stiction.
  const float minimumVx=min(moveCommand.maxSpeed,
                            POST_BRIDGE_E18_FORWARD_MIN_SPEED_MM_S);
  if(fabsf(vx)<minimumVx)vx=copysignf(minimumVx,errorX);
  const float middleStrength=stopLine.normalized[offset+1];
  const float insideStrength=stopLine.normalized[offset+2];
  const float pairStrength=middleStrength+insideStrength;
  const float linePosition=pairStrength>1.0f
      ? 1000.0f*insideStrength/pairStrength
      : POINT_B_PAIR_TARGET_POSITION;
  PID &pid=rightGroup?rightLinePid:leftLinePid;
  const int8_t controlSign=rightGroup?RIGHT_STOP_CONTROL_SIGN:
                                        LEFT_STOP_CONTROL_SIGN;
  const float vy=controlSign*pid.updateError(
      POINT_B_PAIR_TARGET_POSITION-linePosition,dt,
      -X_ALIGN_MAX_SPEED_MM_S,X_ALIGN_MAX_SPEED_MM_S);
  setRobotVelocity(vx,vy,headingControlFine(dt));
  return false;
}

bool autoHeadingAligned(){
  if(!bnoValid){autoHeadingStableSince=0;return false;}
  if(fabsf(shortestAngleError(targetYawDeg,currentYawDeg))<=HEADING_TOLERANCE_DEG &&
     fabsf(yawRateDegS)<=STATION_HEADING_SETTLED_RATE_DEG_S){
    if(!autoHeadingStableSince)autoHeadingStableSince=millis();
    return millis()-autoHeadingStableSince>=HEADING_STABLE_TIME_MS;
  }
  autoHeadingStableSince=0;return false;
}

bool autoPickVerified(){
  // The side-eye/H1-H2 line alignment was confirmed in the preceding states.
  // Keep it latched while final heading correction rotates the chassis.
  return xAligned&&yAligned&&autoHeadingAligned();
}

void reportStationLineInfo(const char *point,bool rightGroup) {
  const uint8_t offset=rightGroup?3:0;
  Serial.print("SAFETY,AUTO_LINE_REFERENCE,");Serial.print(point);Serial.print(',');
  Serial.print(rightGroup?"RIGHT":"LEFT");
  for(uint8_t i=0;i<3;i++){
    Serial.print(',');Serial.print(stopLine.normalized[offset+i]);
  }
  Serial.println();
}

// ============================================================
// Non-blocking ESP32 and USB command parsers
// ============================================================

template <size_t N>
class LineReceiver {
 public:
  bool feed(Stream &stream,char *output,size_t outputCapacity) {
    if(output==nullptr||outputCapacity==0)return false;
    while(stream.available()){
      const char c=static_cast<char>(stream.read());
      if(c=='\r'||c=='\n'){
        if(length){
          buffer[length]='\0';
          const size_t copyLength=min(length,outputCapacity-1);
          memcpy(output,buffer,copyLength);
          output[copyLength]='\0';
          length=0;
          return true;
        }
      }else if(length<N-1)buffer[length++]=c;
      else length=0;
    }
    return false;
  }
 private: char buffer[N]={};size_t length=0;
};

LineReceiver<256> telemetryReceiver;
LineReceiver<256> usbCableReceiver;
uint32_t espCommandSentMs=0;

bool isMechanismReplyForBuzzer(const char *line) {
  if(!line||!*line)return false;
  return strcmp(line,"DONE_THA2A")==0 ||
         strncmp(line,"DONE",4)==0 ||
         strncmp(line,"ACK,",4)==0 ||
         strncmp(line,"ERR,",4)==0 ||
         strncmp(line,"EVENT,",6)==0 ||
         strncmp(line,"HELLO,",6)==0;
}

void sendMechanismFrame(const char *command,bool announce=true) {
  if(!command||!*command)return;
  mechanismClient.send(command);
  if(!announce)return;
  // The dashboard refreshes an unchanged JOG command for its watchdog. Beep
  // only when that command changes, otherwise holding a lever would sound
  // continuously and overflow the non-blocking buzzer queue.
  static char lastJogCommand[64]={};
  if(strncmp(command,"JOG,",4)==0){
    if(strcmp(command,lastJogCommand)==0)return;
    strncpy(lastJogCommand,command,sizeof(lastJogCommand)-1);
    lastJogCommand[sizeof(lastJogCommand)-1]=0;
  }else lastJogCommand[0]=0;
  buzzer.beep(1);
}

void sendEsp(const char *command) {
  if(!ESP32_UART_ENABLED){
    espDone=false;
    setFault(FAULT_ESP_TIMEOUT,"ESP_UART_DISABLED_PIN_CONFLICT");
    return;
  }
  strncpy(lastEspCommand,command,sizeof(lastEspCommand)-1);
  lastEspCommand[sizeof(lastEspCommand)-1]='\0';
  strcpy(lastEspResponse,"WAIT");
  sendMechanismFrame(command);espDone=false;espCommandSentMs=millis();
}

void sendDoneTha2ARequest() {
  if(doneTha2AAttempts>=DONE_THA2A_MAX_ATTEMPTS)return;
  sendEsp("THA_2A");
  ++doneTha2AAttempts;
  lastDoneTha2ASendMs=millis();
  Serial.print("ACK,AUTO_TX_THA_2A,ATTEMPT,");
  Serial.println(doneTha2AAttempts);
}

void sendDoneTha2BRequest() {
  if(doneTha2BAttempts>=DONE_THA2B_MAX_ATTEMPTS)return;
  sendEsp("THA_2B");
  ++doneTha2BAttempts;
  lastDoneTha2BSendMs=millis();
  Serial.print("ACK,AUTO_TX_THA_2B,ATTEMPT,");
  Serial.println(doneTha2BAttempts);
}

const char *autoMechanismModeName() {
  return "REAL";
}

const char *autoTelemetryModeName() {
  return autoTelemetryMode == AUTO_TELEMETRY_SILENT ? "SILENT" : "ON_REQUEST";
}

void printAutoStartConfiguration(const char *source) {
  Serial.print("SAFETY,AUTO_CONFIG,");Serial.print(source);Serial.print(',');
  Serial.print(selectedField==FIELD_BLUE?"BLUE":"RED");Serial.print(',');
  Serial.println(autoMechanismModeName());
}

void printAutoTelemetryMode() {
  Serial.print("AUTO_TELEM,MODE,");
  Serial.println(autoTelemetryModeName());
}

void requestAutoTelemetrySnapshot() {
  if(autoTelemetryMode == AUTO_TELEMETRY_SILENT) {
    autoTelemetrySnapshotStage=0;
    Serial.println("ACK,AUTO_TELEM,SILENT");
    return;
  }
  ++autoTelemetrySnapshotSequence;
  if(autoTelemetrySnapshotSequence==0)autoTelemetrySnapshotSequence=1;
  autoTelemetrySnapshotStage=1;
  Serial.print("AUTO_TELEM,SNAPSHOT_BEGIN,");
  Serial.println(autoTelemetrySnapshotSequence);
}

void printAutoMechanismStatus() {
  const uint32_t elapsed = autoMechanismWaiting ? millis() - autoMechanismStartedMs : 0;
  constexpr uint32_t remaining = 0;
  Serial.print("AUTO_MECH,");Serial.print(autoMechanismModeName());Serial.print(',');
  Serial.print(autoMechanismWaiting?1:0);Serial.print(',');
  Serial.print(elapsed);Serial.print(',');Serial.print(remaining);Serial.print(',');
  Serial.println(autoMechanismAction);
}

void printMechanismTelemetryStatus() {
  const MechanismTelemetrySnapshot &mechanism=mechanismTelemetry.snapshot();
  char line[192];
  snprintf(line,sizeof(line),
      "MECH_STATUS,%u,%s,%u,%u,%.3f,%.3f,%u,%s,%u,%u,%.1f,%.1f",
      mechanism.online?1U:0U,mechanism.state,mechanism.busy?1U:0U,
      static_cast<unsigned>(mechanism.fault),mechanism.positionAmm,
      mechanism.positionBmm,static_cast<unsigned>(mechanism.valveMask),
      mechanism.activeProfile,static_cast<unsigned>(mechanism.sequenceStep),
      mechanism.zeroed?1U:0U,mechanism.jogSpeedA,mechanism.jogSpeedB);
  Serial.println(line);
}

void beginAutoMechanismAction(const char *command) {
  strncpy(autoMechanismAction,command,sizeof(autoMechanismAction)-1);
  autoMechanismAction[sizeof(autoMechanismAction)-1]='\0';
  autoMechanismWaiting=true;
  autoMechanismStartedMs=millis();
  espDone=false;
  sendEsp(command);
  Serial.print("ACK,AUTO_MECH,START,");Serial.print(autoMechanismModeName());
  Serial.print(',');Serial.println(command);
}

bool autoMechanismActionComplete() {
  if(!autoMechanismWaiting)return false;
  const bool complete = espDone;
  if(!complete)return false;
  autoMechanismWaiting=false;
  espDone=false;
  Serial.print("ACK,AUTO_MECH,DONE,");Serial.print(autoMechanismModeName());
  Serial.print(',');Serial.println(autoMechanismAction);
  return true;
}

char *trimAscii(char *text) {
  while(*text==' '||*text=='\t')text++;
  char *end=text+strlen(text);
  while(end>text&&(end[-1]==' '||end[-1]=='\t'||end[-1]=='\r'||end[-1]=='\n'))--end;
  *end='\0';
  return text;
}

bool mechanismDoneMatchesActiveAction(const char *line) {
  if(!line||!autoMechanismWaiting)return false;
  if(strcmp(autoMechanismAction,"ROBOT START")==0)
    return strcmp(line,"DONE,ROBOT_START")==0;
  if(strcmp(autoMechanismAction,"POINT_A")==0)
    return strcmp(line,"DONE_POINT_A")==0||strcmp(line,"DONE,PICK_A")==0||strcmp(line,"DONE,POINT_A")==0;
  if(strcmp(autoMechanismAction,"POINT_B")==0)
    return strcmp(line,"DONE_POINT_B")==0||strcmp(line,"DONE,PICK_B")==0||strcmp(line,"DONE,POINT_B")==0;
  if(strcmp(autoMechanismAction,"THA_2A")==0)
    return strcmp(line,"DONE_THA_2A")==0||strcmp(line,"DONE_THA2A")==0||strcmp(line,"DONE,THA2A")==0;
  if(strcmp(autoMechanismAction,"THA_2B")==0)
    return strcmp(line,"DONE_THA_2B")==0||strcmp(line,"DONE_THA2B")==0||strcmp(line,"DONE,THA2B")==0;
  return false;
}

void handleMechanismLine(const char *line, void *) {
  if(!line||!*line)return;
  if(isMechanismReplyForBuzzer(line))buzzer.beepFast(2);
  if(strcmp(line,"DONE_THA_2A")==0||strcmp(line,"DONE_THA2A")==0){
    espDoneTha2A=true;
    Serial.println("ACK,AUTO_RX_DONE_THA2A");
  }else if(strcmp(line,"DONE_THA_2B")==0||strcmp(line,"DONE_THA2B")==0){
    espDoneTha2B=true;
    Serial.println("ACK,AUTO_RX_DONE_THA2B");
  }
  if(strcmp(line,"EVENT,E18_BOTH")==0||
     strcmp(line,"EVENT,E18_BOTH,1")==0){
    espE18BothDetected=true;
    Serial.println("ACK,AUTO_E18_BOTH_ACTIVE");
  }else if(strcmp(line,"EVENT,E18_BOTH,0")==0){
    espE18BothDetected=false;
    Serial.println("ACK,AUTO_E18_BOTH_CLEAR");
  }
  unsigned e18A=0,e18B=0,e18Armed=0;
  const bool e18StatusRecord=
      sscanf(line,"E18,STATUS,%u,%u,%u",&e18A,&e18B,&e18Armed)==3;
  if(e18StatusRecord)espE18BothDetected=e18Armed&&e18A&&e18B;
  if(mechanismDoneMatchesActiveAction(line))espDone=true;
  const bool statusRecord=mechanismTelemetry.consume(line);
  // STATUS is emitted as one canonical record by printTelemetry(). Do not
  // forward the raw 5 Hz record here: two independently timed streams could
  // otherwise arrive as a corrupt ESP_RX,STTEL... line at the dashboard.
  if(statusRecord||e18StatusRecord)return;
  strncpy(lastEspResponse,line,sizeof(lastEspResponse)-1);
  lastEspResponse[sizeof(lastEspResponse)-1]='\0';
  char frame[288];
  snprintf(frame,sizeof(frame),"ESP_RX,%s",line);
  Serial.println(frame);
}

void updateEspProtocol() {
  if(!ESP32_UART_ENABLED)return;
  mechanismClient.update(handleMechanismLine,nullptr);
  mechanismTelemetry.updateTimeout(2500);
}

enum TestMode:uint8_t {TEST_NONE,TEST_MOTORS,TEST_ENCODERS,TEST_IMU,TEST_CENTER_LINE,
                       TEST_STOP_LINE,TEST_OPTICAL_FLOW,TEST_DISTANCE,TEST_MANUAL_DRIVE,
                       TEST_POSITION_MOVE,TEST_ABSOLUTE_MOVE,TEST_AUTO_HEAD_LOCK,TEST_WHEEL_TUNE};
TestMode testMode=TEST_NONE;
uint32_t testStartedMs=0;
uint32_t lastManualDriveMs=0;
uint32_t lastTunerHeartbeatMs=0;
uint32_t userMoveDeadlineMs=0;
float manualVx=0,manualVy=0,manualWz=0;
bool manualHeadingCaptured=false;
bool manualHeadingLockEnabled=false;
uint8_t tunedWheel=FL;
float tunedWheelTargetRpm=0;

const char *tunerWheelName(uint8_t index) {
  static const char *names[4]={"FL","FR","BL","BR"};
  return index<4?names[index]:"?";
}

int tunerWheelIndex(const char *name) {
  if(strcmp(name,"FL")==0)return FL;
  if(strcmp(name,"FR")==0)return FR;
  if(strcmp(name,"BL")==0||strcmp(name,"RL")==0)return RL;
  if(strcmp(name,"BR")==0||strcmp(name,"RR")==0)return RR;
  return -1;
}

float wheelRpmToMmS(float rpm) { return rpm*PI*WHEEL_DIAMETER_MM/60.0f; }
float wheelMmSToRpm(float speed) { return speed*60.0f/(PI*WHEEL_DIAMETER_MM); }

void sendTunerConfig() {
  Serial.println("CFG,META,ROBOCON_KOSEN_F0_MECANUM,1");
  for(uint8_t i=0;i<4;i++){
    Serial.print("CFG,WHEEL,");Serial.print(tunerWheelName(i));Serial.print(',');
    Serial.print(wheels[i].pid.kp,6);Serial.print(',');Serial.print(wheels[i].pid.ki,6);
    Serial.print(',');Serial.print(wheels[i].pid.kd,6);Serial.print(',');
    Serial.print(wheels[i].kff,6);Serial.print(',');Serial.println(wheels[i].deadzone);
  }
  Serial.print("CFG,HEADING,0,");Serial.print(headingPid.kp,6);Serial.print(',');
  Serial.print(headingPid.ki,6);Serial.print(',');Serial.println(headingPid.kd,6);
  Serial.print("CFG,HEADING_LIMITS,");Serial.print(maxManualHeadingWzRadS,4);Serial.print(',');
  Serial.print(maxAutoHeadLockWzRadS,4);Serial.print(',');Serial.print(headingOutputDeadbandDeg,3);
  Serial.print(',');Serial.println(headingMinWzRadS,4);
  Serial.print("CFG,MOTOR_LIMIT,");Serial.println(maxWheelSpeedMmS,1);
  Serial.print("CFG,LINE,0,");Serial.print(linePid.kp,6);Serial.print(',');
  Serial.print(linePid.ki,6);Serial.print(',');Serial.println(linePid.kd,6);
  Serial.print("CFG,YAW_TARGET,");Serial.println(targetYawDeg,2);
  Serial.print("CFG,SAFETY,");Serial.print(competitionConfigurationValid());Serial.print(',');
  Serial.print(wheelTuneConfigurationValid());Serial.print(',');
  Serial.print(faultFlags);Serial.print(',');Serial.println(stopInputActive());
  Serial.print("CFG,MODE,");Serial.print(controlMode==CONTROL_AUTO?"AUTO":"MANUAL");
  Serial.print(',');Serial.print(selectedField==FIELD_BLUE?"BLUE":"RED");
  Serial.print(',');Serial.println(softwareEStopLatched?1:0);
  Serial.print("CFG,AUTO_MECH,");Serial.print(autoMechanismModeName());
  Serial.print(',');Serial.println(0);
  Serial.print("CFG,AUTO_TELEM,");Serial.println(autoTelemetryModeName());
  Serial.println("CFG_END");
}

void enterPassiveTest(TestMode mode) {
  armed=false;moveCommand.active=false;stopRobot();transitionTo(WAIT_START);testMode=mode;
}

void processCommand(const char *line) {
  if(strncmp(line,"MECH,",5)==0){
    if(!ESP32_UART_ENABLED){Serial.println("ERR,MECH_UART_DISABLED");return;}
    const char *command=line+5;
    if(!*command){Serial.println("ERR,MECH_EMPTY_COMMAND");return;}
    if(strncmp(command,"JOG,",4)==0 && strcmp(command,"JOG,STOP")!=0 && strcmp(command,"JOG,0,0")!=0){
      if(armed || stopInputActive()){
        sendMechanismFrame("JOG,STOP");
        Serial.println("ERR,MECH_JOG,DISARM_AND_RELEASE_ESTOP");return;
      }
    }
    sendMechanismFrame(command);
    Serial.print("ACK,MECH,TX,");Serial.println(command);
    return;
  }
  if(strcmp(line,"HEARTBEAT")==0){lastTunerHeartbeatMs=millis();return;}
  else if(strcmp(line,"PING")==0)Serial.println("HELLO,RBT/1,ROBOCON_KOSEN_F0_MECANUM");
  else if(strcmp(line,"GET_CONFIG")==0)sendTunerConfig();
  else if(strcmp(line,"TUNE,STOP")==0){
    testMode=TEST_NONE;armed=false;moveCommand.active=false;tunedWheelTargetRpm=0;stopRobot();
    transitionTo(WAIT_START);Serial.println("ACK,TUNE,STOP");}
  else if(strncmp(line,"TUNE,WHEEL,",11)==0){
    char name[4]={};float targetRpm=0;
    if(sscanf(line+11,"%3[^,],%f",name,&targetRpm)!=2){
      Serial.println("ERR,TUNE_FORMAT: TUNE,WHEEL,FL,rpm");return;}
    const int index=tunerWheelIndex(name);
    const float maxRpm=wheelMmSToRpm(maxWheelSpeedMmS);
    if(index<0||!isfinite(targetRpm)||fabsf(targetRpm)>maxRpm){
      Serial.print("ERR,TUNE_RANGE,MAX_RPM=");Serial.println(maxRpm,1);return;}
    if(stopInputActive()){Serial.println("ERR,STOP_ACTIVE");return;}
    if(faultFlags!=FAULT_NONE){Serial.println("ERR,RESET_REQUIRED");return;}
    if(!wheelTuneConfigurationValid()){Serial.println("ERR,WHEEL_TUNE_CONFIG_NOT_VERIFIED");return;}
    transitionTo(WAIT_START);resetAllControllers();tunedWheel=static_cast<uint8_t>(index);
    tunedWheelTargetRpm=targetRpm;lastTunerHeartbeatMs=millis();
    testMode=TEST_WHEEL_TUNE;armed=true;
    Serial.print("ACK,TUNE,WHEEL,");Serial.print(tunerWheelName(tunedWheel));
    Serial.print(',');Serial.println(tunedWheelTargetRpm,1);
  }
  else if(strcmp(line,"SAVE_CONFIG")==0||strcmp(line,"PROFILE_SAVE")==0){
    if(armed){Serial.println("ERR,STOP_BEFORE_SAVE");return;}
    Serial.println("ACK,SAVE_CONFIG,RAM_ONLY: EDIT RobotConfig.h TO PERSIST");}
  else if(strncmp(line,"LIMITS,HEADING,",15)==0){
    float manualLimit=0,autoLimit=0,deadband=0,minimum=0;
    if(armed){Serial.println("ERR,STOP_BEFORE_LIMIT_CHANGE");return;}
    if(sscanf(line+15,"%f,%f,%f,%f",&manualLimit,&autoLimit,&deadband,&minimum)==4&&
       isfinite(manualLimit)&&manualLimit>=0.05f&&manualLimit<=1.5f&&
       isfinite(autoLimit)&&autoLimit>=0.05f&&autoLimit<=2.0f&&
       isfinite(deadband)&&deadband>=0.0f&&deadband<=5.0f&&
       isfinite(minimum)&&minimum>=0.0f&&minimum<=1.0f&&minimum<=max(manualLimit,autoLimit)){
      maxManualHeadingWzRadS=manualLimit;maxAutoHeadLockWzRadS=autoLimit;
      headingOutputDeadbandDeg=deadband;headingMinWzRadS=minimum;headingPid.reset();
      Serial.println("ACK,LIMITS,HEADING");
    }else Serial.println("ERR,HEADING_LIMITS_RANGE");
  }
  else if(strncmp(line,"LIMITS,MOTOR,",13)==0){
    float limit=0;
    if(armed){Serial.println("ERR,STOP_BEFORE_LIMIT_CHANGE");return;}
    if(sscanf(line+13,"%f",&limit)==1&&isfinite(limit)&&limit>=100.0f&&limit<=2000.0f){
      maxWheelSpeedMmS=limit;Serial.println("ACK,LIMITS,MOTOR");
    }else Serial.println("ERR,MOTOR_LIMIT_RANGE_100_2000");
  }
  else if(strcmp(line,"FACTORY_RESET")==0){
    if(armed){Serial.println("ERR,STOP_BEFORE_FACTORY_RESET");return;}
    applyFactoryPidConfig();sendTunerConfig();
    Serial.println("ACK,FACTORY_RESET");}
  else if(strcmp(line,"PROFILE_LOAD")==0){
    if(armed){Serial.println("ERR,STOP_BEFORE_PROFILE_LOAD");return;}
    applyFactoryPidConfig();sendTunerConfig();
    Serial.println("ACK,PROFILE_LOAD,COMPILED_DEFAULTS");}
  else if(strcmp(line,"MODE,MANUAL")==0){
    if(armed){Serial.println("ERR,DISARM_BEFORE_MODE_CHANGE");return;}
    supervisedAutoRun=false;testMode=TEST_NONE;armed=false;moveCommand.active=false;stopRobot();
    transitionTo(WAIT_START);controlMode=CONTROL_MANUAL;
    Serial.println("ACK,MODE,MANUAL");
  }
  else if(strcmp(line,"MODE,AUTO")==0){
    if(armed){Serial.println("ERR,DISARM_BEFORE_MODE_CHANGE");return;}
    supervisedAutoRun=false;testMode=TEST_NONE;armed=false;moveCommand.active=false;stopRobot();
    transitionTo(WAIT_START);controlMode=CONTROL_AUTO;
    if(bnoValid)targetYawDeg=currentYawDeg;
    headingPid.reset();Serial.println("ACK,MODE,AUTO");
  }
  else if(strcmp(line,"FIELD,RED")==0||strcmp(line,"FIELD,BLUE")==0){
    if(armed){Serial.println("ERR,DISARM_BEFORE_FIELD_CHANGE");return;}
    selectedField=strcmp(line,"FIELD,BLUE")==0?FIELD_BLUE:FIELD_RED;
    Serial.print("ACK,FIELD,");Serial.println(selectedField==FIELD_BLUE?"BLUE":"RED");
  }
  else if(strcmp(line,"AUTO_MECH,REAL")==0){
    if(armed||state!=WAIT_START){Serial.println("ERR,DISARM_BEFORE_AUTO_MECH_CHANGE");return;}
    autoMechanismMode=AUTO_MECHANISM_REAL;
    autoMechanismWaiting=false;espDone=false;
    strcpy(autoMechanismAction,"NONE");
    Serial.print("ACK,AUTO_MECH,MODE,");Serial.println(autoMechanismModeName());
    printAutoMechanismStatus();
  }
  else if(strcmp(line,"AUTO_TELEM,ON_REQUEST")==0||
          strcmp(line,"AUTO_TELEM,SILENT")==0){
    autoTelemetryMode=strcmp(line,"AUTO_TELEM,SILENT")==0
        ?AUTO_TELEMETRY_SILENT:AUTO_TELEMETRY_ON_REQUEST;
    autoTelemetrySnapshotStage=0;
    Serial.print("ACK,AUTO_TELEM,MODE,");
    Serial.println(autoTelemetryModeName());
    printAutoTelemetryMode();
  }
  else if(strcmp(line,"AUTO_TELEM,SNAPSHOT")==0){
    if(controlMode!=CONTROL_AUTO){Serial.println("ERR,MODE_AUTO_REQUIRED");return;}
    requestAutoTelemetrySnapshot();
  }
  else if(strcmp(line,"ESP_TEST,POINT_A")==0||strcmp(line,"ESP_TEST,POINT_B")==0){
    if(armed){Serial.println("ERR,DISARM_BEFORE_ESP_TEST");return;}
    const char *command=strcmp(line,"ESP_TEST,POINT_B")==0?"POINT_B":"POINT_A";
    sendEsp(command);Serial.print("ACK,ESP_TEST,");Serial.println(command);
  }
  else if(strncmp(line,"AUTO,YAW,",9)==0){
    float yaw=0;
    if(controlMode!=CONTROL_AUTO){Serial.println("ERR,MODE_AUTO_REQUIRED");return;}
    if(sscanf(line+9,"%f",&yaw)!=1||!isfinite(yaw)){
      Serial.println("ERR,AUTO_YAW_FORMAT: AUTO,YAW,degrees");return;}
    targetYawDeg=wrap180(yaw);headingPid.reset();
    Serial.print("ACK,AUTO,YAW,");Serial.println(targetYawDeg,2);
  }
  else if(strcmp(line,"ARM")==0||strcmp(line,"AUTO,ARM")==0){
    if(controlMode!=CONTROL_AUTO){Serial.println("ERR,MODE_AUTO_REQUIRED");return;}
    if(stopInputActive()){Serial.println("ERR,STOP_ACTIVE");return;}
    if(faultFlags!=FAULT_NONE){Serial.println("ERR,RESET_REQUIRED");return;}
    if(!hardwareConfigurationValid()){Serial.println("ERR,CONFIG_NOT_VERIFIED");return;}
    if(!bnoValid){Serial.println("ERR,BNO085_NOT_VALID");return;}
    transitionTo(WAIT_START);resetAllControllers();moveCommand.active=false;
    lastTunerHeartbeatMs=millis();testMode=TEST_AUTO_HEAD_LOCK;armed=true;
    Serial.print("ACK,ARM,AUTO_HEAD_LOCK,");Serial.println(targetYawDeg,2);
  }
  else if(strcmp(line,"START")==0||strcmp(line,"RUN")==0){
    if(controlMode!=CONTROL_AUTO){Serial.println("ERR,MODE_AUTO_REQUIRED");return;}
    if(stopInputActive()){Serial.println("ERR,STOP_ACTIVE");return;}
    if(faultFlags!=FAULT_NONE){Serial.println("ERR,RESET_REQUIRED");return;}
    if(!competitionConfigurationValid()){
      Serial.println("ERR,AUTO_LOCKED: VERIFY_SIGNS_LINE_CALIBRATION_SENSOR_ORDER_AND_ESP_UART");return;}
    supervisedAutoRun=false;testMode=TEST_NONE;armed=true;resetAllControllers();resetPose();
    competitionHeadingDeg=currentYawDeg;targetYawDeg=competitionHeadingDeg;
#if ROBOCON_AFTER_BRIDGE_TEST_BUILD
    transitionTo(DI_LEN_DEM_LINE);
#else
    transitionTo(DOI_HOME_A_READY);
#endif
    printAutoStartConfiguration("UART5");
    Serial.println("SAFETY,AUTO_STARTED");
#if ROBOCON_AFTER_BRIDGE_TEST_BUILD
    Serial.println("ACK,START_AFTER_BRIDGE_TEST");
#else
    Serial.println("ACK,START");
#endif
  }else if(strcmp(line,"START_TEST")==0){
    if(controlMode!=CONTROL_AUTO){Serial.println("ERR,MODE_AUTO_REQUIRED");return;}
    if(stopInputActive()){Serial.println("ERR,STOP_ACTIVE");return;}
    if(faultFlags!=FAULT_NONE){Serial.println("ERR,RESET_REQUIRED");return;}
    if(!competitionConfigurationValid()){
      Serial.println("ERR,AUTO_LOCKED: VERIFY_SIGNS_LINE_CALIBRATION_SENSOR_ORDER_AND_ESP_UART");return;}
    testMode=TEST_NONE;armed=true;resetAllControllers();resetPose();
    competitionHeadingDeg=currentYawDeg;targetYawDeg=competitionHeadingDeg;lastTunerHeartbeatMs=millis();supervisedAutoRun=true;
#if ROBOCON_AFTER_BRIDGE_TEST_BUILD
    transitionTo(DI_LEN_DEM_LINE);
#else
    transitionTo(DOI_HOME_A_READY);
#endif
    printAutoStartConfiguration("UART5_TEST");
    Serial.println("SAFETY,AUTO_STARTED");Serial.println("ACK,START_TEST");
  }else if(strcmp(line,"START_BRIDGE_TEST")==0){
    if(controlMode!=CONTROL_AUTO){Serial.println("ERR,MODE_AUTO_REQUIRED");return;}
    if(stopInputActive()){Serial.println("ERR,STOP_ACTIVE");return;}
    if(faultFlags!=FAULT_NONE){Serial.println("ERR,RESET_REQUIRED");return;}
    if(!competitionConfigurationValid()){
      Serial.println("ERR,AUTO_LOCKED: VERIFY_SIGNS_LINE_CALIBRATION_SENSOR_ORDER_AND_ESP_UART");return;}
    if(!bnoValid){Serial.println("ERR,BNO085_NOT_VALID");return;}
    testMode=TEST_NONE;armed=true;resetAllControllers();resetPose();
    targetYawDeg=currentYawDeg;lastTunerHeartbeatMs=millis();supervisedAutoRun=true;
    transitionTo(DI_SANG_C);Serial.println("ACK,START_BRIDGE_TEST");
  }else if(strcmp(line,"ESTOP")==0){
    if(ESP32_UART_ENABLED)sendMechanismFrame("STOP");
    supervisedAutoRun=false;softwareEStopLatched=true;testMode=TEST_NONE;armed=false;moveCommand.active=false;
    stopRobot();transitionTo(WAIT_START);
    Serial.println("SAFETY,ESTOP,LATCHED");Serial.println("ACK,ESTOP,LATCHED");}
  else if(strcmp(line,"STOP")==0||strcmp(line,"DISARM")==0){
    if(autoMechanismWaiting&&ESP32_UART_ENABLED)sendMechanismFrame("STOP");
    autoMechanismWaiting=false;espDone=false;strcpy(autoMechanismAction,"NONE");
    supervisedAutoRun=false;testMode=TEST_NONE;armed=false;moveCommand.active=false;stopRobot();transitionTo(WAIT_START);
    Serial.println("SAFETY,DISARMED");Serial.println("ACK,DISARM");}
  else if(strcmp(line,"RESET")==0){
    supervisedAutoRun=false;softwareEStopLatched=false;testMode=TEST_NONE;clearFaults();
    if(!stopInputActive())transitionTo(WAIT_START);}
  else if(strcmp(line,"CAL_LINE")==0){
    enterPassiveTest(TEST_NONE);calibrationActive=!calibrationActive;
    calibrationCenterOnly=false;calibrationStopOnly=false;
    if(calibrationActive){
      for(auto &v:centerCalMin)v=1023;
      for(auto &v:centerCalMax)v=0;
      for(auto &v:stopCalMin)v=1023;
      for(auto &v:stopCalMax)v=0;
      Serial.println("ACK,CAL_START");}
    else{printLineCalibration();printCenterCalibrationTelemetry();
      printStopCalibrationTelemetry();Serial.println("ACK,CAL_END");}}
  else if(strcmp(line,"CAL_CENTER,START")==0){
    enterPassiveTest(TEST_CENTER_LINE);calibrationActive=true;
    calibrationCenterOnly=true;calibrationStopOnly=false;
    for(auto &v:centerCalMin)v=1023;
    for(auto &v:centerCalMax)v=0;
    Serial.println("ACK,CAL_CENTER,START");
  }
  else if(strcmp(line,"CAL_CENTER,STOP")==0){
    calibrationActive=false;calibrationCenterOnly=false;calibrationStopOnly=false;
    printCenterCalibrationTelemetry();Serial.println("ACK,CAL_CENTER,STOP");
  }
  else if(strcmp(line,"CAL_STOP,START")==0){
    enterPassiveTest(TEST_STOP_LINE);calibrationActive=true;
    calibrationCenterOnly=false;calibrationStopOnly=true;
    for(auto &v:stopCalMin)v=1023;
    for(auto &v:stopCalMax)v=0;
    Serial.println("ACK,CAL_STOP,START");
  }
  else if(strcmp(line,"CAL_STOP,STOP")==0){
    calibrationActive=false;calibrationCenterOnly=false;calibrationStopOnly=false;
    printStopCalibrationTelemetry();Serial.println("ACK,CAL_STOP,STOP");
  }
  else if(strcmp(line,"TEST_MOTOR")==0){
    enterPassiveTest(TEST_MOTORS);testStartedMs=millis();}
  else if(strcmp(line,"TEST_ENCODER")==0)enterPassiveTest(TEST_ENCODERS);
  else if(strcmp(line,"TEST_BNO")==0)enterPassiveTest(TEST_IMU);
  else if(strcmp(line,"TEST_LINE_CENTER")==0)enterPassiveTest(TEST_CENTER_LINE);
  else if(strcmp(line,"TEST_LINE_STOP")==0)enterPassiveTest(TEST_STOP_LINE);
  else if(strcmp(line,"TEST_FLOW")==0){
    if(OPTICAL_FLOW_ENABLED)enterPassiveTest(TEST_OPTICAL_FLOW);
    else Serial.println("ERR,FLOW_DISABLED");}
  else if(strcmp(line,"TEST_TOF")==0){
    if(TOF_ENABLED)enterPassiveTest(TEST_DISTANCE);
    else Serial.println("ERR,TOF_DISABLED");}
  else if(strcmp(line,"STATUS")==0){
    if(controlMode==CONTROL_AUTO&&armed)requestAutoTelemetrySnapshot();
    else lastTelemetryMs=0;
  }
  else if(strcmp(line,"MANUAL,HEADING,ON")==0){
    if(controlMode!=CONTROL_MANUAL){Serial.println("ERR,MODE_MANUAL_REQUIRED");return;}
    if(!bnoValid){Serial.println("ERR,BNO085_NOT_VALID");return;}
    manualHeadingLockEnabled=true;manualHeadingCaptured=true;
    targetYawDeg=currentYawDeg;headingPid.reset();
    Serial.print("ACK,MANUAL,HEADING,ON,");Serial.println(targetYawDeg,2);
  }
  else if(strcmp(line,"MANUAL,HEADING,OFF")==0){
    manualHeadingLockEnabled=false;manualHeadingCaptured=false;headingPid.reset();
    Serial.println("ACK,MANUAL,HEADING,OFF");
  }
  else if(strncmp(line,"MANUAL,YAW,",11)==0){
    float yaw=0.0f;
    if(controlMode!=CONTROL_MANUAL){Serial.println("ERR,MODE_MANUAL_REQUIRED");return;}
    if(!bnoValid){Serial.println("ERR,BNO085_NOT_VALID");return;}
    if(sscanf(line+11,"%f",&yaw)!=1||!isfinite(yaw)){
      Serial.println("ERR,MANUAL_YAW_FORMAT: MANUAL,YAW,degrees");return;}
    targetYawDeg=wrap180(yaw);manualHeadingLockEnabled=true;
    manualHeadingCaptured=true;headingPid.reset();
    Serial.print("ACK,MANUAL,YAW,");Serial.println(targetYawDeg,2);
  }
  else if(strncmp(line,"DRIVE,",6)==0){
    float vx=0,vy=0,wz=0;
    if(sscanf(line+6,"%f,%f,%f",&vx,&vy,&wz)!=3){
      Serial.println("ERR,DRIVE_FORMAT: DRIVE,vx_mm_s,vy_mm_s,wz_rad_s");return;}
    if(!isfinite(vx)||!isfinite(vy)||!isfinite(wz)){
      Serial.println("ERR,DRIVE_NON_FINITE");return;}
    if(controlMode!=CONTROL_MANUAL){Serial.println("ERR,MODE_MANUAL_REQUIRED");return;}
    if(stopInputActive()){Serial.println("ERR,STOP_ACTIVE");return;}
    // Manual velocity does not use IMU feedback. The user explicitly allows
    // manual DRIVE with BNO unavailable; every other fault remains blocking.
    const uint32_t blockingFaults=faultFlags&~static_cast<uint32_t>(FAULT_IMU_TIMEOUT);
    if(blockingFaults!=FAULT_NONE){Serial.println("ERR,RESET_REQUIRED");return;}
    if(faultFlags&FAULT_IMU_TIMEOUT){
      faultFlags&=~static_cast<uint32_t>(FAULT_IMU_TIMEOUT);
      Serial.println("ACK,DRIVE_IMU_BYPASS");
    }
    if(!hardwareConfigurationValid()){setFault(FAULT_CONFIG,"CONFIG_NOT_VERIFIED");return;}
    const bool enteringManual=testMode!=TEST_MANUAL_DRIVE;
    if(enteringManual){
      moveCommand.active=false;
      transitionTo(WAIT_START);resetAllControllers();testMode=TEST_MANUAL_DRIVE;
      manualHeadingCaptured=bnoValid;
      if(bnoValid)targetYawDeg=currentYawDeg;
    }
    manualVx=constrain(vx,-maxWheelSpeedMmS,maxWheelSpeedMmS);
    manualVy=constrain(vy,-maxWheelSpeedMmS,maxWheelSpeedMmS);
    // The user's wheel-speed limit is the single physical ceiling. Mecanum
    // kinematics still normalizes all four targets together if Vx/Vy/Wz are
    // combined, preserving the requested direction.
    const float maxManualWz=mecanumKmm()>0.0f?maxWheelSpeedMmS/mecanumKmm():0.0f;
    manualWz=constrain(wz,-maxManualWz,maxManualWz);
    lastManualDriveMs=millis();armed=true;
    if(enteringManual)Serial.println("ACK,DRIVE");
  }
  else if(strncmp(line,"MOVE,",5)==0){
    float dx=0,dy=0,maxSpeed=0;
    if(sscanf(line+5,"%f,%f,%f",&dx,&dy,&maxSpeed)!=3){
      Serial.println("ERR,MOVE_FORMAT: MOVE,dx_mm,dy_mm,max_speed_mm_s");return;}
    if(controlMode!=CONTROL_MANUAL){Serial.println("ERR,MODE_MANUAL_REQUIRED");return;}
    if(stopInputActive()){Serial.println("ERR,STOP_ACTIVE");return;}
    if(faultFlags!=FAULT_NONE){Serial.println("ERR,RESET_REQUIRED");return;}
    if(!hardwareConfigurationValid()){Serial.println("ERR,CONFIG_NOT_VERIFIED");return;}
    if(!bnoValid){Serial.println("ERR,BNO085_NOT_VALID");return;}
    if(!isfinite(dx)||!isfinite(dy)||!isfinite(maxSpeed)||
       fabsf(dx)>10000.0f||fabsf(dy)>10000.0f||maxSpeed<30.0f||
       maxSpeed>maxWheelSpeedMmS){Serial.println("ERR,MOVE_SPEED_EXCEEDS_MOTOR_LIMIT");return;}
    transitionTo(WAIT_START);resetAllControllers();resetPose(0,0,currentYawDeg);
    targetYawDeg=currentYawDeg;beginRelativeMove(dx,dy,maxSpeed);
    const float travel=hypotf(dx,dy);
    const uint32_t expectedMs=static_cast<uint32_t>(travel/maxSpeed*1000.0f)+5000UL;
    userMoveDeadlineMs=millis()+constrain(expectedMs,5000UL,60000UL);
    lastTunerHeartbeatMs=millis();testMode=TEST_POSITION_MOVE;armed=true;
    Serial.print("ACK,MOVE,");Serial.print(dx,1);Serial.print(',');
    Serial.print(dy,1);Serial.print(',');Serial.println(maxSpeed,1);
  }
  else if(strncmp(line,"GOTO,",5)==0){
    float targetX=0,targetY=0,targetYaw=0,maxSpeed=0;
    if(sscanf(line+5,"%f,%f,%f,%f",&targetX,&targetY,&targetYaw,&maxSpeed)!=4){
      Serial.println("ERR,GOTO_FORMAT: GOTO,x_mm,y_mm,yaw_deg,max_speed_mm_s");return;}
    if(controlMode!=CONTROL_AUTO){Serial.println("ERR,MODE_AUTO_REQUIRED");return;}
    if(stopInputActive()){Serial.println("ERR,STOP_ACTIVE");return;}
    if(faultFlags!=FAULT_NONE){Serial.println("ERR,RESET_REQUIRED");return;}
    if(!hardwareConfigurationValid()){Serial.println("ERR,CONFIG_NOT_VERIFIED");return;}
    if(!bnoValid){Serial.println("ERR,BNO085_NOT_VALID");return;}
    if(!isfinite(targetX)||!isfinite(targetY)||!isfinite(targetYaw)||!isfinite(maxSpeed)||
       fabsf(targetX)>10000.0f||fabsf(targetY)>10000.0f||fabsf(targetYaw)>180.0f||
       maxSpeed<30.0f||maxSpeed>maxWheelSpeedMmS){Serial.println("ERR,GOTO_RANGE");return;}
    transitionTo(WAIT_START);resetAllControllers();moveCommand.active=false;
    beginAbsoluteMove(targetX,targetY,targetYaw,maxSpeed);
    const float travel=hypotf(targetX-pose.x_mm,targetY-pose.y_mm);
    const uint32_t expectedMs=static_cast<uint32_t>(travel/maxSpeed*1000.0f)+8000UL;
    userMoveDeadlineMs=millis()+constrain(expectedMs,8000UL,90000UL);
    lastTunerHeartbeatMs=millis();testMode=TEST_ABSOLUTE_MOVE;armed=true;
    Serial.print("ACK,GOTO,");Serial.print(targetX,1);Serial.print(',');
    Serial.print(targetY,1);Serial.print(',');Serial.print(wrap180(targetYaw),1);
    Serial.print(',');Serial.println(maxSpeed,1);
  }
  else if(strcmp(line,"POSE_RESET")==0){
    if(armed){Serial.println("ERR,STOP_BEFORE_POSE_RESET");return;}
    resetPose();Serial.println("ACK,POSE_RESET");
  }
  else if(strncmp(line,"YAW,",4)==0){
    float yaw=0;
    if(sscanf(line+4,"%f",&yaw)==1){targetYawDeg=wrap180(yaw);headingPid.reset();Serial.println("ACK,YAW");}
    else Serial.println("ERR,YAW_FORMAT: YAW,degrees");
  }
  else if(strncmp(line,"PID,WHEEL,",10)==0){
    char name[4]={};float kp=0,ki=0,kd=0;
    if(sscanf(line+10,"%3[^,],%f,%f,%f",name,&kp,&ki,&kd)==4&&kp>=0&&ki>=0&&kd>=0){
      int first=-1,last=-1;
      if(strcmp(name,"FL")==0)first=last=FL;else if(strcmp(name,"FR")==0)first=last=FR;
      else if(strcmp(name,"RL")==0||strcmp(name,"BL")==0)first=last=RL;
      else if(strcmp(name,"RR")==0||strcmp(name,"BR")==0)first=last=RR;
      else if(strcmp(name,"ALL")==0){first=FL;last=RR;}
      if(first>=0){for(int i=first;i<=last;i++){
        wheels[i].pid.kp=kp;wheels[i].pid.ki=ki;wheels[i].pid.kd=kd;wheels[i].pid.reset();}
        Serial.println("ACK,PID,WHEEL");}
      else Serial.println("ERR,WHEEL_NAME");
    }else Serial.println("ERR,PID_WHEEL_FORMAT");
  }
  else if(strncmp(line,"PID,HEADING,",12)==0){
    unsigned channel=0;float kp=0,ki=0,kd=0;
    if(sscanf(line+12,"%u,%f,%f,%f",&channel,&kp,&ki,&kd)==4&&kp>=0&&ki>=0&&kd>=0){
      headingPid.kp=kp;headingPid.ki=ki;headingPid.kd=kd;headingPid.reset();
      Serial.println("ACK,PID,HEADING");
    }else Serial.println("ERR,PID_HEADING_FORMAT");
  }
  else if(strncmp(line,"PID,LINE,",9)==0){
    unsigned channel=0;float kp=0,ki=0,kd=0;
    if(sscanf(line+9,"%u,%f,%f,%f",&channel,&kp,&ki,&kd)==4&&kp>=0&&ki>=0&&kd>=0){
      linePid.kp=kp;linePid.ki=ki;linePid.kd=kd;linePid.reset();
      Serial.println("ACK,PID,LINE");
    }else Serial.println("ERR,PID_LINE_FORMAT");
  }
  else if(strncmp(line,"SET_PID,",8)==0){
    unsigned wheelIndex=0;float kp=0,ki=0,kd=0,kff=0;int deadzone=0;
    if(sscanf(line+8,"%u,%f,%f,%f,%f,%d",&wheelIndex,&kp,&ki,&kd,&kff,&deadzone)==6&&
       wheelIndex<WHEEL_COUNT&&kp>=0&&ki>=0&&kd>=0&&kff>=0&&deadzone>=0&&deadzone<PWM_MAX){
      WheelControl &wheel=wheels[wheelIndex];
      wheel.pid.kp=kp;wheel.pid.ki=ki;wheel.pid.kd=kd;
      wheel.kff=kff;wheel.deadzone=deadzone;wheel.pid.reset();
      Serial.print("ACK,SET_PID,");Serial.println(wheelIndex);
    }else Serial.println("ERR,SET_PID_FORMAT: SET_PID,wheel(0-3),kp,ki,kd,kff,deadzone");
  }
  else Serial.println("ERR,UNKNOWN_COMMAND");
}

void updateUsbCommands() {
  char line[256];
  while(telemetryReceiver.feed(Serial,line,sizeof(line)))processCommand(line);
  while(usbCableReceiver.feed(usbCommandStream(),line,sizeof(line))){
    usbCommandStream().print("USB_RX,");usbCommandStream().println(line);
    processCommand(line);
  }
}

void updateTestMode(float dt) {
  if(testMode==TEST_WHEEL_TUNE){
    if(millis()-lastTunerHeartbeatMs>TUNER_WATCHDOG_MS){
      stopRobot();armed=false;testMode=TEST_NONE;tunedWheelTargetRpm=0;
      Serial.println("ERR,TUNER_HEARTBEAT_TIMEOUT");return;}
    if(!wheelTuneConfigurationValid()||faultFlags!=FAULT_NONE||stopInputActive()){
      setFault(FAULT_CONFIG,"TUNE_CONFIG_LOCK");testMode=TEST_NONE;return;}
    for(uint8_t i=0;i<4;i++)
      wheels[i].targetMmS=(i==tunedWheel)?wheelRpmToMmS(tunedWheelTargetRpm):0.0f;
    return;
  }
  if(testMode==TEST_MANUAL_DRIVE){
    if(millis()-lastManualDriveMs>MANUAL_DRIVE_WATCHDOG_MS){
      stopRobot();armed=false;testMode=TEST_NONE;Serial.println("ACK,DRIVE_TIMEOUT");return;}
    const bool translationActive=fabsf(manualVx)>0.5f||fabsf(manualVy)>0.5f;
    const bool rotationActive=fabsf(manualWz)>0.001f;
    if(!translationActive&&!rotationActive){
      headingPid.reset();setRobotVelocity(0,0,0);return;
    }
    float commandedWz=manualWz;
    if(rotationActive){
      headingPid.reset();
      if(bnoValid){targetYawDeg=currentYawDeg;manualHeadingCaptured=true;}
    }else if(manualHeadingLockEnabled&&bnoValid){
      if(!manualHeadingCaptured){
        targetYawDeg=currentYawDeg;headingPid.reset();manualHeadingCaptured=true;
      }
      commandedWz=headingControl(dt,maxManualHeadingWzRadS);
    }else{
      if(!manualHeadingLockEnabled)manualHeadingCaptured=false;
      headingPid.reset();commandedWz=0;
    }
    setRobotVelocity(manualVx,manualVy,commandedWz);return;
  }
  if(testMode==TEST_POSITION_MOVE){
    if(millis()-lastTunerHeartbeatMs>POSITION_LINK_WATCHDOG_MS){
      moveCommand.active=false;stopRobot();armed=false;testMode=TEST_NONE;
      Serial.println("ERR,MOVE_HEARTBEAT_TIMEOUT");return;}
    if(static_cast<int32_t>(millis()-userMoveDeadlineMs)>=0){
      moveCommand.active=false;stopRobot();armed=false;testMode=TEST_NONE;
      Serial.println("ERR,MOVE_TIMEOUT");return;}
    if(updateRelativeMove(dt)){
      stopRobot();armed=false;testMode=TEST_NONE;Serial.println("ACK,MOVE_DONE");}
    return;
  }
  if(testMode==TEST_ABSOLUTE_MOVE){
    if(millis()-lastTunerHeartbeatMs>POSITION_LINK_WATCHDOG_MS){
      moveCommand.active=false;stopRobot();armed=false;testMode=TEST_NONE;
      Serial.println("ERR,GOTO_HEARTBEAT_TIMEOUT");return;}
    if(static_cast<int32_t>(millis()-userMoveDeadlineMs)>=0){
      moveCommand.active=false;stopRobot();armed=false;testMode=TEST_NONE;
      Serial.println("ERR,GOTO_TIMEOUT");return;}
    if(updateAbsoluteMove(dt)){
      stopRobot();armed=false;testMode=TEST_NONE;Serial.println("ACK,GOTO_DONE");}
    return;
  }
  if(testMode==TEST_AUTO_HEAD_LOCK){
    if(millis()-lastTunerHeartbeatMs>TUNER_WATCHDOG_MS){
      stopRobot();armed=false;testMode=TEST_NONE;
      Serial.println("ERR,AUTO_HEARTBEAT_TIMEOUT");return;}
    if(controlMode!=CONTROL_AUTO||faultFlags!=FAULT_NONE||stopInputActive()){
      stopRobot();armed=false;testMode=TEST_NONE;return;}
    if(!bnoValid){
      setFault(FAULT_IMU_TIMEOUT,"BNO085_TIMEOUT");testMode=TEST_NONE;return;}
    setRobotVelocity(0.0f,0.0f,headingControl(dt,maxAutoHeadLockWzRadS));
    return;
  }
  if(testMode!=TEST_MOTORS)return;
  if(!configurationAllowsMotion()){setFault(FAULT_CONFIG,"TEST_CONFIG_LOCK");testMode=TEST_NONE;return;}
  armed=true;const uint32_t elapsed=millis()-testStartedMs;
  if(elapsed>=8000){stopRobot();armed=false;testMode=TEST_NONE;Serial.println("ACK,TEST_MOTOR_DONE");return;}
  const uint8_t index=static_cast<uint8_t>(min(elapsed/2000UL,3UL));
  for(uint8_t i=0;i<4;i++)wheels[i].targetMmS=(i==index)?120.0f:0.0f;
}

// ============================================================
// Competition state machine through the start of bridge line following.
// Current chassis convention remains: vx forward, vy right, wz clockwise.
// ============================================================

bool stateEntered=false;
uint32_t stateStartedMs=0,stateTimeoutMs=STATE_DEFAULT_TIMEOUT_MS;
uint32_t stateWaitStartedMs=0;
uint8_t bridgeCrossCount=0;
float bridgeCommandSpeedMmS=0.0f;
bool bridgeCrossLatched=false;
uint32_t bridgeCrossClearSince=0;
uint32_t lineCenterStableSince=0;
uint32_t bridgeLineValidSince=0;
uint32_t bridgeEntryCenterStableSince=0;
uint32_t startHoldLineStableSince=0;
uint32_t tha2BHorizontalSinceMs=0;
uint32_t tha2BHorizontalClearSinceMs=0;
bool tha2BCoarseLineArmed=false;
uint32_t tha2BCoarseClearSinceMs=0;
uint32_t tha2BCoarseHitSinceMs=0;
RobotPose bridgeAcquireStartPose;
int8_t bridgeAcquireSearchDirectionSign=1;

bool updateTha2BCoarseRightLineLatch(){
  if(!moveCommand.active)return false;
  const float worldDx=pose.x_mm-moveCommand.start.x_mm;
  const float worldDy=pose.y_mm-moveCommand.start.y_mm;
  const float yaw=-moveCommand.start.yaw_deg*PI/180.0f;
  const float localY=sinf(yaw)*worldDx+cosf(yaw)*worldDy;
  const bool pairTouched=
      stopLine.normalized[4]>=THA2B_COARSE_LINE_THRESHOLD||
      stopLine.normalized[5]>=THA2B_COARSE_LINE_THRESHOLD;

  if(!tha2BCoarseLineArmed){
    tha2BCoarseHitSinceMs=0;
    if(fabsf(localY)<THA2B_COARSE_LINE_ARM_DISTANCE_MM){
      tha2BCoarseClearSinceMs=0;
      return false;
    }
    if(pairTouched){
      tha2BCoarseClearSinceMs=0;
      return false;
    }
    if(!tha2BCoarseClearSinceMs)tha2BCoarseClearSinceMs=millis();
    if(millis()-tha2BCoarseClearSinceMs>=THA2B_COARSE_CLEAR_CONFIRM_MS){
      tha2BCoarseLineArmed=true;
      Serial.println("ACK,AUTO_THA2B_NEXT_VERTICAL_LINE_ARMED");
    }
    return false;
  }

  if(!pairTouched){
    tha2BCoarseHitSinceMs=0;
    return false;
  }
  if(!tha2BCoarseHitSinceMs)tha2BCoarseHitSinceMs=millis();
  return millis()-tha2BCoarseHitSinceMs>=THA2B_COARSE_HIT_CONFIRM_MS;
}

const char *stateName(AutoState s) {
  static const char *names[]={"WAIT_START","START_MECHANISM_ASYNC","START_FORWARD_ENCODER_170MM","LEGACY_START_FORWARD","LEGACY_TOF_H1_H2",
  "MOVE_LEFT_PICK_1","ALIGN_X_PICK_1","ALIGN_Y_PICK_1","VERIFY_PICK_1",
  "WAIT_PICK_1_DONE","MOVE_LEFT_PICK_2","ALIGN_X_PICK_2","ALIGN_Y_PICK_2",
  "VERIFY_PICK_2","WAIT_PICK_2_DONE","POST_B_LEFT_1300MM","UNUSED","UNUSED",
  "ACQUIRE_AND_CENTER_8_EYE_LINE","POINT_A_RIGHT_ALIGN","FOLLOW_LINE_UNTIL_END_MARKER","HOME_CENTER","HOME_DROP",
  "HOME_TO_B","TIEN_34CM_B","DOI_THA_2B","CHO_SAU_B","LUI_33CM_A","DOI_THA_2A",
  "CHO_SAU_THA_A","LUI_3M","SANG_TRAI_1M5","POST_BRIDGE_RIGHT_190_ALIGN_B_E18",
  "POST_BRIDGE_FORWARD_70MM","WAIT_DONE_THA2A","MOVE_RIGHT_400_BACK_200_THA2B","ALIGN_RIGHT_PAIR_VERTICAL_THA2B","REVERSE_FOLLOW_VERTICAL_THA2B","REVERSE_CLEAR_HORIZONTAL_THA2B","WAIT_DONE_THA2B","POST_THA2B_DIAGONAL_RIGHT_2000_REVERSE_3000","FINISH","FAULT_STOP"};
  return names[static_cast<uint8_t>(s)];
}

void onExitState(AutoState) { resetAllControllers(); }

void transitionTo(AutoState next) {
  onExitState(state);state=next;stateEntered=false;stateStartedMs=millis();
  Serial.print("STATE,");Serial.println(stateName(state));
  if(state==FINISH)Serial.println("SAFETY,AUTO_COMPLETE");
}

void onEnterState() {
  stateEntered=true;stateStartedMs=millis();stateTimeoutMs=STATE_DEFAULT_TIMEOUT_MS;
  switch(state){
    case WAIT_START:
      armed=false;stopRobot();autoMechanismWaiting=false;espDone=false;
      espDoneTha2A=false;espDoneTha2B=false;
      strcpy(autoMechanismAction,"NONE");stateTimeoutMs=0;break;
    case DOI_HOME_A_READY:
      stopRobot();
      beginAutoMechanismAction("ROBOT START");
      autoMechanismWaiting=false;espDone=false;
      Serial.println("SAFETY,AUTO_MECH_START_ASYNC");
      stateTimeoutMs=0;break;
    case CAN_START:
      stopArrayCountingLocked=false;
      // The VL53L3CX is below its reliable minimum at the starting wall.
      // Ignore ranging until the encoder-only escape move has completed.
      tofAcceptanceEnabled=false;tofValid=false;tofSampleIndex=0;tofSampleCount=0;
      targetYawDeg=competitionHeadingDeg;
      headingPid.reset();fineHeadingPid.reset();
      beginRelativeMove(START_ENCODER_DISTANCE_MM,0,START_ENCODER_SPEED_MM_S);
      stateTimeoutMs=5000;break;
    case CAN_START_FAST:
      startHoldLineStableSince=0;
      headingPid.reset();
      beginRelativeMove(START_ENCODER_DISTANCE_MM,0,START_ENCODER_SPEED_MM_S);
      stateTimeoutMs=5000;break;
    case CAN_START_FINE:
      // We are now far enough from the wall to accept fresh VL53L3CX data.
      tofAcceptanceEnabled=true;tofValid=false;tofSampleIndex=0;tofSampleCount=0;
      lastTofMs=millis();startHoldLineStableSince=0;
      resetLateralLineHold();headingPid.reset();tofDistancePid.reset();
      stateTimeoutMs=START_HOLD_LINE_SEARCH_TIMEOUT_MS;break;
    case DI_SANG_TRAI_A:
      // Enable ToF only after the encoder launch has left the near-wall zone.
      tofAcceptanceEnabled=true;tofValid=false;
      tofSampleIndex=0;tofSampleCount=0;lastTofMs=millis();
      firstLateralStartedYmm=pose.y_mm;
      // Start a fresh lateral count here. The start marking cannot be counted;
      // point A is the second complete line.
      firstLateralGuidanceArmed=true;
      resetLateralLineHold();
      lateralHoldXMm=pose.x_mm;
      lateralHoldTofCaptured=tofValid;
      if(tofValid)lateralHoldTofMm=tofDistanceMm;
      pointATargetPairTracking=false;
      pointAMiddlePassed=false;pointAInsidePassed=false;
      pointASearchActive=false;
      pointASearchDirectionSign=selectedFieldLateralSign();
      pointAPassDirectionSign=selectedFieldLateralSign();
      rightAutoCounter.reset();leftAutoCounter.reset();stateTimeoutMs=15000;break;
    case BU_TAM_A:
      xAligned=false;autoXStableSince=0;rightLinePid.reset();leftLinePid.reset();
      pointASidePairLocked=false;
      pointAHSearchCenterXmm=pose.x_mm;
      pointAHSearchDirectionSign=1;
      resetLateralLineHold();
      stateTimeoutMs=POINT_A_H_SEARCH_TIMEOUT_MS;break;
    case CAN_YAW_A:
      yAligned=false;tofDistancePid.reset();stateTimeoutMs=8000;break;
    case CAN_A:
      autoHeadingStableSince=0;stationHeadingCommandWz=0.0f;
      fineHeadingPid.reset();stateTimeoutMs=5000;break;
    case DOI_GAP_A:
      // A mechanism wait must never inherit a final heading/line command.
      stopRobot();
      // UART TX/RX tones are emitted centrally.
      beginAutoMechanismAction("POINT_A");
      Serial.println("SAFETY,AUTO_WAIT,POINT_A,ESP32_DONE");
      stateTimeoutMs=ESP32_REPLY_TIMEOUT_MS;break;
    case DI_SANG_TRAI_B:
      // Point A is the new encoder origin. Move the measured 200 mm toward B,
      // then let the opposite 3-eye array perform the final line alignment.
      resetPose(0.0f,0.0f,currentYawDeg);
      firstLateralGuidanceArmed=true;
      resetLateralLineHold();
      lateralHoldXMm=pose.x_mm;
      lateralHoldTofCaptured=tofValid;
      if(tofValid)lateralHoldTofMm=tofDistanceMm;
      beginRelativeMove(0.0f,
          selectedFieldLateralSign()*POINT_A_TO_B_DISTANCE_MM,
          POINT_A_TO_B_SPEED_MM_S);
      rightAutoCounter.reset();leftAutoCounter.reset();stateTimeoutMs=6000;break;
    case BU_TAM_B:
      xAligned=false;autoXStableSince=0;rightLinePid.reset();leftLinePid.reset();
      fineHeadingPid.reset();stationHeadingCommandWz=0.0f;
      pointBSearchCenterYmm=pose.y_mm;
      pointBPairLostSinceMs=0;
      // The encoder leg can overshoot B. Search back toward A first, then
      // sweep both directions inside POINT_B_PAIR_SEARCH_RADIUS_MM.
      pointBSearchDirectionSign=-selectedFieldLateralSign();
      Serial.print("ACK,AUTO_POINT_B_SENSOR_GROUP,");
      Serial.println(secondPickUsesRightStopGroup()?"RIGHT":"LEFT");
      stateTimeoutMs=POINT_B_PAIR_SEARCH_TIMEOUT_MS;break;
    case CAN_YAW_B:
      yAligned=false;tofDistancePid.reset();stateTimeoutMs=8000;break;
    case CAN_B:
      autoHeadingStableSince=0;stationHeadingCommandWz=0.0f;
      fineHeadingPid.reset();stateTimeoutMs=5000;break;
    case DOI_GAP_B:
      // Hold every motor at zero while REAL waits for DONE or SIM waits 4 s.
      stopRobot();
      // UART TX/RX tones are emitted centrally.
      beginAutoMechanismAction("POINT_B");
      Serial.println("ACK,AUTO_POINT_B_VERIFIED_WAIT_ESP32_DONE");
      stateTimeoutMs=ESP32_REPLY_TIMEOUT_MS;break;
    case DI_SANG_C:
      stopArrayCountingLocked=true;
      // Verified on-floor on 2026-09-05: this drivetrain's current physical
      // mapping moves LEFT for positive relative Y. Keep this post-B test
      // independent of field mirroring.
      beginRelativeMove(0.0f,POST_B_LEFT_DISTANCE_MM,POST_B_LEFT_SPEED_MM_S);
      stateTimeoutMs=8000;break;
    case BU_TAM_C:
      beginRelativeMove(0,selectedFieldLateralSign()*C_OFFSET_MM,180);break;
    case CAN_YAW_C: break;
    case CHAY_LEN_XANH:
      linePid.reset();bridgeLineValidSince=0;
      bridgeAcquireStartPose=pose;
      bridgeAcquireSearchDirectionSign=1;
      fineHeadingPid.reset();stationHeadingCommandWz=0.0f;
      stateTimeoutMs=BRIDGE_LINE_ACQUIRE_TIMEOUT_MS;break;
    case CAN_GIUA_LINE_C:
      autoXStableSince=0;resetLateralLineHold();
      pointARightMiddleSeen=false;pointARightInsideSeen=false;
      pointARightPairStartedMs=0;
      stateTimeoutMs=POINT_A_RIGHT_SEARCH_TIMEOUT_MS;
      Serial.println("ACK,AUTO_H1_H2_ALIGNED_SEARCH_POINT_A_RIGHT");break;
    case DI_LEN_DEM_LINE:
      bridgeCrossCount=0;bridgeCrossLatched=false;bridgeCrossClearSince=0;
      bridgeCommandSpeedMmS=BRIDGE_FORWARD_SPEED_MM_S;
#if ROBOCON_AFTER_BRIDGE_TEST_BUILD
      // Standalone commissioning build: the robot is already behind the
      // bridge, so start directly in the post-descent marker-search phase.
      bridgeSlopePhase=BRIDGE_PHASE_DESCENDING;
#else
      bridgeSlopePhase=BRIDGE_PHASE_APPROACH;
#endif
      bridgeLevelRollDeg=currentRollDeg;
      bridgeLevelPitchDeg=currentPitchDeg;
      bridgeRelativeTiltDeg=0.0f;
      bridgeInclineSinceMs=0;bridgeLevelSinceMs=0;bridgeDescentSinceMs=0;
      bridgeDescentLevelSinceMs=0;
#if ROBOCON_AFTER_BRIDGE_TEST_BUILD
      bridgeEndMarkerArmed=true;
#else
      bridgeEndMarkerArmed=false;
#endif
      bridgeEndMarkerSinceMs=0;bridgeSideMarkerSinceMs=0;
      bridgeEndMarkerSource=BRIDGE_MARKER_NONE;
      bridgeEndMarkerAdvanceTargetMm=0.0f;
      bridgeEndMarkerPose=pose;
      bridgeAscentDetectedMs=0;bridgeSlopeAxis=0;
      moveCommand.active=false;
      positionErrorX=0.0f;positionErrorY=0.0f;
      linePid.reset();stateTimeoutMs=BRIDGE_FOLLOW_TIMEOUT_MS;
#if ROBOCON_AFTER_BRIDGE_TEST_BUILD
      Serial.println("ACK,AFTER_BRIDGE_TEST_LINE_FOLLOW_MARKERS_ARMED");
#endif
      break;
    case HOME_CENTER: lineCenterStableSince=0;break;
    case HOME_DROP: beginRelativeMove(HOME_TO_B_MM,0,250);break;
    case HOME_TO_B: break;
    case TIEN_34CM_B:
      beginAutoMechanismAction("THA_2B");
      stateTimeoutMs=ESP32_REPLY_TIMEOUT_MS;break;
    case DOI_THA_2B: stateTimeoutMs=ESP32_REPLY_TIMEOUT_MS;break;
    case CHO_SAU_B: stateWaitStartedMs=millis();stateTimeoutMs=2500;break;
    case LUI_33CM_A: beginRelativeMove(-RETURN_A_MM,0,250);break;
    case DOI_THA_2A:
      beginAutoMechanismAction("THA_2A");
      stateTimeoutMs=ESP32_REPLY_TIMEOUT_MS;break;
    case CHO_SAU_THA_A: stateWaitStartedMs=millis();stateTimeoutMs=2500;break;
    case LUI_3M: beginRelativeMove(-RETURN_LONG_MM,0,500);break;
    case SANG_TRAI_1M5:
      beginRelativeMove(0,-selectedFieldLateralSign()*RETURN_LATERAL_MM,500);break;
    case CAN_PHAI_LINE_B_E18:
      stopRobot();espE18BothDetected=false;postBridgeE18LineSinceMs=0;
      lastPostBridgeE18PollMs=0;
      postBridgeCoarseMoveDone=false;xAligned=false;autoXStableSince=0;
      rightLinePid.reset();leftLinePid.reset();
      fineHeadingPid.reset();stationHeadingCommandWz=0.0f;
      sendEsp("E18,ARM");
      // Physical calibration: -Y is robot-right. First make the measured
      // coarse move, then reuse point B's sensor PID for final alignment.
      beginRelativeMove(0.0f,-POST_BRIDGE_E18_RIGHT_DISTANCE_MM,
                        POST_BRIDGE_E18_RIGHT_SPEED_MM_S);
      stateTimeoutMs=POST_BRIDGE_E18_SEARCH_TIMEOUT_MS;
      Serial.println("ACK,AUTO_POST_BRIDGE_RIGHT_190MM_THEN_ALIGN_POINT_B_PAIR_E18");break;
    case TIEN_E18_70:
      pointBSearchCenterYmm=pose.y_mm;pointBSearchDirectionSign=1;
      // This state is reachable only after the required pair was confirmed.
      // Preserve that latch across the transition instead of rejecting the
      // move because of one noisy sample on the next 10 ms control tick.
      autoXStableSince=0;xAligned=true;pointBPairLostSinceMs=0;
      rightLinePid.reset();leftLinePid.reset();
      beginRelativeMove(POST_BRIDGE_E18_FORWARD_DISTANCE_MM,0.0f,
                        POST_BRIDGE_E18_FORWARD_SPEED_MM_S);
      stateTimeoutMs=8000;
      Serial.println("ACK,AUTO_LINE_PRIMARY_FORWARD_70MM_WITH_PAIR_HOLD");break;
    case DOI_DONE_THA2A:
      stopRobot();espDoneTha2A=false;doneTha2AAttempts=0;
      lastDoneTha2ASendMs=0;sendDoneTha2ARequest();
      stateTimeoutMs=DONE_THA2A_TOTAL_TIMEOUT_MS;break;
    case DI_CHEO_PHAI_THA2B:
      targetYawDeg=competitionHeadingDeg;
      headingPid.reset();fineHeadingPid.reset();
      tha2BCoarseLineArmed=false;
      tha2BCoarseClearSinceMs=0;tha2BCoarseHitSinceMs=0;
      beginRelativeMove(-POST_THA2A_BACKWARD_DISTANCE_MM,
                        -POST_THA2A_RIGHT_DISTANCE_MM,
                        POST_THA2A_DIAGONAL_SPEED_MM_S);
      stateTimeoutMs=THA2B_DIAGONAL_TIMEOUT_MS;
      Serial.println("ACK,AUTO_THA2B_MOVE_RIGHT_400_BACK_200");break;
    case CAN_LINE_DOC_PHAI_THA2B:
      stopRobot();xAligned=false;autoXStableSince=0;pointBPairLostSinceMs=0;
      // If the coarse move has just passed the line, search back toward
      // robot-left first instead of continuing farther to the right.
      pointBSearchCenterYmm=pose.y_mm;pointBSearchDirectionSign=1;
      rightLinePid.reset();fineHeadingPid.reset();
      stateTimeoutMs=THA2B_LINE_SEARCH_TIMEOUT_MS;
      Serial.println("ACK,AUTO_THA2B_ALIGN_RIGHT_MIDDLE_INSIDE");break;
    case LUI_BAM_LINE_DOC_THA2B:
      moveCommand.active=false;xAligned=true;pointBPairLostSinceMs=0;
      tha2BHorizontalSinceMs=0;tha2BHorizontalClearSinceMs=0;
      rightLinePid.reset();fineHeadingPid.reset();
      stateTimeoutMs=THA2B_REVERSE_TIMEOUT_MS;
      Serial.println("ACK,AUTO_THA2B_REVERSE_FOLLOW_VERTICAL_LINE");break;
    case LUI_RA_LINE_NGANG_THA2B:
      tha2BHorizontalClearSinceMs=0;pointBPairLostSinceMs=0;
      rightLinePid.reset();
      stateTimeoutMs=4000;
      Serial.println("ACK,AUTO_THA2B_HORIZONTAL_FOUND_REVERSE_SLOW_TO_CLEAR");break;
    case DOI_DONE_THA2B:
      stopRobot();espDoneTha2B=false;doneTha2BAttempts=0;
      lastDoneTha2BSendMs=0;sendDoneTha2BRequest();
      stateTimeoutMs=DONE_THA2B_TOTAL_TIMEOUT_MS;break;
    case SAU_THA2B_DI_CHEO_PHAI_2M_LUI_3M:
      targetYawDeg=competitionHeadingDeg;
      headingPid.reset();fineHeadingPid.reset();
      beginRelativeMove(-POST_THA2B_EXIT_REVERSE_DISTANCE_MM,
                        -POST_THA2B_EXIT_RIGHT_DISTANCE_MM,
                        POST_THA2B_EXIT_DIAGONAL_SPEED_MM_S);
      stateTimeoutMs=POST_THA2B_EXIT_DIAGONAL_TIMEOUT_MS;
      Serial.println("ACK,AUTO_POST_THA2B_DIAGONAL_RIGHT_2000_REVERSE_3000");break;
    case FINISH: armed=false;stopRobot();stateTimeoutMs=0;break;
    case FAULT_STOP: armed=false;stopRobot();stateTimeoutMs=0;break;
  }
}

void checkStateTimeout() {
  if(stateTimeoutMs&&millis()-stateStartedMs>stateTimeoutMs){
    if((state==DOI_HOME_A_READY||state==DOI_GAP_A||state==DOI_GAP_B||state==TIEN_34CM_B||state==DOI_THA_2B||state==DOI_THA_2A||state==DOI_DONE_THA2A||state==DOI_DONE_THA2B))
      setFault(FAULT_ESP_TIMEOUT,"ESP_STATE_TIMEOUT");
    else setFault(FAULT_STATE_TIMEOUT,"STATE_TIMEOUT");
    transitionTo(FAULT_STOP);
  }
}

void updateCompetition(float dt) {
  if(!stateEntered)onEnterState();
  if(state==WAIT_START||state==FINISH||state==FAULT_STOP)return;
  if(faultFlags!=FAULT_NONE){transitionTo(FAULT_STOP);return;}
  checkStateTimeout();if(state==FAULT_STOP)return;

  switch(state){
    case DOI_HOME_A_READY:
      stopRobot();
      Serial.println("ACK,AUTO_CHASSIS_START_WITH_MECHANISM");
      transitionTo(CAN_START);
      break;
    case CAN_START:
      if(updateRelativeMove(dt)){
        stopRobot();targetYawDeg=competitionHeadingDeg;
        transitionTo(DI_SANG_TRAI_A);
        Serial.println("ACK,AUTO_START_ENCODER_170MM_DONE_SEARCH_LINE_2");
      }
      break;
    case CAN_START_FAST:
      if(updateRelativeMove(dt)){
        stopRobot();targetYawDeg=competitionHeadingDeg;
        transitionTo(CAN_START_FINE);
        Serial.println("ACK,AUTO_START_ENCODER_170MM_DONE_TOF_ENABLED");
      }
      break;
    case CAN_START_FINE:{
      const bool tailOnLine=stopLine.holdNormalized[0]>=LATERAL_HOLD_LINE_THRESHOLD;
      const bool frontOnLine=stopLine.holdNormalized[1]>=LATERAL_HOLD_LINE_THRESHOLD;
      if(tailOnLine&&frontOnLine){
        setRobotVelocity(0.0f,0.0f,0.0f);
        // H1/H2 are the primary alignment sensors. ToF only confirms that the
        // detected black line is the expected one near the calibrated 160 mm.
        const bool tofConfirms=tofValid&&
          fabsf(TOF_TARGET_MM-tofDistanceMm)<=TOF_TOLERANCE_MM;
        if(tofConfirms&&!startHoldLineStableSince)
          startHoldLineStableSince=millis();
        if(!tofConfirms)startHoldLineStableSince=0;
        if(tofConfirms&&
           millis()-startHoldLineStableSince>=START_HOLD_LINE_CONFIRM_MS){
          stopRobot();
          Serial.println("ACK,AUTO,H1_H2_BLACK_LINE_FOUND_TOF_CONFIRMED");
          transitionTo(CAN_GIUA_LINE_C);
        }
      }else{
        startHoldLineStableSince=0;
        float vx=START_HOLD_LINE_SEARCH_SPEED_MM_S;
        // Once either eye has touched black, let the H1/H2 pair pull the other
        // eye onto the line. If both momentarily lose it, continue briefly in
        // the last correction direction instead of restarting a blind search.
        if(tailOnLine||frontOnLine||lateralLineHoldLastSeenMs!=0)
          vx=lateralLineHoldVelocity(dt);
        setRobotVelocity(vx,0.0f,
                         headingControl(dt,MAX_BRIDGE_HEADING_WZ_RAD_S));
      }
      break;}
    case DI_SANG_TRAI_A:
      // Find the next complete line; no H1/H2 longitudinal correction here.
      if(updateAutoPickTravel(dt,selectedFieldUsesRightStopGroup(),PICK_A_LINE_TARGET))
        transitionTo(BU_TAM_A);
      break;
    case BU_TAM_A:
      if(updateAutoPickX(dt,selectedFieldUsesRightStopGroup()))transitionTo(CAN_YAW_A);
      break;
    case CAN_YAW_A:
      // H1/H2 plus the required side-eye pair are the station reference.
      // VL53L3CX is diagnostic only here and must never block the route.
      if(!pointARequiredSensorsOnLine(selectedFieldUsesRightStopGroup())){
        transitionTo(BU_TAM_A);
      }else{
        yAligned=true;
        reportStationLineInfo("POINT_A",selectedFieldUsesRightStopGroup());
        transitionTo(CAN_A);
      }
      break;
    case CAN_A:
      // Do not let ToF/yaw correction pull the robot away from the four line
      // conditions already established at point A.
      if(!pointARequiredSensorsOnLine(selectedFieldUsesRightStopGroup())){
        transitionTo(BU_TAM_A);
        break;
      }
      setRobotVelocity(0,0,headingControlStationary(dt));
      if(autoPickVerified())transitionTo(DOI_GAP_A);
      break;
    case DOI_GAP_A:
      stopRobot();
      if(!espDone&&
         millis()-stateStartedMs>5500&&millis()-espCommandSentMs>5000)
        sendEsp("POINT_A");
      if(autoMechanismActionComplete())transitionTo(DI_SANG_TRAI_B);
      break;
    case DI_SANG_TRAI_B:
      if(updateRelativeLateralMoveWithHold(dt)){
        Serial.println("ACK,AUTO_A_TO_B_ENCODER_200MM_DONE");
        transitionTo(BU_TAM_B);
      }
      break;
    case BU_TAM_B:
      if(updateAutoPickX(dt,secondPickUsesRightStopGroup()))transitionTo(CAN_YAW_B);
      break;
    case CAN_YAW_B:{
      // Keep the selected B-side pair live while entering heading lock.
      // A short ADC/filter dropout must not restart the entire B search.
      const bool rightGroup=secondPickUsesRightStopGroup();
      if(pointBPairLostBeyondGrace(rightGroup)){
        xAligned=false;transitionTo(BU_TAM_B);
      }else if(pointBRequiredSensorsOnLine(rightGroup)){
        yAligned=true;reportStationLineInfo("POINT_B",rightGroup);
        transitionTo(CAN_B);
      }else{
        setRobotVelocity(0,0,headingControlStationary(dt));
      }
      break;}
    case CAN_B:{
      const bool rightGroup=secondPickUsesRightStopGroup();
      const bool pairOnLine=pointBRequiredSensorsOnLine(rightGroup);
      if(!pairOnLine){
        if(!pointBPairLostSinceMs)pointBPairLostSinceMs=millis();
        if(millis()-pointBPairLostSinceMs>=POINT_B_PAIR_LOST_GRACE_MS)
          xAligned=false;
      }else pointBPairLostSinceMs=0;
      // Stay inside the bounded CAN_B state and actively reacquire the pair.
      // Returning to BU_TAM_B here used to reset its timeout indefinitely.
      if(!xAligned){
        autoHeadingStableSince=0;
        if(updateAutoPickX(dt,rightGroup)){
          yAligned=true;pointBPairLostSinceMs=0;
        }
        break;
      }
      setRobotVelocity(0,0,headingControlStationary(dt));
      const bool preciseAligned=autoPickVerified();
      const float headingError=fabsf(
          shortestAngleError(targetYawDeg,currentYawDeg));
      const bool boundedAligned=
          millis()-stateStartedMs>=POINT_B_HEADING_MAX_SETTLE_MS&&
          headingError<=POINT_B_HEADING_FALLBACK_TOLERANCE_DEG&&
          fabsf(yawRateDegS)<=POINT_B_HEADING_FALLBACK_RATE_DEG_S;
      if(preciseAligned||boundedAligned){
        if(!preciseAligned)
          Serial.println("ACK,AUTO_POINT_B_HEADING_BOUNDED_ACCEPT");
        transitionTo(DOI_GAP_B);
      }
      break;}
    case DOI_GAP_B:
      stopRobot();
      if(!espDone&&
         millis()-stateStartedMs>5500&&millis()-espCommandSentMs>5000)
        sendEsp("POINT_B");
      if(autoMechanismActionComplete()){
        stopArrayCountingLocked=true;
        Serial.println("ACK,AUTO_STOP_ARRAYS_LOCKED_AFTER_POINT_B");
        transitionTo(DI_SANG_C);
      }break;
    case DI_SANG_C:
      if(updateRelativeMove(dt)){
        stopRobot();
        Serial.println("ACK,AUTO_POST_B_LEFT_1300MM_DONE_ACQUIRE_8_EYE_LINE");
        transitionTo(CHAY_LEN_XANH);
      }
      break;
    case BU_TAM_C: if(updateRelativeMove(dt))transitionTo(CAN_YAW_C);break;
    case CAN_YAW_C: if(updateYawAlignment(dt))transitionTo(DI_LEN_DEM_LINE);break;
    case CHAY_LEN_XANH:{
      if(centerLine.valid){
        const bool centered=
            fabsf(centerLine.position)<=BRIDGE_ENTRY_CENTER_TOLERANCE;
        if(centered){
          setRobotVelocity(0.0f,0.0f,headingControlStationary(dt));
          if(!bridgeLineValidSince)bridgeLineValidSince=millis();
          if(millis()-bridgeLineValidSince>=BRIDGE_LINE_ACQUIRE_CONFIRM_MS){
            stopRobot();
            Serial.println("ACK,AUTO_8_EYE_LINE_CENTERED_FOLLOW_UNTIL_END_MARKER");
            transitionTo(DI_LEN_DEM_LINE);
          }
        }else{
          bridgeLineValidSince=0;
          const float error=CENTER_LINE_CONTROL_SIGN*centerLine.position;
          const float vy=linePid.updateError(
              error,dt,-BRIDGE_ENTRY_CENTER_MAX_VY_MM_S,
              BRIDGE_ENTRY_CENTER_MAX_VY_MM_S);
          setRobotVelocity(0.0f,vy,headingControlLateral(dt));
        }
      }else{
        bridgeLineValidSince=0;
        linePid.reset();
        const float worldDx=pose.x_mm-bridgeAcquireStartPose.x_mm;
        const float worldDy=pose.y_mm-bridgeAcquireStartPose.y_mm;
        const float yaw=-bridgeAcquireStartPose.yaw_deg*PI/180.0f;
        const float localY=sinf(yaw)*worldDx+cosf(yaw)*worldDy;
        if(localY>=BRIDGE_LINE_SEARCH_RADIUS_MM)
          bridgeAcquireSearchDirectionSign=-1;
        else if(localY<=-BRIDGE_LINE_SEARCH_RADIUS_MM)
          bridgeAcquireSearchDirectionSign=1;
        setRobotVelocity(
            0.0f,
            bridgeAcquireSearchDirectionSign*BRIDGE_LINE_SEARCH_SPEED_MM_S,
            headingControlLateral(dt));
      }
      break;}
    case CAN_GIUA_LINE_C:{
      // Verified by a live isolated-array test: with only the physical RIGHT
      // group on black, normalized[3..5] read 1000 while [0..2] stayed near 0.
      // Order within the group is outside/middle/inside.
      const bool rightMiddleOnLine=
        stopLine.normalized[4]>=POINT_A_RIGHT_SENSOR_THRESHOLD;
      const bool rightInsideOnLine=
        stopLine.normalized[5]>=POINT_A_RIGHT_SENSOR_THRESHOLD;
      if(rightMiddleOnLine||rightInsideOnLine){
        if(!pointARightPairStartedMs)pointARightPairStartedMs=millis();
        pointARightMiddleSeen|=rightMiddleOnLine;
        pointARightInsideSeen|=rightInsideOnLine;
      }
      if(pointARightPairStartedMs&&
         millis()-pointARightPairStartedMs>POINT_A_RIGHT_PAIR_WINDOW_MS){
        pointARightMiddleSeen=rightMiddleOnLine;
        pointARightInsideSeen=rightInsideOnLine;
        pointARightPairStartedMs=(rightMiddleOnLine||rightInsideOnLine)?millis():0;
      }
      if(pointARightMiddleSeen&&pointARightInsideSeen){
        stopRobot();
        if(!autoXStableSince)autoXStableSince=millis();
        if(millis()-autoXStableSince>=POINT_A_RIGHT_CONFIRM_MS){
          stopRobot();
          Serial.println("ACK,AUTO_POINT_A_RIGHT_MIDDLE_INSIDE_FOUND");
          transitionTo(DOI_GAP_A);
        }
      }else{
        autoXStableSince=0;
        const float holdVx=lateralLineHoldVelocity(dt);
        setRobotVelocity(holdVx,
          selectedFieldLateralSign()*POINT_A_RIGHT_ALIGN_SPEED_MM_S,
          headingControlLateral(dt));
      }
      break;}
    case DI_LEN_DEM_LINE:
      if(updateBridgeLineUntilEndMarker(dt)){
        stopRobot();
        Serial.println("ACK,AUTO_BRIDGE_END_BLACK_MARKER_DETECTED_STOP");
        transitionTo(CAN_PHAI_LINE_B_E18);
      }
      break;
    case CAN_PHAI_LINE_B_E18:{
      const bool rightGroup=secondPickUsesRightStopGroup();
      if(millis()-lastPostBridgeE18PollMs>=POST_BRIDGE_E18_POLL_MS){
        sendMechanismFrame("GET,E18",false);
        lastPostBridgeE18PollMs=millis();
      }
      if(!postBridgeCoarseMoveDone){
        // The point-B eye pair is authoritative even during the 190 mm
        // encoder move. Stop lateral travel as soon as both eyes are stable
        // on black, then advance immediately without waiting for 190 mm.
        if(pointBRequiredSensorsOnLine(rightGroup)){
          stopRobot();
          if(!postBridgeE18LineSinceMs)postBridgeE18LineSinceMs=millis();
          if(millis()-postBridgeE18LineSinceMs>=POST_BRIDGE_E18_LINE_CONFIRM_MS){
            Serial.print("ACK,AUTO_POINT_B_PAIR_EARLY_CONFIRMED,E18_AUX,");
            Serial.println(espE18BothDetected?1:0);
            espE18BothDetected=false;moveCommand.active=false;xAligned=true;
            sendMechanismFrame("E18,DISARM");
            transitionTo(TIEN_E18_70);
          }
          break;
        }
        postBridgeE18LineSinceMs=0;
        if(updateRelativeMove(dt)){
          stopRobot();postBridgeCoarseMoveDone=true;
          pointBSearchCenterYmm=pose.y_mm;pointBPairLostSinceMs=0;
          // The 190 mm coarse move may pass the line. Search back first,
          // exactly like the point-B arrival path.
          pointBSearchDirectionSign=1;
          autoXStableSince=0;rightLinePid.reset();leftLinePid.reset();
          Serial.print("ACK,AUTO_POST_BRIDGE_190MM_DONE_ALIGN_GROUP,");
          Serial.println(rightGroup?"RIGHT":"LEFT");
        }
        break;
      }

      if(!xAligned){
        if(updateAutoPickX(dt,rightGroup)){
          xAligned=true;postBridgeE18LineSinceMs=0;
          Serial.println("ACK,AUTO_POST_BRIDGE_POINT_B_PAIR_ALIGNED_WAIT_E18");
        }
        break;
      }

      const bool pointBPairStillOnLine=pointBRequiredSensorsOnLine(rightGroup);
      if(!pointBPairStillOnLine){
        xAligned=false;autoXStableSince=0;postBridgeE18LineSinceMs=0;
        Serial.println("ACK,AUTO_POST_BRIDGE_POINT_B_PAIR_LOST_REALIGN");
        break;
      }

      // Line is authoritative. E18 remains visible in diagnostics but cannot
      // prevent the robot from continuing after the point-B pair is aligned.
      stopRobot();
      if(!postBridgeE18LineSinceMs)postBridgeE18LineSinceMs=millis();
      if(millis()-postBridgeE18LineSinceMs>=POST_BRIDGE_E18_LINE_CONFIRM_MS){
        Serial.print("ACK,AUTO_POINT_B_PAIR_CONFIRMED,E18_AUX,");
        Serial.println(espE18BothDetected?1:0);
        espE18BothDetected=false;moveCommand.active=false;
        sendMechanismFrame("E18,DISARM");
        transitionTo(TIEN_E18_70);
      }
      break;}
    case TIEN_E18_70:
      if(updateForwardKeepingPointBPair(
             dt,secondPickUsesRightStopGroup())){
        stopRobot();Serial.println("ACK,AUTO_POST_BRIDGE_FORWARD_70MM_DONE");
        transitionTo(DOI_DONE_THA2A);
      }
      break;
    case DOI_DONE_THA2A:
      stopRobot();
      if(espDoneTha2A){
        Serial.println("ACK,AUTO_DONE_THA2A_HANDSHAKE_COMPLETE");
        transitionTo(DI_CHEO_PHAI_THA2B);
      }else if(millis()-lastDoneTha2ASendMs>=DONE_THA2A_RETRY_INTERVAL_MS){
        if(doneTha2AAttempts<DONE_THA2A_MAX_ATTEMPTS){
          sendDoneTha2ARequest();
        }else{
          setFault(FAULT_ESP_TIMEOUT,"DONE_THA2A_RETRY_EXHAUSTED");
          transitionTo(FAULT_STOP);
        }
      }
      break;
    case DI_CHEO_PHAI_THA2B:
      if(updateTha2BCoarseRightLineLatch()){
        moveCommand.active=false;stopRobot();
        Serial.println("ACK,AUTO_THA2B_RIGHT_PAIR_EDGE_CAUGHT_DURING_COARSE_MOVE");
        transitionTo(CAN_LINE_DOC_PHAI_THA2B);
      }else if(updateRelativeMove(dt)){
        Serial.println("ACK,AUTO_THA2B_COARSE_DISTANCE_DONE_START_PAIR_SEARCH");
        transitionTo(CAN_LINE_DOC_PHAI_THA2B);
      }
      break;
    case CAN_LINE_DOC_PHAI_THA2B:
      if(updateAutoPickX(dt,true)){
        xAligned=true;pointBPairLostSinceMs=0;
        Serial.println("ACK,AUTO_THA2B_RIGHT_PAIR_ON_VERTICAL_LINE");
        transitionTo(LUI_BAM_LINE_DOC_THA2B);
      }
      break;
    case LUI_BAM_LINE_DOC_THA2B:
      if(!updateReverseKeepingRightPair(dt,THA2B_LINE_REVERSE_SPEED_MM_S)){
        Serial.println("ACK,AUTO_THA2B_RIGHT_PAIR_LOST_REALIGN");
        transitionTo(CAN_LINE_DOC_PHAI_THA2B);
        break;
      }
      if(tha2BHorizontalMarkerPresent()){
        if(!tha2BHorizontalSinceMs)tha2BHorizontalSinceMs=millis();
        if(millis()-tha2BHorizontalSinceMs>=THA2B_HORIZONTAL_CONFIRM_MS){
          stopRobot();
          Serial.println("ACK,AUTO_THA2B_BOTH_SIDE_GROUPS_ON_HORIZONTAL");
          transitionTo(LUI_RA_LINE_NGANG_THA2B);
        }
      }else tha2BHorizontalSinceMs=0;
      break;
    case LUI_RA_LINE_NGANG_THA2B:
      if(!updateReverseKeepingRightPair(dt,THA2B_LINE_CLEAR_SPEED_MM_S)){
        Serial.println("ACK,AUTO_THA2B_RIGHT_PAIR_LOST_WHILE_CLEARING");
        transitionTo(CAN_LINE_DOC_PHAI_THA2B);
        break;
      }
      if(tha2BHorizontalMarkerCleared()){
        if(!tha2BHorizontalClearSinceMs)tha2BHorizontalClearSinceMs=millis();
        if(millis()-tha2BHorizontalClearSinceMs>=
           THA2B_HORIZONTAL_CLEAR_CONFIRM_MS){
          stopRobot();
          Serial.println("ACK,AUTO_THA2B_BOTH_SIDE_GROUPS_JUST_CLEARED_STOP");
          transitionTo(DOI_DONE_THA2B);
        }
      }else tha2BHorizontalClearSinceMs=0;
      break;
    case DOI_DONE_THA2B:
      stopRobot();
      if(espDoneTha2B){
        Serial.println("ACK,AUTO_DONE_THA2B_HANDSHAKE_COMPLETE_START_EXIT");
        transitionTo(SAU_THA2B_DI_CHEO_PHAI_2M_LUI_3M);
      }else if(millis()-lastDoneTha2BSendMs>=DONE_THA2B_RETRY_INTERVAL_MS){
        if(doneTha2BAttempts<DONE_THA2B_MAX_ATTEMPTS){
          sendDoneTha2BRequest();
        }else{
          setFault(FAULT_ESP_TIMEOUT,"DONE_THA2B_RETRY_EXHAUSTED");
          transitionTo(FAULT_STOP);
        }
      }
      break;
    case SAU_THA2B_DI_CHEO_PHAI_2M_LUI_3M:
      if(updateRelativeMove(dt))transitionTo(FINISH);
      break;
    case HOME_CENTER:{
      if(!centerLine.valid){lineCenterStableSince=0;setRobotVelocity(0,centerLine.lastPosition<0?-60:60,headingControl(dt));break;}
      const float vy=constrain(
          CENTER_LINE_CONTROL_SIGN*centerLine.position*0.08f,-100.0f,100.0f);
      setRobotVelocity(0,vy,headingControl(dt,MAX_LINE_ALIGN_WZ_RAD_S));
      if(fabsf(centerLine.position)<100){if(!lineCenterStableSince)lineCenterStableSince=millis();}
      else lineCenterStableSince=0;
      if(lineCenterStableSince&&millis()-lineCenterStableSince>180){stopRobot();resetPose();transitionTo(HOME_DROP);}
      break;}
    case HOME_DROP: transitionTo(HOME_TO_B);break;
    case HOME_TO_B: if(updateRelativeMoveWithLine(dt))transitionTo(TIEN_34CM_B);break;
    case TIEN_34CM_B: transitionTo(DOI_THA_2B);break;
    case DOI_THA_2B:
      if(autoMechanismActionComplete())transitionTo(CHO_SAU_B);
      break;
    case CHO_SAU_B: if(millis()-stateWaitStartedMs>=1000)transitionTo(LUI_33CM_A);break;
    case LUI_33CM_A: if(updateRelativeMoveWithLine(dt))transitionTo(DOI_THA_2A);break;
    case DOI_THA_2A:
      if(autoMechanismActionComplete())transitionTo(CHO_SAU_THA_A);
      break;
    case CHO_SAU_THA_A: if(millis()-stateWaitStartedMs>=1000)transitionTo(LUI_3M);break;
    case LUI_3M: if(updateRelativeMove(dt))transitionTo(SANG_TRAI_1M5);break;
    case SANG_TRAI_1M5: if(updateRelativeMove(dt))transitionTo(FINISH);break;
    default: break;
  }
}

// ============================================================
// Debug telemetry at 10 Hz
// ============================================================

void printAutoTelemetrySnapshotStage() {
  if(autoTelemetrySnapshotStage==0)return;
  const uint32_t sequence=autoTelemetrySnapshotSequence;
  if(autoTelemetrySnapshotStage==1){
    Serial.print("AUTO_SNAPSHOT,");Serial.print(sequence);Serial.print(',');
    Serial.print(millis());Serial.print(',');Serial.print(stateName(state));Serial.print(',');
    Serial.print(faultFlags);Serial.print(',');Serial.print(armed?1:0);Serial.print(',');
    Serial.print(robotVx,1);Serial.print(',');Serial.print(robotVy,1);Serial.print(',');
    Serial.print(robotWz,3);Serial.print(',');Serial.print(pose.x_mm,1);Serial.print(',');
    Serial.print(pose.y_mm,1);Serial.print(',');Serial.print(pose.yaw_deg,2);Serial.print(',');
    Serial.print(currentYawDeg,2);Serial.print(',');Serial.print(targetYawDeg,2);Serial.print(',');
    Serial.print(tofDistanceMm,1);Serial.print(',');Serial.println(tofValid?1:0);
  }else if(autoTelemetrySnapshotStage==2){
    const float rpmFactor=60.0f/(PI*WHEEL_DIAMETER_MM);
    Serial.print("AUTO_WHEELS,");Serial.print(sequence);
    for(uint8_t i=0;i<4;i++){
      Serial.print(',');Serial.print(wheels[i].targetMmS*rpmFactor,1);
      Serial.print(',');Serial.print(wheels[i].measuredMmS*rpmFactor,1);
      Serial.print(',');Serial.print(wheels[i].pwm);
      Serial.print(',');Serial.print(wheels[i].count);
    }
    Serial.println();
  }else if(autoTelemetrySnapshotStage==3){
    Serial.print("AUTO_SENSORS,");Serial.print(sequence);Serial.print(',');
    Serial.print(centerLine.position,1);Serial.print(',');Serial.print(centerLine.valid?1:0);
    Serial.print(',');Serial.print(centerLine.activeCount);
    for(uint8_t i=0;i<6;i++){Serial.print(',');Serial.print(stopLine.normalized[i]);}
    Serial.print(',');Serial.print(stopLine.holdNormalized[0]);
    Serial.print(',');Serial.print(stopLine.holdNormalized[1]);
    Serial.print(',');Serial.print(rightAutoCounter.count);
    Serial.print(',');Serial.print(leftAutoCounter.count);
    Serial.print(',');Serial.print(bridgeAutoCounter.count);
    Serial.print(',');Serial.print(xAligned?1:0);
    Serial.print(',');Serial.println(yAligned?1:0);
  }else{
    Serial.print("AUTO_TELEM,SNAPSHOT_END,");Serial.println(sequence);
    autoTelemetrySnapshotStage=0;
    return;
  }
  ++autoTelemetrySnapshotStage;
}

void printTelemetry() {
  // During any armed AUTO operation, suppress the large periodic burst. A
  // requested snapshot is emitted as one short record per loop iteration so
  // Serial5 cannot starve the 100 Hz sensor/control loop.
  if(controlMode==CONTROL_AUTO&&armed){
    if(autoTelemetryMode==AUTO_TELEMETRY_ON_REQUEST)
      printAutoTelemetrySnapshotStage();
    else autoTelemetrySnapshotStage=0;
    return;
  }
  if(millis()-lastTelemetryMs<TELEMETRY_PERIOD_MS)return;
  lastTelemetryMs=millis();
  if(DEBUG_TELEMETRY){
    Serial.print("DBG,STATE=");Serial.print(stateName(state));
    Serial.print(",V=");Serial.print(robotVx,1);Serial.print(',');Serial.print(robotVy,1);Serial.print(',');Serial.print(robotWz,3);
    Serial.print(",WT=");for(uint8_t i=0;i<4;i++){if(i)Serial.print('/');Serial.print(wheels[i].targetMmS,1);}
    Serial.print(",WS=");for(uint8_t i=0;i<4;i++){if(i)Serial.print('/');Serial.print(wheels[i].measuredMmS,1);}
    Serial.print(",PWM=");for(uint8_t i=0;i<4;i++){if(i)Serial.print('/');Serial.print(wheels[i].pwm);}
    Serial.print(",ENC=");for(uint8_t i=0;i<4;i++){if(i)Serial.print('/');Serial.print(wheels[i].count);}
    Serial.print(",YAW=");Serial.print(currentYawDeg,2);Serial.print('/');Serial.print(targetYawDeg,2);Serial.print('/');Serial.print(shortestAngleError(targetYawDeg,currentYawDeg),2);
    Serial.print(",CENTER=");Serial.print(centerLine.position,1);Serial.print('/');Serial.print(centerLine.valid);
    Serial.print(",STOP=");Serial.print(stopLine.leftPosition,1);Serial.print('/');Serial.print(stopLine.rightPosition,1);
    Serial.print(",FLOW=");Serial.print(flow.deltaX,1);Serial.print('/');Serial.print(flow.deltaY,1);Serial.print('/');Serial.print(flow.quality,0);Serial.print('/');Serial.print(flow.valid);
    Serial.print(",TOF=");Serial.print(tofDistanceMm,1);
    Serial.print(",POSE=");Serial.print(pose.x_mm,1);Serial.print('/');Serial.print(pose.y_mm,1);Serial.print('/');Serial.print(pose.yaw_deg,2);
    Serial.print(",FAULT=");Serial.println(faultFlags);
  }

  if(testMode==TEST_CENTER_LINE){
    Serial.print("LINE_CENTER_RAW");
    for(uint8_t i=0;i<8;i++){Serial.print(',');Serial.print(centerLine.raw[i]);}
    Serial.println();
    Serial.print("LINE_CENTER_NORM");
    for(uint8_t i=0;i<8;i++){Serial.print(',');Serial.print(centerLine.normalized[i]);}
    Serial.print(',');Serial.print(centerLine.position,1);Serial.print(',');
    Serial.print(centerLine.valid?1:0);Serial.print(',');
    Serial.print(centerLine.activeCount);Serial.print(',');
    Serial.println(static_cast<uint8_t>(centerLine.state));
    if(calibrationActive&&calibrationCenterOnly)printCenterCalibrationTelemetry();
  }else if(testMode==TEST_STOP_LINE){
    Serial.print("LINE_STOP_RAW");
    for(uint8_t i=0;i<6;i++){Serial.print(',');Serial.print(stopLine.raw[i]);}
    Serial.println();
    Serial.print("LINE_STOP_NORM");
    for(uint8_t i=0;i<6;i++){Serial.print(',');Serial.print(stopLine.normalized[i]);}
    Serial.print(',');Serial.print(stopLine.leftPosition,1);Serial.print(',');
    Serial.print(stopLine.leftValid?1:0);Serial.print(',');
    Serial.print(stopLine.rightPosition,1);Serial.print(',');
    Serial.println(stopLine.rightValid?1:0);
    Serial.print("LINE_HOLD_RAW,");Serial.print(stopLine.holdRaw[0]);
    Serial.print(',');Serial.println(stopLine.holdRaw[1]);
    Serial.print("LINE_HOLD_NORM,");Serial.print(stopLine.holdNormalized[0]);
    Serial.print(',');Serial.println(stopLine.holdNormalized[1]);
    if(calibrationActive&&calibrationStopOnly)printStopCalibrationTelemetry();
  }

  // RBT/1-compatible compact stream for the existing pid-tuner interface.
  const float rpmFactor=60.0f/(PI*WHEEL_DIAMETER_MM);
  Serial.print("TEL,");Serial.print(millis());
  for(uint8_t i=0;i<4;i++){Serial.print(',');Serial.print(wheels[i].measuredMmS*rpmFactor,2);}
  Serial.print(',');Serial.print(currentYawDeg,2);
  Serial.print(',');Serial.print(centerLine.position/3500.0f,3);
  Serial.println(",0.0"); // Battery input is not assigned in the supplied hardware map.
  for(uint8_t i=0;i<4;i++){
    Serial.print("WHEEL,");Serial.print(tunerWheelName(i));Serial.print(',');
    Serial.print(wheelMmSToRpm(wheels[i].targetMmS),2);Serial.print(',');
    Serial.print(wheelMmSToRpm(wheels[i].measuredMmS),2);Serial.print(',');
    Serial.print(wheels[i].pwm);Serial.print(',');Serial.println(wheels[i].count);
  }
  Serial.print("POSE,");Serial.print(pose.x_mm,1);Serial.print(',');
  Serial.print(pose.y_mm,1);Serial.print(',');Serial.print(pose.yaw_deg,2);
  Serial.print(',');Serial.println(moveCommand.active?1:0);
  Serial.print("SYS,");Serial.print(stateName(state));Serial.print(',');
  Serial.print(faultFlags);Serial.print(',');Serial.print(armed?1:0);Serial.print(',');
  Serial.print(static_cast<uint8_t>(testMode));Serial.print(',');
  Serial.print(controlMode==CONTROL_AUTO?"AUTO":"MANUAL");Serial.print(',');
  Serial.print(selectedField==FIELD_BLUE?"BLUE":"RED");Serial.print(',');
  Serial.println(softwareEStopLatched?1:0);
  Serial.print("HDG,");Serial.print(currentYawDeg,2);Serial.print(',');
  Serial.print(targetYawDeg,2);Serial.print(',');
  Serial.print(shortestAngleError(targetYawDeg,currentYawDeg),2);Serial.print(',');
  Serial.print(bnoValid?1:0);Serial.print(',');Serial.println(bnoInitialized?1:0);
  Serial.print("TILT,");Serial.print(currentRollDeg,2);Serial.print(',');
  Serial.print(currentPitchDeg,2);Serial.print(',');
  Serial.print(bridgeRelativeTiltDeg,2);Serial.print(',');
  Serial.print(static_cast<uint8_t>(bridgeSlopePhase));Serial.print(',');
  Serial.println(bridgeSlopeAxis);
  Serial.print("TOF,");Serial.print(tofDistanceMm,1);Serial.print(',');
  Serial.print(tofValid?1:0);Serial.print(',');Serial.print(TOF_TARGET_MM);Serial.print(',');
  Serial.println(TOF_TARGET_MM-tofDistanceMm,1);
  Serial.print("TOFI,");Serial.print(tofInitModeStatus);Serial.print(',');
  Serial.print(tofInitBudgetStatus);Serial.print(',');Serial.print(tofInitStartStatus);
  Serial.print(',');Serial.println(tofInitialized?1:0);
  Serial.print("STOP6");
  for(uint8_t i=0;i<6;i++){Serial.print(',');Serial.print(stopLine.normalized[i]);}
  Serial.println();
  Serial.print("HOLD2,");Serial.print(stopLine.holdNormalized[0]);
  Serial.print(',');Serial.println(stopLine.holdNormalized[1]);
  Serial.print("AUTO,");Serial.print(rightAutoCounter.count);Serial.print(',');
  Serial.print(leftAutoCounter.count);Serial.print(',');Serial.print(bridgeAutoCounter.count);
  Serial.print(',');Serial.print(xAligned?1:0);Serial.print(',');Serial.print(yAligned?1:0);
  Serial.print(',');Serial.print(lastEspCommand);Serial.print(',');Serial.print(lastEspResponse);
  Serial.print(',');Serial.print(bridgeCrossCount);Serial.print(',');
  Serial.println(BRIDGE_STOP_CROSS_TARGET);
  printAutoMechanismStatus();
  printMechanismTelemetryStatus();
  if(state==DI_SANG_TRAI_A||state==BU_TAM_A||
     state==DI_SANG_TRAI_B||state==BU_TAM_B||
     state==CAN_YAW_B||state==CAN_B||
     state==CAN_PHAI_LINE_B_E18||state==TIEN_E18_70){
    const bool rightGroup=(state==DI_SANG_TRAI_A||state==BU_TAM_A)
                            ?selectedFieldUsesRightStopGroup()
                            :secondPickUsesRightStopGroup();
    const uint8_t offset=rightGroup?3:0;
    Serial.print("PICK_SCAN,");Serial.print(stateName(state));Serial.print(',');
    Serial.print(rightGroup?"RIGHT":"LEFT");Serial.print(',');
    Serial.print(stopLine.normalized[offset]);Serial.print(',');
    Serial.print(stopLine.normalized[offset+1]);Serial.print(',');
    Serial.print(stopLine.normalized[offset+2]);Serial.print(',');
    Serial.print(rightGroup?stopLine.rightPosition:stopLine.leftPosition,1);
    Serial.print(',');Serial.print(stopGroupActiveCount(offset));
    Serial.print(',');Serial.println(robotVy,2);
    if(state==CAN_PHAI_LINE_B_E18||state==TIEN_E18_70){
      Serial.print("POST_E18,");Serial.print(postBridgeCoarseMoveDone?1:0);
      Serial.print(',');Serial.print(xAligned?1:0);
      Serial.print(',');Serial.print(pointBRequiredSensorsOnLine(rightGroup)?1:0);
      Serial.print(',');Serial.println(espE18BothDetected?1:0);
    }
  }
  if(state==DI_SANG_C||state==CHAY_LEN_XANH||state==DI_LEN_DEM_LINE){
    Serial.print("BRIDGE_SCAN,");Serial.print(stateName(state));Serial.print(',');
    Serial.print(bridgeAutoCounter.count);Serial.print(',');
    Serial.print(centerLine.activeCount);Serial.print(',');
    Serial.print(centerLine.cross?1:0);Serial.print(',');
    Serial.print(centerLine.valid?1:0);Serial.print(',');
    Serial.print(centerLine.position,1);
    for(uint8_t i=0;i<8;i++){
      Serial.print(',');Serial.print(centerLine.normalized[i]);
    }
    Serial.println();
  }
  bool motionActive=moveCommand.active||fabsf(robotVx)>0.5f||fabsf(robotVy)>0.5f||fabsf(robotWz)>0.002f;
  for(uint8_t i=0;i<4;i++)motionActive=motionActive||fabsf(wheels[i].targetMmS)>0.5f;
  Serial.print("MOTION,");Serial.print(millis());Serial.print(',');
  Serial.print(motionActive?1:0);Serial.print(',');
  Serial.print(robotVx,2);Serial.print(',');Serial.print(robotVy,2);Serial.print(',');
  Serial.print(robotWz,4);Serial.print(',');Serial.print(positionErrorX,2);Serial.print(',');
  Serial.println(positionErrorY,2);
}

// ============================================================
// Physical field/start buttons and buzzer
// ============================================================

void confirmPhysicalField(FieldSide field) {
  selectedField = field;
  buzzer.beep(field == FIELD_RED ? 2 : 3);
  Serial.print("ACK,FIELD,PHYSICAL,");
  Serial.println(field == FIELD_RED ? "RED" : "BLUE");
}

void startAutoFromPhysicalButton() {
  buzzer.beep(1);
  if (pendingPhysicalFieldClicks == 1) {
    pendingPhysicalFieldClicks = 0;
    confirmPhysicalField(FIELD_RED);
  }
  if (armed || state != WAIT_START) {
    Serial.println("ERR,PHYSICAL_START,ROBOT_NOT_IDLE");
    return;
  }
  if (stopInputActive()) {
    Serial.println("ERR,STOP_ACTIVE");
    return;
  }
  if (faultFlags != FAULT_NONE) {
    Serial.println("ERR,RESET_REQUIRED");
    return;
  }
  if (!bnoValid) {
    Serial.println("ERR,PHYSICAL_START,BNO085_NOT_VALID");
    return;
  }
  if (!competitionConfigurationValid()) {
    setFault(FAULT_CONFIG, "START_CONFIG_LOCKED");
    return;
  }

  controlMode = CONTROL_AUTO;
  supervisedAutoRun = false;
  testMode = TEST_NONE;
  armed = true;
  resetAllControllers();
  resetPose();
  competitionHeadingDeg = currentYawDeg;
  targetYawDeg = competitionHeadingDeg;
#if ROBOCON_AFTER_BRIDGE_TEST_BUILD
  transitionTo(DI_LEN_DEM_LINE);
#else
  transitionTo(DOI_HOME_A_READY);
#endif
  printAutoStartConfiguration("PHYSICAL_BUTTON_PIN34");
  Serial.println("SAFETY,AUTO_STARTED");
#if ROBOCON_AFTER_BRIDGE_TEST_BUILD
  Serial.println("ACK,PHYSICAL_START_AFTER_BRIDGE_TEST");
#endif
}

void updatePhysicalControls() {
  buzzer.update();
  if (!PHYSICAL_CONTROL_BUTTONS_ENABLED) return;
  const uint32_t now = millis();

  if (pendingPhysicalFieldClicks == 1 &&
      now - lastPhysicalFieldClickMs >= PHYSICAL_FIELD_DOUBLE_CLICK_MS) {
    pendingPhysicalFieldClicks = 0;
    confirmPhysicalField(FIELD_RED);
  }

  if (physicalFieldButton.pressed()) {
    if (armed || state != WAIT_START) {
      pendingPhysicalFieldClicks = 0;
      Serial.println("ERR,PHYSICAL_FIELD,DISARM_REQUIRED");
      buzzer.alignmentAlarm();
    } else if (pendingPhysicalFieldClicks == 1 &&
               now - lastPhysicalFieldClickMs < PHYSICAL_FIELD_DOUBLE_CLICK_MS) {
      pendingPhysicalFieldClicks = 0;
      confirmPhysicalField(FIELD_BLUE);
    } else {
      pendingPhysicalFieldClicks = 1;
      lastPhysicalFieldClickMs = now;
    }
  }

  if (physicalStartButton.pressed()) startAutoFromPhysicalButton();
}

// ============================================================
// Setup and cooperative loop
// ============================================================

void setup() {
  beginUsbCommandStream();
  Serial.setRX(TELEMETRY_RX_PIN);
  Serial.setTX(TELEMETRY_TX_PIN);
  Serial.begin(TELEMETRY_BAUD);
  if(ESP32_UART_ENABLED){
    ESP32_UART.setRX(ESP32_RX_PIN);
    ESP32_UART.setTX(ESP32_TX_PIN);
    mechanismClient.begin(ESP32_RX_PIN,ESP32_TX_PIN,ESP32_BAUD);
  }
  if (OPTICAL_FLOW_ENABLED) FLOW_UART.begin(FLOW_BAUD);

  buzzer.begin();
  if(PHYSICAL_CONTROL_BUTTONS_ENABLED){
    physicalStartButton.begin(PHYSICAL_START_BUTTON_PIN);
    physicalFieldButton.begin(PHYSICAL_FIELD_BUTTON_PIN);
  }
  pinMode(MCP_CENTER_CS,OUTPUT);pinMode(MCP_STOP_CS,OUTPUT);
  digitalWrite(MCP_CENTER_CS,HIGH);digitalWrite(MCP_STOP_CS,HIGH);
  SPI.begin();

  analogWriteResolution(8);
  for(uint8_t i=0;i<4;i++){
    WheelControl &wheel=wheels[i];
    pinMode(wheel.rpwm,OUTPUT);pinMode(wheel.lpwm,OUTPUT);
    analogWriteFrequency(wheel.rpwm,PWM_FREQUENCY_HZ);
    analogWriteFrequency(wheel.lpwm,PWM_FREQUENCY_HZ);
    wheel.pid=PID(WHEEL_KP[i],WHEEL_KI[i],WHEEL_KD[i],WHEEL_INTEGRAL_LIMIT);
    wheel.kff=WHEEL_KFF[i];wheel.deadzone=WHEEL_DEADZONE_PWM[i];
    wheel.previousCount=wheel.encoder->read()*wheel.encoderSign;
    wheel.lastEncoderMotionMs=millis();writeMotor(wheel,0);
  }
  applyFactoryPidConfig();

  memcpy(centerCalMin,CENTER_SENSOR_MIN,sizeof(centerCalMin));
  memcpy(centerCalMax,CENTER_SENSOR_MAX,sizeof(centerCalMax));
  memcpy(stopCalMin,STOP_SENSOR_MIN,sizeof(stopCalMin));
  memcpy(stopCalMax,STOP_SENSOR_MAX,sizeof(stopCalMax));

  Wire1.begin();Wire1.setClock(SENSOR_I2C_HZ);
  for(uint8_t attempt=0;attempt<3&&!bnoInitialized;attempt++){
    initializeBno085();
    if(!bnoInitialized){
      const uint32_t retryAt=millis()+200;
      while(static_cast<int32_t>(millis()-retryAt)<0)yield();
    }
  }
  const uint32_t imuWait=millis();
  while(bnoInitialized&&!bnoValid&&millis()-imuWait<500)updateBno085();
  if(bnoValid){yawReferenceDeg=currentYawDeg;currentYawDeg=0;targetYawDeg=0;}

  if(TOF_ENABLED){
    Wire1.setClock(SENSOR_I2C_HZ);
    tof.begin(Wire1);
    const VL53LX_Error modeStatus=tof.setDistanceMode(DIST_LONG);
    const VL53LX_Error budgetStatus=tof.setTimingBudget(100000);
    const VL53LX_Error startStatus=tof.startMeasurement();
    tofInitModeStatus = static_cast<int16_t>(modeStatus);
    tofInitBudgetStatus = static_cast<int16_t>(budgetStatus);
    tofInitStartStatus = static_cast<int16_t>(startStatus);
    tofInitialized=modeStatus==VL53LX_ERROR_NONE&&
                   budgetStatus==VL53LX_ERROR_NONE&&
                   startStatus==VL53LX_ERROR_NONE;
  }

  resetPose();lastControlUs=micros();
  Serial.println("ROBOCON_MECANUM_BOOT");
#if ROBOCON_AFTER_BRIDGE_TEST_BUILD
  Serial.println("BUILD=AFTER_BRIDGE_TEST");
#else
  Serial.println("BUILD=COMPETITION");
#endif
  Serial.print("TELEMETRY=Serial5,TX20,RX21,BAUD=");Serial.println(TELEMETRY_BAUD);
  Serial.print("ESP32_UART=");
  if(ESP32_UART_ENABLED){
    Serial.print("Serial6,TX24,RX25,BAUD=");Serial.println(ESP32_BAUD);
  }else Serial.println("DISABLED_PIN_CONFLICT");
  Serial.println("PID_CONFIG=ROBOTCONFIG_H_RAM_ONLY");
  Serial.print("CONFIG_VERIFIED=");Serial.println(HARDWARE_CONFIG_VERIFIED);
  Serial.print("BNO085=");Serial.print(bnoInitialized);Serial.print(",TOF=");Serial.println(tofInitialized);
  if(!HARDWARE_CONFIG_VERIFIED||!dimensionsValid())
    Serial.println("MOTION_LOCKED: fill RobotConfig.h then set HARDWARE_CONFIG_VERIFIED=true");
}

void loop() {
  updateUsbCommands();updateEspProtocol();updateOpticalFlow();updateBno085();updateTof();

  updatePhysicalControls();

  const uint32_t nowUs=micros();
  if(nowUs-lastControlUs>=CONTROL_PERIOD_US){
    const float dt=(nowUs-lastControlUs)*1.0e-6f;lastControlUs=nowUs;
    updateLineArrays();updateEncoderMeasurements(dt);updateOdometry(dt);
    if(armed&&testMode!=TEST_WHEEL_TUNE&&testMode!=TEST_MANUAL_DRIVE&&!bnoValid)
      setFault(FAULT_IMU_TIMEOUT,"BNO085_TIMEOUT");
    if(supervisedAutoRun&&millis()-lastTunerHeartbeatMs>POSITION_LINK_WATCHDOG_MS){
      supervisedAutoRun=false;armed=false;moveCommand.active=false;stopRobot();
      transitionTo(WAIT_START);Serial.println("ERR,AUTO_TEST_HEARTBEAT_TIMEOUT");
    }
    updateTestMode(dt);
    if(testMode==TEST_NONE)updateCompetition(dt);
    updateWheelSpeedPid(dt);
  }
  printTelemetry();
}

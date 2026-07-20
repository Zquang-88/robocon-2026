/*
 * ROBOCON 2026 - AUTO ROBOT / TEENSY 4.1
 * ================================================================
 * Chuc nang:
 *   - Dieu khien 4 banh omni bang PID toc do (don vi RPM).
 *   - Giu heading bang BNO055.
 *   - Bam line 8 mat qua MCP3008.
 *   - State machine AUTO mau, de thay bang kich ban thi dau.
 *   - Truyen/nhan PID va telemetry qua radio SiK/3DR (UART).
 *
 * Ket noi telemetry:
 *   USB telemetry (PC)  <~~~ radio ~~~>  Air telemetry (robot)
 *   Air TX -> Teensy RX5 pin 34
 *   Air RX -> Teensy TX5 pin 33
 *   Air GND -> Teensy GND
 *
 * Giao thuc day du: pid-tuner/TELEMETRY_UART_PROTOCOL.md
 * ================================================================
 */

#include <Arduino.h>
#include <Encoder.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// ================================================================
// 1. CAU HINH HE THONG
// ================================================================
namespace Config {
constexpr uint32_t DEBUG_BAUD = 115200;
constexpr uint32_t TELEMETRY_BAUD = 57600;

constexpr uint32_t CONTROL_PERIOD_US = 10000;   // 100 Hz
constexpr uint32_t TELEMETRY_PERIOD_MS = 20;    // 50 Hz
constexpr uint32_t RADIO_TIMEOUT_MS = 1500;

// Sua theo encoder thuc te: xung/vong sau khi tinh quadrature.
constexpr float ENCODER_COUNTS_PER_REV = 2048.0f;
constexpr float MAX_WHEEL_RPM = 180.0f;
constexpr float MAX_PWM = 255.0f;
constexpr float INTEGRAL_LIMIT = 400.0f;

constexpr int LINE_THRESHOLD = 500;
constexpr int LINE_LOST = 9999;
constexpr uint8_t MCP3008_CS = 10;

constexpr uint8_t FL_RPWM = 36;
constexpr uint8_t FL_LPWM = 37;
constexpr uint8_t FR_RPWM = 4;
constexpr uint8_t FR_LPWM = 5;
constexpr uint8_t RL_RPWM = 25;
constexpr uint8_t RL_LPWM = 24;
constexpr uint8_t RR_RPWM = 3;
constexpr uint8_t RR_LPWM = 2;

constexpr uint8_t START_BUTTON = 6; // INPUT_PULLUP, nhan = LOW
}  // namespace Config

#define TELEMETRY Serial5

// ================================================================
// 2. KIEU DU LIEU VA TRANG THAI
// ================================================================
struct PIDGains {
  float kp;
  float ki;
  float kd;
};

struct PIDState {
  PIDGains gains;
  float integral;
  float lastError;
  float output;
};

struct Wheel {
  const char *id;
  Encoder *encoder;
  uint8_t rpwmPin;
  uint8_t lpwmPin;
  int8_t encoderSign;
  int8_t motorSign;

  long lastCount;
  float targetRpm;
  float measuredRpm;
  int pwm;
  PIDState pid;
};

enum class DriveMode : uint8_t {
  STOP,
  MANUAL,
  HEADING_HOLD,
  LINE_FOLLOW,
  AUTO
};

enum class AutoState : uint8_t {
  IDLE,
  FORWARD_1M,
  STRAFE_RIGHT_500,
  FOLLOW_LINE,
  FINISHED,
  FAULT
};

// ================================================================
// 3. PHAN CUNG
// ================================================================
Encoder encoderFL(18, 19);
Encoder encoderFR(22, 23);
Encoder encoderRL(15, 14);
Encoder encoderRR(29, 28);
Adafruit_BNO055 bno(55, 0x28, &Wire1);

Wheel wheels[4] = {
  {"FL", &encoderFL, Config::FL_RPWM, Config::FL_LPWM, +1, +1, 0, 0, 0, 0, {{0.82f, 0.035f, 0.014f}, 0, 0, 0}},
  {"FR", &encoderFR, Config::FR_RPWM, Config::FR_LPWM, +1, +1, 0, 0, 0, 0, {{0.79f, 0.032f, 0.012f}, 0, 0, 0}},
  {"RL", &encoderRL, Config::RL_RPWM, Config::RL_LPWM, +1, +1, 0, 0, 0, 0, {{0.84f, 0.038f, 0.015f}, 0, 0, 0}},
  {"RR", &encoderRR, Config::RR_RPWM, Config::RR_LPWM, +1, +1, 0, 0, 0, 0, {{0.81f, 0.034f, 0.013f}, 0, 0, 0}}
};

PIDState headingPid = {{3.20f, 0.018f, 0.240f}, 0, 0, 0};
PIDState linePid = {{1.45f, 0.012f, 0.180f}, 0, 0, 0};

DriveMode driveMode = DriveMode::STOP;
AutoState autoState = AutoState::IDLE;

bool imuReady = false;
bool emergencyStop = true; // Boot luon an toan.
bool telemetryEverConnected = false;

float headingDeg = 0.0f;
float targetHeadingDeg = 0.0f;
float headingErrorDeg = 0.0f;
float lineError = 0.0f;
float batteryVoltage = 0.0f; // Gan ADC pin neu robot co mach chia ap.

float commandVx = 0.0f;     // RPM tien/lui
float commandVy = 0.0f;     // RPM trai/phai
float commandOmega = 0.0f;  // RPM quay

uint16_t lineAdc[8] = {};
long autoStartCounts[4] = {};
uint32_t stateStartedMs = 0;
uint32_t lastLineSeenMs = 0;
uint32_t lastRadioRxMs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t lastControlUs = 0;

char rxLine[160];
size_t rxLength = 0;

// ================================================================
// 4. HAM TIEN ICH
// ================================================================
float clampFloat(float value, float minimum, float maximum) {
  return value < minimum ? minimum : (value > maximum ? maximum : value);
}

float wrapAngle180(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

void resetPID(PIDState &pid) {
  pid.integral = 0.0f;
  pid.lastError = 0.0f;
  pid.output = 0.0f;
}

float updatePID(PIDState &pid, float error, float dt, float outputLimit) {
  if (dt <= 0.0f) return pid.output;

  pid.integral += error * dt;
  pid.integral = clampFloat(pid.integral,
                            -Config::INTEGRAL_LIMIT,
                            Config::INTEGRAL_LIMIT);

  const float derivative = (error - pid.lastError) / dt;
  pid.lastError = error;
  pid.output = pid.gains.kp * error
             + pid.gains.ki * pid.integral
             + pid.gains.kd * derivative;
  pid.output = clampFloat(pid.output, -outputLimit, outputLimit);
  return pid.output;
}

int wheelIndexFromId(const char *id) {
  for (int i = 0; i < 4; ++i) {
    if (strcmp(id, wheels[i].id) == 0) return i;
  }
  return -1;
}

const char *driveModeName() {
  switch (driveMode) {
    case DriveMode::STOP: return "STOP";
    case DriveMode::MANUAL: return "MANUAL";
    case DriveMode::HEADING_HOLD: return "HEADING";
    case DriveMode::LINE_FOLLOW: return "LINE";
    case DriveMode::AUTO: return "AUTO";
  }
  return "UNKNOWN";
}

// ================================================================
// 5. MOTOR, ENCODER VA PID TOC DO
// ================================================================
void writeMotor(const Wheel &wheel, int requestedPwm) {
  int pwm = constrain(requestedPwm * wheel.motorSign, -255, 255);
  if (pwm > 0) {
    analogWrite(wheel.rpwmPin, pwm);
    analogWrite(wheel.lpwmPin, 0);
  } else if (pwm < 0) {
    analogWrite(wheel.rpwmPin, 0);
    analogWrite(wheel.lpwmPin, -pwm);
  } else {
    analogWrite(wheel.rpwmPin, 0);
    analogWrite(wheel.lpwmPin, 0);
  }
}

void stopAllMotors() {
  commandVx = commandVy = commandOmega = 0.0f;
  for (Wheel &wheel : wheels) {
    wheel.targetRpm = 0.0f;
    wheel.pwm = 0;
    resetPID(wheel.pid);
    writeMotor(wheel, 0);
  }
}

void readWheelSpeeds(float dt) {
  for (Wheel &wheel : wheels) {
    const long count = wheel.encoder->read() * wheel.encoderSign;
    const long delta = count - wheel.lastCount;
    wheel.lastCount = count;
    wheel.measuredRpm =
      (delta * 60.0f) / (Config::ENCODER_COUNTS_PER_REV * dt);
  }
}

void mixOmni(float vx, float vy, float omega) {
  // X-drive / omni 4 banh:
  // FL = +vx -vy +w, FR = +vx +vy -w
  // RL = +vx +vy +w, RR = +vx -vy -w
  float mixed[4] = {
    vx - vy + omega,
    vx + vy - omega,
    vx + vy + omega,
    vx - vy - omega
  };

  float largest = 1.0f;
  for (float value : mixed) largest = max(largest, fabsf(value) / Config::MAX_WHEEL_RPM);
  for (int i = 0; i < 4; ++i) wheels[i].targetRpm = mixed[i] / largest;
}

void updateWheelControllers(float dt) {
  readWheelSpeeds(dt);

  for (Wheel &wheel : wheels) {
    if (emergencyStop || fabsf(wheel.targetRpm) < 0.5f) {
      wheel.pwm = 0;
      resetPID(wheel.pid);
    } else {
      const float error = wheel.targetRpm - wheel.measuredRpm;
      // Feed-forward don gian theo ty le RPM toi da.
      const float feedForward = (wheel.targetRpm / Config::MAX_WHEEL_RPM) * 210.0f;
      const float correction = updatePID(wheel.pid, error, dt, Config::MAX_PWM);
      wheel.pwm = static_cast<int>(clampFloat(feedForward + correction,
                                              -Config::MAX_PWM,
                                              Config::MAX_PWM));
    }
    writeMotor(wheel, wheel.pwm);
  }
}

// ================================================================
// 6. HEADING BNO055
// ================================================================
float readHeading() {
  if (!imuReady) return headingDeg;
  const imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  return euler.x();
}

float calculateHeadingCorrection(float dt) {
  headingDeg = readHeading();
  headingErrorDeg = wrapAngle180(targetHeadingDeg - headingDeg);
  return updatePID(headingPid, headingErrorDeg, dt, 60.0f);
}

void setHeadingTarget(float degrees) {
  targetHeadingDeg = degrees;
  resetPID(headingPid);
}

// ================================================================
// 7. CAM BIEN LINE MCP3008
// ================================================================
uint16_t readMCP3008(uint8_t channel) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(Config::MCP3008_CS, LOW);
  SPI.transfer(0x01);
  const uint8_t highByte = SPI.transfer((0x08 | (channel & 0x07)) << 4);
  const uint8_t lowByte = SPI.transfer(0x00);
  digitalWrite(Config::MCP3008_CS, HIGH);
  SPI.endTransaction();
  return ((highByte & 0x03) << 8) | lowByte;
}

int calculateLineError() {
  static const int weights[8] = {-350, -250, -150, -50, 50, 150, 250, 350};
  long weightedSum = 0;
  int activeCount = 0;

  for (int i = 0; i < 8; ++i) {
    lineAdc[i] = readMCP3008(i);
    if (lineAdc[i] > Config::LINE_THRESHOLD) {
      weightedSum += weights[i];
      ++activeCount;
    }
  }
  return activeCount == 0 ? Config::LINE_LOST : weightedSum / activeCount;
}

float calculateLineCorrection(float dt) {
  const int measuredError = calculateLineError();
  if (measuredError == Config::LINE_LOST) {
    lineError = Config::LINE_LOST;
    resetPID(linePid);
    return 0.0f;
  }
  lineError = measuredError;
  // Chia 100 de gain tren dashboard de doc va tune hon.
  return updatePID(linePid, lineError / 100.0f, dt, 50.0f);
}

// ================================================================
// 8. STATE MACHINE AUTO
// ================================================================
float averageTravelCounts() {
  float sum = 0.0f;
  for (int i = 0; i < 4; ++i) {
    sum += fabsf(static_cast<float>(wheels[i].encoder->read() - autoStartCounts[i]));
  }
  return sum / 4.0f;
}

void enterAutoState(AutoState next) {
  autoState = next;
  stateStartedMs = millis();
  lastLineSeenMs = stateStartedMs;
  for (int i = 0; i < 4; ++i) autoStartCounts[i] = wheels[i].encoder->read();
}

void startAuto() {
  emergencyStop = false;
  driveMode = DriveMode::AUTO;
  setHeadingTarget(readHeading());
  enterAutoState(AutoState::FORWARD_1M);
}

void updateAuto(float dt) {
  const float headingCorrection = calculateHeadingCorrection(dt);

  switch (autoState) {
    case AutoState::IDLE:
      stopAllMotors();
      break;

    case AutoState::FORWARD_1M:
      commandVx = 100.0f;
      commandVy = 0.0f;
      commandOmega = headingCorrection;
      // Can hieu chinh counts theo duong kinh banh va ti so truyen.
      if (averageTravelCounts() >= 5600.0f) enterAutoState(AutoState::STRAFE_RIGHT_500);
      break;

    case AutoState::STRAFE_RIGHT_500:
      commandVx = 0.0f;
      commandVy = 80.0f;
      commandOmega = headingCorrection;
      if (averageTravelCounts() >= 2800.0f) enterAutoState(AutoState::FOLLOW_LINE);
      break;

    case AutoState::FOLLOW_LINE: {
      const float correction = calculateLineCorrection(dt);
      if (lineError == Config::LINE_LOST) {
        if (millis() - lastLineSeenMs > 1000) enterAutoState(AutoState::FAULT);
        commandVx = 35.0f;
        commandVy = commandOmega = 0.0f;
      } else {
        lastLineSeenMs = millis();
        commandVx = 75.0f;
        commandVy = -correction;
        commandOmega = headingCorrection;
      }
      // Kich ban mau: bam line 8 giay roi dung.
      if (millis() - stateStartedMs > 8000) enterAutoState(AutoState::FINISHED);
      break;
    }

    case AutoState::FINISHED:
      stopAllMotors();
      driveMode = DriveMode::STOP;
      break;

    case AutoState::FAULT:
      emergencyStop = true;
      stopAllMotors();
      break;
  }
}

// ================================================================
// 9. TELEMETRY UART RBT/1
// ================================================================
void sendAck(const char *topic, const char *detail = nullptr) {
  TELEMETRY.print("ACK,");
  TELEMETRY.print(topic);
  if (detail) {
    TELEMETRY.print(',');
    TELEMETRY.print(detail);
  }
  TELEMETRY.println();
}

void sendError(const char *errorCode) {
  TELEMETRY.print("ERR,");
  TELEMETRY.println(errorCode);
}

void applyWheelGains(const char *target, const PIDGains &gains) {
  if (strcmp(target, "ALL") == 0) {
    for (Wheel &wheel : wheels) {
      wheel.pid.gains = gains;
      resetPID(wheel.pid);
    }
    return;
  }
  const int index = wheelIndexFromId(target);
  if (index >= 0) {
    wheels[index].pid.gains = gains;
    resetPID(wheels[index].pid);
  }
}

void parsePidCommand(char *savePtr) {
  char *scope = strtok_r(nullptr, ",", &savePtr);
  char *target = strtok_r(nullptr, ",", &savePtr);
  char *kpText = strtok_r(nullptr, ",", &savePtr);
  char *kiText = strtok_r(nullptr, ",", &savePtr);
  char *kdText = strtok_r(nullptr, ",", &savePtr);
  if (!scope || !target || !kpText || !kiText || !kdText) {
    sendError("BAD_PID_PACKET");
    return;
  }

  const PIDGains gains = {atof(kpText), atof(kiText), atof(kdText)};
  if (!isfinite(gains.kp) || !isfinite(gains.ki) || !isfinite(gains.kd)
      || gains.kp < 0 || gains.ki < 0 || gains.kd < 0) {
    sendError("BAD_PID_VALUE");
    return;
  }

  if (strcmp(scope, "WHEEL") == 0) {
    if (strcmp(target, "ALL") != 0 && wheelIndexFromId(target) < 0) {
      sendError("BAD_WHEEL");
      return;
    }
    applyWheelGains(target, gains);
  } else if (strcmp(scope, "HEADING") == 0) {
    headingPid.gains = gains;
    resetPID(headingPid);
  } else if (strcmp(scope, "LINE") == 0) {
    linePid.gains = gains;
    resetPID(linePid);
  } else {
    sendError("BAD_SCOPE");
    return;
  }
  sendAck("PID", scope);
}

void parseCommand(char *packet) {
  char *savePtr = nullptr;
  char *command = strtok_r(packet, ",", &savePtr);
  if (!command) return;

  lastRadioRxMs = millis();
  telemetryEverConnected = true;

  if (strcmp(command, "PID") == 0) {
    parsePidCommand(savePtr);
  } else if (strcmp(command, "ESTOP") == 0) {
    emergencyStop = true;
    driveMode = DriveMode::STOP;
    stopAllMotors();
    sendAck("ESTOP");
  } else if (strcmp(command, "RUN") == 0) {
    emergencyStop = false;
    driveMode = DriveMode::MANUAL;
    sendAck("RUN");
  } else if (strcmp(command, "AUTO") == 0) {
    startAuto();
    sendAck("AUTO");
  } else if (strcmp(command, "TARGET") == 0) {
    char *vx = strtok_r(nullptr, ",", &savePtr);
    char *vy = strtok_r(nullptr, ",", &savePtr);
    char *omega = strtok_r(nullptr, ",", &savePtr);
    if (!vx || !vy || !omega) {
      sendError("BAD_TARGET");
      return;
    }
    commandVx = clampFloat(atof(vx), -Config::MAX_WHEEL_RPM, Config::MAX_WHEEL_RPM);
    commandVy = clampFloat(atof(vy), -Config::MAX_WHEEL_RPM, Config::MAX_WHEEL_RPM);
    commandOmega = clampFloat(atof(omega), -60.0f, 60.0f);
    driveMode = DriveMode::MANUAL;
    sendAck("TARGET");
  } else if (strcmp(command, "YAW") == 0) {
    char *yaw = strtok_r(nullptr, ",", &savePtr);
    if (!yaw) {
      sendError("BAD_YAW");
      return;
    }
    setHeadingTarget(atof(yaw));
    driveMode = DriveMode::HEADING_HOLD;
    sendAck("YAW");
  } else if (strcmp(command, "LINE") == 0) {
    char *speed = strtok_r(nullptr, ",", &savePtr);
    commandVx = speed ? clampFloat(atof(speed), -120.0f, 120.0f) : 60.0f;
    driveMode = DriveMode::LINE_FOLLOW;
    sendAck("LINE");
  } else if (strcmp(command, "PING") == 0) {
    sendAck("PONG");
  } else {
    sendError("UNKNOWN_COMMAND");
  }
}

void readTelemetryCommands() {
  while (TELEMETRY.available()) {
    const char character = static_cast<char>(TELEMETRY.read());
    if (character == '\n') {
      rxLine[rxLength] = '\0';
      if (rxLength > 0) parseCommand(rxLine);
      rxLength = 0;
    } else if (character != '\r') {
      if (rxLength < sizeof(rxLine) - 1) rxLine[rxLength++] = character;
      else {
        rxLength = 0;
        sendError("PACKET_TOO_LONG");
      }
    }
  }
}

void sendTelemetry() {
  if (millis() - lastTelemetryMs < Config::TELEMETRY_PERIOD_MS) return;
  lastTelemetryMs = millis();

  TELEMETRY.print("TEL,");
  TELEMETRY.print(millis());
  for (const Wheel &wheel : wheels) {
    TELEMETRY.print(',');
    TELEMETRY.print(wheel.measuredRpm, 1);
  }
  TELEMETRY.print(',');
  TELEMETRY.print(headingDeg, 2);
  TELEMETRY.print(',');
  TELEMETRY.print(lineError, 2);
  TELEMETRY.print(',');
  TELEMETRY.println(batteryVoltage, 2);
}

// ================================================================
// 10. BO DIEU KHIEN TRUNG TAM
// ================================================================
void updateDriveMode(float dt) {
  if (emergencyStop) {
    stopAllMotors();
    return;
  }

  switch (driveMode) {
    case DriveMode::STOP:
      commandVx = commandVy = commandOmega = 0.0f;
      break;

    case DriveMode::MANUAL:
      break;

    case DriveMode::HEADING_HOLD:
      commandOmega = calculateHeadingCorrection(dt);
      break;

    case DriveMode::LINE_FOLLOW:
      commandVy = -calculateLineCorrection(dt);
      commandOmega = calculateHeadingCorrection(dt);
      break;

    case DriveMode::AUTO:
      updateAuto(dt);
      break;
  }

  mixOmni(commandVx, commandVy, commandOmega);
}

void runSafetyChecks() {
  // Chi ap dung timeout sau khi giao dien da tung ket noi va dang dieu khien.
  const bool remoteControlMode =
    driveMode == DriveMode::MANUAL ||
    driveMode == DriveMode::HEADING_HOLD ||
    driveMode == DriveMode::LINE_FOLLOW;

  if (telemetryEverConnected && remoteControlMode
      && millis() - lastRadioRxMs > Config::RADIO_TIMEOUT_MS) {
    emergencyStop = true;
    driveMode = DriveMode::STOP;
    stopAllMotors();
    sendError("RADIO_TIMEOUT");
  }
}

// ================================================================
// 11. KHOI TAO
// ================================================================
void setupMotorHardware() {
  analogWriteResolution(8);
  for (Wheel &wheel : wheels) {
    pinMode(wheel.rpwmPin, OUTPUT);
    pinMode(wheel.lpwmPin, OUTPUT);
    analogWriteFrequency(wheel.rpwmPin, 20000);
    analogWriteFrequency(wheel.lpwmPin, 20000);
    wheel.lastCount = wheel.encoder->read() * wheel.encoderSign;
  }
  stopAllMotors();
}

void setupSensors() {
  pinMode(Config::MCP3008_CS, OUTPUT);
  digitalWrite(Config::MCP3008_CS, HIGH);
  SPI.begin();

  Wire1.begin();
  imuReady = bno.begin();
  if (imuReady) {
    bno.setExtCrystalUse(true);
    delay(250);
    headingDeg = readHeading();
    targetHeadingDeg = headingDeg;
  }
}

void setup() {
  Serial.begin(Config::DEBUG_BAUD);
  TELEMETRY.begin(Config::TELEMETRY_BAUD, SERIAL_8N1);
  pinMode(Config::START_BUTTON, INPUT_PULLUP);

  setupMotorHardware();
  setupSensors();

  lastRadioRxMs = millis();
  lastControlUs = micros();

  Serial.println("ROBOCON AUTO V1 READY");
  Serial.println(imuReady ? "BNO055 OK" : "BNO055 NOT FOUND");
  TELEMETRY.println("ACK,BOOT,AUTO_V1");
}

// ================================================================
// 12. VONG LAP CHINH - KHONG DUNG delay()
// ================================================================
void loop() {
  readTelemetryCommands();
  runSafetyChecks();

  // Nut tren robot cho phep chay AUTO khong can radio.
  static bool lastButton = HIGH;
  const bool button = digitalRead(Config::START_BUTTON);
  if (lastButton == HIGH && button == LOW) startAuto();
  lastButton = button;

  const uint32_t nowUs = micros();
  if (nowUs - lastControlUs >= Config::CONTROL_PERIOD_US) {
    const float dt = (nowUs - lastControlUs) / 1000000.0f;
    lastControlUs = nowUs;

    updateDriveMode(dt);
    updateWheelControllers(dt);
  }

  sendTelemetry();
}

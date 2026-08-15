#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Encoder.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <VL53L1X.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

// ============================================================
// ROBOCON KOSEN F0 - Teensy 4.1 / 4-wheel mecanum
// Review every pin and direction before powering the motors.
// ============================================================

namespace Config {
constexpr uint32_t DEBUG_BAUD = 115200;
constexpr uint32_t TELEMETRY_BAUD = 57600;
constexpr uint32_t MECHANISM_BAUD = 115200;
constexpr uint32_t FIRMWARE_VERSION = 0x00010000; // 1.0.0
constexpr uint32_t CONFIG_VERSION = 1;
constexpr uint32_t CONTROL_PERIOD_US = 10000; // 100 Hz
constexpr uint32_t TELEMETRY_PERIOD_MS = 20;  // 50 Hz
constexpr uint32_t COMMAND_TIMEOUT_MS = 500;
constexpr int PWM_FREQUENCY = 20000;
constexpr int PWM_MAX = 255;

constexpr uint8_t FL_RPWM = 36, FL_LPWM = 37;
constexpr uint8_t FR_RPWM = 4,  FR_LPWM = 5;
constexpr uint8_t RL_RPWM = 25, RL_LPWM = 24;
constexpr uint8_t RR_RPWM = 3,  RR_LPWM = 2;

constexpr uint8_t FL_ENC_A = 18, FL_ENC_B = 19;
constexpr uint8_t FR_ENC_A = 22, FR_ENC_B = 23;
constexpr uint8_t RL_ENC_A = 15, RL_ENC_B = 14;
constexpr uint8_t RR_ENC_A = 29, RR_ENC_B = 28;

constexpr int8_t MOTOR_SIGN[4] = {1, 1, 1, 1};
constexpr int8_t ENCODER_SIGN[4] = {1, 1, 1, 1};
constexpr float ENCODER_CPR = 330.0f; // TODO: replace with counts/rev at wheel
constexpr float WHEEL_DIAMETER_MM = 100.0f;

constexpr uint8_t MCP3008_CS = 10;
constexpr uint16_t LINE_THRESHOLD = 500;
constexpr int16_t LINE_WEIGHT[8] = {-350, -250, -150, -50, 50, 150, 250, 350};

constexpr uint8_t START_PIN = 32; // active LOW
constexpr uint8_t ESTOP_PIN = 31; // active LOW
constexpr uint8_t BATTERY_PIN = A17;
constexpr float ADC_REFERENCE_V = 3.3f;
constexpr float BATTERY_DIVIDER_RATIO = 11.0f; // TODO: match resistor divider

constexpr float MAX_WHEEL_RPM = 220.0f;
constexpr float AUTO_SPEED_RPM = 80.0f;
constexpr float POSITION_TOLERANCE_MM = 20.0f;
constexpr uint32_t AUTO_STEP_TIMEOUT_MS = 8000;
}

#define TELEMETRY Serial5 // RX5 pin 34, TX5 pin 33
#define MECHANISM Serial4 // confirm pins on your Teensy carrier

struct PIDGains { float kp, ki, kd; };

struct PIDController {
  PIDGains gains;
  float integral = 0.0f;
  float previousError = 0.0f;
  float integralLimit = 300.0f;
  float outputLimit = 255.0f;

  void reset() { integral = 0.0f; previousError = 0.0f; }

  float update(float target, float measured, float dt) {
    if (dt <= 0.0f) return 0.0f;
    const float error = target - measured;
    integral = constrain(integral + error * dt, -integralLimit, integralLimit);
    const float derivative = (error - previousError) / dt;
    previousError = error;
    return constrain(gains.kp * error + gains.ki * integral + gains.kd * derivative,
                     -outputLimit, outputLimit);
  }
};

enum WheelIndex : uint8_t { FL, FR, RL, RR, WHEEL_COUNT };

struct Wheel {
  uint8_t rpwm;
  uint8_t lpwm;
  Encoder *encoder;
  int8_t motorSign;
  int8_t encoderSign;
  long lastCount = 0;
  float rpm = 0.0f;
  float targetRpm = 0.0f;
  PIDController pid;
};

Encoder encoderFL(Config::FL_ENC_A, Config::FL_ENC_B);
Encoder encoderFR(Config::FR_ENC_A, Config::FR_ENC_B);
Encoder encoderRL(Config::RL_ENC_A, Config::RL_ENC_B);
Encoder encoderRR(Config::RR_ENC_A, Config::RR_ENC_B);

Wheel wheels[WHEEL_COUNT] = {
  {Config::FL_RPWM, Config::FL_LPWM, &encoderFL, Config::MOTOR_SIGN[0], Config::ENCODER_SIGN[0]},
  {Config::FR_RPWM, Config::FR_LPWM, &encoderFR, Config::MOTOR_SIGN[1], Config::ENCODER_SIGN[1]},
  {Config::RL_RPWM, Config::RL_LPWM, &encoderRL, Config::MOTOR_SIGN[2], Config::ENCODER_SIGN[2]},
  {Config::RR_RPWM, Config::RR_LPWM, &encoderRR, Config::MOTOR_SIGN[3], Config::ENCODER_SIGN[3]}
};

Adafruit_BNO055 bno(55, 0x28, &Wire1);
VL53L1X tof;
PIDController headingPID{{3.2f, 0.018f, 0.24f}};
PIDController linePID{{1.45f, 0.012f, 0.18f}};

bool bnoReady = false;
bool tofReady = false;
bool armed = false;
bool emergencyStop = true;
bool autonomousEnabled = false;
float headingDeg = 0.0f;
float headingTargetDeg = 0.0f;
float lineError = 0.0f;
float batteryVoltage = 0.0f;
float commandVx = 0.0f; // forward RPM
float commandVy = 0.0f; // right RPM
float commandWz = 0.0f; // rotation contribution RPM
uint32_t lastDriveCommandMs = 0;
uint32_t lastControlUs = 0;
uint32_t lastTelemetryMs = 0;
char telemetryBuffer[160];
size_t telemetryLength = 0;

enum class AutoState : uint8_t {
  IDLE,
  LEAVE_START,
  WAIT_MECHANISM,
  RETURN_HOME,
  FINISHED,
  FAULT
};

AutoState autoState = AutoState::IDLE;
uint32_t stateStartedMs = 0;
long stateStartCounts[WHEEL_COUNT] = {};

static float wrapAngle(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

static bool validNumber(float value) {
  return isfinite(value) && fabsf(value) < 100000.0f;
}

void writeMotor(Wheel &wheel, int pwm) {
  pwm = constrain(pwm * wheel.motorSign, -Config::PWM_MAX, Config::PWM_MAX);
  analogWrite(wheel.rpwm, pwm > 0 ? pwm : 0);
  analogWrite(wheel.lpwm, pwm < 0 ? -pwm : 0);
}

void stopAllMotors() {
  commandVx = commandVy = commandWz = 0.0f;
  for (Wheel &wheel : wheels) {
    wheel.targetRpm = 0.0f;
    wheel.pid.reset();
    writeMotor(wheel, 0);
  }
}

void setDrive(float vx, float vy, float wz) {
  commandVx = constrain(vx, -Config::MAX_WHEEL_RPM, Config::MAX_WHEEL_RPM);
  commandVy = constrain(vy, -Config::MAX_WHEEL_RPM, Config::MAX_WHEEL_RPM);
  commandWz = constrain(wz, -Config::MAX_WHEEL_RPM, Config::MAX_WHEEL_RPM);
}

void applyMecanumKinematics(float vx, float vy, float wz) {
  wheels[FL].targetRpm = vx + vy + wz;
  wheels[FR].targetRpm = vx - vy - wz;
  wheels[RL].targetRpm = vx - vy + wz;
  wheels[RR].targetRpm = vx + vy - wz;
  float largest = 1.0f;
  for (const Wheel &wheel : wheels)
    largest = max(largest, fabsf(wheel.targetRpm) / Config::MAX_WHEEL_RPM);
  for (Wheel &wheel : wheels) wheel.targetRpm /= largest;
}

uint16_t readMCP3008(uint8_t channel) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(Config::MCP3008_CS, LOW);
  SPI.transfer(0x01);
  const uint8_t high = SPI.transfer((0x08 | (channel & 0x07)) << 4);
  const uint8_t low = SPI.transfer(0x00);
  digitalWrite(Config::MCP3008_CS, HIGH);
  SPI.endTransaction();
  return ((high & 0x03) << 8) | low;
}

float readLineError() {
  long weightedSum = 0;
  uint8_t detected = 0;
  for (uint8_t i = 0; i < 8; ++i) {
    const uint16_t value = readMCP3008(i);
    if (value > Config::LINE_THRESHOLD) {
      weightedSum += Config::LINE_WEIGHT[i];
      ++detected;
    }
  }
  if (detected == 0) return lineError; // keep last direction when line is lost
  // Normalize to about -1..+1 so the PID tuner uses portable gains.
  return (static_cast<float>(weightedSum) / detected) / 350.0f;
}

float readBatteryVoltage() {
  const float adc = analogRead(Config::BATTERY_PIN);
  return adc * Config::ADC_REFERENCE_V / 1023.0f * Config::BATTERY_DIVIDER_RATIO;
}

void updateSensors() {
  if (bnoReady) headingDeg = bno.getVector(Adafruit_BNO055::VECTOR_EULER).x();
  lineError = readLineError();
  batteryVoltage = 0.95f * batteryVoltage + 0.05f * readBatteryVoltage();
}

void updateWheelControl(float dt) {
  for (Wheel &wheel : wheels) {
    const long count = wheel.encoder->read() * wheel.encoderSign;
    const long delta = count - wheel.lastCount;
    wheel.lastCount = count;
    const float rawRpm = delta * 60.0f / (Config::ENCODER_CPR * dt);
    wheel.rpm = 0.7f * wheel.rpm + 0.3f * rawRpm;
    if (!armed || emergencyStop) {
      wheel.pid.reset();
      writeMotor(wheel, 0);
      continue;
    }
    const float feedForward = wheel.targetRpm / Config::MAX_WHEEL_RPM * Config::PWM_MAX;
    const int pwm = lroundf(feedForward + wheel.pid.update(wheel.targetRpm, wheel.rpm, dt));
    writeMotor(wheel, pwm);
  }
}

void sendAck(const char *message) { TELEMETRY.print("ACK,"); TELEMETRY.println(message); }
void sendError(const char *message) { TELEMETRY.print("ERR,"); TELEMETRY.println(message); }

void printPidConfig(const char *scope, const char *target, const PIDGains &gains) {
  TELEMETRY.print("CFG,"); TELEMETRY.print(scope); TELEMETRY.print(',');
  TELEMETRY.print(target); TELEMETRY.print(',');
  TELEMETRY.print(gains.kp, 6); TELEMETRY.print(',');
  TELEMETRY.print(gains.ki, 6); TELEMETRY.print(',');
  TELEMETRY.println(gains.kd, 6);
}

void sendCurrentConfig() {
  static const char *wheelNames[WHEEL_COUNT] = {"FL", "FR", "RL", "RR"};
  TELEMETRY.print("CFG,META,"); TELEMETRY.print(Config::FIRMWARE_VERSION);
  TELEMETRY.print(','); TELEMETRY.println(Config::CONFIG_VERSION);
  for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    printPidConfig("WHEEL", wheelNames[i], wheels[i].pid.gains);
  printPidConfig("HEADING", "0", headingPID.gains);
  printPidConfig("LINE", "CENTER", linePID.gains);
  TELEMETRY.print("CFG,YAW_TARGET,"); TELEMETRY.println(headingTargetDeg, 2);
  TELEMETRY.println("CFG_END");
}

int wheelIndex(const char *id) {
  if (!strcmp(id, "FL")) return FL;
  if (!strcmp(id, "FR")) return FR;
  if (!strcmp(id, "RL") || !strcmp(id, "BL")) return RL;
  if (!strcmp(id, "RR") || !strcmp(id, "BR")) return RR;
  return -1;
}

bool parseFloatStrict(const char *text, float &value) {
  if (!text) return false;
  char *end = nullptr;
  value = strtof(text, &end);
  return end != text && *end == '\0' && validNumber(value);
}

void applyPidCommand(char *scope, char *target, char *kpText, char *kiText, char *kdText) {
  float kp, ki, kd;
  if (!scope || !target || !parseFloatStrict(kpText, kp) ||
      !parseFloatStrict(kiText, ki) || !parseFloatStrict(kdText, kd) ||
      kp < 0.0f || ki < 0.0f || kd < 0.0f) {
    sendError("BAD_PID"); return;
  }
  const PIDGains gains{kp, ki, kd};
  if (!strcmp(scope, "WHEEL")) {
    if (!strcmp(target, "ALL")) {
      for (Wheel &wheel : wheels) wheel.pid.gains = gains;
    } else {
      const int index = wheelIndex(target);
      if (index < 0) { sendError("BAD_WHEEL"); return; }
      wheels[index].pid.gains = gains;
    }
  } else if (!strcmp(scope, "HEADING")) headingPID.gains = gains;
  else if (!strcmp(scope, "LINE")) linePID.gains = gains;
  else { sendError("BAD_SCOPE"); return; }
  TELEMETRY.print("ACK,PID,"); TELEMETRY.print(scope); TELEMETRY.print(','); TELEMETRY.println(target);
}

void processTelemetryCommand(char *packet) {
  char *save = nullptr;
  char *command = strtok_r(packet, ",", &save);
  if (!command) return;

  if (!strcmp(command, "PING")) {
    TELEMETRY.print("HELLO,ROBOCON_KOSEN_F0,");
    TELEMETRY.print(Config::FIRMWARE_VERSION); TELEMETRY.print(',');
    TELEMETRY.println(Config::CONFIG_VERSION);
    return;
  }
  if (!strcmp(command, "HEARTBEAT")) {
    TELEMETRY.print("HB,"); TELEMETRY.print(millis()); TELEMETRY.print(',');
    TELEMETRY.print(armed ? 1 : 0); TELEMETRY.print(',');
    TELEMETRY.println(emergencyStop ? 1 : 0);
    return;
  }
  if (!strcmp(command, "GET_CONFIG")) {
    sendCurrentConfig();
    return;
  }

  if (!strcmp(command, "ESTOP")) {
    emergencyStop = true; armed = false; autonomousEnabled = false;
    autoState = AutoState::IDLE; stopAllMotors(); sendAck("ESTOP"); return;
  }
  if (!strcmp(command, "RUN")) {
    if (digitalRead(Config::ESTOP_PIN) == LOW) { sendError("PHYSICAL_ESTOP"); return; }
    emergencyStop = false; armed = true; lastDriveCommandMs = millis(); sendAck("RUN"); return;
  }
  if (!strcmp(command, "AUTO")) {
    if (!armed || emergencyStop) { sendError("NOT_ARMED"); return; }
    autonomousEnabled = true; autoState = AutoState::LEAVE_START; stateStartedMs = millis();
    for (uint8_t i = 0; i < WHEEL_COUNT; ++i) stateStartCounts[i] = wheels[i].encoder->read();
    sendAck("AUTO"); return;
  }
  if (!strcmp(command, "DRIVE")) {
    float vx, vy, wz;
    if (!parseFloatStrict(strtok_r(nullptr, ",", &save), vx) ||
        !parseFloatStrict(strtok_r(nullptr, ",", &save), vy) ||
        !parseFloatStrict(strtok_r(nullptr, ",", &save), wz)) { sendError("BAD_DRIVE"); return; }
    autonomousEnabled = false; setDrive(vx, vy, wz); lastDriveCommandMs = millis(); sendAck("DRIVE"); return;
  }
  if (!strcmp(command, "YAW")) {
    float target;
    if (!parseFloatStrict(strtok_r(nullptr, ",", &save), target)) { sendError("BAD_YAW"); return; }
    headingTargetDeg = target; sendAck("YAW"); return;
  }
  if (!strcmp(command, "PID")) {
    // Tokenize sequentially: C++ does not guarantee function argument evaluation order.
    char *scope = strtok_r(nullptr, ",", &save);
    char *target = strtok_r(nullptr, ",", &save);
    char *kp = strtok_r(nullptr, ",", &save);
    char *ki = strtok_r(nullptr, ",", &save);
    char *kd = strtok_r(nullptr, ",", &save);
    applyPidCommand(scope, target, kp, ki, kd);
    return;
  }
  sendError("UNKNOWN_COMMAND");
}

void updateTelemetryInput() {
  while (TELEMETRY.available()) {
    const char c = static_cast<char>(TELEMETRY.read());
    if (c == '\n' || c == '\r') {
      if (telemetryLength) {
        telemetryBuffer[telemetryLength] = '\0';
        processTelemetryCommand(telemetryBuffer);
        telemetryLength = 0;
      }
    } else if (telemetryLength < sizeof(telemetryBuffer) - 1) {
      telemetryBuffer[telemetryLength++] = c;
    } else {
      telemetryLength = 0; sendError("PACKET_TOO_LONG");
    }
  }
}

void sendTelemetry() {
  if (millis() - lastTelemetryMs < Config::TELEMETRY_PERIOD_MS) return;
  lastTelemetryMs = millis();
  TELEMETRY.printf("TEL,%lu,%.1f,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f\n",
                   millis(), wheels[FL].rpm, wheels[FR].rpm, wheels[RL].rpm,
                   wheels[RR].rpm, headingDeg, lineError, batteryVoltage);
}

float stateDistanceMm() {
  float counts = 0.0f;
  for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    counts += (wheels[i].encoder->read() - stateStartCounts[i]) * wheels[i].encoderSign;
  counts /= WHEEL_COUNT;
  return counts / Config::ENCODER_CPR * PI * Config::WHEEL_DIAMETER_MM;
}

void enterAutoState(AutoState next) {
  autoState = next; stateStartedMs = millis();
  for (uint8_t i = 0; i < WHEEL_COUNT; ++i) stateStartCounts[i] = wheels[i].encoder->read();
}

void updateAutonomous(float dt) {
  if (!autonomousEnabled) return;
  if (millis() - stateStartedMs > Config::AUTO_STEP_TIMEOUT_MS &&
      autoState != AutoState::FINISHED && autoState != AutoState::IDLE) {
    stopAllMotors(); emergencyStop = true; armed = false; autoState = AutoState::FAULT;
    sendError("AUTO_TIMEOUT"); return;
  }

  const float headingCorrection = bnoReady
      ? headingPID.update(0.0f, wrapAngle(headingDeg - headingTargetDeg), dt) : 0.0f;
  const float lineCorrection = linePID.update(0.0f, lineError, dt);

  switch (autoState) {
    case AutoState::LEAVE_START:
      // Safe example only. Replace distances/states with the official Kosen field sequence.
      setDrive(Config::AUTO_SPEED_RPM, lineCorrection, headingCorrection);
      if (stateDistanceMm() >= 1000.0f - Config::POSITION_TOLERANCE_MM) {
        stopAllMotors(); MECHANISM.println("POINT_A"); enterAutoState(AutoState::WAIT_MECHANISM);
      }
      break;
    case AutoState::WAIT_MECHANISM:
      stopAllMotors();
      if (MECHANISM.available()) {
        char response[16] = {};
        const size_t n = MECHANISM.readBytesUntil('\n', response, sizeof(response) - 1);
        response[n] = '\0';
        if (strstr(response, "DONE")) enterAutoState(AutoState::RETURN_HOME);
      }
      break;
    case AutoState::RETURN_HOME:
      setDrive(-Config::AUTO_SPEED_RPM, -lineCorrection, headingCorrection);
      if (stateDistanceMm() <= -1000.0f + Config::POSITION_TOLERANCE_MM) {
        stopAllMotors(); enterAutoState(AutoState::FINISHED); sendAck("AUTO_FINISHED");
      }
      break;
    case AutoState::FINISHED:
    case AutoState::IDLE:
    case AutoState::FAULT:
      stopAllMotors();
      break;
  }
}

void setup() {
  Serial.begin(Config::DEBUG_BAUD);
  TELEMETRY.begin(Config::TELEMETRY_BAUD);
  MECHANISM.begin(Config::MECHANISM_BAUD);
  MECHANISM.setTimeout(5);

  pinMode(Config::START_PIN, INPUT_PULLUP);
  pinMode(Config::ESTOP_PIN, INPUT_PULLUP);
  pinMode(Config::MCP3008_CS, OUTPUT);
  digitalWrite(Config::MCP3008_CS, HIGH);
  SPI.begin();
  analogReadResolution(10);
  analogWriteResolution(8);

  for (Wheel &wheel : wheels) {
    pinMode(wheel.rpwm, OUTPUT); pinMode(wheel.lpwm, OUTPUT);
    analogWriteFrequency(wheel.rpwm, Config::PWM_FREQUENCY);
    analogWriteFrequency(wheel.lpwm, Config::PWM_FREQUENCY);
    wheel.pid.gains = {0.82f, 0.035f, 0.014f};
    wheel.pid.outputLimit = 150.0f;
    wheel.lastCount = wheel.encoder->read() * wheel.encoderSign;
    writeMotor(wheel, 0);
  }

  Wire1.begin();
  bnoReady = bno.begin();
  if (bnoReady) { bno.setExtCrystalUse(true); delay(50); headingDeg = bno.getVector(Adafruit_BNO055::VECTOR_EULER).x(); headingTargetDeg = headingDeg; }
  tof.setBus(&Wire1);
  tof.setTimeout(30);
  tofReady = tof.init();
  if (tofReady) { tof.setDistanceMode(VL53L1X::Long); tof.startContinuous(50); }
  linePID.outputLimit = 35.0f;
  linePID.integralLimit = 2.0f;
  headingPID.outputLimit = 60.0f;

  lastControlUs = micros();
  Serial.printf("ROBOCON_KOSEN_F0 ready | BNO=%s TOF=%s | waiting RUN\n",
                bnoReady ? "OK" : "FAIL", tofReady ? "OK" : "FAIL");
}

void loop() {
  updateTelemetryInput();
  sendTelemetry();

  if (digitalRead(Config::ESTOP_PIN) == LOW) {
    emergencyStop = true; armed = false; autonomousEnabled = false; stopAllMotors();
  }

  static bool lastStart = HIGH;
  const bool startNow = digitalRead(Config::START_PIN);
  if (lastStart == HIGH && startNow == LOW && !emergencyStop && armed) {
    autonomousEnabled = true; headingTargetDeg = headingDeg; enterAutoState(AutoState::LEAVE_START);
  }
  lastStart = startNow;

  const uint32_t nowUs = micros();
  if (nowUs - lastControlUs >= Config::CONTROL_PERIOD_US) {
    const float dt = (nowUs - lastControlUs) * 1.0e-6f;
    lastControlUs = nowUs;
    updateSensors();
    updateAutonomous(dt);

    if (!autonomousEnabled && armed && millis() - lastDriveCommandMs > Config::COMMAND_TIMEOUT_MS)
      setDrive(0.0f, 0.0f, 0.0f);

    if (!emergencyStop && armed) applyMecanumKinematics(commandVx, commandVy, commandWz);
    else stopAllMotors();
    updateWheelControl(dt);
  }
}

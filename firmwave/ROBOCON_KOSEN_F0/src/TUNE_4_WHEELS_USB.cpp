#include <Arduino.h>
#include <Encoder.h>
#include "RobotConfig.h"

using namespace RobotConfig;

// Standalone four-wheel speed PID tuner over Teensy USB Serial (COM14).
// Build/upload with PlatformIO environment: wheel_pid_usb
// Individual-wheel and four-wheel tests are supported. Every motion command
// has an automatic timeout and STOP always zeros all four PWM outputs.
//TEST BR 60 3
enum WheelId : uint8_t { FL, FR, BL, BR, WHEEL_COUNT };

struct PidController {
  float kp, ki, kd;
  float integral = 0.0f;
  float previousError = 0.0f;

  void reset() { integral = 0.0f; previousError = 0.0f; }

  float update(float error, float dt, float low, float high) {
    if (dt <= 0.0f || !isfinite(error)) return 0.0f;
    const float derivative = (error - previousError) / dt;
    const float candidateIntegral = constrain(
        integral + error * dt, -WHEEL_INTEGRAL_LIMIT, WHEEL_INTEGRAL_LIMIT);
    const float candidate = kp * error + ki * candidateIntegral + kd * derivative;
    if (!((candidate > high && error > 0.0f) ||
          (candidate < low && error < 0.0f))) {
      integral = candidateIntegral;
    }
    previousError = error;
    return constrain(kp * error + ki * integral + kd * derivative, low, high);
  }
};

struct WheelChannel {
  Encoder *encoder;
  uint8_t rpwm;
  uint8_t lpwm;
  int8_t motorSign;
  int8_t encoderSign;
  PidController pid;
  float kffPositive;
  float kffNegative;
  int deadzonePositive;
  int deadzoneNegative;
  long count = 0;
  long previousCount = 0;
  float speedMmS = 0.0f;
  float targetMmS = 0.0f;
  int pwm = 0;
};

Encoder encoderFL(FL_ENC_A, FL_ENC_B);
Encoder encoderFR(FR_ENC_A, FR_ENC_B);
Encoder encoderBL(RL_ENC_A, RL_ENC_B);
Encoder encoderBR(RR_ENC_A, RR_ENC_B);

WheelChannel wheels[WHEEL_COUNT] = {
    {&encoderFL, FL_RPWM, FL_LPWM, MOTOR_SIGN_FL, ENCODER_SIGN_FL,
     {WHEEL_KP[0], WHEEL_KI[0], WHEEL_KD[0]}, WHEEL_KFF_POSITIVE[0],
     WHEEL_KFF_NEGATIVE[0], WHEEL_DEADZONE_PWM_POSITIVE[0],
     WHEEL_DEADZONE_PWM_NEGATIVE[0]},
    {&encoderFR, FR_RPWM, FR_LPWM, MOTOR_SIGN_FR, ENCODER_SIGN_FR,
     {WHEEL_KP[1], WHEEL_KI[1], WHEEL_KD[1]}, WHEEL_KFF_POSITIVE[1],
     WHEEL_KFF_NEGATIVE[1], WHEEL_DEADZONE_PWM_POSITIVE[1],
     WHEEL_DEADZONE_PWM_NEGATIVE[1]},
    {&encoderBL, RL_RPWM, RL_LPWM, MOTOR_SIGN_RL, ENCODER_SIGN_RL,
     {WHEEL_KP[2], WHEEL_KI[2], WHEEL_KD[2]}, WHEEL_KFF_POSITIVE[2],
     WHEEL_KFF_NEGATIVE[2], WHEEL_DEADZONE_PWM_POSITIVE[2],
     WHEEL_DEADZONE_PWM_NEGATIVE[2]},
    {&encoderBR, RR_RPWM, RR_LPWM, MOTOR_SIGN_RR, ENCODER_SIGN_RR,
     {WHEEL_KP[3], WHEEL_KI[3], WHEEL_KD[3]}, WHEEL_KFF_POSITIVE[3],
     WHEEL_KFF_NEGATIVE[3], WHEEL_DEADZONE_PWM_POSITIVE[3],
     WHEEL_DEADZONE_PWM_NEGATIVE[3]}
};

const char *const WHEEL_NAMES[WHEEL_COUNT] = {"FL", "FR", "BL", "BR"};
constexpr uint32_t USB_BAUD = 115200;
// Human-readable USB Serial table at 5 Hz. PID control remains at 100 Hz.
constexpr uint32_t TELEMETRY_MS = 200;
constexpr uint32_t DEFAULT_TEST_MS = 5000;
constexpr uint32_t MAX_TEST_MS = 15000;
constexpr uint8_t RPM_BAR_WIDTH = 20;

uint8_t activeWheelMask = 0;
uint32_t testEndsMs = 0;
uint32_t lastControlUs = 0;
uint32_t lastTelemetryMs = 0;
char commandBuffer[96] = {};
size_t commandLength = 0;

float rpmToMmS(float rpm) { return rpm * PI * WHEEL_DIAMETER_MM / 60.0f; }
float mmSToRpm(float mmS) { return mmS * 60.0f / (PI * WHEEL_DIAMETER_MM); }

int wheelIndex(const char *name) {
  for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    if (strcmp(name, WHEEL_NAMES[i]) == 0) return i;
  if (strcmp(name, "RL") == 0) return BL;
  if (strcmp(name, "RR") == 0) return BR;
  return -1;
}

void writeMotor(WheelChannel &wheel, int pwm) {
  pwm = constrain(pwm * wheel.motorSign, -PWM_MAX, PWM_MAX);
  analogWrite(wheel.rpwm, pwm > 0 ? pwm : 0);
  analogWrite(wheel.lpwm, pwm < 0 ? -pwm : 0);
}

void stopAll() {
  activeWheelMask = 0;
  testEndsMs = 0;
  for (WheelChannel &wheel : wheels) {
    wheel.targetMmS = 0.0f;
    wheel.pwm = 0;
    wheel.pid.reset();
    writeMotor(wheel, 0);
  }
}

void printHelp() {
  Serial.println("COMMANDS:");
  Serial.println("  TEST <FL|FR|BL|BR> <rpm> [seconds]  (seconds 1..15)");
  Serial.println("  RUN4_RPM <FL> <FR> <BL> <BR> [seconds]");
  Serial.println("  RUN4_MMS <FL> <FR> <BL> <BR> [seconds]");
  Serial.println("  PID <FL|FR|BL|BR> <kp> <ki> <kd>");
  Serial.println("  FF <FL|FR|BL|BR> <kff>");
  Serial.println("  FFDIR <FL|FR|BL|BR> <positive> <negative>");
  Serial.println("  DEADZONE <FL|FR|BL|BR> <pwm>");
  Serial.println("  DEADZONEDIR <FL|FR|BL|BR> <positive> <negative>");
  Serial.println("  STOP | STATUS | HELP");
  Serial.println("Example: TEST FL 60 5");
  Serial.println("Example: RUN4_MMS 300 300 300 300 5");
  Serial.println("Use negative speed to reverse a wheel; STOP stops immediately.");
}

void printStatus() {
  Serial.println("CONFIG,wheel,kp,ki,kd,kff_pos,kff_neg,dz_pos,dz_neg");
  for (uint8_t i = 0; i < WHEEL_COUNT; ++i) {
    Serial.print("CONFIG,"); Serial.print(WHEEL_NAMES[i]); Serial.print(',');
    Serial.print(wheels[i].pid.kp, 6); Serial.print(',');
    Serial.print(wheels[i].pid.ki, 6); Serial.print(',');
    Serial.print(wheels[i].pid.kd, 6); Serial.print(',');
    Serial.print(wheels[i].kffPositive, 6); Serial.print(',');
    Serial.print(wheels[i].kffNegative, 6); Serial.print(',');
    Serial.print(wheels[i].deadzonePositive); Serial.print(',');
    Serial.println(wheels[i].deadzoneNegative);
  }
}

void processCommand(char *line) {
  char name[4] = {};
  float a = 0.0f, b = 0.0f, c = 0.0f;
  int index = -1;
  if (strcmp(line, "STOP") == 0) {
    stopAll(); Serial.println("ACK,STOP"); return;
  }
  if (strcmp(line, "HELP") == 0) { printHelp(); return; }
  if (strcmp(line, "STATUS") == 0) { printStatus(); return; }

  float speed4[4] = {};
  unsigned run4Seconds = DEFAULT_TEST_MS / 1000;
  int run4Fields = sscanf(line, "RUN4_RPM %f %f %f %f %u",
                          &speed4[0], &speed4[1], &speed4[2], &speed4[3],
                          &run4Seconds);
  bool run4UsesRpm = run4Fields >= 4;
  if (!run4UsesRpm) {
    run4Seconds = DEFAULT_TEST_MS / 1000;
    run4Fields = sscanf(line, "RUN4_MMS %f %f %f %f %u",
                        &speed4[0], &speed4[1], &speed4[2], &speed4[3],
                        &run4Seconds);
  }
  if (run4Fields >= 4) {
    bool valid = true;
    for (uint8_t i = 0; i < WHEEL_COUNT; ++i) {
      const float mmS = run4UsesRpm ? rpmToMmS(speed4[i]) : speed4[i];
      if (!isfinite(mmS) || fabsf(mmS) > MAX_WHEEL_SPEED_MM_S) valid = false;
    }
    if (!valid) {
      Serial.println("ERR,RUN4_SPEED_RANGE");
      return;
    }
    run4Seconds = constrain(run4Seconds, 1U, MAX_TEST_MS / 1000U);
    stopAll();
    for (uint8_t i = 0; i < WHEEL_COUNT; ++i) {
      wheels[i].targetMmS = run4UsesRpm ? rpmToMmS(speed4[i]) : speed4[i];
      wheels[i].pid.reset();
      if (fabsf(wheels[i].targetMmS) >= 0.5f) activeWheelMask |= (1U << i);
    }
    testEndsMs = millis() + run4Seconds * 1000UL;
    Serial.print("ACK,"); Serial.print(run4UsesRpm ? "RUN4_RPM" : "RUN4_MMS");
    for (uint8_t i = 0; i < WHEEL_COUNT; ++i) {
      Serial.print(','); Serial.print(speed4[i], 2);
    }
    Serial.print(','); Serial.println(run4Seconds);
    return;
  }

  unsigned seconds = DEFAULT_TEST_MS / 1000;
  const int testFields = sscanf(line, "TEST %3s %f %u", name, &a, &seconds);
  if (testFields >= 2 && (index = wheelIndex(name)) >= 0 && isfinite(a) &&
      fabsf(a) <= mmSToRpm(MAX_WHEEL_SPEED_MM_S)) {
    seconds = constrain(seconds, 1U, MAX_TEST_MS / 1000U);
    stopAll();
    activeWheelMask = (1U << index);
    wheels[index].targetMmS = rpmToMmS(a);
    wheels[index].pid.reset();
    testEndsMs = millis() + seconds * 1000UL;
    Serial.print("ACK,TEST,"); Serial.print(WHEEL_NAMES[index]); Serial.print(',');
    Serial.print(a, 2); Serial.print(','); Serial.println(seconds);
    return;
  }
  if (sscanf(line, "PID %3s %f %f %f", name, &a, &b, &c) == 4 &&
      (index = wheelIndex(name)) >= 0 && a >= 0 && b >= 0 && c >= 0) {
    wheels[index].pid.kp = a; wheels[index].pid.ki = b; wheels[index].pid.kd = c;
    wheels[index].pid.reset(); Serial.println("ACK,PID"); return;
  }
  if (sscanf(line, "FFDIR %3s %f %f", name, &a, &b) == 3 &&
      (index = wheelIndex(name)) >= 0 && a >= 0 && a <= 1.0f &&
      b >= 0 && b <= 1.0f) {
    wheels[index].kffPositive = a; wheels[index].kffNegative = b;
    wheels[index].pid.reset(); Serial.println("ACK,FFDIR"); return;
  }
  if (sscanf(line, "FF %3s %f", name, &a) == 2 &&
      (index = wheelIndex(name)) >= 0 && a >= 0 && a <= 1.0f) {
    wheels[index].kffPositive = a; wheels[index].kffNegative = a;
    wheels[index].pid.reset();
    Serial.println("ACK,FF"); return;
  }
  int deadzone = 0;
  int deadzoneNegative = 0;
  if (sscanf(line, "DEADZONEDIR %3s %d %d", name, &deadzone,
             &deadzoneNegative) == 3 &&
      (index = wheelIndex(name)) >= 0 && deadzone >= 0 &&
      deadzone < PWM_MAX && deadzoneNegative >= 0 &&
      deadzoneNegative < PWM_MAX) {
    wheels[index].deadzonePositive = deadzone;
    wheels[index].deadzoneNegative = deadzoneNegative;
    wheels[index].pid.reset(); Serial.println("ACK,DEADZONEDIR"); return;
  }
  if (sscanf(line, "DEADZONE %3s %d", name, &deadzone) == 2 &&
      (index = wheelIndex(name)) >= 0 && deadzone >= 0 && deadzone < PWM_MAX) {
    wheels[index].deadzonePositive = deadzone;
    wheels[index].deadzoneNegative = deadzone;
    wheels[index].pid.reset();
    Serial.println("ACK,DEADZONE"); return;
  }
  Serial.println("ERR,BAD_COMMAND (type HELP)");
}

void readCommands() {
  while (Serial.available()) {
    const char ch = static_cast<char>(Serial.read());
    if (ch == '\n' || ch == '\r') {
      if (commandLength) {
        commandBuffer[commandLength] = '\0';
        processCommand(commandBuffer);
        commandLength = 0;
      }
    } else if (commandLength + 1 < sizeof(commandBuffer)) {
      commandBuffer[commandLength++] = ch;
    } else {
      commandLength = 0;
      Serial.println("ERR,COMMAND_TOO_LONG");
    }
  }
}

void updateControl(float dt) {
  const float distancePerCount = RobotConfig::mmPerCount();
  for (uint8_t i = 0; i < WHEEL_COUNT; ++i) {
    WheelChannel &wheel = wheels[i];
    wheel.count = wheel.encoder->read() * wheel.encoderSign;
    const long delta = wheel.count - wheel.previousCount;
    wheel.previousCount = wheel.count;
    const float rawSpeed = dt > 0.0f ? delta * distancePerCount / dt : 0.0f;
    wheel.speedMmS += WHEEL_SPEED_FILTER_ALPHA * (rawSpeed - wheel.speedMmS);

    if (!(activeWheelMask & (1U << i)) || fabsf(wheel.targetMmS) < 0.5f) {
      wheel.pwm = 0; wheel.pid.reset(); writeMotor(wheel, 0); continue;
    }
    const bool positive = wheel.targetMmS > 0.0f;
    const float sign = positive ? 1.0f : -1.0f;
    const float kff = positive ? wheel.kffPositive : wheel.kffNegative;
    const int deadzone = positive ? wheel.deadzonePositive
                                  : wheel.deadzoneNegative;
    const float ff = kff * wheel.targetMmS + sign * deadzone;
    const float correction = wheel.pid.update(
        wheel.targetMmS - wheel.speedMmS, dt, -PWM_MAX - ff, PWM_MAX - ff);
    wheel.pwm = constrain(lroundf(ff + correction), -PWM_MAX, PWM_MAX);
    writeMotor(wheel, wheel.pwm);
  }
}

void makeRpmBar(char *out, float targetRpm, float actualRpm) {
  const float scale = max(20.0f, max(fabsf(targetRpm), fabsf(actualRpm)) * 1.15f);
  const int actualCells = constrain(
      static_cast<int>(lroundf(fabsf(actualRpm) / scale * RPM_BAR_WIDTH)),
      0, RPM_BAR_WIDTH);
  const int targetCell = constrain(
      static_cast<int>(lroundf(fabsf(targetRpm) / scale * (RPM_BAR_WIDTH - 1))),
      0, RPM_BAR_WIDTH - 1);
  out[0] = '[';
  for (uint8_t i = 0; i < RPM_BAR_WIDTH; ++i) out[i + 1] = i < actualCells ? '=' : ' ';
  out[targetCell + 1] = '|';
  out[RPM_BAR_WIDTH + 1] = ']';
  out[RPM_BAR_WIDTH + 2] = '\0';
}

void printWheelBox(uint8_t i) {
  const float target = mmSToRpm(wheels[i].targetMmS);
  const float actual = mmSToRpm(wheels[i].speedMmS);
  char bar[RPM_BAR_WIDTH + 3] = {};
  makeRpmBar(bar, target, actual);
  Serial.println("+----------------------------+");
  Serial.print("| "); Serial.print(WHEEL_NAMES[i]);
  Serial.print(" TARGET: "); Serial.print(target, 2); Serial.println(" RPM");
  Serial.print("| ACTUAL: "); Serial.print(actual, 2);
  Serial.print("  ERROR: "); Serial.println(target - actual, 2);
  Serial.print("| PWM: "); Serial.print(wheels[i].pwm);
  Serial.print("  ENC: "); Serial.println(wheels[i].count);
  Serial.print("| ACT "); Serial.println(bar);
  Serial.println("+----------------------------+");
}

void printTelemetry() {
  if (millis() - lastTelemetryMs < TELEMETRY_MS) return;
  lastTelemetryMs = millis();
  if (activeWheelMask == 0) return;
  Serial.print("\033[2J\033[H");
  Serial.print("ROBOCON PID - 4 O BANH XE     TIME: ");
  Serial.print(millis()); Serial.println(" ms");
  Serial.println("'=' RPM thuc te, '|' RPM target");
  Serial.println("========== HANG TRUOC ==========");
  printWheelBox(FL);
  printWheelBox(FR);
  Serial.println("=========== HANG SAU ===========");
  printWheelBox(BL);
  printWheelBox(BR);
  Serial.println("STOP | RUN4_RPM FL FR BL BR giay");
}

void setup() {
  Serial.begin(USB_BAUD);
  analogWriteResolution(8);
  for (WheelChannel &wheel : wheels) {
    pinMode(wheel.rpwm, OUTPUT); pinMode(wheel.lpwm, OUTPUT);
    analogWriteFrequency(wheel.rpwm, PWM_FREQUENCY_HZ);
    analogWriteFrequency(wheel.lpwm, PWM_FREQUENCY_HZ);
    wheel.previousCount = wheel.encoder->read() * wheel.encoderSign;
    writeMotor(wheel, 0);
  }
  delay(800);
  Serial.println("ROBOCON_WHEEL_PID_USB_TUNER_READY");
  Serial.println("WHEEL_ORDER,FL,FR,BL,BR");
  Serial.println("USB_DASHBOARD_4_WHEELS");
  printStatus();
  printHelp();
  lastControlUs = micros();
}

void loop() {
  readCommands();
  if (activeWheelMask != 0 && static_cast<int32_t>(millis() - testEndsMs) >= 0) {
    stopAll(); Serial.println("ACK,TEST_TIMEOUT_STOP");
  }
  const uint32_t nowUs = micros();
  if (nowUs - lastControlUs >= CONTROL_PERIOD_US) {
    const float dt = (nowUs - lastControlUs) * 1.0e-6f;
    lastControlUs = nowUs;
    updateControl(dt);
  }
  printTelemetry();
}

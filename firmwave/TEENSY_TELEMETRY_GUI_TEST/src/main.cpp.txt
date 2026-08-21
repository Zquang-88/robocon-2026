#include <Arduino.h>
#include <EEPROM.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// Ground radio: USB -> PC.
// Air radio: TX -> Teensy RX8 pin 34, RX -> Teensy TX8 pin 35, GND -> GND.
// Teensy 4.1 UART is 3.3 V only.
#define TELEMETRY Serial8

namespace Settings {
constexpr uint32_t TELEMETRY_BAUD = 57600;
constexpr uint32_t USB_DEBUG_BAUD = 115200;
constexpr uint8_t TELEMETRY_RX_PIN = 34;
constexpr uint8_t TELEMETRY_TX_PIN = 35;
// 20 Hz keeps the full text packet below the usable air bandwidth of common
// SiK/3DR radios configured with a low AIR_SPEED. At 50 Hz the tail of each
// packet can be dropped even though the UART itself is set to 57600 baud.
constexpr uint32_t TELEMETRY_PERIOD_MS = 50;  // 20 Hz
constexpr uint32_t LINK_TIMEOUT_MS = 500;
constexpr uint8_t STATUS_LED_PIN = LED_BUILTIN;
constexpr uint32_t CONFIG_MAGIC = 0x54475549;  // "TGUI"
constexpr uint16_t CONFIG_VERSION = 1;
constexpr size_t COMMAND_BUFFER_SIZE = 192;
constexpr float MAX_SIM_RPM = 2000.0f;
}  // namespace Settings

struct PIDGains {
  float kp;
  float ki;
  float kd;
};

struct RobotConfig {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved;
  PIDGains wheel[4];
  PIDGains heading;
  PIDGains line;
  float targetYaw;
  uint32_t crc;
};

enum WheelIndex : uint8_t { FL = 0, FR = 1, BL = 2, BR = 3, WHEEL_COUNT = 4 };

RobotConfig config{};
float targetRpm[WHEEL_COUNT] = {1200.0f, 1200.0f, 1200.0f, 1200.0f};
float actualRpm[WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
float yawDeg = 0.0f;
float lineError = 0.0f;
float batteryVoltage = 24.6f;
bool emergencyStop = false;
bool armed = true;
uint32_t lastTelemetryMs = 0;
uint32_t lastSimulationUs = 0;
uint32_t lastPcCommandMs = 0;
char commandBuffer[Settings::COMMAND_BUFFER_SIZE]{};
size_t commandLength = 0;

uint32_t crc32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xFFFFFFFFu;
  while (length--) {
    crc ^= *data++;
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}

bool validGain(float value) {
  return isfinite(value) && value >= 0.0f && value <= 10000.0f;
}

bool validConfig(const RobotConfig &candidate) {
  if (candidate.magic != Settings::CONFIG_MAGIC ||
      candidate.version != Settings::CONFIG_VERSION)
    return false;
  const uint32_t expected = crc32(
      reinterpret_cast<const uint8_t *>(&candidate), offsetof(RobotConfig, crc));
  if (candidate.crc != expected || !isfinite(candidate.targetYaw)) return false;
  for (const PIDGains &gains : candidate.wheel)
    if (!validGain(gains.kp) || !validGain(gains.ki) || !validGain(gains.kd))
      return false;
  return validGain(candidate.heading.kp) && validGain(candidate.heading.ki) &&
         validGain(candidate.heading.kd) && validGain(candidate.line.kp) &&
         validGain(candidate.line.ki) && validGain(candidate.line.kd);
}

void loadDefaults() {
  config = {};
  config.magic = Settings::CONFIG_MAGIC;
  config.version = Settings::CONFIG_VERSION;
  config.wheel[FL] = {0.82f, 0.031f, 0.012f};
  config.wheel[FR] = {0.80f, 0.029f, 0.013f};
  config.wheel[BL] = {0.86f, 0.034f, 0.011f};
  config.wheel[BR] = {0.84f, 0.032f, 0.012f};
  config.heading = {3.20f, 0.018f, 0.240f};
  config.line = {1.45f, 0.012f, 0.180f};
  config.targetYaw = 90.0f;
}

void saveConfig() {
  config.magic = Settings::CONFIG_MAGIC;
  config.version = Settings::CONFIG_VERSION;
  config.crc = crc32(reinterpret_cast<const uint8_t *>(&config),
                     offsetof(RobotConfig, crc));
  EEPROM.put(0, config);
}

void loadConfig() {
  RobotConfig stored{};
  EEPROM.get(0, stored);
  if (validConfig(stored)) {
    config = stored;
    Serial.println("[EEPROM] Loaded valid configuration");
  } else {
    loadDefaults();
    saveConfig();
    Serial.println("[EEPROM] Invalid/empty; defaults saved");
  }
}

void sendAck(const char *message) {
  TELEMETRY.print("ACK,");
  TELEMETRY.println(message);
}

void sendError(const char *message) {
  TELEMETRY.print("ERR,");
  TELEMETRY.println(message);
}

void printPidConfig(const char *scope, const char *target,
                    const PIDGains &gains) {
  TELEMETRY.print("CFG,");
  TELEMETRY.print(scope);
  TELEMETRY.print(',');
  TELEMETRY.print(target);
  TELEMETRY.print(',');
  TELEMETRY.print(gains.kp, 6);
  TELEMETRY.print(',');
  TELEMETRY.print(gains.ki, 6);
  TELEMETRY.print(',');
  TELEMETRY.println(gains.kd, 6);
  // SiK radios can drop a long back-to-back UART burst while their air buffer
  // is still draining. Config sync is infrequent, so pace each record.
  TELEMETRY.flush();
  delay(25);
}

void sendCurrentConfig() {
  static const char *const wheelNames[WHEEL_COUNT] = {"FL", "FR", "BL", "BR"};
  TELEMETRY.println("CFG,META,1.0.0,1");
  TELEMETRY.flush();
  delay(25);
  for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    printPidConfig("WHEEL", wheelNames[i], config.wheel[i]);
  printPidConfig("HEADING", "0", config.heading);
  printPidConfig("LINE", "0", config.line);
  TELEMETRY.print("CFG,YAW_TARGET,");
  TELEMETRY.println(config.targetYaw, 2);
  TELEMETRY.flush();
  delay(25);
  TELEMETRY.println("CFG_END");
  TELEMETRY.flush();
  lastTelemetryMs = millis();
}

int wheelIndex(const char *id) {
  if (!id) return -1;
  if (!strcmp(id, "FL")) return FL;
  if (!strcmp(id, "FR")) return FR;
  if (!strcmp(id, "BL") || !strcmp(id, "RL")) return BL;
  if (!strcmp(id, "BR") || !strcmp(id, "RR")) return BR;
  return -1;
}

bool parseFloatStrict(const char *text, float &value) {
  if (!text) return false;
  char *end = nullptr;
  value = strtof(text, &end);
  return end != text && *end == '\0' && isfinite(value);
}

void applyPid(char *scope, char *target, char *kpText, char *kiText,
              char *kdText) {
  float kp = 0.0f, ki = 0.0f, kd = 0.0f;
  if (!scope || !target || !parseFloatStrict(kpText, kp) ||
      !parseFloatStrict(kiText, ki) || !parseFloatStrict(kdText, kd) ||
      !validGain(kp) || !validGain(ki) || !validGain(kd)) {
    sendError("BAD_PID");
    return;
  }

  const PIDGains gains{kp, ki, kd};
  if (!strcmp(scope, "WHEEL")) {
    if (!strcmp(target, "ALL")) {
      for (PIDGains &wheel : config.wheel) wheel = gains;
    } else {
      const int index = wheelIndex(target);
      if (index < 0) {
        sendError("BAD_WHEEL");
        return;
      }
      config.wheel[index] = gains;
    }
  } else if (!strcmp(scope, "HEADING")) {
    config.heading = gains;
  } else if (!strcmp(scope, "LINE")) {
    config.line = gains;
  } else {
    sendError("BAD_SCOPE");
    return;
  }

  TELEMETRY.print("ACK,PID,");
  TELEMETRY.print(scope);
  TELEMETRY.print(',');
  TELEMETRY.println(target);
}

void stopSimulation() {
  for (float &target : targetRpm) target = 0.0f;
}

void applyDirectionalDrive(const char *direction, float speed) {
  speed = constrain(fabsf(speed), 0.0f, Settings::MAX_SIM_RPM);
  if (!strcmp(direction, "FORWARD")) {
    targetRpm[FL] = speed; targetRpm[FR] = speed;
    targetRpm[BL] = speed; targetRpm[BR] = speed;
  } else if (!strcmp(direction, "BACKWARD")) {
    targetRpm[FL] = -speed; targetRpm[FR] = -speed;
    targetRpm[BL] = -speed; targetRpm[BR] = -speed;
  } else if (!strcmp(direction, "LEFT")) {
    targetRpm[FL] = -speed; targetRpm[FR] = speed;
    targetRpm[BL] = speed; targetRpm[BR] = -speed;
  } else if (!strcmp(direction, "RIGHT")) {
    targetRpm[FL] = speed; targetRpm[FR] = -speed;
    targetRpm[BL] = -speed; targetRpm[BR] = speed;
  } else if (!strcmp(direction, "ROTATE_CW")) {
    targetRpm[FL] = speed; targetRpm[FR] = -speed;
    targetRpm[BL] = speed; targetRpm[BR] = -speed;
  } else if (!strcmp(direction, "ROTATE_CCW")) {
    targetRpm[FL] = -speed; targetRpm[FR] = speed;
    targetRpm[BL] = -speed; targetRpm[BR] = speed;
  } else if (!strcmp(direction, "STOP")) {
    stopSimulation();
  } else {
    sendError("BAD_DIRECTION");
    return;
  }
  sendAck("DRIVE");
}

void applyNumericDrive(float vx, float vy, float wz) {
  const float scale = 500.0f;
  targetRpm[FL] = constrain((vx - vy - wz) * scale, -Settings::MAX_SIM_RPM, Settings::MAX_SIM_RPM);
  targetRpm[FR] = constrain((vx + vy + wz) * scale, -Settings::MAX_SIM_RPM, Settings::MAX_SIM_RPM);
  targetRpm[BL] = constrain((vx + vy - wz) * scale, -Settings::MAX_SIM_RPM, Settings::MAX_SIM_RPM);
  targetRpm[BR] = constrain((vx - vy + wz) * scale, -Settings::MAX_SIM_RPM, Settings::MAX_SIM_RPM);
  sendAck("DRIVE");
}

void processCommand(char *packet) {
  Serial.print("RX: ");
  Serial.println(packet);
  lastPcCommandMs = millis();

  char *save = nullptr;
  char *command = strtok_r(packet, ",", &save);
  if (!command) return;

  if (!strcmp(command, "PING")) {
    TELEMETRY.println("HELLO,TEENSY_GUI_TEST,1.0.0,1");
    return;
  }
  if (!strcmp(command, "HEARTBEAT")) {
    TELEMETRY.print("HB,");
    TELEMETRY.print(millis());
    TELEMETRY.print(',');
    TELEMETRY.print(armed ? 1 : 0);
    TELEMETRY.print(',');
    TELEMETRY.println(emergencyStop ? 1 : 0);
    return;
  }
  if (!strcmp(command, "GET_CONFIG")) {
    sendCurrentConfig();
    return;
  }
  if (!strcmp(command, "SAVE_CONFIG") || !strcmp(command, "PROFILE_SAVE")) {
    saveConfig();
    sendAck("CONFIG_SAVED");
    return;
  }
  if (!strcmp(command, "PROFILE_LOAD")) {
    loadConfig();
    sendCurrentConfig();
    sendAck("CONFIG_LOADED");
    return;
  }
  if (!strcmp(command, "FACTORY_RESET")) {
    loadDefaults();
    saveConfig();
    sendCurrentConfig();
    sendAck("FACTORY_RESET");
    return;
  }
  if (!strcmp(command, "ESTOP")) {
    emergencyStop = true;
    armed = false;
    stopSimulation();
    sendAck("ESTOP");
    return;
  }
  if (!strcmp(command, "RUN") || !strcmp(command, "CLEAR_ESTOP")) {
    emergencyStop = false;
    armed = true;
    sendAck("RUN");
    return;
  }
  if (!strcmp(command, "YAW")) {
    float target = 0.0f;
    if (!parseFloatStrict(strtok_r(nullptr, ",", &save), target)) {
      sendError("BAD_YAW");
      return;
    }
    config.targetYaw = fmodf(target + 360.0f, 360.0f);
    sendAck("YAW");
    return;
  }
  if (!strcmp(command, "PID")) {
    char *scope = strtok_r(nullptr, ",", &save);
    char *target = strtok_r(nullptr, ",", &save);
    char *kp = strtok_r(nullptr, ",", &save);
    char *ki = strtok_r(nullptr, ",", &save);
    char *kd = strtok_r(nullptr, ",", &save);
    applyPid(scope, target, kp, ki, kd);
    return;
  }
  if (!strcmp(command, "DRIVE")) {
    char *first = strtok_r(nullptr, ",", &save);
    if (!first) {
      sendError("BAD_DRIVE");
      return;
    }
    if (isalpha(static_cast<unsigned char>(first[0]))) {
      float speed = 1200.0f;
      const char *speedText = strtok_r(nullptr, ",", &save);
      if (speedText && !parseFloatStrict(speedText, speed)) {
        sendError("BAD_SPEED");
        return;
      }
      applyDirectionalDrive(first, speed);
    } else {
      float vx = 0.0f, vy = 0.0f, wz = 0.0f;
      if (!parseFloatStrict(first, vx) ||
          !parseFloatStrict(strtok_r(nullptr, ",", &save), vy) ||
          !parseFloatStrict(strtok_r(nullptr, ",", &save), wz)) {
        sendError("BAD_DRIVE");
        return;
      }
      applyNumericDrive(vx, vy, wz);
    }
    return;
  }

  sendError("UNKNOWN_COMMAND");
}

void updateTelemetryInput() {
  while (TELEMETRY.available()) {
    const char c = static_cast<char>(TELEMETRY.read());
    if (c == '\n' || c == '\r') {
      if (commandLength > 0) {
        commandBuffer[commandLength] = '\0';
        processCommand(commandBuffer);
        commandLength = 0;
      }
    } else if (commandLength < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLength++] = c;
    } else {
      commandLength = 0;
      sendError("PACKET_TOO_LONG");
    }
  }
}

float wrapAngleError(float target, float current) {
  float error = fmodf(target - current + 540.0f, 360.0f) - 180.0f;
  return error;
}

void updateSimulation() {
  const uint32_t nowUs = micros();
  if (lastSimulationUs == 0) {
    lastSimulationUs = nowUs;
    return;
  }
  const float dt = constrain((nowUs - lastSimulationUs) * 1e-6f, 0.0f, 0.05f);
  lastSimulationUs = nowUs;

  for (uint8_t i = 0; i < WHEEL_COUNT; ++i) {
    const float wanted = (armed && !emergencyStop) ? targetRpm[i] : 0.0f;
    const float response = constrain(0.5f + config.wheel[i].kp * 2.5f, 0.5f, 8.0f);
    const float simulatedMeasurement =
        wanted + sinf(millis() * 0.004f + i * 1.7f) * 4.0f;
    actualRpm[i] += (simulatedMeasurement - actualRpm[i]) *
                    constrain(response * dt, 0.0f, 1.0f);
  }

  const float yawError = wrapAngleError(config.targetYaw, yawDeg);
  yawDeg += yawError * constrain((0.05f + config.heading.kp * 0.08f) * dt, 0.0f, 1.0f);
  if (yawDeg < 0.0f) yawDeg += 360.0f;
  if (yawDeg >= 360.0f) yawDeg -= 360.0f;
  lineError = sinf(millis() * 0.0016f) * 1.8f;
  batteryVoltage = 24.6f + sinf(millis() * 0.0002f) * 0.08f;
}

void sendTelemetry() {
  if (millis() - lastTelemetryMs < Settings::TELEMETRY_PERIOD_MS) return;
  lastTelemetryMs = millis();
  TELEMETRY.printf("TEL,%lu,%.1f,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f\n",
                   millis(), actualRpm[FL], actualRpm[FR], actualRpm[BL],
                   actualRpm[BR], yawDeg, lineError, batteryVoltage);
}

void updateStatusLed() {
  const bool linkActive = millis() - lastPcCommandMs <= Settings::LINK_TIMEOUT_MS;
  digitalWrite(Settings::STATUS_LED_PIN,
               linkActive ? HIGH : ((millis() / 500) % 2 ? HIGH : LOW));
}

void setup() {
  pinMode(Settings::STATUS_LED_PIN, OUTPUT);
  Serial.begin(Settings::USB_DEBUG_BAUD);
  TELEMETRY.setRX(Settings::TELEMETRY_RX_PIN);
  TELEMETRY.setTX(Settings::TELEMETRY_TX_PIN);
  TELEMETRY.begin(Settings::TELEMETRY_BAUD);
  delay(250);
  loadConfig();
  lastPcCommandMs = millis() - Settings::LINK_TIMEOUT_MS - 1;
  Serial.println("TEENSY TELEMETRY GUI TEST v1.0.0");
  Serial.println("Serial8: RX8 pin 34, TX8 pin 35, 57600 baud");
}

void loop() {
  updateTelemetryInput();
  updateSimulation();
  sendTelemetry();
  updateStatusLed();
}

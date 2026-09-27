#include "UartProtocol.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "HardwareConfig.h"

namespace {
constexpr uint8_t POINT_B_FINAL_LIFT_ACTIVE_STEP = 6;
constexpr uint8_t DROP_2B_HOME_ACTIVE_STEP = 2;

char *trimAscii(char *text) {
  while (*text) {
    const uint8_t c = static_cast<uint8_t>(*text);
    if (c == ' ' || c == '\t' || c < 0x20 || c > 0x7E) ++text;
    else break;
  }
  char *end = text + strlen(text);
  while (end > text) {
    const uint8_t c = static_cast<uint8_t>(end[-1]);
    if (c == ' ' || c == '\t' || c < 0x20 || c > 0x7E) --end;
    else break;
  }
  *end = '\0';
  return text;
}

void uppercaseAscii(char *text) {
  while (*text) {
    *text = static_cast<char>(toupper(static_cast<unsigned char>(*text)));
    ++text;
  }
}

}  // namespace

UartProtocol::UartProtocol(Stream &usb, Stream &robot,
                           MechanismController &controller,
                           MechanismConfig &config,
                           PersistentConfig &persistentConfig)
    : usb_(usb), robot_(robot), controller_(controller), config_(config),
      persistentConfig_(persistentConfig) {}

void UartProtocol::begin() {
  cancelButtonGestures();
  pinMode(HardwareConfig::E18_A_PIN, INPUT_PULLUP);
  pinMode(HardwareConfig::E18_B_PIN, INPUT_PULLUP);
  emit("BOOT,ESP32_MECHANISM,1");
  emit("ACK,UART2,RX16,TX15,57600");
  emit("ACK,HOME_SWITCHES,A36,B37,ACTIVE_LOW");
  emit("ACK,LOCAL_BUTTONS,VALVES11,HOME12,ACTIVE_LOW");
  emit("ACK,E18_INPUTS,A17,B18,ACTIVE_LOW");
  emit("ACK,RGB_FIELD_LED,G19,R20,B21");
  emitFieldStatus();
  emitCapabilities();
  if (controller_.valvesAvailable()) {
    char pcf[40];
    snprintf(pcf, sizeof(pcf), "ACK,PCF8574,0X%02X",
             controller_.pcf8574Address());
    emit(pcf);
  } else {
    emit("ERROR_PCF8574");
  }
  emitHomeStatus();
  emitE18Status();
  emitStatus();
}

void UartProtocol::emit(const char *line) {
  usb_.println(line);
  robot_.println(line);
}

void UartProtocol::emitAck(const char *token) {
  char line[96];
  snprintf(line, sizeof(line), "ACK,%s", token ? token : "OK");
  emit(line);
}

void UartProtocol::emitE18Status() {
  const bool a = digitalRead(HardwareConfig::E18_A_PIN) ==
                 HardwareConfig::E18_ACTIVE_LEVEL;
  const bool b = digitalRead(HardwareConfig::E18_B_PIN) ==
                 HardwareConfig::E18_ACTIVE_LEVEL;
  char line[48];
  snprintf(line, sizeof(line), "E18,STATUS,%u,%u,%u",
           a ? 1U : 0U, b ? 1U : 0U, e18Armed_ ? 1U : 0U);
  emit(line);
}

void UartProtocol::emitHomeStatus() {
  char line[64];
  snprintf(line, sizeof(line), "HOME,STATUS,%u,%u,%u,%u,0X%02X",
           controller_.homeSwitchAActive() ? 1U : 0U,
           controller_.homeSwitchBActive() ? 1U : 0U,
           controller_.zeroed() ? 1U : 0U,
           controller_.valvesAvailable() ? 1U : 0U,
           controller_.pcf8574Address());
  emit(line);
}

void UartProtocol::updateE18() {
  const bool a = digitalRead(HardwareConfig::E18_A_PIN) ==
                 HardwareConfig::E18_ACTIVE_LEVEL;
  const bool b = digitalRead(HardwareConfig::E18_B_PIN) ==
                 HardwareConfig::E18_ACTIVE_LEVEL;
  if (!e18Armed_) {
    e18BothSinceMs_ = 0;
    return;
  }
  if (!(a && b)) {
    e18BothSinceMs_ = 0;
    if (e18Latched_) {
      e18Latched_ = false;
      emit("EVENT,E18_BOTH,0");
      emitE18Status();
    }
    return;
  }
  if (e18Latched_) return;
  if (e18BothSinceMs_ == 0) e18BothSinceMs_ = millis();
  if (millis() - e18BothSinceMs_ < HardwareConfig::E18_BOTH_CONFIRM_MS) return;
  e18Latched_ = true;
  emit("EVENT,E18_BOTH,1");
  emitE18Status();
}

void UartProtocol::cancelButtonGestures() {
  valveClickPending_ = false;
  valveDoubleClickCanEnable_ = false;
  valveButton_.begin(HardwareConfig::VALVE_BUTTON_PIN);
  homeButton_.begin(HardwareConfig::HOME_BUTTON_PIN);
}

void UartProtocol::emitFieldStatus() {
  char line[32];
  snprintf(line, sizeof(line), "ACK,FIELD,%s",
           controller_.competitionFieldName());
  emit(line);
}

void UartProtocol::closeValvesFromButton() {
  // Stop the active sequence first, otherwise its next step could overwrite
  // the requested closed state. A latched fault is never cleared here.
  const bool canEnergizeCloseSide =
      controller_.fault() == MechanismFault::None;
  readyQueued_ = false;
  e18Armed_ = false;
  e18Latched_ = false;
  e18BothSinceMs_ = 0;
  controller_.emergencyStop("STOP");
  resetCompetitionLatches();
  emit("STOPPED");
  if (!controller_.valvesAvailable()) {
    emit("ERROR_PCF8574");
  } else if (!canEnergizeCloseSide) {
    emitError("BUTTON11_CLOSE_REJECTED", controller_.faultText());
  } else if (!controller_.setValveMask(0xF0)) {
    emitError("BUTTON11_CLOSE_REJECTED", controller_.faultText());
  } else {
    // Enabled-mask 0xF0 means P4-P7 active and P0-P3 inactive. The
    // ValveController applies the common 50 ms break-before-make interval.
    emitAck("BUTTON11,P4_P7_ON_CLOSED");
  }
  emitStatus();
}

void UartProtocol::updateLocalButtons() {
  const uint32_t now = millis();
  const bool valvePress = valveButton_.takePress();
  const bool homePress = homeButton_.takePress();

  if (homePress) {
    valveClickPending_ = false;
    if (controller_.busy()) {
      emit("BUSY");
      emitError("BUTTON12_HOME_REJECTED", "BUSY");
    } else if (!controller_.startHome(true)) {
      emitError("BUTTON12_HOME_REJECTED", controller_.faultText());
    } else {
      readyQueued_ = false;
      resetCompetitionLatches();
      emitAck("BUTTON12,HOME_B_THEN_A,8000,5500");
      emitHomeStatus();
      emitStatus();
    }
    return;
  }

  // Expire only the possibility of a second click. The first click has already
  // requested the P4-P7 closed direction.
  if (valveClickPending_ &&
      now - valveFirstClickMs_ > HardwareConfig::BUTTON_DOUBLE_CLICK_MS) {
    valveClickPending_ = false;
    valveDoubleClickCanEnable_ = false;
  }
  if (!valvePress) return;
  if (!valveClickPending_) {
    valveDoubleClickCanEnable_ =
        !controller_.busy() && controller_.fault() == MechanismFault::None;
    valveFirstClickMs_ = now;
    valveClickPending_ = true;
    closeValvesFromButton();
    return;
  }

  valveClickPending_ = false;
  if (!valveDoubleClickCanEnable_) {
    valveDoubleClickCanEnable_ = false;
    emit("BUSY");
    emitError("BUTTON11_ON_REJECTED", "BUSY");
  } else if (controller_.busy()) {
    valveDoubleClickCanEnable_ = false;
    emit("BUSY");
    emitError("BUTTON11_ON_REJECTED", "BUSY");
  } else if (!controller_.setValveMask(0x0F)) {
    valveDoubleClickCanEnable_ = false;
    emitError("BUTTON11_ON_REJECTED", controller_.faultText());
  } else {
    valveDoubleClickCanEnable_ = false;
    // One PCF mask selects all four work coils together, after the existing
    // 50 ms pair interlock. Opposite coils P4-P7 remain HIGH/OFF.
    emitAck("BUTTON11,P0_P3_ON_REQUESTED");
    emitStatus();
  }
}

void UartProtocol::emitError(const char *code, const char *detail) {
  char line[128];
  if (detail && *detail) snprintf(line, sizeof(line), "ERR,%s,%s", code, detail);
  else snprintf(line, sizeof(line), "ERR,%s", code);
  emit(line);
}

void UartProtocol::readStream(Stream &stream, Receiver &receiver) {
  size_t budget = HardwareConfig::UART_LINE_CAPACITY;
  while (budget-- && stream.available()) {
    const char c = static_cast<char>(stream.read());
    if (c == '\r' || c == '\n') {
      if (receiver.dropping) { receiver.dropping = false; receiver.length = 0; continue; }
      if (receiver.length) {
        receiver.buffer[receiver.length] = '\0';
        processLine(receiver.buffer);
        receiver.length = 0;
      }
    } else if (receiver.dropping) {
      continue;
    } else if (receiver.length < sizeof(receiver.buffer) - 1) {
      receiver.buffer[receiver.length++] = c;
    } else {
      receiver.length = 0;
      receiver.dropping = true;
      emitError("LINE_TOO_LONG");
    }
  }
}

bool UartProtocol::runNamedProfile(const char *name) {
  ProfileId id;
  if (!parseProfileId(name, id)) { emitError("PROFILE_UNKNOWN", name); return false; }
  if (!controller_.startProfile(id)) {
    emitError(controller_.busy() ? "BUSY" : "PROFILE_REJECTED", controller_.faultText());
    return false;
  }
  char ack[48];
  snprintf(ack, sizeof(ack), "STARTED,%s", profileName(id));
  emitAck(ack);
  return true;
}

void UartProtocol::resetCompetitionLatches() {
  pointAComplete_ = false;
  pointBComplete_ = false;
  pointCComplete_ = false;
  bridgeBComplete_ = false;
  drop2AComplete_ = false;
  drop2BComplete_ = false;
}

bool UartProtocol::startCompetitionCommand(const char *wireCommand,
                                            CompetitionAction action) {
  const char *operation = "ROBOT_START";
  bool alreadyComplete = false;
  if (action == CompetitionAction::PointA) {
    operation = "PICK_A";
    alreadyComplete = pointAComplete_;
  } else if (action == CompetitionAction::PointB) {
    operation = "PICK_B";
    alreadyComplete = pointBComplete_;
  } else if (action == CompetitionAction::PointC) {
    operation = "PICK_C";
    alreadyComplete = pointCComplete_;
  } else if (action == CompetitionAction::BridgeBFinish) {
    operation = "BRIDGE_B";
    alreadyComplete = bridgeBComplete_;
  } else if (action == CompetitionAction::Drop2A) {
    operation = "THA2A";
    alreadyComplete = drop2AComplete_;
  } else if (action == CompetitionAction::Drop2B) {
    operation = "THA2B";
    alreadyComplete = drop2BComplete_;
  }

  if (action == CompetitionAction::ReadyHomeA) {
    if (controller_.homing()) {
      resetCompetitionLatches();
      readyQueued_ = true;
      emitAck("QUEUED,ROBOT_START,WAIT_HOME");
      return true;
    }
    resetCompetitionLatches();
  } else if (alreadyComplete) {
    if (action == CompetitionAction::PointA) emit("DONE_POINT_A");
    else if (action == CompetitionAction::PointB) emit("DONE_POINT_B");
    else if (action == CompetitionAction::PointC) emit("DONE_POINT_C");
    else if (action == CompetitionAction::BridgeBFinish) emit("DONE_BRIDGE_B");
    else if (action == CompetitionAction::Drop2A) emit("DONE_THA_2A");
    else if (action == CompetitionAction::Drop2B) emit("DONE_THA_2B");
    return true;
  }

  if (controller_.busy()) {
    emit("BUSY");
    return false;
  }
  if (!controller_.zeroed()) {
    emitError("NOT_ZEROED", wireCommand);
    return false;
  }
  if (!controller_.startCompetitionAction(action)) {
    emitError("ACTION_REJECTED", controller_.faultText());
    return false;
  }
  char ack[56];
  snprintf(ack, sizeof(ack), "STARTED,%s", operation);
  emitAck(ack);
  return true;
}

bool UartProtocol::updateProfile(const char *arguments) {
  char name[16] = {};
  float distanceA, distanceB, speedA, speedB, accelA, accelB;
  float stepsRevA, stepsRevB, pitchA, pitchB, correctionA, correctionB;
  int directionA, directionB, absoluteMode;
  unsigned long postWaitMs, timeoutMs;
  const int parsed = sscanf(arguments,
      "%15[^,],%f,%f,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%lu,%lu,%d",
      name, &distanceA, &distanceB, &directionA, &directionB,
      &speedA, &speedB, &accelA, &accelB, &stepsRevA, &stepsRevB,
      &pitchA, &pitchB, &correctionA, &correctionB,
      &postWaitMs, &timeoutMs, &absoluteMode);
  if (parsed != 18) { emitError("PROFILE_FORMAT"); return false; }
  uppercaseAscii(name);
  ProfileId id;
  if (!parseProfileId(name, id)) { emitError("PROFILE_UNKNOWN", name); return false; }

  MechanismConfig candidate = config_;
  StepperProfile &p = candidate.profiles[static_cast<uint8_t>(id)];
  p.distanceAmm = distanceA; p.distanceBmm = distanceB;
  p.directionA = static_cast<int8_t>(directionA);
  p.directionB = static_cast<int8_t>(directionB);
  p.maxSpeedA = speedA; p.maxSpeedB = speedB;
  p.accelerationA = accelA; p.accelerationB = accelB;
  p.stepsPerRevA = stepsRevA; p.stepsPerRevB = stepsRevB;
  p.screwPitchAmm = pitchA; p.screwPitchBmm = pitchB;
  p.correctionA = correctionA; p.correctionB = correctionB;
  p.postWaitMs = static_cast<uint32_t>(postWaitMs);
  p.timeoutMs = static_cast<uint32_t>(timeoutMs);
  p.absoluteMode = absoluteMode != 0;
  if (p.stepCount >= 3 && p.steps[2].type == SequenceStepType::WaitTime)
    p.steps[2].durationMs = p.postWaitMs;

  char reason[32];
  if (!validateMechanismConfig(candidate, reason, sizeof(reason))) {
    emitError("CONFIG_REJECTED", reason); return false;
  }
  config_ = candidate;
  config_.crc32 = mechanismConfigCrc(config_);
  emitAck("PROFILE_UPDATED_RAM");
  emitProfile(id);
  return true;
}

void UartProtocol::emitCapabilities() {
  emit("CAPS,JOG,1,500,10,30000");
  emit("CAPS,PCF,0X20,ACTIVE_LOW,PAIRS,P0-P7,P1-P6,P2-P5,P3-P4");
  emit("CAPS,BRIDGE_STAGED_LIFT,A1700,B566.7_PLUS_1133.3");
}

void UartProtocol::processJog(const char *arguments) {
  if (strcmp(arguments, "STOP") == 0) {
    controller_.stopJog();
    emitAck("JOG_STOPPED");
    emitStatus();
    return;
  }
  float a = 0.0f, b = 0.0f;
  int consumed = 0;
  if (sscanf(arguments, "%f,%f%n", &a, &b, &consumed) != 2 ||
      consumed == 0 || arguments[consumed] != '\0') {
    emitError("JOG_FORMAT_SIGNED_SPEED_A_B");
    return;
  }
  const bool changed = a != controller_.jogSpeed(0) || b != controller_.jogSpeed(1);
  if (!controller_.setJog(a, b)) {
    emitError(controller_.busy() && !controller_.jogging() ? "BUSY" : "JOG_REJECTED",
              controller_.faultText());
    return;
  }
  if (changed) { emitAck("JOG_UPDATED"); emitStatus(); }
}

void UartProtocol::processLine(char *rawLine) {
  char *line = trimAscii(rawLine);
  uppercaseAscii(line);
  if (!*line) return;

  if (strcmp(line, "STOP") == 0 || strcmp(line, "ESTOP") == 0 ||
      strcmp(line, "CMD,STOP") == 0) {
    cancelButtonGestures();
    readyQueued_ = false;
    e18Armed_ = false; e18Latched_ = false; e18BothSinceMs_ = 0;
    controller_.emergencyStop("STOP");
    emit("STOPPED");
    if (!controller_.valvesAvailable()) emit("ERROR_PCF8574");
    emitStatus();
    return;
  }
  if (strcmp(line, "GET,STATUS") == 0 || strcmp(line, "STATUS") == 0 ||
      strcmp(line, "POS") == 0 || strcmp(line, "PCF,STATUS") == 0) {
    emitStatus(); return;
  }
  if (strcmp(line, "HELP") == 0) {
    emit("HELP,PCF,P0,ON");
    emit("HELP,PCF,P0,OFF");
    emit("HELP,PCF,ALL,OFF");
    emit("HELP,PCF,STATUS");
    emit("HELP,LEGACY,VALVE,1-8,0-1");
    emit("HELP,BRIDGE_STABLE,FINISH_B_REMAINING_1133.3MM");
    return;
  }
  // Idempotent post-bridge handshake. A retry must be safe and must not
  // start a mechanism profile again. Acknowledge it even if the controller
  // is busy so Teensy cannot remain blocked after completing its 70 mm move.
  if (strcmp(line, "E18,ARM") == 0) {
    e18Armed_ = true; e18Latched_ = false; e18BothSinceMs_ = 0;
    emitAck("E18_ARMED"); emitE18Status(); return;
  }
  if (strcmp(line, "E18,DISARM") == 0) {
    e18Armed_ = false; e18Latched_ = false; e18BothSinceMs_ = 0;
    emitAck("E18_DISARMED"); emitE18Status(); return;
  }
  if (strcmp(line, "GET,E18") == 0 || strcmp(line, "E18,STATUS") == 0) {
    emitE18Status(); return;
  }
  if (strcmp(line, "ROBOT START") == 0 ||
      strcmp(line, "ROBOT_START") == 0 || strcmp(line, "AUTO") == 0) {
    startCompetitionCommand(line, CompetitionAction::ReadyHomeA);
    return;
  }
  if (strcmp(line, "POINT_A") == 0) {
    startCompetitionCommand(line, CompetitionAction::PointA);
    return;
  }
  if (strcmp(line, "POINT_B") == 0) {
    startCompetitionCommand(line, CompetitionAction::PointB);
    return;
  }
  if (strcmp(line, "POINT_C") == 0) {
    startCompetitionCommand(line, CompetitionAction::PointC);
    return;
  }
  if (strcmp(line, "BRIDGE_STABLE") == 0) {
    if (!pointBComplete_ && !pointCComplete_) {
      emitError("SEQUENCE", "BRIDGE_REQUIRES_POINT_B_OR_C");
      return;
    }
    startCompetitionCommand(line, CompetitionAction::BridgeBFinish);
    return;
  }
  if (strcmp(line, "DONE_THA_2A") == 0 ||
      strcmp(line, "DONE_THA2A") == 0 || strcmp(line, "THA_2A") == 0) {
    startCompetitionCommand(line, CompetitionAction::Drop2A);
    return;
  }
  if (strcmp(line, "CHO_THA2B") == 0 ||
      strcmp(line, "CHO_THA_2B") == 0 ||
      strcmp(line, "DONE_THA_2B") == 0 ||
      strcmp(line, "DONE_THA2B") == 0 || strcmp(line, "THA_2B") == 0) {
    startCompetitionCommand(line, CompetitionAction::Drop2B);
    return;
  }
  // Refresh and release are allowed while manual jog owns the controller.
  if (strncmp(line, "JOG,", 4) == 0) { processJog(line + 4); return; }
  if (controller_.busy()) { emitError("BUSY", controller_.activeProfileName()); return; }

  if (strcmp(line, "PING") == 0) emit("HELLO,ESP32_MECHANISM,1");
  else if (strcmp(line, "PCF,ALL,OFF") == 0) {
    if (controller_.allValveCoilsOff()) {
      emitAck("PCF,ALL,OFF");
      emitStatus();
    } else {
      emitError("PCF_ALL_OFF_REJECTED", controller_.faultText());
    }
  }
  else if (strncmp(line, "PCF,P", 5) == 0) {
    unsigned index = 0;
    char state[8] = {};
    int consumed = 0;
    const int parsed = sscanf(line, "PCF,P%u,%7s%n", &index, state, &consumed);
    const bool stateValid = strcmp(state, "ON") == 0 ||
                            strcmp(state, "OFF") == 0;
    if (parsed != 2 || line[consumed] != '\0' ||
        index >= HardwareConfig::VALVE_COUNT || !stateValid) {
      emitError("PCF_FORMAT_USE_P0_TO_P7_ON_OFF");
    } else {
      const bool enabled = strcmp(state, "ON") == 0;
      if (!controller_.setValve(static_cast<uint8_t>(index), enabled))
        emitError("PCF_VALVE_REJECTED", controller_.faultText());
      else {
        char ack[32];
        snprintf(ack, sizeof(ack), "PCF,P%u,%s", index,
                 enabled ? "ON" : "OFF");
        emitAck(ack);
        emitStatus();
      }
    }
  }
  else if (strcmp(line, "GET,CONFIG") == 0 || strcmp(line, "READ_CONFIG") == 0) emitConfig();
  else if (strcmp(line, "SAVE") == 0 || strcmp(line, "SAVE_CONFIG") == 0) {
    char reason[32];
    if (persistentConfig_.save(config_, reason, sizeof(reason))) { emitAck("CONFIG_SAVED_NVS"); emitConfig(); }
    else emitError("NVS_SAVE", reason);
  } else if (strcmp(line, "RESTORE_DEFAULT") == 0 || strcmp(line, "DEFAULTS") == 0) {
    loadDefaultMechanismConfig(config_); emitAck("DEFAULTS_LOADED_RAM_SAVE_TO_PERSIST"); emitConfig();
  } else if (strcmp(line, "CLEAR_FAULT") == 0 || strcmp(line, "RESET") == 0) {
    controller_.clearFault(); emitAck("FAULT_CLEARED");
  } else if (strcmp(line, "SET_ZERO") == 0 || strcmp(line, "SET0") == 0) {
    controller_.setSoftwareZero(); emitAck("SOFTWARE_ZERO_SET"); emitStatus();
  } else if (strcmp(line, "HOME") == 0 || strcmp(line, "CMD,HOME") == 0) {
    if (!controller_.startHome()) emitError("HOME_REJECTED", controller_.faultText());
    else emitAck("STARTED,HOME");
  } else if (strncmp(line, "CMD,", 4) == 0) runNamedProfile(line + 4);
  else if (strcmp(line, "MANUAL") == 0) {
    cancelButtonGestures();
    readyQueued_ = false;
    controller_.emergencyStop("STOP"); emitAck("MANUAL");
  }
  if (strcmp(line, "GET,FIELD") == 0) {
    emitFieldStatus();
    return;
  }
  if (strcmp(line, "FIELD,RED") == 0 || strcmp(line, "FIELD,BLUE") == 0) {
    const MechanismField requested = strcmp(line, "FIELD,BLUE") == 0
        ? MechanismField::Blue : MechanismField::Red;
    if (!controller_.setCompetitionField(requested)) {
      emitError("BUSY", "FIELD_CHANGE");
      return;
    }
    emitFieldStatus();
    emitStatus();
    return;
  }
  else if (strncmp(line, "VALVE,", 6) == 0) {
    unsigned index = 0, enabled = 0;
    if (sscanf(line + 6, "%u,%u", &index, &enabled) != 2 || index < 1 ||
        index > HardwareConfig::VALVE_COUNT || enabled > 1) emitError("VALVE_FORMAT");
    else if (!controller_.setValve(static_cast<uint8_t>(index - 1), enabled != 0)) emitError("VALVE_REJECTED", controller_.faultText());
    else { char ack[32]; snprintf(ack, sizeof(ack), "VALVE,%u,%u", index, enabled); emitAck(ack); emitStatus(); }
  } else if (strncmp(line, "SET,PROFILE,", 12) == 0) updateProfile(line + 12);
  else if (strncmp(line, "SPEED ", 6) == 0) {
    const float speed = strtof(line + 6, nullptr);
    if (!isfinite(speed) || speed < 10 || speed > 30000) emitError("SPEED_RANGE_10_30000");
    else { manualSpeed_ = speed; emitAck("MANUAL_SPEED_UPDATED"); }
  } else if (strncmp(line, "NANG ", 5) == 0 || strncmp(line, "HA ", 3) == 0) {
    float a = 0, b = 0; const bool raise = line[0] == 'N';
    const char *arguments = line + (raise ? 5 : 3);
    if (sscanf(arguments, "%f %f", &a, &b) != 2 ||
        !controller_.startDirectMove((raise ? -1.0f : 1.0f) * a,
                                     (raise ? -1.0f : 1.0f) * b, manualSpeed_))
      emitError("DIRECT_MOVE_REJECTED");
    else emitAck("DIRECT_MOVE_STARTED");
  } else emitError("UNKNOWN_COMMAND", line);
}

void UartProtocol::emitProfile(ProfileId id) {
  const StepperProfile &p = config_.profiles[static_cast<uint8_t>(id)];
  char line[256];
  snprintf(line, sizeof(line),
      "CONFIG,PROFILE,%s,%.3f,%.3f,%d,%d,%.1f,%.1f,%.1f,%.1f,%.3f,%.3f,%.3f,%.3f,%.5f,%.5f,%lu,%lu,%u",
      profileName(id), p.distanceAmm, p.distanceBmm, p.directionA, p.directionB,
      p.maxSpeedA, p.maxSpeedB, p.accelerationA, p.accelerationB,
      p.stepsPerRevA, p.stepsPerRevB, p.screwPitchAmm, p.screwPitchBmm,
      p.correctionA, p.correctionB, static_cast<unsigned long>(p.postWaitMs),
      static_cast<unsigned long>(p.timeoutMs), p.absoluteMode ? 1U : 0U);
  emit(line);
}

void UartProtocol::emitConfig() {
  emitCapabilities();
  char header[128];
  snprintf(header, sizeof(header), "CONFIG,HEADER,%u,%lu,%.1f,%.1f,%.1f,%.1f",
      config_.version, static_cast<unsigned long>(config_.crc32),
      config_.minimumPositionAmm, config_.maximumPositionAmm,
      config_.minimumPositionBmm, config_.maximumPositionBmm);
  emit(header);
  for (uint8_t i = 0; i < PROFILE_COUNT; ++i) emitProfile(static_cast<ProfileId>(i));
  emit("CONFIG,END");
}

void UartProtocol::emitStatus() {
  char line[192];
  snprintf(line, sizeof(line), "STATUS,%s,%u,%u,%.3f,%.3f,%u,%s,%u,%u,%.1f,%.1f",
      controller_.stateName(), controller_.busy() ? 1U : 0U,
      static_cast<unsigned>(controller_.fault()), controller_.positionAmm(),
      controller_.positionBmm(), controller_.valveMask(),
      controller_.activeProfileName(), controller_.activeStep(), controller_.zeroed() ? 1U : 0U,
      controller_.jogSpeed(0), controller_.jogSpeed(1));
  emit(line); lastStatusMs_ = millis();
}

void UartProtocol::update() {
  // Check timeout before processing a late refresh: it may not restart motion.
  controller_.update();
  // Read local inputs first so an UART STOP received in this iteration wins.
  updateLocalButtons();
  readStream(robot_, robotReceiver_);
  readStream(usb_, usbReceiver_);
  updateE18();
  controller_.update();
  char eventText[32];
  if (controller_.takeFaultEvent(eventText, sizeof(eventText))) {
    cancelButtonGestures();
    if (strstr(eventText, "PCF8574") != nullptr)
      emit("ERROR_PCF8574");
    emitError("MECHANISM_FAULT", eventText);
  }
  char valveLog[80];
  if (controller_.takeValveLogEvent(valveLog, sizeof(valveLog))) emit(valveLog);
  // Hold the chassis while A/B return together to READY_HOME_A. Release it
  // when A starts its 1700 mm lift and B starts its first 850 mm stage, so
  // Teensy bridge travel and mechanism motion run concurrently.
  const bool pointBOrCActive =
      strcmp(controller_.activeProfileName(), "PICK_B") == 0 ||
      strcmp(controller_.activeProfileName(), "PICK_C") == 0;
  if (controller_.competitionActionRunning() && pointBOrCActive &&
      controller_.activeStep() >= POINT_B_FINAL_LIFT_ACTIVE_STEP) {
    bool releasedNow = false;
    if (strcmp(controller_.activeProfileName(), "PICK_C") == 0 &&
        !pointCComplete_) {
      pointCComplete_ = true;
      emit("DONE_POINT_C");
      releasedNow = true;
    } else if (strcmp(controller_.activeProfileName(), "PICK_B") == 0 &&
               !pointBComplete_) {
      pointBComplete_ = true;
      emit("DONE_POINT_B");
      releasedNow = true;
    }
    // Chi phat trang thai dung mot lan luc nha Teensy. Neu phat o moi vong
    // lap trong buoc nang cuoi, UART se bi ngap du lieu va giao dien bi lag.
    if (releasedNow) emitStatus();
  }
  // THA_2B pneumatic work is complete at step 2. Release Teensy now so the
  // robot can travel while ESP32 homes the mechanism in the background.
  if (!drop2BComplete_ && controller_.competitionActionRunning() &&
      strcmp(controller_.activeProfileName(), "THA2B") == 0 &&
      controller_.activeStep() >= DROP_2B_HOME_ACTIVE_STEP) {
    drop2BComplete_ = true;
    emit("DONE_THA_2B");
    emitStatus();
  }
  if (controller_.takeDoneEvent(eventText, sizeof(eventText))) {
    const bool pointBAlreadyReleased =
        strcmp(eventText, "PICK_B") == 0 && pointBComplete_;
    const bool pointCAlreadyReleased =
        strcmp(eventText, "PICK_C") == 0 && pointCComplete_;
    const bool drop2BAlreadyReleased =
        strcmp(eventText, "THA2B") == 0 && drop2BComplete_;
    if (strcmp(eventText, "PICK_A") == 0) pointAComplete_ = true;
    else if (strcmp(eventText, "PICK_B") == 0) pointBComplete_ = true;
    else if (strcmp(eventText, "PICK_C") == 0) pointCComplete_ = true;
    else if (strcmp(eventText, "BRIDGE_B") == 0) bridgeBComplete_ = true;
    else if (strcmp(eventText, "THA2A") == 0) drop2AComplete_ = true;
    else if (strcmp(eventText, "THA2B") == 0) drop2BComplete_ = true;

    if (strcmp(eventText, "PICK_A") == 0) emit("DONE_POINT_A");
    else if (strcmp(eventText, "PICK_B") == 0 && !pointBAlreadyReleased)
      emit("DONE_POINT_B");
    else if (strcmp(eventText, "PICK_C") == 0 && !pointCAlreadyReleased)
      emit("DONE_POINT_C");
    else if (strcmp(eventText, "BRIDGE_B") == 0) emit("DONE_BRIDGE_B");
    else if (strcmp(eventText, "THA2A") == 0) emit("DONE_THA_2A");
    else if (strcmp(eventText, "THA2B") == 0 && !drop2BAlreadyReleased)
      emit("DONE_THA_2B");
    emitStatus();
    if (strcmp(eventText, "HOME") == 0) emitHomeStatus();
  }
  if (readyQueued_ && !controller_.busy() && controller_.zeroed() &&
      controller_.fault() == MechanismFault::None) {
    readyQueued_ = false;
    startCompetitionCommand("ROBOT START", CompetitionAction::ReadyHomeA);
  }
  if (millis() - lastStatusMs_ >= HardwareConfig::STATUS_PERIOD_MS) emitStatus();
}


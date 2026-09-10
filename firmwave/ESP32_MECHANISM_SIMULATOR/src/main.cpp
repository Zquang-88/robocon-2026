#include <Arduino.h>
#include <cstring>

namespace {

// Physical UART wiring confirmed on the robot:
// ESP32 RX GPIO16 <- Teensy TX6 pin 24
// ESP32 TX GPIO15 -> Teensy RX6 pin 25
// Both controllers must share GND.
constexpr int kRobotRxPin = 16;
constexpr int kRobotTxPin = 15;
constexpr uint32_t kRobotBaud = 57600;
constexpr uint32_t kSimulatedActionMs = 4000;
constexpr size_t kLineCapacity = 32;

HardwareSerial robotSerial(2);

char lineBuffer[kLineCapacity] = {};
size_t lineLength = 0;
char pendingCommand[kLineCapacity] = {};
bool actionPending = false;
uint32_t actionStartedMs = 0;

bool isSupportedCommand(const char *command) {
  return strcmp(command, "POINT_A") == 0 ||
         strcmp(command, "ROBOT START") == 0 ||
         strcmp(command, "POINT_B") == 0 ||
         strcmp(command, "THA_2B") == 0 ||
         strcmp(command, "THA_2A") == 0 ||
         strcmp(command, "DONE_THA_2A") == 0 ||
         strcmp(command, "DONE_THA_2B") == 0 ||
         strcmp(command, "CHO_THA2B") == 0;
}

const char *simulationAction(const char *command) {
  return strncmp(command, "SIM,", 4) == 0 ? command + 4 : command;
}

char *trimAscii(char *text) {
  while (*text == ' ' || *text == '\t') {
    ++text;
  }

  char *end = text + strlen(text);
  while (end > text && (end[-1] == ' ' || end[-1] == '\t')) {
    --end;
  }
  *end = '\0';
  return text;
}

void startSimulatedAction(const char *command) {
  if (strcmp(command, "STOP") == 0 || strcmp(command, "ESTOP") == 0) {
    actionPending = false;
    pendingCommand[0] = '\0';
    robotSerial.println("ACK,STOPPED_SAFE");
#ifdef LED_BUILTIN
    digitalWrite(LED_BUILTIN, LOW);
#endif
    Serial.println("SIMULATION_STOPPED");
    return;
  }

  const char *action = simulationAction(command);
  if (!isSupportedCommand(action)) {
    Serial.print("IGNORED_UNKNOWN_COMMAND: ");
    Serial.println(command);
    return;
  }

  // The robot sends only one mechanism command at a time. If electrical noise
  // creates a duplicate while waiting, do not restart the action timer.
  if (actionPending) {
    Serial.print("IGNORED_WHILE_BUSY: ");
    Serial.println(command);
    return;
  }

  strncpy(pendingCommand, action, sizeof(pendingCommand) - 1);
  pendingCommand[sizeof(pendingCommand) - 1] = '\0';
  actionStartedMs = millis();
  actionPending = true;

#ifdef LED_BUILTIN
  digitalWrite(LED_BUILTIN, HIGH);
#endif

  Serial.print("RECEIVED: ");
  Serial.print(pendingCommand);
  Serial.println("; simulating mechanism for 4 seconds");
}

void processCompleteLine() {
  lineBuffer[lineLength] = '\0';
  char *command = trimAscii(lineBuffer);
  if (*command != '\0') {
    startSimulatedAction(command);
  }
  lineLength = 0;
}

void readRobotCommands() {
  while (robotSerial.available() > 0) {
    const char received = static_cast<char>(robotSerial.read());

    // Teensy uses println(), which normally produces CRLF. Process the line on
    // the first delimiter and ignore the empty delimiter that follows it.
    if (received == '\r' || received == '\n') {
      if (lineLength > 0) {
        processCompleteLine();
      }
      continue;
    }

    if (lineLength < sizeof(lineBuffer) - 1) {
      lineBuffer[lineLength++] = received;
    } else {
      lineLength = 0;
      Serial.println("IGNORED_RX_LINE_TOO_LONG");
    }
  }
}

void finishSimulatedActionWhenDue() {
  if (!actionPending || millis() - actionStartedMs < kSimulatedActionMs) {
    return;
  }

  const char *response = "DONE";
  if (strcmp(pendingCommand, "ROBOT START") == 0)
    response = "DONE,ROBOT_START";
  else if (strcmp(pendingCommand, "POINT_A") == 0)
    response = "DONE,PICK_A";
  else if (strcmp(pendingCommand, "POINT_B") == 0)
    response = "DONE,PICK_B";
  else if (strcmp(pendingCommand, "DONE_THA_2A") == 0)
    response = "DONE_THA2A";
  else if (strcmp(pendingCommand, "DONE_THA_2B") == 0 ||
           strcmp(pendingCommand, "CHO_THA2B") == 0)
    response = "DONE_THA2B";
  robotSerial.println(response);

  Serial.print("SENT: ");
  Serial.print(response);
  Serial.print(" for ");
  Serial.println(pendingCommand);

  actionPending = false;
  pendingCommand[0] = '\0';

#ifdef LED_BUILTIN
  digitalWrite(LED_BUILTIN, LOW);
#endif
}

}  // namespace

void setup() {
  Serial.begin(115200);
  robotSerial.begin(kRobotBaud, SERIAL_8N1, kRobotRxPin, kRobotTxPin);

#ifdef LED_BUILTIN
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
#endif

  Serial.println();
  Serial.println("ESP32_MECHANISM_SIMULATOR_READY");
  Serial.println("UART2 RX=GPIO16 TX=GPIO15 BAUD=57600");
  Serial.println("Commands: POINT_A, POINT_B, THA_2B, THA_2A, DONE_THA_2A, DONE_THA_2B, SIM,<command>, STOP");
  Serial.println("Commands normally receive DONE after 4 seconds; DONE_THA_2A receives DONE_THA2A; DONE_THA_2B receives DONE_THA2B");
}

void loop() {
  readRobotCommands();
  finishSimulatedActionWhenDue();
}

#include <Arduino.h>
#include <cstring>

namespace {

// ESP32 DevKit V1 UART2 pins. Connect RX2 to Teensy TX6 (pin 24), TX2 to
// Teensy RX6 (pin 25), and connect the grounds of both controllers.
constexpr int kRobotRxPin = 16;
constexpr int kRobotTxPin = 17;
constexpr uint32_t kRobotBaud = 115200;
constexpr uint32_t kSimulatedActionMs = 10000;
constexpr size_t kLineCapacity = 32;

HardwareSerial robotSerial(2);

char lineBuffer[kLineCapacity] = {};
size_t lineLength = 0;
char pendingCommand[kLineCapacity] = {};
bool actionPending = false;
uint32_t actionStartedMs = 0;

bool isSupportedCommand(const char *command) {
  return strcmp(command, "POINT_A") == 0 ||
         strcmp(command, "POINT_B") == 0 ||
         strcmp(command, "THA_2B") == 0 ||
         strcmp(command, "THA_2A") == 0;
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
  if (!isSupportedCommand(command)) {
    Serial.print("IGNORED_UNKNOWN_COMMAND: ");
    Serial.println(command);
    return;
  }

  // The robot sends only one mechanism command at a time. If electrical noise
  // creates a duplicate while waiting, do not restart the 10-second timer.
  if (actionPending) {
    Serial.print("IGNORED_WHILE_BUSY: ");
    Serial.println(command);
    return;
  }

  strncpy(pendingCommand, command, sizeof(pendingCommand) - 1);
  pendingCommand[sizeof(pendingCommand) - 1] = '\0';
  actionStartedMs = millis();
  actionPending = true;

#ifdef LED_BUILTIN
  digitalWrite(LED_BUILTIN, HIGH);
#endif

  Serial.print("RECEIVED: ");
  Serial.print(pendingCommand);
  Serial.println("; simulating mechanism for 10 seconds");
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

  // The main Teensy firmware performs an exact, case-sensitive comparison
  // against "DONE" and expects a CR/LF-delimited line.
  robotSerial.println("DONE");

  Serial.print("SENT: DONE for ");
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
  Serial.println("UART2 RX=GPIO16 TX=GPIO17 BAUD=115200");
  Serial.println("Commands: POINT_A, POINT_B, THA_2B, THA_2A");
  Serial.println("Each valid command receives DONE after 10 seconds");
}

void loop() {
  readRobotCommands();
  finishSimulatedActionWhenDue();
}


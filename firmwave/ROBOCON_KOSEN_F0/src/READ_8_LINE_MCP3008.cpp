#include <Arduino.h>
#include <SPI.h>

// ============================================================
// Standalone diagnostic: 8 center line sensors via MCP3008
// Board : Teensy 4.1
// SPI   : SPI0 (MOSI 11, MISO 12, SCK 13)
// CS    : pin 26
// Serial: USB Serial, 115200 baud
//
// PlatformIO environment: line_reader
// This program never configures or drives the motor pins.
// ============================================================

namespace LineReaderConfig {
constexpr uint8_t MCP3008_CS = 26;
constexpr uint32_t SPI_HZ = 1000000;
constexpr uint32_t PRINT_PERIOD_MS = 100;  // 10 samples/second
constexpr uint8_t SENSOR_COUNT = 8;
}

uint16_t rawValue[LineReaderConfig::SENSOR_COUNT] = {};
uint16_t sensorMin[LineReaderConfig::SENSOR_COUNT] = {};
uint16_t sensorMax[LineReaderConfig::SENSOR_COUNT] = {};
bool streamingEnabled = true;
bool calibrationActive = false;
uint32_t lastPrintMs = 0;

uint16_t readMCP3008Raw(uint8_t channel) {
  digitalWrite(LineReaderConfig::MCP3008_CS, LOW);
  SPI.transfer(0x01);
  const uint8_t high = SPI.transfer((0x08 | (channel & 0x07)) << 4);
  const uint8_t low = SPI.transfer(0x00);
  digitalWrite(LineReaderConfig::MCP3008_CS, HIGH);
  return static_cast<uint16_t>(((high & 0x03) << 8) | low);
}

void readAllSensors() {
  SPI.beginTransaction(SPISettings(LineReaderConfig::SPI_HZ, MSBFIRST, SPI_MODE0));
  for (uint8_t i = 0; i < LineReaderConfig::SENSOR_COUNT; ++i) {
    rawValue[i] = readMCP3008Raw(i);
  }
  SPI.endTransaction();

  if (calibrationActive) {
    for (uint8_t i = 0; i < LineReaderConfig::SENSOR_COUNT; ++i) {
      sensorMin[i] = min(sensorMin[i], rawValue[i]);
      sensorMax[i] = max(sensorMax[i], rawValue[i]);
    }
  }
}

void resetCalibration() {
  for (uint8_t i = 0; i < LineReaderConfig::SENSOR_COUNT; ++i) {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }
}

void printArray(const char *label, const uint16_t *values) {
  Serial.print(label);
  for (uint8_t i = 0; i < LineReaderConfig::SENSOR_COUNT; ++i) {
    Serial.print(',');
    Serial.print(values[i]);
  }
  Serial.println();
}

void printCalibration() {
  printArray("CENTER_MIN", sensorMin);
  printArray("CENTER_MAX", sensorMax);
  Serial.print("CENTER_RANGE");
  for (uint8_t i = 0; i < LineReaderConfig::SENSOR_COUNT; ++i) {
    Serial.print(',');
    Serial.print(sensorMax[i] >= sensorMin[i] ? sensorMax[i] - sensorMin[i] : 0);
  }
  Serial.println();

  Serial.print("constexpr uint16_t CENTER_SENSOR_MIN[8] = {");
  for (uint8_t i = 0; i < LineReaderConfig::SENSOR_COUNT; ++i) {
    if (i) Serial.print(',');
    Serial.print(sensorMin[i]);
  }
  Serial.println("};");
  Serial.print("constexpr uint16_t CENTER_SENSOR_MAX[8] = {");
  for (uint8_t i = 0; i < LineReaderConfig::SENSOR_COUNT; ++i) {
    if (i) Serial.print(',');
    Serial.print(sensorMax[i]);
  }
  Serial.println("};");
}

void printHelp() {
  Serial.println("COMMANDS:");
  Serial.println("  R = resume raw stream");
  Serial.println("  P = pause raw stream");
  Serial.println("  C = start calibration (move all eyes over floor and line)");
  Serial.println("  S = stop calibration and print MIN/MAX/RANGE");
  Serial.println("  H = show this help");
}

void processSerialCommands() {
  while (Serial.available() > 0) {
    const char command = static_cast<char>(toupper(Serial.read()));
    if (command == 'R') {
      streamingEnabled = true;
      Serial.println("ACK,STREAM,ON");
    } else if (command == 'P') {
      streamingEnabled = false;
      Serial.println("ACK,STREAM,OFF");
    } else if (command == 'C') {
      resetCalibration();
      calibrationActive = true;
      streamingEnabled = true;
      Serial.println("ACK,CALIBRATION,START");
    } else if (command == 'S') {
      calibrationActive = false;
      Serial.println("ACK,CALIBRATION,STOP");
      printCalibration();
    } else if (command == 'H') {
      printHelp();
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LineReaderConfig::MCP3008_CS, OUTPUT);
  digitalWrite(LineReaderConfig::MCP3008_CS, HIGH);
  SPI.begin();
  resetCalibration();

  const uint32_t waitStartedMs = millis();
  while (!Serial && millis() - waitStartedMs < 2000) {}
  Serial.println("READ_8_LINE_MCP3008_READY");
  Serial.println("FORMAT: LINE8,millis,C1,C2,C3,C4,C5,C6,C7,C8");
  printHelp();
}

void loop() {
  processSerialCommands();
  readAllSensors();

  const uint32_t now = millis();
  if (streamingEnabled && now - lastPrintMs >= LineReaderConfig::PRINT_PERIOD_MS) {
    lastPrintMs = now;
    Serial.print("LINE8,");
    Serial.print(now);
    for (uint8_t i = 0; i < LineReaderConfig::SENSOR_COUNT; ++i) {
      Serial.print(',');
      Serial.print(rawValue[i]);
    }
    Serial.println();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

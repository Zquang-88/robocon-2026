#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <string.h>

#define OPTICAL_FLOW_BAUD       115200
#define MAVLINK_V1_STX          0xFE
#define MAVLINK_DISTANCE_SENSOR 132

enum ParserState {
  WAIT_STX,
  READ_LENGTH,
  READ_SEQUENCE,
  READ_SYSTEM_ID,
  READ_COMPONENT_ID,
  READ_MESSAGE_ID,
  READ_PAYLOAD,
  READ_CRC_1,
  READ_CRC_2
};

ParserState parserState = WAIT_STX;

uint8_t payload[255];
uint8_t payloadLength = 0;
uint8_t payloadIndex = 0;
uint8_t messageID = 0;

float distanceCm = -1.0f;
bool distanceValid = false;

unsigned long lastMessageTime = 0;
unsigned long lastPrintTime = 0;

template <typename T>
T readValue(const uint8_t *data, uint8_t offset) {
  T value;
  memcpy(&value, data + offset, sizeof(T));
  return value;
}
void resetParser() {
  parserState = WAIT_STX;
  payloadLength = 0;
  payloadIndex = 0;
  messageID = 0;
}

void decodeDistanceSensor() {
  if (payloadLength < 14) {
    return;
  }

  uint16_t minDistance =
      readValue<uint16_t>(payload, 4);

  uint16_t maxDistance =
      readValue<uint16_t>(payload, 6);

  uint16_t currentDistance =
      readValue<uint16_t>(payload, 8);

  bool valid =
      currentDistance != 0 &&
      currentDistance != UINT16_MAX &&
      currentDistance >= minDistance &&
      currentDistance <= maxDistance;

  if (valid) {
    distanceCm = currentDistance;
    distanceValid = true;
    lastMessageTime = millis();
  } else {
    distanceValid = false;
  }
}

void processMessage() {
  if (messageID == MAVLINK_DISTANCE_SENSOR) {
    decodeDistanceSensor();
  }
}

void parseMavlinkByte(uint8_t data) {
  switch (parserState) {
    case WAIT_STX:
      if (data == MAVLINK_V1_STX) {
        parserState = READ_LENGTH;
      }
      break;

    case READ_LENGTH:
      payloadLength = data;
      payloadIndex = 0;
      parserState = READ_SEQUENCE;
      break;

    case READ_SEQUENCE:
      parserState = READ_SYSTEM_ID;
      break;

    case READ_SYSTEM_ID:
      parserState = READ_COMPONENT_ID;
      break;

    case READ_COMPONENT_ID:
      parserState = READ_MESSAGE_ID;
      break;

    case READ_MESSAGE_ID:
      messageID = data;

      if (payloadLength == 0) {
        parserState = READ_CRC_1;
      } else {
        parserState = READ_PAYLOAD;
      }
      break;

    case READ_PAYLOAD:
      if (payloadIndex < sizeof(payload)) {
        payload[payloadIndex++] = data;
      } else {
        resetParser();
        break;
      }

      if (payloadIndex >= payloadLength) {
        parserState = READ_CRC_1;
      }
      break;

    case READ_CRC_1:
      parserState = READ_CRC_2;
      break;

    case READ_CRC_2:
      processMessage();
      resetParser();
      break;

    default:
      resetParser();
      break;
  }
}

void setup() {
  Serial.begin(115200);

  while (!Serial && millis() < 3000) {
  }

  // Teensy 4.1 Serial8:
  // Optical Flow TX -> chân 34 RX8
  // Chân 35 TX8 không cần nối nếu chỉ nhận
  Serial8.begin(OPTICAL_FLOW_BAUD);

  Serial.println("=== DOC KHOANG CACH OPTICAL FLOW ===");
}

void loop() {
  while (Serial8.available() > 0) {
    parseMavlinkByte((uint8_t)Serial8.read());
  }

  // Mất dữ liệu quá 500 ms thì báo không hợp lệ
  if (millis() - lastMessageTime > 500) {
    distanceValid = false;
  }

  // In kết quả mỗi 100 ms
  if (millis() - lastPrintTime >= 100) {
    lastPrintTime = millis();

    if (distanceValid) {
      Serial.print("Distance: ");
      Serial.print(distanceCm, 0);
      Serial.print(" cm | ");
      Serial.print(distanceCm / 100.0f, 2);
      Serial.println(" m");
    } else {
      Serial.println("Distance: KHONG CO DU LIEU");
    }
  }
}

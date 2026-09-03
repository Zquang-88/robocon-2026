#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>

// Standalone VL53L1X diagnostic for Teensy 4.1.
// Wire1: SDA pin 17, SCL pin 16. USB Serial: 115200 baud.
// No motor or encoder pins are configured by this program.

namespace TofReaderConfig {
constexpr uint8_t EXPECTED_ADDRESS = 0x29;
constexpr uint32_t I2C_HZ = 100000;
constexpr uint32_t MEASUREMENT_PERIOD_MS = 50;
constexpr uint32_t PRINT_PERIOD_MS = 100;
constexpr uint16_t MIN_VALID_MM = 35;
constexpr uint16_t MAX_VALID_MM = 3500;
}

VL53L1X tof;
bool tofReady = false;
uint32_t lastPrintMs = 0;
uint32_t sampleCount = 0;

void scanI2cBus() {
  Serial.println("I2C_SCAN_BEGIN");
  uint8_t found = 0;
  for (uint8_t address = 1; address < 127; ++address) {
    Wire1.beginTransmission(address);
    if (Wire1.endTransmission() == 0) {
      Serial.print("I2C_DEVICE,0x");
      if (address < 16) Serial.print('0');
      Serial.println(address, HEX);
      ++found;
    }
  }
  Serial.print("I2C_SCAN_END,COUNT=");
  Serial.println(found);
}

bool initializeTof() {
  tof.setBus(&Wire1);
  tof.setTimeout(60);
  if (!tof.init()) {
    Serial.println("ERR,VL53L1X_NOT_FOUND_AT_0x29");
    return false;
  }

  tof.setDistanceMode(VL53L1X::Long);
  tof.setMeasurementTimingBudget(50000);
  tof.startContinuous(TofReaderConfig::MEASUREMENT_PERIOD_MS);
  Serial.println("ACK,VL53L1X_READY,LONG_MODE,50MS");
  return true;
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  const uint32_t waitStartedMs = millis();
  while (!Serial && millis() - waitStartedMs < 2000) {}

  Wire1.begin();
  Wire1.setClock(TofReaderConfig::I2C_HZ);
  Serial.println("READ_TOF_VL53L1X_READY");
  Serial.println("WIRE1,SDA=17,SCL=16,I2C=100000");
  scanI2cBus();
  tofReady = initializeTof();
  Serial.println("FORMAT: TOF,millis,distance_mm,range_status,valid,sample_count");
}

void loop() {
  static uint32_t lastPeriodicScanMs = 0;
  if (millis() - lastPeriodicScanMs >= 2000) {
    lastPeriodicScanMs = millis();
    scanI2cBus();
  }
  if (!tofReady) {
    static uint32_t lastRetryMs = 0;
    if (millis() - lastRetryMs >= 2000) {
      lastRetryMs = millis();
      scanI2cBus();
      tofReady = initializeTof();
    }
    return;
  }

  if (!tof.dataReady()) return;
  const uint16_t distanceMm = tof.read(false);
  const bool timedOut = tof.timeoutOccurred();
  const uint8_t rangeStatus = tof.ranging_data.range_status;
  const bool valid = !timedOut && rangeStatus == VL53L1X::RangeValid &&
                     distanceMm >= TofReaderConfig::MIN_VALID_MM &&
                     distanceMm <= TofReaderConfig::MAX_VALID_MM;
  ++sampleCount;

  const uint32_t now = millis();
  if (now - lastPrintMs >= TofReaderConfig::PRINT_PERIOD_MS) {
    lastPrintMs = now;
    Serial.print("TOF,");
    Serial.print(now);Serial.print(',');
    Serial.print(distanceMm);Serial.print(',');
    Serial.print(rangeStatus);Serial.print(',');
    Serial.print(valid ? 1 : 0);Serial.print(',');
    Serial.println(sampleCount);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

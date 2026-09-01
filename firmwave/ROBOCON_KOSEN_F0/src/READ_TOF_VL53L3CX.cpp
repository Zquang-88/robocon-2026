#include <Arduino.h>
#include <Wire.h>
#include <PWFusion_VL53L3C.h>

// Standalone reader for DFRobot SEN0378 / VL53L3CX on Teensy 4.1.
// Wiring: VCC->3V3 or 5V, GND->GND, SDA->17, SCL->16.
// XSHUT and GPIO1 are not required for this polling-mode test.

namespace {

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t I2C_CLOCK_HZ = 100000;
constexpr uint32_t TIMING_BUDGET_US = 100000;
constexpr uint32_t NO_DATA_WARNING_MS = 1000;

VL53L3C tof;
uint32_t lastMeasurementMs = 0;
uint32_t lastWarningMs = 0;

bool readRegister8(uint16_t reg, uint8_t &value) {
  Wire1.beginTransmission(0x29);
  Wire1.write(static_cast<uint8_t>(reg >> 8));
  Wire1.write(static_cast<uint8_t>(reg & 0xFF));
  const uint8_t status = Wire1.endTransmission(false);
  if (status != 0) {
    Serial.print("ERR,I2C_REGISTER_SELECT,status=");
    Serial.println(status);
    return false;
  }
  if (Wire1.requestFrom(static_cast<uint8_t>(0x29),
                        static_cast<uint8_t>(1)) != 1) {
    Serial.println("ERR,I2C_REGISTER_READ");
    return false;
  }
  value = static_cast<uint8_t>(Wire1.read());
  return true;
}

void printMeasurement(const MeasurmentResult &measurement) {
  Serial.print("VL53L3CX,objects=");
  Serial.print(measurement.numObjs);

  for (uint8_t i = 0; i < measurement.numObjs; ++i) {
    const RangeData &target = measurement.rangeData[i];
    Serial.print(",target=");
    Serial.print(i);
    Serial.print(",distance_mm=");
    Serial.print(target.Range);
    Serial.print(",min_mm=");
    Serial.print(target.RangeMin);
    Serial.print(",max_mm=");
    Serial.print(target.RangeMax);
    Serial.print(",signal_mcps=");
    Serial.print(target.SignalRate, 3);
    Serial.print(",ambient_mcps=");
    Serial.print(target.AmbientRate, 3);
    Serial.print(",sigma_mm=");
    Serial.print(target.Sigma, 2);
  }
  Serial.println();
}

}  // namespace

void setup() {
  Serial.begin(SERIAL_BAUD);
  const uint32_t serialWaitStartedMs = millis();
  while (!Serial && millis() - serialWaitStartedMs < 2000) {
    yield();
  }

  Serial.println("VL53L3CX_TEST_START");
  Serial.println("I2C=Wire1,SDA=17,SCL=16,address=0x29");

  Wire1.begin();
  Wire1.setClock(I2C_CLOCK_HZ);

  uint8_t modelId = 0;
  uint8_t moduleType = 0;
  if (readRegister8(0x010F, modelId) && readRegister8(0x0110, moduleType)) {
    Serial.print("VL53L3CX_ID,model=0x");
    Serial.print(modelId, HEX);
    Serial.print(",module=0x");
    Serial.println(moduleType, HEX);
  } else {
    Serial.println("FATAL,VL53L3CX_ID_READ_FAILED");
    return;
  }

  tof.begin(Wire1);

  VL53LX_Error status = tof.setDistanceMode(DIST_LONG);
  if (status != VL53LX_ERROR_NONE) {
    Serial.print("ERR,setDistanceMode,status=");
    Serial.println(status);
  }

  status = tof.setTimingBudget(TIMING_BUDGET_US);
  if (status != VL53LX_ERROR_NONE) {
    Serial.print("ERR,setTimingBudget,status=");
    Serial.println(status);
  }

  status = tof.startMeasurement();
  if (status != VL53LX_ERROR_NONE) {
    Serial.print("ERR,startMeasurement,status=");
    Serial.println(status);
  } else {
    Serial.println("VL53L3CX_MEASUREMENT_STARTED");
  }

  lastMeasurementMs = millis();
}

void loop() {
  if (tof.dataIsReady()) {
    MeasurmentResult measurement{};
    const VL53LX_Error status = tof.getMeasurmentData(&measurement);

    if (status == VL53LX_ERROR_NONE) {
      printMeasurement(measurement);
      lastMeasurementMs = millis();
    } else {
      Serial.print("ERR,getMeasurement,status=");
      Serial.println(status);
    }

    const VL53LX_Error restartStatus = tof.startNextMeasurement();
    if (restartStatus != VL53LX_ERROR_NONE) {
      Serial.print("ERR,startNextMeasurement,status=");
      Serial.println(restartStatus);
    }
  }

  const uint32_t nowMs = millis();
  if (nowMs - lastMeasurementMs > NO_DATA_WARNING_MS &&
      nowMs - lastWarningMs > NO_DATA_WARNING_MS) {
    lastWarningMs = nowMs;
    Serial.println("WARN,VL53L3CX_NO_NEW_DATA");
  }
}

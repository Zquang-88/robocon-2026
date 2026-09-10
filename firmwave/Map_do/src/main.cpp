#include <AccelStepper.h>
#include <Arduino.h>

#include "HardwareConfig.h"
#include "MechanismController.h"
#include "PersistentConfig.h"
#include "StepperProfiles.h"
#include "UartProtocol.h"
#include "ValveController.h"

HardwareSerial robotSerial(2);
AccelStepper motorA(AccelStepper::DRIVER,
                    HardwareConfig::STEP_A_PIN, HardwareConfig::DIR_A_PIN);
AccelStepper motorB(AccelStepper::DRIVER,
                    HardwareConfig::STEP_B_PIN, HardwareConfig::DIR_B_PIN);
ValveController valves;
MechanismConfig mechanismConfig;
PersistentConfig persistentConfig;
MechanismController mechanism(motorA, motorB, valves);
UartProtocol protocol(Serial, robotSerial, mechanism,
                      mechanismConfig, persistentConfig);

void setup() {
  // Safety first: if ESP32 resets while PCF8574 remains powered, its previous
  // output latch may still energize a coil. Write 0xFF before UART/NVS/HOME.
  valves.begin();

  Serial.begin(HardwareConfig::USB_BAUD);
  robotSerial.begin(HardwareConfig::ROBOT_UART_BAUD, SERIAL_8N1,
                    HardwareConfig::ROBOT_UART_RX_PIN,
                    HardwareConfig::ROBOT_UART_TX_PIN);

  char reason[32];
  if (!persistentConfig.load(mechanismConfig, reason, sizeof(reason))) {
    loadDefaultMechanismConfig(mechanismConfig);
    Serial.print("WARN,CONFIG_DEFAULTS_RAM,");
    Serial.println(reason);
  } else {
    Serial.println("ACK,CONFIG_LOADED_NVS");
  }

  mechanism.begin(mechanismConfig);
  protocol.begin();
}

void loop() {
  protocol.update();
}

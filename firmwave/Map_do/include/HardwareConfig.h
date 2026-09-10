#pragma once

#include <Arduino.h>

namespace HardwareConfig {

// USB Serial is reserved for local diagnostics.  The robot link is a separate
// UART so uploading/monitoring the ESP32 cannot consume the Teensy channel.
constexpr uint32_t USB_BAUD = 115200;
constexpr uint32_t ROBOT_UART_BAUD = 57600;
constexpr uint8_t ROBOT_UART_RX_PIN = 16;  // <- Teensy TX6 pin 24
constexpr uint8_t ROBOT_UART_TX_PIN = 15;  // -> Teensy RX6 pin 25

// Existing, physically wired step/dir outputs.  GPIO45 is a boot strapping pin
// and GPIO48 also drives the on-board RGB LED on ESP32-S3-DevKitM-1.  They are
// retained because the current mechanism is already wired this way.
constexpr uint8_t STEP_A_PIN = 1;
constexpr uint8_t DIR_A_PIN = 2;
constexpr uint8_t STEP_B_PIN = 45;
constexpr uint8_t DIR_B_PIN = 48;
constexpr uint16_t STEPPER_MIN_PULSE_US = 10;

// Eight pneumatic outputs are driven by one PCF8574. ESP32-S3 DevKitM-1 uses
// GPIO8/GPIO3 as SDA/SCL and the fixed address 0x20. The firmware refuses
// to drive a different I2C device.
constexpr uint8_t VALVE_COUNT = 8;
constexpr uint8_t PCF8574_SDA_PIN = 8;
constexpr uint8_t PCF8574_SCL_PIN = 3;
constexpr uint8_t PCF8574_ADDRESS = 0x20;
constexpr uint32_t PCF8574_I2C_HZ = 100000;
constexpr bool PCF8574_AUTO_DETECT = false;
constexpr bool PCF8574_OUTPUT_ACTIVE_LOW = true;
constexpr uint8_t VALVE_SAFE_MASK = 0x00;

// Bit i identifies the opposite coil of the same double-solenoid valve.
// P0<->P7, P1<->P6, P2<->P5 and P3<->P4 may never be LOW together.
constexpr uint8_t VALVE_INTERLOCK_MASK[VALVE_COUNT] = {
    0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};
constexpr uint32_t VALVE_SWITCH_DEADTIME_MS = 50;
constexpr uint32_t VALVE_SEQUENCE_WAIT_MS = 700;
constexpr uint32_t THA2B_FIRST_TO_SECOND_OUTPUT_MS = 350;

// Limit/home and optional sequence sensors were not present in the supplied
// Physical active-low HOME switches independently establish zero for each axis.
constexpr bool HAS_HOME_SWITCH = true;
constexpr int8_t HOME_SWITCH_A_PIN = 36;
constexpr int8_t HOME_SWITCH_B_PIN = 37;
constexpr uint8_t HOME_SWITCH_ACTIVE_LEVEL = LOW;
constexpr int8_t HOME_DIRECTION_A = 1;
constexpr int8_t HOME_DIRECTION_B = 1;
constexpr float HOME_SPEED_STEPS_S = 1200.0f;
constexpr float HOME_ACCEL_STEPS_S2 = 1000.0f;
constexpr uint32_t HOME_SWITCH_CONFIRM_MS = 25;
constexpr uint32_t HOME_TIMEOUT_MS = 20000;
constexpr long HOME_SEARCH_STEPS = 200000L;

// After THA_2B: lower B alone by 400 mm, then lower A/B together until B
// reaches HOME37, and finally return only A to HOME36. Both axes use the same
// raw step speed/acceleration requested for this recovery sequence.
constexpr int8_t POST_THA2B_LOWER_DIRECTION_A = -HOME_DIRECTION_A;
constexpr int8_t POST_THA2B_LOWER_DIRECTION_B = HOME_DIRECTION_B;
constexpr float POST_THA2B_B_PRELOWER_MM = 400.0f;
constexpr float POST_THA2B_SPEED_STEPS_S = 8000.0f;
constexpr float POST_THA2B_ACCEL_STEPS_S2 = 5000.0f;
constexpr uint32_t POST_THA2B_HOME_TIMEOUT_MS = 45000;
constexpr int8_t E18_SENSOR_PIN = -1;  // Legacy profile sensor slot.
constexpr int8_t LINE_SENSOR_PIN = -1;

// Two E18 object sensors used by the mobile robot after the bridge. E18 NPN
// outputs are active LOW; internal pull-ups keep disconnected inputs safe.
constexpr uint8_t E18_A_PIN = 11;
constexpr uint8_t E18_B_PIN = 18;
constexpr uint8_t E18_ACTIVE_LEVEL = LOW;
constexpr uint32_t E18_BOTH_CONFIRM_MS = 30;

// Absolute mechanism coordinates after the physical HOME switches establish 0.
// A: +1 moves toward HOME/up. B: +1 moves toward HOME/down.
constexpr float READY_HOME_A_POSITION_A_MM = -920.0f;
constexpr float READY_HOME_A_POSITION_B_MM = -520.0f;
constexpr float PICK_LOWER_DISTANCE_MM = 600.0f;
constexpr int8_t PICK_LOWER_DIRECTION_A = -1;
constexpr int8_t PICK_LOWER_DIRECTION_B = 1;
constexpr float POST_POINT_B_RAISE_DISTANCE_MM = 1600.0f;
constexpr int8_t POST_POINT_B_RAISE_DIRECTION_A = 1;
constexpr int8_t POST_POINT_B_RAISE_DIRECTION_B = -1;
constexpr float COMPETITION_SPEED_STEPS_S = 7000.0f;
constexpr float COMPETITION_ACCEL_STEPS_S2 = 5000.0f;
constexpr uint32_t VALVE_PULSE_MS = 300;
constexpr uint32_t COMPETITION_ACTION_TIMEOUT_MS = 30000;

constexpr uint32_t STATUS_PERIOD_MS = 200;
constexpr uint32_t JOG_COMMAND_TIMEOUT_MS = 500;
constexpr float JOG_MIN_SPEED = 10.0f;
constexpr float JOG_MAX_SPEED = 30000.0f;
constexpr size_t UART_LINE_CAPACITY = 256;

}  // namespace HardwareConfig



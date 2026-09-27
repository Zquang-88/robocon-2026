#pragma once

#include <Arduino.h>

namespace HardwareConfig {

// USB Serial is reserved for local diagnostics.  The robot link is a separate
// UART so uploading/monitoring the ESP32 cannot consume the Teensy channel.
constexpr uint32_t USB_BAUD = 115200;
constexpr uint32_t ROBOT_UART_BAUD = 57600;
constexpr uint8_t ROBOT_UART_RX_PIN = 16;  // <- Teensy TX6 pin 24
constexpr uint8_t ROBOT_UART_TX_PIN = 15;  // -> Teensy RX6 pin 25

// External three-channel RGB field indicator. Common-cathode/direct HIGH is
// the default; set RGB_LED_ACTIVE_HIGH=false for a common-anode driver.
constexpr uint8_t RGB_GREEN_PIN = 19;
constexpr uint8_t RGB_RED_PIN = 20;
constexpr uint8_t RGB_BLUE_PIN = 21;
constexpr bool RGB_LED_ACTIVE_HIGH = true;

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
constexpr float HOME_SPEED_STEPS_S = 7000.0f;
constexpr float HOME_ACCEL_STEPS_S2 = 5000.0f;
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
constexpr uint8_t E18_A_PIN = 17;
constexpr uint8_t E18_B_PIN = 18;
constexpr uint8_t E18_ACTIVE_LEVEL = LOW;
constexpr uint32_t E18_BOTH_CONFIRM_MS = 30;

// Local active-low buttons: connect each button between GPIO and GND.
constexpr uint8_t VALVE_BUTTON_PIN = 11;
constexpr uint8_t HOME_BUTTON_PIN = 12;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 30;
constexpr uint32_t BUTTON_DOUBLE_CLICK_MS = 400;
constexpr float BUTTON_HOME_SPEED_STEPS_S = 8000.0f;
constexpr float BUTTON_HOME_ACCEL_STEPS_S2 = 5500.0f;
constexpr uint32_t BUTTON_HOME_TIMEOUT_MS = 45000;
static_assert(E18_A_PIN != VALVE_BUTTON_PIN && E18_B_PIN != VALVE_BUTTON_PIN &&
              E18_A_PIN != HOME_BUTTON_PIN && E18_B_PIN != HOME_BUTTON_PIN,
              "Local buttons must not share the E18 input pins");

// Absolute mechanism coordinates after the physical HOME switches establish 0.
// A: +1 moves toward HOME/up. B: +1 moves toward HOME/down.
constexpr float READY_HOME_A_POSITION_A_MM = -920.0f;
constexpr float READY_HOME_A_POSITION_B_MM = -520.0f;
// Independently calibrated lowering distances for each field, axis and
// station. Keep RED and BLUE separate so tuning one field cannot silently
// change the other. At both fields POINT_A lowers deeper than POINT_B.
// constexpr float RED_POINT_A_LOWER_DISTANCE_A_MM = 610.0f;
// constexpr float RED_POINT_A_LOWER_DISTANCE_B_MM = 545.0f;
// constexpr float RED_POINT_B_LOWER_DISTANCE_A_MM = 520.0f;
// constexpr float RED_POINT_B_LOWER_DISTANCE_B_MM = 480.0f;
// constexpr float BLUE_POINT_A_LOWER_DISTANCE_A_MM = 610.0f;
// constexpr float BLUE_POINT_A_LOWER_DISTANCE_B_MM = 545.0f;
// constexpr float BLUE_POINT_B_LOWER_DISTANCE_A_MM = 520.0f;
// constexpr float BLUE_POINT_B_LOWER_DISTANCE_B_MM = 480.0f;

constexpr float RED_POINT_A_LOWER_DISTANCE_A_MM = 610.0f;
constexpr float RED_POINT_A_LOWER_DISTANCE_B_MM = 545.0f;

constexpr float RED_POINT_B_LOWER_DISTANCE_A_MM = 510.0f;
constexpr float RED_POINT_B_LOWER_DISTANCE_B_MM = 470.0f;


constexpr float BLUE_POINT_A_LOWER_DISTANCE_A_MM = 610.0f;
constexpr float BLUE_POINT_A_LOWER_DISTANCE_B_MM = 545.0f;

constexpr float BLUE_POINT_B_LOWER_DISTANCE_A_MM = 510.0f;
constexpr float BLUE_POINT_B_LOWER_DISTANCE_B_MM = 470.0f;
// POINT_C is the recovery pickup and deliberately reuses the proven POINT_B
// lowering geometry.  Keep separate names so it can be tuned independently.
constexpr float POINT_C_LOWER_DISTANCE_A_MM = 520.0f;
constexpr float POINT_C_LOWER_DISTANCE_B_MM = 480.0f;
constexpr int8_t PICK_LOWER_DIRECTION_A = -1;
constexpr int8_t PICK_LOWER_DIRECTION_B = 1;
// POINT_A and the lowering phase of POINT_B use these motion limits.
constexpr float COMPETITION_SPEED_STEPS_S = 10000.0f;
constexpr float COMPETITION_ACCEL_STEPS_S2 = 8000.0f;
// After POINT_B valves fire, return A and B together to READY_HOME_A. Once
// both axes are ready, A lifts the full distance while B lifts only one third
// holds there. Teensy drives over the bridge concurrently; after it confirms
// a stable level floor, BRIDGE_STABLE releases B for the remaining two thirds.
// Synchronized moves compensate for the two different screw pitches.
constexpr float POINT_B_FINAL_RAISE_DISTANCE_MM = 1800.0f;
constexpr float POINT_B_FIRST_STAGE_A_DISTANCE_MM =
    POINT_B_FINAL_RAISE_DISTANCE_MM * 0.5f;
constexpr float POINT_B_FIRST_STAGE_B_DISTANCE_MM =
    POINT_B_FINAL_RAISE_DISTANCE_MM / 3.0f;
constexpr int8_t POINT_B_FINAL_RAISE_DIRECTION_A = 1;
constexpr int8_t POINT_B_FINAL_RAISE_DIRECTION_B = -1;
constexpr float POINT_B_LIFT_SPEED_STEPS_S = 9000.0f;
constexpr float POINT_B_LIFT_ACCEL_STEPS_S2 = 6000.0f;
constexpr uint32_t VALVE_PULSE_MS = 300;
constexpr uint32_t COMPETITION_ACTION_TIMEOUT_MS = 30000;

constexpr uint32_t STATUS_PERIOD_MS = 200;
constexpr uint32_t JOG_COMMAND_TIMEOUT_MS = 500;
constexpr float JOG_MIN_SPEED = 10.0f;
constexpr float JOG_MAX_SPEED = 30000.0f;
constexpr size_t UART_LINE_CAPACITY = 256;

}  // namespace HardwareConfig

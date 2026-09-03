#pragma once

#include <Arduino.h>

namespace RobotConfig {

// Set true only after every TODO value and direction has been measured.
// Motor/encoder wiring and encoder CPR verified for safe one-wheel PID tuning.
// Autonomous motion remains locked until dimensionsValid() is also true.
constexpr bool HARDWARE_CONFIG_VERIFIED = true;
// Keep the long DBG record off on the 57600-baud air link. The compact TEL,
// WHEEL and selected raw-line records remain enabled for the tuner at 10 Hz.
constexpr bool DEBUG_TELEMETRY = false;

// ---------------- Mechanical configuration ----------------
constexpr float WHEEL_DIAMETER_MM = 100.0f;
constexpr float ROBOT_LENGTH_MM = 260.0f; // front/rear wheel-centre distance
constexpr float ROBOT_WIDTH_MM = 350.0f;  // left/right wheel-centre distance
constexpr float ENCODER_COUNTS_PER_WHEEL_REV = 300.0f; // X4 counts at wheel, after gearbox
constexpr float DISTANCE_CALIBRATION = 1.0f;
// Lateral route mirror selected at run time by FIELD,RED / FIELD,BLUE.
// Verified on the physical chassis: +Vy moves toward the red-field left
// route, while -Vy moves toward the mirrored blue-field right route.
constexpr int8_t RED_FIELD_LATERAL_SIGN = 1;
constexpr int8_t BLUE_FIELD_LATERAL_SIGN = -1;

constexpr int8_t MOTOR_SIGN_FL = 1;
constexpr int8_t MOTOR_SIGN_FR = -1;
// Rear hardware channels were identified in the opposite physical positions:
// old RR is physical RL, and old RL is physical RR.
constexpr int8_t MOTOR_SIGN_RL = 1;
constexpr int8_t MOTOR_SIGN_RR = -1;
// Verified from live 30 RPM tune telemetry: positive motor command counted down.
constexpr int8_t ENCODER_SIGN_FL = -1;
// Verified from live 10 RPM tune telemetry: positive motor command counted down.
constexpr int8_t ENCODER_SIGN_FR = 1;
constexpr int8_t ENCODER_SIGN_RL = 1;
constexpr int8_t ENCODER_SIGN_RR = -1;

// X-configuration wheel equations:
// FL=vx-vy-k*wz, FR=vx+vy+k*wz, RL=vx+vy-k*wz, RR=vx-vy+k*wz.
constexpr bool MECANUM_X_CONFIGURATION = true; // TODO: confirm roller layout

// ---------------- Motor and encoder pins ----------------
constexpr uint8_t FL_RPWM = 36, FL_LPWM = 37;
constexpr uint8_t FR_RPWM = 22, FR_LPWM = 23;
constexpr uint8_t RL_RPWM = 18, RL_LPWM = 19;
constexpr uint8_t RR_RPWM = 14, RR_LPWM = 15;

constexpr uint8_t FL_ENC_A = 0, FL_ENC_B = 1;
constexpr uint8_t FR_ENC_A = 6, FR_ENC_B = 7;
constexpr uint8_t RL_ENC_A = 2, RL_ENC_B = 3;
constexpr uint8_t RR_ENC_A = 4, RR_ENC_B = 5;

constexpr int PWM_FREQUENCY_HZ = 20000;
constexpr int PWM_MAX = 255;
constexpr float MAX_WHEEL_SPEED_MM_S = 1300.0f;
constexpr float WHEEL_SPEED_FILTER_ALPHA = 0.35f;
constexpr uint32_t CONTROL_PERIOD_US = 10000; // 100 Hz

// kp, ki, kd, feed-forward PWM/(mm/s), dead-zone PWM.
// Measured with the robot lifted, verified at 30 and 100 RPM on 2026-08-30.
constexpr float WHEEL_KP[4] = {0.15f, 0.15f, 0.15f, 0.15f};
constexpr float WHEEL_KI[4] = {0.05f, 0.05f, 0.05f, 0.05f};
constexpr float WHEEL_KD[4] = {0.0f, 0.0f, 0.0f, 0.0f};
constexpr float WHEEL_KFF[4] = {0.038f, 0.034f, 0.036f, 0.035f};
constexpr int WHEEL_DEADZONE_PWM[4] = {32, 27, 27, 25};
constexpr float WHEEL_INTEGRAL_LIMIT = 250.0f;

// ---------------- BNO085 + I2C ----------------
// Teensy 4.1 Wire1 defaults: SDA pin 17, SCL pin 16. TODO: confirm wiring.
constexpr uint8_t BNO085_I2C_ADDRESS = 0x4A;
constexpr uint32_t SENSOR_I2C_HZ = 100000;
constexpr bool BNO_USE_GAME_ROTATION_VECTOR = true;
// Verified from the 2026-08-28 position log: positive commanded wz previously
// drove the reported yaw negative, turning heading hold into positive feedback.
// With +1, positive Mecanum wz and reported BNO085 yaw use the same sign.
constexpr int8_t BNO_YAW_SIGN = 1;
constexpr uint32_t BNO_REPORT_INTERVAL_US = 10000;
constexpr uint32_t BNO_TIMEOUT_MS = 250;
constexpr uint32_t BNO_RETRY_INTERVAL_MS = 1000;
// Ground step-response tuned on 2026-08-30 in both yaw directions.
constexpr float HEADING_KP = 0.10f;
constexpr float HEADING_KI = 0.0f;
constexpr float HEADING_KD = 0.004f;
constexpr float HEADING_INTEGRAL_LIMIT = 30.0f;
constexpr float MAX_HEADING_WZ_RAD_S = 1.2f;
constexpr float MAX_MANUAL_HEADING_WZ_RAD_S = 0.60f;
constexpr float MAX_AUTO_HEAD_LOCK_WZ_RAD_S = 1.00f;
constexpr float MAX_LINE_ALIGN_WZ_RAD_S = 0.45f;
constexpr float MAX_BRIDGE_HEADING_WZ_RAD_S = 0.25f;
constexpr float HEADING_OUTPUT_DEADBAND_DEG = 0.50f;
constexpr float HEADING_MIN_WZ_RAD_S = 0.18f;
constexpr float FINE_HEADING_KP = 0.120f;
constexpr float FINE_HEADING_KD = 0.0030f;
constexpr float FINE_HEADING_DEADBAND_DEG = 0.25f;
constexpr float FINE_HEADING_MIN_WZ_RAD_S = 0.180f;
constexpr float FINE_HEADING_MAX_WZ_RAD_S = 0.35f;

// ---------------- Two MCP3008 devices ----------------
constexpr uint8_t MCP_CENTER_CS = 26; // CH0..7: eight center sensors
constexpr uint8_t MCP_STOP_CS = 8;
// Stop-array cables are crossed: keep internal order left first, right second.
// Both groups are ordered outside / middle / inside: -1000, 0, +1000.
constexpr uint8_t STOP_LEFT_CHANNELS[3]  = {3, 4, 5};
constexpr uint8_t STOP_RIGHT_CHANNELS[3] = {0, 1, 2};
constexpr uint32_t MCP_SPI_HZ = 1000000;
// Verified polarity for both MCP3008 arrays:
// black line -> higher ADC than floor -> normalized value approaches 1000.
// A sensor is logically HIGH/on-line when normalized >= LINE_ACTIVE_NORMALIZED.
constexpr bool LINE_IS_HIGHER_THAN_FLOOR = true;
constexpr uint16_t CENTER_SENSOR_MIN[8] = {84,79,76,75,74,75,81,83};
constexpr uint16_t CENTER_SENSOR_MAX[8] = {1023,1023,1023,1023,969,866,1023,1023};
// Logical order after the crossed-cable channel map: left CH3..5, right CH0..2.
constexpr uint16_t STOP_SENSOR_MIN[6] = {76,76,76,80,83,81};
constexpr uint16_t STOP_SENSOR_MAX[6] = {1023,1023,1023,1023,1023,1023};
constexpr bool CENTER_LINE_CALIBRATION_VERIFIED = true;
constexpr bool STOP_LINE_CALIBRATION_VERIFIED = true;
// Confirmed: physical left is CH3..5, physical right is CH0..2; within each
// group the order is outside / middle / inside = -1000 / 0 / +1000.
constexpr bool STOP_SENSOR_ORDER_VERIFIED = true;
constexpr float LINE_FILTER_ALPHA = 0.55f;
constexpr uint16_t LINE_ACTIVE_NORMALIZED = 550;
constexpr uint8_t CROSS_MIN_ACTIVE_SENSORS = 6;
constexpr uint32_t CROSS_CONFIRM_MS = 35;
constexpr uint32_t CROSS_RELEASE_MS = 100;
constexpr uint32_t LINE_SHORT_LOST_MS = 180;
constexpr uint32_t LINE_FAULT_TIMEOUT_MS = 650;
constexpr float LINE_KP = 0.16f;
constexpr float LINE_KI = 0.02f;
constexpr float LINE_KD = 0.003f;
// Bridge test: positive center-array position requires negative robot Vy.
constexpr int8_t CENTER_LINE_CONTROL_SIGN = -1;
// Mecanum lateral authority while following the bridge line. This must be
// comparable to forward speed or the robot cannot recover near an edge.
constexpr float MAX_LINE_VY_MM_S = 220.0f;
constexpr float LINE_LOST_SEARCH_VY_MM_S = 140.0f;

// Local order for both mirrored side arrays: outside / middle / inside.
constexpr int16_t STOP_WEIGHTS[3] = {-1000, 0, 1000};
constexpr float STOP_POSITION_TOLERANCE = 120.0f;
constexpr float STOP_SIDE_DIFFERENCE_TOLERANCE = 150.0f;
constexpr uint32_t STOP_ALIGNMENT_CONFIRM_MS = 180;
constexpr uint32_t STOP_CROSS_CONFIRM_MS = 40;
constexpr uint32_t STOP_CROSS_RELEASE_MS = 100;

// ---------------- Autonomous route (current chassis convention) ----------------
// Existing convention is preserved: +vx forward, +vy right, +wz clockwise.
// Counts belong to mission points A/B, not to a physical side. The active
// three-eye group is selected from the field direction at run time.
constexpr uint8_t PICK_A_LINE_TARGET = 2;
constexpr uint8_t PICK_B_LINE_TARGET = 1;
// After POINT_B, the center 8-eye array ignores the line currently under the
// robot, then counts three complete transverse lines before turning forward.
constexpr uint8_t BRIDGE_LINE_TARGET = 3;
constexpr uint32_t LINE_DEBOUNCE_MS = 40;
constexpr uint32_t LINE_CLEAR_DEBOUNCE_MS = 100;
// The side arrays cross narrow pick lines quickly. Two control samples are
// sufficient because detection already requires the middle eye or 2/3 eyes.
constexpr uint16_t PICK_LINE_DETECT_NORMALIZED = 400;
constexpr uint32_t PICK_LINE_CONFIRM_MS = 8;
constexpr uint32_t PICK_LINE_CLEAR_MS = 60;
constexpr uint32_t X_ALIGN_STABLE_TIME_MS = 180;
constexpr uint32_t LINE_SEARCH_TIMEOUT_MS = 5000;
constexpr uint32_t HEADING_STABLE_TIME_MS = 80;
constexpr float HEADING_TOLERANCE_DEG = 0.60f;
constexpr float MOVE_LEFT_SPEED_MM_S = 400.0f;
// Lateral scan from point B to the third center-array line.
constexpr float BRIDGE_LATERAL_SCAN_SPEED_MM_S = 500.0f;
constexpr float X_ALIGN_MAX_SPEED_MM_S = 80.0f;
constexpr float X_ALIGN_SEARCH_SPEED_MM_S = 35.0f;
constexpr float TOF_ALIGN_MAX_SPEED_MM_S = 100.0f;
constexpr float BRIDGE_FORWARD_SPEED_MM_S = 1150.0f;
constexpr float BRIDGE_MIN_FORWARD_SPEED_MM_S = 950.0f; // Minimum while center line remains valid on ramp
constexpr float BRIDGE_ENTRY_CENTER_KP = 0.08f;
// Minimum command overcomes drivetrain stiction while centering on line 3.
constexpr float BRIDGE_ENTRY_CENTER_MIN_VY_MM_S = 90.0f;
constexpr float BRIDGE_ENTRY_CENTER_MAX_VY_MM_S = 140.0f;
constexpr float BRIDGE_ENTRY_CENTER_TOLERANCE = 300.0f;
constexpr uint32_t BRIDGE_ENTRY_CENTER_CONFIRM_MS = 80;
// Stop this commissioning route after the center array has followed the
// bridge line and detected this many complete transverse markers.
constexpr uint8_t BRIDGE_STOP_CROSS_TARGET = 3;
constexpr uint32_t BRIDGE_LINE_ACQUIRE_CONFIRM_MS = 180;
constexpr uint32_t BRIDGE_LINE_ACQUIRE_TIMEOUT_MS = 3500;
constexpr uint32_t BRIDGE_FOLLOW_TIMEOUT_MS = 20000;
// TODO: set to +1 or -1 only after low-speed tests. Zero locks AUTO START.
constexpr int8_t TOF_CONTROL_SIGN = 1;
// The two logical arrays are ordered outside -> body. Live AUTO logging showed
// the left array oscillating with -1 while the right array converged with +1.
constexpr int8_t LEFT_STOP_CONTROL_SIGN = 1;
constexpr int8_t RIGHT_STOP_CONTROL_SIGN = -1;

// ---------------- Optical flow ----------------
// Temporary conflict-free choice. TODO: confirm module UART and baud.
// Disabled while FL encoder uses pins 0/1. Move optical flow to another UART
// before enabling it; Serial1 would change the pin mux and break FL encoder.
constexpr bool OPTICAL_FLOW_ENABLED = false;
#define FLOW_UART Serial1
constexpr uint32_t FLOW_BAUD = 115200;
constexpr uint8_t MAVLINK_OPTICAL_FLOW_RAD_ID = 106;
constexpr uint8_t MAVLINK_OPTICAL_FLOW_RAD_LEN = 44;
constexpr uint8_t MAVLINK_OPTICAL_FLOW_RAD_CRC_EXTRA = 138;
constexpr int8_t FLOW_X_FROM_SENSOR_AXIS = 0; // 0=sensor X, 1=sensor Y
constexpr int8_t FLOW_Y_FROM_SENSOR_AXIS = 1;
constexpr int8_t FLOW_X_SIGN = 1; // TODO
constexpr int8_t FLOW_Y_SIGN = 1; // TODO
constexpr float FLOW_SCALE_X_MM = 1.0f; // TODO: scale integrated radians/distance to mm
constexpr float FLOW_SCALE_Y_MM = 1.0f; // TODO
constexpr uint8_t FLOW_MIN_QUALITY = 80;
constexpr uint32_t FLOW_TIMEOUT_MS = 250;
constexpr float ODOM_ENCODER_WEIGHT = 1.0f;
constexpr float ODOM_FLOW_WEIGHT = 0.0f;
constexpr float LATERAL_SLIP_THRESHOLD_MM_S = 180.0f;

// ---------------- VL53L3CX (DFRobot SEN0378) ----------------
// Shares Teensy 4.1 Wire1 (SDA17/SCL16) with BNO085.
// BNO085 uses 0x4A and VL53L3CX uses its default address 0x29.
constexpr bool TOF_ENABLED = true;
constexpr uint8_t TOF_I2C_ADDRESS = 0x29;
constexpr uint16_t TOF_TARGET_MM = 220;
constexpr uint16_t TOF_MIN_VALID_MM = 35;
constexpr uint16_t TOF_MAX_VALID_MM = 800;
// Reject implausible single-step jumps caused by VL53L3CX ghost targets.
constexpr uint16_t TOF_MAX_SAMPLE_JUMP_MM = 250;
constexpr uint32_t TOF_TIMEOUT_MS = 1500;
// Hold the robot stopped while VL53L3CX recovers from short motion/power-noise
// dropouts. The 30 cm target and tolerance remain unchanged.
constexpr uint32_t TOF_ALIGNMENT_RECOVERY_MS = 3000;
// At the starting mark the sensor is only about 20 mm from the wall, below
// the reliable VL53L3CX range. Creep forward by encoder until ToF can take over.
// Initial launch is encoder-only; ToF is intentionally ignored here.
constexpr float START_ENCODER_DISTANCE_MM = 260.0f;
constexpr float START_ENCODER_SPEED_MM_S = 250.0f;
constexpr float TOF_START_ESCAPE_DISTANCE_MM = 250.0f;
constexpr float TOF_START_ESCAPE_SPEED_MM_S = 300.0f;
constexpr uint8_t TOF_MEDIAN_SAMPLES = 5;
constexpr float TOF_COARSE_SPEED_MM_S = 180.0f;
constexpr float TOF_FINE_SPEED_MM_S = 65.0f;
constexpr float TOF_TOLERANCE_MM = 30.0f;
constexpr uint32_t TOF_CONFIRM_MS = 200;

// ---------------- UART and safety ----------------
// RBT/1 GUI telemetry through the CP210x/air-radio link (PC COM5).
// Teensy 4.1 Serial5: RX5 pin 21, TX5 pin 20.
#define TELEMETRY_UART Serial5
constexpr uint32_t TELEMETRY_BAUD = 57600;
constexpr uint8_t TELEMETRY_RX_PIN = 21;
constexpr uint8_t TELEMETRY_TX_PIN = 20;

// ESP32 link on Teensy 4.1 Serial6, separate from Serial5 telemetry.
// Confirmed free from the motor driver: TX6 pin 24, RX6 pin 25.
constexpr bool ESP32_UART_ENABLED = true;
#define ESP32_UART Serial6
constexpr uint8_t ESP32_TX_PIN = 24;
constexpr uint8_t ESP32_RX_PIN = 25;
constexpr uint32_t ESP32_BAUD = 57600;
// The mechanism simulator intentionally answers after 10 s. Keep enough margin
// for scheduling and UART transmission before declaring an ESP32 timeout.
constexpr uint32_t ESP32_REPLY_TIMEOUT_MS = 15000;
constexpr uint32_t STATE_DEFAULT_TIMEOUT_MS = 12000;
// 5 Hz leaves margin on the half-duplex radio while four motors create EMI.
// The 100 Hz control loop and 500 ms motion watchdog are unchanged.
constexpr uint32_t TELEMETRY_PERIOD_MS = 200;
constexpr uint32_t TUNER_WATCHDOG_MS = 600;
constexpr uint32_t MANUAL_DRIVE_WATCHDOG_MS = 500;
constexpr uint32_t POSITION_LINK_WATCHDOG_MS = 600;
// Position tests use a gentler heading correction so k*wz cannot dominate
// the translational wheel targets while the robot is converging on X/Y.
constexpr float MAX_POSITION_HEADING_WZ_RAD_S = 0.35f;

// Pins 31/32 are unused. START/STOP remain available through RBT/1 telemetry.
constexpr bool PHYSICAL_START_STOP_ENABLED = false;
constexpr uint8_t START_PIN = 32;
constexpr uint8_t STOP_PIN = 31;

// ---------------- Mission constants copied from V10 ----------------
constexpr float SENSOR_TO_CENTER_MM = 20.0f;
constexpr float START_APPROACH_MM = 125.0f;
constexpr float C_OFFSET_MM = 150.0f;
constexpr float HOME_TO_B_MM = 410.0f;
constexpr float RETURN_A_MM = 330.0f;
constexpr float RETURN_LONG_MM = 2650.0f;
constexpr float RETURN_LATERAL_MM = 1650.0f;
constexpr float MOVE_SPEED_MM_S = 450.0f;
// Commissioning limits: do not jump from the 180 mm/s acquisition speed to
// 700 mm/s. Ramp up only after the center line is confirmed.
constexpr float BRIDGE_SPEED_FAST_MM_S = 1150.0f;
constexpr float BRIDGE_SPEED_SLOW_MM_S = 1000.0f;
constexpr float BRIDGE_ACCEL_MM_S2 = 1800.0f;
constexpr float BRIDGE_DECEL_MM_S2 = 700.0f;

inline bool dimensionsValid() {
  return ROBOT_LENGTH_MM > 0.0f && ROBOT_WIDTH_MM > 0.0f &&
         ENCODER_COUNTS_PER_WHEEL_REV > 0.0f &&
         RED_FIELD_LATERAL_SIGN == 1 && BLUE_FIELD_LATERAL_SIGN == -1;
}

inline float mmPerCount() {
  if (ENCODER_COUNTS_PER_WHEEL_REV <= 0.0f) return 0.0f;
  return PI * WHEEL_DIAMETER_MM * DISTANCE_CALIBRATION /
         ENCODER_COUNTS_PER_WHEEL_REV;
}

inline float mecanumKmm() {
  return 0.5f * (ROBOT_LENGTH_MM + ROBOT_WIDTH_MM);
}

} // namespace RobotConfig

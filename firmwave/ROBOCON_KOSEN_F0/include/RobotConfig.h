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
// Synchronized wheel-target ramp. It limits the initial torque step without
// changing the Mecanum wheel-speed ratios. Safety stops still bypass this ramp.
constexpr float WHEEL_TARGET_ACCEL_MM_S2 = 1400.0f;
constexpr float WHEEL_TARGET_DECEL_MM_S2 = 2400.0f;

// All controller gains live in this file. Telemetry tuning changes RAM only;
// reset/reboot restores these compiled values. EEPROM is not used for PID.
// Direct USB tuning with all four wheels lifted, both directions, 40/80/120 RPM
// on 2026-09-05. Recheck under load before competition use.
// Array order everywhere in firmware: FL, FR, BL/RL, BR/RR.
constexpr float WHEEL_KP[4] = {
    0.15f, // FL - front left
    0.14f, // FR - front right
    0.14f, // BL/RL - back left
    0.15f  // BR/RR - back right
};
constexpr float WHEEL_KI[4] = {
    0.08f, // FL
    0.08f, // FR
    0.12f, // BL/RL
    0.10f  // BR/RR
};
constexpr float WHEEL_KD[4] = {
    0.0f, // FL: derivative disabled to avoid encoder-quantization kicks
    0.0f, // FR
    0.0f, // BL/RL
    0.0f  // BR/RR
};
constexpr float WHEEL_KFF[4] = {0.035f, 0.045f, 0.034f, 0.034f}; // FL, FR, BL, BR
constexpr int WHEEL_DEADZONE_PWM[4] = {36, 34, 34, 34}; // FL, FR, BL, BR
constexpr float WHEEL_INTEGRAL_LIMIT = 250.0f;

// Position, stop-line and distance controllers.
constexpr float POSITION_X_KP = 1.40f;
constexpr float POSITION_X_KI = 0.05f;
constexpr float POSITION_X_KD = 0.01f;
constexpr float POSITION_X_INTEGRAL_LIMIT = 400.0f;
constexpr float POSITION_Y_KP = 1.40f;
constexpr float POSITION_Y_KI = 0.05f;
constexpr float POSITION_Y_KD = 0.01f;
constexpr float POSITION_Y_INTEGRAL_LIMIT = 400.0f;
constexpr float STOP_FORWARD_KP = 0.20f;
constexpr float STOP_FORWARD_KI = 0.01f;
constexpr float STOP_FORWARD_KD = 0.002f;
constexpr float STOP_FORWARD_INTEGRAL_LIMIT = 600.0f;
constexpr float STOP_YAW_KP = 0.0018f;
constexpr float STOP_YAW_KI = 0.0001f;
constexpr float STOP_YAW_KD = 0.00002f;
constexpr float STOP_YAW_INTEGRAL_LIMIT = 500.0f;
constexpr float TOF_DISTANCE_KP = 0.80f;
constexpr float TOF_DISTANCE_KI = 0.0f;
constexpr float TOF_DISTANCE_KD = 0.0f;
constexpr float TOF_DISTANCE_INTEGRAL_LIMIT = 300.0f;
constexpr float RIGHT_LINE_KP = 0.08f;
constexpr float RIGHT_LINE_KI = 0.0f;
constexpr float RIGHT_LINE_KD = 0.0f;
constexpr float RIGHT_LINE_INTEGRAL_LIMIT = 1000.0f;
constexpr float LEFT_LINE_KP = 0.08f;
constexpr float LEFT_LINE_KI = 0.0f;
constexpr float LEFT_LINE_KD = 0.0f;
constexpr float LEFT_LINE_INTEGRAL_LIMIT = 1000.0f;

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
constexpr float HEADING_KP = 0.101f;
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
// Stationary/ToF alignment at pick points A and B. Keep these values gentler
// than the moving heading controller so motor dead-zone compensation cannot
// excite a left/right yaw oscillation around the target.
constexpr float STATION_HEADING_DEADBAND_DEG = 0.45f;
constexpr float STATION_HEADING_MIN_WZ_RAD_S = 0.05f;
constexpr float STATION_HEADING_MAX_WZ_RAD_S = 0.16f;
constexpr float STATION_HEADING_SLEW_RAD_S2 = 0.55f;
constexpr float STATION_HEADING_SETTLED_RATE_DEG_S = 2.0f;
// Dedicated, gentler heading hold for lateral AUTO travel. A slew limit avoids
// the abrupt +/-Wz step that previously rotated the chassis across pick lines.
constexpr float LATERAL_HEADING_DEADBAND_DEG = 0.45f;
constexpr float LATERAL_HEADING_MIN_WZ_RAD_S = 0.06f;
constexpr float LATERAL_HEADING_MAX_WZ_RAD_S = 0.20f;
constexpr float LATERAL_HEADING_SLEW_RAD_S2 = 0.80f;
constexpr float LATERAL_LINE_COUNT_MAX_YAW_ERROR_DEG = 2.0f;
constexpr float LATERAL_HEADING_RECOVERY_SPEED_FACTOR = 0.45f;

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
// MCP STOP CS8 remaining inputs: physical CH6 and CH7 (sensor numbers 7/8).
// Replace these defaults after both sensors have seen white floor and black line.
constexpr uint16_t LATERAL_HOLD_SENSOR_MIN[2] = {0, 0};
constexpr uint16_t LATERAL_HOLD_SENSOR_MAX[2] = {1023, 1023};
// H1 is mounted at the tail (CH6), H2 at the front (CH7). Black line is HIGH.
constexpr uint8_t LATERAL_HOLD_TAIL_CHANNEL = 6;
constexpr uint8_t LATERAL_HOLD_FRONT_CHANNEL = 7;
constexpr uint16_t LATERAL_HOLD_LINE_THRESHOLD = 500;
// +1: tail leaves line -> +Vx (forward), front leaves line -> -Vx (backward).
// Change to -1 only if the first low-speed floor test corrects the wrong way.
constexpr int8_t LATERAL_HOLD_CONTROL_SIGN = 1;
constexpr float LATERAL_HOLD_CORRECTION_SPEED_MM_S = 65.0f;
constexpr float LATERAL_HOLD_SEARCH_SPEED_MM_S = 35.0f;
constexpr float LATERAL_HOLD_SLEW_MM_S2 = 220.0f;
constexpr uint32_t LATERAL_HOLD_LOST_TIMEOUT_MS = 450;
constexpr uint32_t LATERAL_HOLD_ACQUIRE_TIMEOUT_MS = 1200;
// After the 170 mm launch, travel sideways this distance before enabling H1/H2
// and the point-A line counter. This avoids treating the start marking as the
// longitudinal guide line.
constexpr float FIRST_LATERAL_BLIND_DISTANCE_MM = 400.0f;
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
// The start marking is ignored while the robot first moves forward by encoder.
// Counting begins only when lateral travel starts; point A is line number 2.
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
constexpr uint32_t HEADING_STABLE_TIME_MS = 350;
constexpr float HEADING_TOLERANCE_DEG = 0.60f;
constexpr float MOVE_LEFT_SPEED_MM_S = 400.0f;
// Do not use ToF/encoder X correction while crossing lateral pick lines. ToF
// changes with wall geometry and previously injected 60-75 mm/s longitudinal
// motion. Absolute ToF alignment remains active after the robot stops at A/B.
constexpr bool LATERAL_LONGITUDINAL_HOLD_ENABLED = false;
// Encoder-odometry correction that prevents forward/backward drift while strafing.
constexpr float LATERAL_X_HOLD_KP = 1.20f;
constexpr float LATERAL_X_HOLD_DEADBAND_MM = 6.0f;
constexpr float LATERAL_X_HOLD_MAX_SPEED_MM_S = 140.0f;
// Absolute wall-distance hold used while strafing; encoder Mecanum odometry
// alone cannot distinguish real longitudinal drift from wheel slip.
constexpr float LATERAL_TOF_HOLD_KP = 0.90f;
constexpr float LATERAL_TOF_HOLD_DEADBAND_MM = 8.0f;
constexpr float LATERAL_TOF_HOLD_MAX_SPEED_MM_S = 130.0f;
// Lateral scan from point B to the third center-array line.
constexpr float BRIDGE_LATERAL_SCAN_SPEED_MM_S = 500.0f;
constexpr float X_ALIGN_MAX_SPEED_MM_S = 80.0f;
// Loaded-wheel minimum at A/B: commands below this can remain inside static
// friction and leave the state machine waiting even though line error exists.
constexpr float X_ALIGN_MIN_SPEED_MM_S = 55.0f;
constexpr float X_ALIGN_SEARCH_SPEED_MM_S = 55.0f;
// Point B is accepted only when the middle (0) and inside (+1000) eyes are
// simultaneously on black. Their geometric midpoint is +500.
constexpr float POINT_B_PAIR_TARGET_POSITION = 500.0f;
// If the 200 mm A-to-B encoder move passes the line completely, search back
// first and then sweep around the arrival point instead of drifting forever.
constexpr float POINT_B_PAIR_SEARCH_RADIUS_MM = 120.0f;
constexpr uint32_t POINT_B_PAIR_SEARCH_TIMEOUT_MS = 10000;
// Reject short threshold dropouts while heading settles at B.
constexpr uint32_t POINT_B_PAIR_LOST_GRACE_MS = 250;
// Preserve the precise 0.6 degree condition first. If BNO rate noise prevents
// 350 ms of perfect settling, accept this still-tight bounded result instead.
constexpr uint32_t POINT_B_HEADING_MAX_SETTLE_MS = 1800;
constexpr float POINT_B_HEADING_FALLBACK_TOLERANCE_DEG = 1.0f;
constexpr float POINT_B_HEADING_FALLBACK_RATE_DEG_S = 4.0f;
constexpr float TOF_ALIGN_MAX_SPEED_MM_S = 100.0f;
constexpr float BRIDGE_FORWARD_SPEED_MM_S = 1300.0f;
constexpr float BRIDGE_APPROACH_SPEED_MM_S = 1000.0f;
constexpr float BRIDGE_CREST_SPEED_MM_S = 350.0f;
constexpr float BRIDGE_DESCENT_SPEED_MM_S = 300.0f;
// Relative roll/pitch thresholds. The level reference is captured immediately
// after the eight-eye array is centred, before forward bridge travel begins.
constexpr float BRIDGE_INCLINE_ENTER_DEG = 5.0f;
constexpr float BRIDGE_CREST_LEVEL_DEG = 4.0f;
constexpr uint32_t BRIDGE_INCLINE_CONFIRM_MS = 150;
constexpr uint32_t BRIDGE_CREST_CONFIRM_MS = 120;
constexpr uint32_t BRIDGE_MIN_ASCENT_BEFORE_CREST_MS = 300;
// After the crest is level, a renewed tilt confirms that the robot is actually
// descending. Transverse-line detection is disabled until this state.
constexpr float BRIDGE_DESCENT_ENTER_DEG = 4.5f;
constexpr uint32_t BRIDGE_DESCENT_CONFIRM_MS = 100;
// Do not arm transverse-line detection at the vibrating foot of the bridge.
// The robot must return close to level continuously for this settling time.
constexpr float BRIDGE_DESCENT_EXIT_LEVEL_DEG = 2.0f;
constexpr uint32_t BRIDGE_DESCENT_EXIT_LEVEL_CONFIRM_MS = 500;
// The longitudinal guide normally covers only 1-2 center eyes. A transverse
// marker covers most of the array, so detect it directly without stacking the
// centerLine.cross debounce with another long confirmation delay.
constexpr uint16_t BRIDGE_CENTER_MARKER_ACTIVE_NORMALIZED = 400;
constexpr uint8_t BRIDGE_CENTER_MARKER_MIN_ACTIVE_SENSORS = 5;
constexpr uint32_t BRIDGE_END_MARKER_CONFIRM_MS = 30;
constexpr uint32_t BRIDGE_SIDE_MARKER_CONFIRM_MS = 80;
// Once an end marker is latched, continue following the longitudinal guide by
// encoder before stopping. The center array is farther forward than the sides.
constexpr float BRIDGE_CENTER_MARKER_ADVANCE_MM = 300.0f;
constexpr float BRIDGE_SIDE_MARKER_ADVANCE_MM = 300.0f;
constexpr float BRIDGE_POST_MARKER_SPEED_MM_S = 200.0f;
// After the end-marker advance, move right 190 mm by encoder, then reuse
// point B's middle (0) + inside (+1000) eye PID to pull back onto the line.
// The point-B eye pair is the primary stop/advance condition. ESP32 E18 is
// auxiliary telemetry only and must never block the following 70 mm move.
// During that move, one remaining point-B eye steers laterally; losing both
// pauses forward motion until the middle+inside pair is stable again.
constexpr float POST_BRIDGE_E18_RIGHT_DISTANCE_MM = 190.0f;
constexpr float POST_BRIDGE_E18_RIGHT_SPEED_MM_S = 200.0f;
constexpr float POST_BRIDGE_E18_ALIGN_MIN_SPEED_MM_S = 90.0f;
constexpr float POST_BRIDGE_E18_ALIGN_MAX_SPEED_MM_S = 140.0f;
constexpr uint32_t POST_BRIDGE_E18_LINE_CONFIRM_MS = 40;
constexpr uint32_t POST_BRIDGE_E18_POLL_MS = 100;
constexpr float POST_BRIDGE_E18_FORWARD_DISTANCE_MM = 70.0f;
constexpr float POST_BRIDGE_E18_FORWARD_SPEED_MM_S = 200.0f;
constexpr float POST_BRIDGE_E18_FORWARD_MIN_SPEED_MM_S = 110.0f;
constexpr uint32_t POST_BRIDGE_FORWARD_PAIR_LOST_CONFIRM_MS = 60;
constexpr uint32_t POST_BRIDGE_E18_SEARCH_TIMEOUT_MS = 13000;
// Route from completed 2A placement to the 2B drop line. Physical right is
// negative body Y on the verified drivetrain; backward is negative body X.
constexpr float POST_THA2A_RIGHT_DISTANCE_MM = 400.0f;
constexpr float POST_THA2A_BACKWARD_DISTANCE_MM = 200.0f;
constexpr float POST_THA2A_DIAGONAL_SPEED_MM_S = 300.0f;
// Watch the right middle/inside eyes during the coarse diagonal move. The
// previous pickup line must first clear, then touching either eye on the next
// vertical line immediately hands control to the slow pair aligner.
constexpr float THA2B_COARSE_LINE_ARM_DISTANCE_MM = 80.0f;
constexpr uint16_t THA2B_COARSE_LINE_THRESHOLD = 450;
constexpr uint32_t THA2B_COARSE_CLEAR_CONFIRM_MS = 40;
constexpr uint32_t THA2B_COARSE_HIT_CONFIRM_MS = 10;
constexpr float THA2B_PAIR_SEARCH_SPEED_MM_S = 60.0f;
constexpr float THA2B_PAIR_ALIGN_MIN_SPEED_MM_S = 25.0f;
constexpr float THA2B_PAIR_ALIGN_MAX_SPEED_MM_S = 90.0f;
constexpr uint32_t THA2B_PAIR_CONFIRM_MS = 80;
constexpr float THA2B_LINE_REVERSE_SPEED_MM_S = 260.0f;
constexpr float THA2B_LINE_CLEAR_SPEED_MM_S = 90.0f;
constexpr uint8_t THA2B_LEFT_MARKER_MIN_ACTIVE = 2;
constexpr uint32_t THA2B_HORIZONTAL_CONFIRM_MS = 50;
constexpr uint32_t THA2B_HORIZONTAL_CLEAR_CONFIRM_MS = 20;
constexpr uint32_t THA2B_DIAGONAL_TIMEOUT_MS = 8000;
constexpr uint32_t THA2B_LINE_SEARCH_TIMEOUT_MS = 13000;
constexpr uint32_t THA2B_REVERSE_TIMEOUT_MS = 15000;
// Final route after ESP32 confirms THA_2B. Physical right is -body Y and
// reverse is -body X on the verified drivetrain.
constexpr float POST_THA2B_EXIT_RIGHT_DISTANCE_MM = 2000.0f;
constexpr float POST_THA2B_EXIT_REVERSE_DISTANCE_MM = 3000.0f;
constexpr float POST_THA2B_EXIT_DIAGONAL_SPEED_MM_S = 1200.0f;
constexpr uint32_t POST_THA2B_EXIT_DIAGONAL_TIMEOUT_MS = 18000;

constexpr float BNO_TILT_FILTER_ALPHA = 0.35f;
constexpr float BRIDGE_LINE_SEARCH_RADIUS_MM = 180.0f;
constexpr float BRIDGE_LINE_SEARCH_SPEED_MM_S = 60.0f;
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
constexpr uint32_t BRIDGE_LINE_ACQUIRE_TIMEOUT_MS = 10000;
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
// This competition route no longer uses ToF. Keep the driver code available
// for the standalone diagnostic build, but do not initialize or poll it here.
constexpr bool TOF_ENABLED = false;
constexpr uint8_t TOF_I2C_ADDRESS = 0x29;
// Measured alignment point: H1 and H2 are centred on the black line at ~160 mm.
constexpr uint16_t TOF_TARGET_MM = 160;
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
constexpr float START_ENCODER_DISTANCE_MM = 170.0f;
constexpr float START_ENCODER_SPEED_MM_S = 280.0f;
constexpr float START_LATERAL_DISTANCE_MM = 1000.0f;
constexpr float START_LATERAL_SPEED_MM_S = 400.0f;
constexpr float START_HOLD_LINE_SEARCH_SPEED_MM_S = 280.0f;
constexpr uint32_t START_HOLD_LINE_CONFIRM_MS = 100;
constexpr uint32_t START_HOLD_LINE_SEARCH_TIMEOUT_MS = 8000;
// After H1/H2 acquire the longitudinal guide, creep sideways until the
// logical right array's middle and inside eyes both see the pickup line.
constexpr float POINT_A_RIGHT_ALIGN_SPEED_MM_S = 50.0f;
constexpr uint16_t POINT_A_RIGHT_SENSOR_THRESHOLD = 500;
constexpr uint32_t POINT_A_RIGHT_CONFIRM_MS = 120;
constexpr uint32_t POINT_A_RIGHT_PAIR_WINDOW_MS = 1200;
constexpr uint32_t POINT_A_RIGHT_SEARCH_TIMEOUT_MS = 15000;
// Measured lateral distance from the start of the sideways leg to point A.
// Encoder distance defines only the search window; the line sensors still
// decide the final stop/alignment because Mecanum wheels can slip.
constexpr float POINT_A_EXPECTED_LATERAL_MM = 1075.0f;
constexpr float POINT_A_SEARCH_HALF_RANGE_MM = 200.0f;
constexpr float POINT_A_SEARCH_SPEED_MM_S = 70.0f;
// Point A becomes the new odometry origin. Move this measured lateral
// distance toward B, then let the B side-array perform the final alignment.
constexpr float POINT_A_TO_B_DISTANCE_MM = 200.0f;
constexpr float POINT_A_TO_B_SPEED_MM_S = 300.0f;
// Once point A''s middle/inside side eyes are locked on the line, keep that
// lateral position and search forward/back for H1/H2 within this safe radius.
constexpr float POINT_A_H_SEARCH_RADIUS_MM = 100.0f;
constexpr float POINT_A_H_SEARCH_SPEED_MM_S = 35.0f;
constexpr uint32_t POINT_A_H_SEARCH_TIMEOUT_MS = 15000;
// After POINT_B, move robot-left by encoder before acquiring the center line.
constexpr float POST_B_LEFT_DISTANCE_MM = 1300.0f;
constexpr float POST_B_LEFT_SPEED_MM_S = 800.0f;
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
// AUTO always uses the real ESP32 mechanism link.
constexpr uint32_t ESP32_REPLY_TIMEOUT_MS = 15000;
constexpr uint32_t ESP32_HOME_READY_TIMEOUT_MS = 30000;
// Post-bridge handshake: Teensy notifies ESP32 after the 70 mm line-held move.
// ESP32 must reply with the exact token DONE_THA2A. Retries are idempotent.
constexpr uint32_t DONE_THA2A_RETRY_INTERVAL_MS = 1000;
constexpr uint8_t DONE_THA2A_MAX_ATTEMPTS = 6;
constexpr uint32_t DONE_THA2A_TOTAL_TIMEOUT_MS = 7000;
// Final 2B action sends CHO_THA2B and waits for exact DONE_THA2B.
constexpr uint32_t DONE_THA2B_RETRY_INTERVAL_MS = 1000;
constexpr uint8_t DONE_THA2B_MAX_ATTEMPTS = 6;
constexpr uint32_t DONE_THA2B_TOTAL_TIMEOUT_MS = 7000;
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

// Active-low physical controls. Each input uses Teensy INPUT_PULLUP, so the
// released level is HIGH and a pressed switch connects the pin to GND.
constexpr bool PHYSICAL_CONTROL_BUTTONS_ENABLED = true;
constexpr uint8_t PHYSICAL_START_BUTTON_PIN = 34;
constexpr uint8_t PHYSICAL_FIELD_BUTTON_PIN = 35;
constexpr uint32_t PHYSICAL_BUTTON_DEBOUNCE_MS = 35;
constexpr uint32_t PHYSICAL_FIELD_DOUBLE_CLICK_MS = 450;

// AO3400A is a low-side switch: HIGH on IN_BUZZER enables the buzzer.
constexpr uint8_t BUZZER_PIN = 40;
constexpr bool BUZZER_ACTIVE_HIGH = true;
constexpr uint16_t BUZZER_BEEP_ON_MS = 110;
constexpr uint16_t BUZZER_BEEP_OFF_MS = 100;
constexpr uint16_t BUZZER_PATTERN_GAP_MS = 180;
// Short double tone used only for command replies from the mechanism ESP32.
constexpr uint16_t BUZZER_UART_FAST_ON_MS = 55;
constexpr uint16_t BUZZER_UART_FAST_OFF_MS = 45;
constexpr uint16_t BUZZER_UART_FAST_GAP_MS = 100;
constexpr uint16_t BUZZER_ALARM_TOGGLE_MS = 120;
constexpr uint32_t BUZZER_ALIGNMENT_ALARM_MS = 2000;

static_assert(PHYSICAL_START_BUTTON_PIN != PHYSICAL_FIELD_BUTTON_PIN &&
              PHYSICAL_START_BUTTON_PIN != BUZZER_PIN &&
              PHYSICAL_FIELD_BUTTON_PIN != BUZZER_PIN,
              "Physical control pins must be unique");

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

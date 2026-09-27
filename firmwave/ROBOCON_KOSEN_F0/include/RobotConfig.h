#pragma once

#include <Arduino.h>

namespace RobotConfig {

// Set true only after every TODO value and direction has been measured.
// Motor/encoder wiring and encoder CPR verified for safe one-wheel PID tuning.
// Autonomous motion remains locked until dimensionsValid() is also true.
constexpr bool HARDWARE_CONFIG_VERIFIED = true; // VI: Cho phép vận hành khi cấu hình phần cứng đã được xác minh.
// Keep the long DBG record off on the 57600-baud air link. The compact TEL,
// WHEEL and selected raw-line records remain enabled for the tuner at 10 Hz.
constexpr bool DEBUG_TELEMETRY = false; // VI: Bật hoặc tắt bản tin telemetry gỡ lỗi dài.

// ---------------- Mechanical configuration ----------------
constexpr float WHEEL_DIAMETER_MM = 100.0f; // VI: Đường kính bánh xe, đơn vị mm.
constexpr float ROBOT_LENGTH_MM = 260.0f; // VI: Khoảng cách tâm bánh trước–sau, đơn vị mm.
constexpr float ROBOT_WIDTH_MM = 350.0f; // VI: Khoảng cách tâm bánh trái–phải, đơn vị mm.
constexpr float ENCODER_COUNTS_PER_WHEEL_REV = 300.0f; // VI: Số xung encoder X4 trên một vòng bánh sau hộp số.
constexpr float DISTANCE_CALIBRATION = 1.0f; // VI: Hệ số hiệu chỉnh quãng đường encoder.
// Lateral route mirror selected at run time by FIELD,RED / FIELD,BLUE.
// Verified on the physical chassis: +Vy moves toward the red-field left
// route, while -Vy moves toward the mirrored blue-field right route.
constexpr int8_t RED_FIELD_LATERAL_SIGN = 1; // VI: Dấu chiều chạy ngang của MAP đỏ.
constexpr int8_t BLUE_FIELD_LATERAL_SIGN = -1; // VI: Dấu chiều chạy ngang của MAP xanh.

constexpr int8_t MOTOR_SIGN_FL = 1; // VI: Chiều điều khiển động cơ bánh trước trái FL.
constexpr int8_t MOTOR_SIGN_FR = -1; // VI: Chiều điều khiển động cơ bánh trước phải FR.
// Rear hardware channels were identified in the opposite physical positions:
// old RR is physical RL, and old RL is physical RR.
constexpr int8_t MOTOR_SIGN_RL = 1; // VI: Chiều điều khiển động cơ bánh sau trái BL/RL.
constexpr int8_t MOTOR_SIGN_RR = -1; // VI: Chiều điều khiển động cơ bánh sau phải BR/RR.
// Verified from live 30 RPM tune telemetry: positive motor command counted down.
constexpr int8_t ENCODER_SIGN_FL = -1; // VI: Chiều đếm encoder bánh trước trái FL.
// Verified from live 10 RPM tune telemetry: positive motor command counted down.
constexpr int8_t ENCODER_SIGN_FR = 1; // VI: Chiều đếm encoder bánh trước phải FR.
constexpr int8_t ENCODER_SIGN_RL = 1; // VI: Chiều đếm encoder bánh sau trái BL/RL.
constexpr int8_t ENCODER_SIGN_RR = -1; // VI: Chiều đếm encoder bánh sau phải BR/RR.

// X-configuration wheel equations:
// FL=vx-vy-k*wz, FR=vx+vy+k*wz, RL=vx+vy-k*wz, RR=vx-vy+k*wz.
constexpr bool MECANUM_X_CONFIGURATION = true; // VI: Chọn cấu hình con lăn Mecanum kiểu X.

// ---------------- Motor and encoder pins ----------------
constexpr uint8_t FL_RPWM = 36, FL_LPWM = 37; // VI: Chân PWM hai chiều bánh FL: RPWM và LPWM.
constexpr uint8_t FR_RPWM = 22, FR_LPWM = 23; // VI: Chân PWM hai chiều bánh FR: RPWM và LPWM.
constexpr uint8_t RL_RPWM = 18, RL_LPWM = 19; // VI: Chân PWM hai chiều bánh BL/RL: RPWM và LPWM.
constexpr uint8_t RR_RPWM = 14, RR_LPWM = 15; // VI: Chân PWM hai chiều bánh BR/RR: RPWM và LPWM.

constexpr uint8_t FL_ENC_A = 0, FL_ENC_B = 1; // VI: Chân encoder A/B của bánh FL.
constexpr uint8_t FR_ENC_A = 6, FR_ENC_B = 7; // VI: Chân encoder A/B của bánh FR.
constexpr uint8_t RL_ENC_A = 2, RL_ENC_B = 3; // VI: Chân encoder A/B của bánh BL/RL.
constexpr uint8_t RR_ENC_A = 4, RR_ENC_B = 5; // VI: Chân encoder A/B của bánh BR/RR.

constexpr int PWM_FREQUENCY_HZ = 20000; // VI: Tần số PWM FREQUENCY, đơn vị Hz.
constexpr int PWM_MAX = 255; // VI: Giá trị PWM lớn nhất.
constexpr float MAX_WHEEL_SPEED_MM_S = 1300.0f; // VI: Tốc độ tối đa bánh xe tốc độ, đơn vị mm/s.
constexpr float WHEEL_SPEED_FILTER_ALPHA = 0.35f; // VI: Hệ số lọc tốc độ bánh; lớn hơn bám nhanh nhưng nhiễu hơn.
constexpr uint32_t CONTROL_PERIOD_US = 10000; // VI: Thời gian điều khiển PERIOD, đơn vị µs.
// Synchronized wheel-target ramp. It limits the initial torque step without
// changing the Mecanum wheel-speed ratios. Safety stops still bypass this ramp.
constexpr float WHEEL_TARGET_ACCEL_MM_S2 = 1400.0f; // VI: Gia tốc bánh xe mục tiêu tăng tốc, đơn vị mm/s².
constexpr float WHEEL_TARGET_DECEL_MM_S2 = 2400.0f; // VI: Gia tốc bánh xe mục tiêu giảm tốc, đơn vị mm/s².

// All controller gains live in this file. Telemetry tuning changes RAM only;
// reset/reboot restores these compiled values. EEPROM is not used for PID.
// Direct USB tuning with all four wheels lifted, both directions, 40/80/120 RPM
// on 2026-09-05. Recheck under load before competition use.
// Array order everywhere in firmware: FL, FR, BL/RL, BR/RR.
constexpr float WHEEL_KP[4] = { // VI: Hệ số P bốn bánh, thứ tự FL, FR, BL/RL, BR/RR.
    0.25f, // FL - bánh trước trái
    0.25f, // FR - bánh trước phải
    0.25f, // BL/RL - bánh sau trái
    0.25f  // BR/RR - bánh sau phải
};
constexpr float WHEEL_KI[4] = { // VI: Hệ số I bốn bánh, thứ tự FL, FR, BL/RL, BR/RR.
    0.12f, // FL - bánh trước trái
    0.12f, // FR - bánh trước phải
    0.12f, // BL/RL - bánh sau trái
    0.12f  // BR/RR - bánh sau phải
};
constexpr float WHEEL_KD[4] = { // VI: Hệ số D bốn bánh, thứ tự FL, FR, BL/RL, BR/RR.
    0.0f, // FL - tắt D để tránh giật do lượng tử xung encoder
    0.0f, // FR - bánh trước phải
    0.0f, // BL/RL - bánh sau trái
    0.0f  // BR/RR - bánh sau phải
};
// Loaded-floor identification on 2026-09-13 showed strong direction-dependent
// friction, especially on BL. Separate feed-forward/dead-zone values let the
// robot reach the requested speed quickly without forcing the integral term to
// supply the full static-load torque.
// Array order: FL, FR, BL/RL, BR/RR.
constexpr float WHEEL_KFF_POSITIVE[4] = {0.0235f, 0.0115f, 0.0382f, 0.0206f}; // VI: Hệ số bù thuận chiều dương, thứ tự FL, FR, BL, BR.
constexpr float WHEEL_KFF_NEGATIVE[4] = {0.0250f, 0.0343f, 0.0291f, 0.0220f}; // VI: Hệ số bù thuận chiều âm, thứ tự FL, FR, BL, BR.
constexpr int WHEEL_DEADZONE_PWM_POSITIVE[4] = {49, 48, 38, 44}; // VI: PWM thắng ma sát chiều dương, thứ tự FL, FR, BL, BR.
constexpr int WHEEL_DEADZONE_PWM_NEGATIVE[4] = {44, 45, 48, 44}; // VI: PWM thắng ma sát chiều âm, thứ tự FL, FR, BL, BR.
constexpr float WHEEL_INTEGRAL_LIMIT = 250.0f; // VI: Giới hạn tích phân PID tốc độ bánh để chống bão hòa.

// Position, stop-line and distance controllers.
constexpr float POSITION_X_KP = 1.40f; // VI: Hệ số P của bộ điều khiển vị trí X.
constexpr float POSITION_X_KI = 0.05f; // VI: Hệ số I của bộ điều khiển vị trí X.
constexpr float POSITION_X_KD = 0.01f; // VI: Hệ số D của bộ điều khiển vị trí X.
constexpr float POSITION_X_INTEGRAL_LIMIT = 400.0f; // VI: Giới hạn tích phân của bộ điều khiển vị trí X.
constexpr float POSITION_Y_KP = 1.40f; // VI: Hệ số P của bộ điều khiển vị trí Y.
constexpr float POSITION_Y_KI = 0.05f; // VI: Hệ số I của bộ điều khiển vị trí Y.
constexpr float POSITION_Y_KD = 0.01f; // VI: Hệ số D của bộ điều khiển vị trí Y.
constexpr float POSITION_Y_INTEGRAL_LIMIT = 400.0f; // VI: Giới hạn tích phân của bộ điều khiển vị trí Y.
constexpr float STOP_FORWARD_KP = 0.20f; // VI: Hệ số P của bộ điều khiển dừng tiến.
constexpr float STOP_FORWARD_KI = 0.01f; // VI: Hệ số I của bộ điều khiển dừng tiến.
constexpr float STOP_FORWARD_KD = 0.002f; // VI: Hệ số D của bộ điều khiển dừng tiến.
constexpr float STOP_FORWARD_INTEGRAL_LIMIT = 600.0f; // VI: Giới hạn tích phân của bộ điều khiển dừng tiến.
constexpr float STOP_YAW_KP = 0.0018f; // VI: Hệ số P của bộ điều khiển dừng góc yaw.
constexpr float STOP_YAW_KI = 0.0001f; // VI: Hệ số I của bộ điều khiển dừng góc yaw.
constexpr float STOP_YAW_KD = 0.00002f; // VI: Hệ số D của bộ điều khiển dừng góc yaw.
constexpr float STOP_YAW_INTEGRAL_LIMIT = 500.0f; // VI: Giới hạn tích phân của bộ điều khiển dừng góc yaw.
constexpr float TOF_DISTANCE_KP = 0.80f; // VI: Hệ số P của bộ điều khiển ToF khoảng cách.
constexpr float TOF_DISTANCE_KI = 0.0f; // VI: Hệ số I của bộ điều khiển ToF khoảng cách.
constexpr float TOF_DISTANCE_KD = 0.0f; // VI: Hệ số D của bộ điều khiển ToF khoảng cách.
constexpr float TOF_DISTANCE_INTEGRAL_LIMIT = 300.0f; // VI: Giới hạn tích phân của bộ điều khiển ToF khoảng cách.
constexpr float RIGHT_LINE_KP = 0.08f; // VI: Hệ số P của bộ điều khiển bên phải line.
constexpr float RIGHT_LINE_KI = 0.0f; // VI: Hệ số I của bộ điều khiển bên phải line.
constexpr float RIGHT_LINE_KD = 0.0f; // VI: Hệ số D của bộ điều khiển bên phải line.
constexpr float RIGHT_LINE_INTEGRAL_LIMIT = 1000.0f; // VI: Giới hạn tích phân của bộ điều khiển bên phải line.
constexpr float LEFT_LINE_KP = 0.08f; // VI: Hệ số P của bộ điều khiển bên trái line.
constexpr float LEFT_LINE_KI = 0.0f; // VI: Hệ số I của bộ điều khiển bên trái line.
constexpr float LEFT_LINE_KD = 0.0f; // VI: Hệ số D của bộ điều khiển bên trái line.
constexpr float LEFT_LINE_INTEGRAL_LIMIT = 1000.0f; // VI: Giới hạn tích phân của bộ điều khiển bên trái line.

// ---------------- BNO085 + I2C ----------------
// Teensy 4.1 Wire1 defaults: SDA pin 17, SCL pin 16. TODO: confirm wiring.
constexpr uint8_t BNO085_I2C_ADDRESS = 0x4A; // VI: Địa chỉ I2C của cảm biến BNO085.
constexpr uint32_t SENSOR_I2C_HZ = 100000; // VI: Tần số cảm biến I2C, đơn vị Hz.
constexpr bool BNO_USE_GAME_ROTATION_VECTOR = true; // VI: Dùng Game Rotation Vector của BNO085.
// Verified from the 2026-08-28 position log: positive commanded wz previously
// drove the reported yaw negative, turning heading hold into positive feedback.
// With +1, positive Mecanum wz and reported BNO085 yaw use the same sign.
constexpr int8_t BNO_YAW_SIGN = 1; // VI: Dấu quy ước chiều của BNO góc yaw.
constexpr uint32_t BNO_REPORT_INTERVAL_US = 10000; // VI: Thời gian BNO báo cáo chu kỳ, đơn vị µs.
constexpr uint32_t BNO_TIMEOUT_MS = 250; // VI: Thời gian BNO TIMEOUT, đơn vị ms.
constexpr uint32_t BNO_RETRY_INTERVAL_MS = 1000; // VI: Thời gian BNO gửi lại chu kỳ, đơn vị ms.
// Ground step-response tuned on 2026-08-30 in both yaw directions.
constexpr float HEADING_KP = 0.101f; // VI: Hệ số P của bộ điều khiển khóa hướng.
constexpr float HEADING_KI = 0.0f; // VI: Hệ số I của bộ điều khiển khóa hướng.
constexpr float HEADING_KD = 0.004f; // VI: Hệ số D của bộ điều khiển khóa hướng.
constexpr float HEADING_INTEGRAL_LIMIT = 30.0f; // VI: Giới hạn tích phân của bộ điều khiển khóa hướng.
constexpr float MAX_HEADING_WZ_RAD_S = 1.2f; // VI: Tốc độ góc tối đa khóa hướng WZ, đơn vị rad/s.
constexpr float MAX_MANUAL_HEADING_WZ_RAD_S = 0.60f; // VI: Tốc độ góc tối đa thủ công khóa hướng WZ, đơn vị rad/s.
constexpr float MAX_AUTO_HEAD_LOCK_WZ_RAD_S = 1.00f; // VI: Tốc độ góc tối đa AUTO HEAD LOCK WZ, đơn vị rad/s.
constexpr float MAX_LINE_ALIGN_WZ_RAD_S = 0.45f; // VI: Tốc độ góc tối đa line ALIGN WZ, đơn vị rad/s.
constexpr float MAX_BRIDGE_HEADING_WZ_RAD_S = 0.25f; // VI: Tốc độ góc tối đa cầu khóa hướng WZ, đơn vị rad/s.
constexpr float HEADING_OUTPUT_DEADBAND_DEG = 0.50f; // VI: Góc hoặc sai số góc khóa hướng đầu ra vùng chết, đơn vị độ.
// Keep the minimum correction gentle. A 0.18 rad/s kick caused the loaded
// chassis to cross the target repeatedly and create longitudinal drift while
// translating sideways.
constexpr float HEADING_MIN_WZ_RAD_S = 0.06f; // VI: Tốc độ góc khóa hướng tối thiểu WZ, đơn vị rad/s.
constexpr float FINE_HEADING_KP = 0.120f; // VI: Hệ số P của bộ điều khiển tinh khóa hướng.
constexpr float FINE_HEADING_KD = 0.0030f; // VI: Hệ số D của bộ điều khiển tinh khóa hướng.
constexpr float FINE_HEADING_DEADBAND_DEG = 0.25f; // VI: Góc hoặc sai số góc tinh khóa hướng vùng chết, đơn vị độ.
constexpr float FINE_HEADING_MIN_WZ_RAD_S = 0.060f; // VI: Tốc độ góc tinh khóa hướng tối thiểu WZ, đơn vị rad/s.
constexpr float FINE_HEADING_MAX_WZ_RAD_S = 0.35f; // VI: Tốc độ góc tinh khóa hướng tối đa WZ, đơn vị rad/s.
// Stationary/ToF alignment at pick points A and B. Keep these values gentler
// than the moving heading controller so motor dead-zone compensation cannot
// excite a left/right yaw oscillation around the target.
constexpr float STATION_HEADING_DEADBAND_DEG = 0.50f; // VI: Vùng chết khóa hướng A/B/C mềm hơn nhưng vẫn nhỏ hơn ngưỡng xác nhận 0,60 độ.
constexpr float STATION_HEADING_MIN_WZ_RAD_S = 0.035f; // VI: Tốc độ góc tối thiểu mềm hơn khi căn tại A/B/C, đơn vị rad/s.
constexpr float STATION_HEADING_MAX_WZ_RAD_S = 0.10f; // VI: Giới hạn tốc độ quay tại A/B/C để tránh phá bám bánh, đơn vị rad/s.
constexpr float STATION_HEADING_SLEW_RAD_S2 = 0.30f; // VI: Gia tốc góc mềm tại A/B/C, đơn vị rad/s².
constexpr float STATION_HEADING_SETTLED_RATE_DEG_S = 2.0f; // VI: Tốc độ góc tại điểm khóa hướng đã ổn định tốc độ, đơn vị độ/s.
// Dedicated, gentler heading hold for lateral AUTO travel. A slew limit avoids
// the abrupt +/-Wz step that previously rotated the chassis across pick lines.
constexpr float LATERAL_HEADING_DEADBAND_DEG = 0.45f; // VI: Góc hoặc sai số góc chạy ngang khóa hướng vùng chết, đơn vị độ.
constexpr float LATERAL_HEADING_MIN_WZ_RAD_S = 0.06f; // VI: Tốc độ góc chạy ngang khóa hướng tối thiểu WZ, đơn vị rad/s.
constexpr float LATERAL_HEADING_MAX_WZ_RAD_S = 0.20f; // VI: Tốc độ góc chạy ngang khóa hướng tối đa WZ, đơn vị rad/s.
constexpr float LATERAL_HEADING_SLEW_RAD_S2 = 0.80f; // VI: Gia tốc góc chạy ngang khóa hướng SLEW, đơn vị rad/s².
constexpr float LATERAL_LINE_COUNT_MAX_YAW_ERROR_DEG = 2.0f; // VI: Góc hoặc sai số góc chạy ngang line đếm tối đa góc yaw sai số, đơn vị độ.
constexpr float LATERAL_HEADING_RECOVERY_SPEED_FACTOR = 0.45f; // VI: Hệ số của chạy ngang khóa hướng phục hồi tốc độ.

// ---------------- Two MCP3008 devices ----------------
constexpr uint8_t MCP_CENTER_CS = 26; // VI: Chân CS của MCP3008 cho cụm 8 mắt giữa.
constexpr uint8_t MCP_STOP_CS = 8; // VI: Chân CS của MCP3008 cho hai cụm bên và H1/H2.
// Physical MCP_STOP wiring:
// CH0 H1 tail, CH1 H2 front, CH2 left inside, CH3 left middle,
// CH4 left outside, CH5 right inside, CH6 right middle, CH7 right outside.
// Keep the internal group order outside / middle / inside: -1000, 0, +1000.
constexpr uint8_t STOP_LEFT_CHANNELS[3]  = {4, 3, 2}; // VI: Thứ tự kênh cụm trái: ngoài, giữa, trong.
constexpr uint8_t STOP_RIGHT_CHANNELS[3] = {7, 6, 5}; // VI: Thứ tự kênh cụm phải: ngoài, giữa, trong.
constexpr uint32_t MCP_SPI_HZ = 1000000; // VI: Tần số MCP SPI, đơn vị Hz.
// Verified polarity for both MCP3008 arrays:
// black line -> higher ADC than floor -> normalized value approaches 1000.
// A sensor is logically HIGH/on-line when normalized >= LINE_ACTIVE_NORMALIZED.
constexpr bool LINE_IS_HIGHER_THAN_FLOOR = true; // VI: Mức ADC line đen cao hơn nền trắng.
constexpr uint16_t CENTER_SENSOR_MIN[8] = {91,88,85,83,83,83,89,91};
constexpr uint16_t CENTER_SENSOR_MAX[8] = {1023,1023,1023,997,988,908,1023,1023};
// constexpr uint16_t CENTER_SENSOR_MIN[8] = {84,79,76,75,74,75,81,83}; // VI: Mức nền thấp hiệu chuẩn cho CH0–CH7 cụm giữa.
// constexpr uint16_t CENTER_SENSOR_MAX[8] = {1023,1023,1023,1023,969,866,1023,1023}; // VI: Mức line cao hiệu chuẩn cho CH0–CH7 cụm giữa.
// Logical calibration order remains left outside/middle/inside, then right.
constexpr uint16_t STOP_SENSOR_MIN[6] = {50,53,53,50,49,50};
constexpr uint16_t STOP_SENSOR_MAX[6] = {479,1023,717,552,895,755};
// constexpr uint16_t STOP_SENSOR_MIN[6] = {76,76,76,81,83,80}; // VI: Mức nền thấp cho trái ngoài/giữa/trong rồi phải ngoài/giữa/trong.
// constexpr uint16_t STOP_SENSOR_MAX[6] = {1023,1023,1023,1023,1023,1023}; // VI: Mức line cao cho trái ngoài/giữa/trong rồi phải ngoài/giữa/trong.
// H1/H2 use physical CH0 and CH1.
// Replace these defaults after both sensors have seen white floor and black line.
constexpr uint16_t LATERAL_HOLD_SENSOR_MIN[2] = {0, 0}; // VI: Mức nền thấp hiệu chuẩn của H1 đuôi và H2 đầu.
constexpr uint16_t LATERAL_HOLD_SENSOR_MAX[2] = {1023, 1023}; // VI: Mức line cao hiệu chuẩn của H1 đuôi và H2 đầu.
// H1 is mounted at the tail (CH0), H2 at the front (CH1). Black line is HIGH.
constexpr uint8_t LATERAL_HOLD_TAIL_CHANNEL = 0; // VI: Kênh MCP3008 của H1 đặt ở đuôi robot.
constexpr uint8_t LATERAL_HOLD_FRONT_CHANNEL = 1; // VI: Kênh MCP3008 của H2 đặt ở đầu robot.
constexpr uint16_t LATERAL_HOLD_LINE_THRESHOLD = 500; // VI: Ngưỡng phát hiện chạy ngang giữ line.
// Keep H1/H2 active during lateral travel to correct longitudinal drift:
// tail leaves first -> move forward; front leaves first -> move backward.
constexpr bool H1_H2_TRAVEL_HOLD_ENABLED = true; // VI: Bật H1/H2 chống trôi dọc khi robot chạy ngang.
// +1: tail leaves line -> +Vx (forward), front leaves line -> -Vx (backward).
// Change to -1 only if the first low-speed floor test corrects the wrong way.
constexpr int8_t LATERAL_HOLD_CONTROL_SIGN = 1; // VI: Dấu quy ước chiều của chạy ngang giữ điều khiển.
constexpr float LATERAL_HOLD_CORRECTION_SPEED_MM_S = 65.0f; // VI: Tốc độ chạy ngang giữ sửa sai tốc độ, đơn vị mm/s.
constexpr float LATERAL_HOLD_SEARCH_SPEED_MM_S = 35.0f; // VI: Tốc độ chạy ngang giữ tìm kiếm tốc độ, đơn vị mm/s.
constexpr float LATERAL_HOLD_SLEW_MM_S2 = 220.0f; // VI: Gia tốc chạy ngang giữ SLEW, đơn vị mm/s².
constexpr uint32_t LATERAL_HOLD_LOST_TIMEOUT_MS = 450; // VI: Thời gian chạy ngang giữ mất TIMEOUT, đơn vị ms.
constexpr uint32_t LATERAL_HOLD_ACQUIRE_TIMEOUT_MS = 1200; // VI: Thời gian chạy ngang giữ bắt TIMEOUT, đơn vị ms.
// After the 170 mm launch, travel sideways this distance before enabling H1/H2
// and the point-A line counter. This avoids treating the start marking as the
// longitudinal guide line.
constexpr float FIRST_LATERAL_BLIND_DISTANCE_MM = 400.0f; // VI: Khoảng cách/kích thước đầu tiên chạy ngang mù cảm biến khoảng cách, đơn vị mm.
constexpr bool CENTER_LINE_CALIBRATION_VERIFIED = true; // VI: Thông số cấu hình cho giữa line hiệu chuẩn VERIFIED.
constexpr bool STOP_LINE_CALIBRATION_VERIFIED = true; // VI: Thông số cấu hình cho dừng line hiệu chuẩn VERIFIED.
// Confirmed logical order after mapping: outside / middle / inside for each
// group = -1000 / 0 / +1000.
constexpr bool STOP_SENSOR_ORDER_VERIFIED = true; // VI: Thông số cấu hình cho dừng cảm biến thứ tự VERIFIED.
constexpr float LINE_FILTER_ALPHA = 0.55f; // VI: Hệ số lọc của line lọc.
constexpr uint16_t LINE_ACTIVE_NORMALIZED = 550; // VI: Thông số cấu hình cho line kích hoạt NORMALIZED.
constexpr uint8_t CROSS_MIN_ACTIVE_SENSORS = 6; // VI: Thông số cấu hình cho vạch ngang tối thiểu kích hoạt SENSORS.
constexpr uint32_t CROSS_CONFIRM_MS = 35; // VI: Thời gian vạch ngang xác nhận, đơn vị ms.
constexpr uint32_t CROSS_RELEASE_MS = 100; // VI: Thời gian vạch ngang nhả, đơn vị ms.
constexpr uint32_t LINE_SHORT_LOST_MS = 180; // VI: Thời gian line ngắn mất, đơn vị ms.
constexpr uint32_t LINE_FAULT_TIMEOUT_MS = 650; // VI: Thời gian line lỗi TIMEOUT, đơn vị ms.
constexpr float LINE_KP = 0.16f; // VI: Hệ số P của bộ điều khiển line.
constexpr float LINE_KI = 0.02f; // VI: Hệ số I của bộ điều khiển line.
constexpr float LINE_KD = 0.003f; // VI: Hệ số D của bộ điều khiển line.
// Bridge test: positive center-array position requires negative robot Vy.
constexpr int8_t CENTER_LINE_CONTROL_SIGN = -1; // VI: Dấu quy ước chiều của giữa line điều khiển.
// Mecanum lateral authority while following the bridge line. This must be
// comparable to forward speed or the robot cannot recover near an edge.
constexpr float MAX_LINE_VY_MM_S = 220.0f; // VI: Tốc độ tối đa line VY, đơn vị mm/s.
constexpr float LINE_LOST_SEARCH_VY_MM_S = 140.0f; // VI: Tốc độ line mất tìm kiếm VY, đơn vị mm/s.

// Local order for both mirrored side arrays: outside / middle / inside.
constexpr int16_t STOP_WEIGHTS[3] = {-1000, 0, 1000}; // VI: Tọa độ ba mắt: ngoài −1000, giữa 0, trong +1000.
constexpr float STOP_POSITION_TOLERANCE = 120.0f; // VI: Sai số cho phép của dừng vị trí.
constexpr float STOP_SIDE_DIFFERENCE_TOLERANCE = 150.0f; // VI: Sai số cho phép của dừng SIDE DIFFERENCE.
constexpr uint32_t STOP_ALIGNMENT_CONFIRM_MS = 180; // VI: Thời gian dừng căn chỉnh xác nhận, đơn vị ms.
constexpr uint32_t STOP_CROSS_CONFIRM_MS = 40; // VI: Thời gian dừng vạch ngang xác nhận, đơn vị ms.
constexpr uint32_t STOP_CROSS_RELEASE_MS = 100; // VI: Thời gian dừng vạch ngang nhả, đơn vị ms.

// ---------------- Autonomous route (current chassis convention) ----------------
// Existing convention is preserved: +vx forward, +vy right, +wz clockwise.
// Counts belong to mission points A/B, not to a physical side. The active
// three-eye group is selected from the field direction at run time.
// The start marking is ignored while the robot first moves forward by encoder.
// Counting begins only when lateral travel starts; point A is line number 2.
constexpr uint8_t PICK_A_LINE_TARGET = 2; // VI: Số thứ tự line cần bắt để xác định điểm A.
constexpr uint8_t PICK_B_LINE_TARGET = 1; // VI: Số thứ tự line cần bắt để xác định điểm B.
// After POINT_B, the center 8-eye array ignores the line currently under the
// robot, then counts three complete transverse lines before turning forward.
constexpr uint8_t BRIDGE_LINE_TARGET = 3; // VI: Số line ngang mục tiêu trước khi vào cầu.
constexpr uint32_t LINE_DEBOUNCE_MS = 40; // VI: Thời gian line DEBOUNCE, đơn vị ms.
constexpr uint32_t LINE_CLEAR_DEBOUNCE_MS = 100; // VI: Thời gian line thoát DEBOUNCE, đơn vị ms.
// The side arrays cross narrow pick lines quickly. Two control samples are
// sufficient because detection already requires the middle eye or 2/3 eyes.
constexpr uint16_t PICK_LINE_DETECT_NORMALIZED = 400; // VI: Thông số cấu hình cho gắp line DETECT NORMALIZED.
constexpr uint32_t PICK_LINE_CONFIRM_MS = 8; // VI: Thời gian gắp line xác nhận, đơn vị ms.
constexpr uint32_t PICK_LINE_CLEAR_MS = 60; // VI: Thời gian gắp line thoát, đơn vị ms.
constexpr uint32_t X_ALIGN_STABLE_TIME_MS = 120; // VI: Thời gian cặp mắt line tại A/B giữ ổn định trước khi chuyển bước, đơn vị ms.
constexpr uint32_t LINE_SEARCH_TIMEOUT_MS = 5000; // VI: Thời gian line tìm kiếm TIMEOUT, đơn vị ms.
constexpr uint32_t HEADING_STABLE_TIME_MS = 200; // VI: Thời gian heading tại A/B giữ ổn định trước khi xác nhận, đơn vị ms.
constexpr float HEADING_TOLERANCE_DEG = 0.60f; // VI: Góc hoặc sai số góc khóa hướng, đơn vị độ.
constexpr float MOVE_LEFT_SPEED_MM_S = 400.0f; // VI: Tốc độ di chuyển bên trái tốc độ, đơn vị mm/s.
// Do not use ToF/encoder X correction while crossing lateral pick lines. ToF
// changes with wall geometry and previously injected 60-75 mm/s longitudinal
// motion. Absolute ToF alignment remains active after the robot stops at A/B.
constexpr bool LATERAL_LONGITUDINAL_HOLD_ENABLED = false; // VI: Bật bù trôi dọc bằng encoder/ToF khi chạy ngang.
// Encoder-odometry correction that prevents forward/backward drift while strafing.
constexpr float LATERAL_X_HOLD_KP = 1.20f; // VI: Hệ số P của bộ điều khiển chạy ngang X giữ.
constexpr float LATERAL_X_HOLD_DEADBAND_MM = 6.0f; // VI: Khoảng cách/kích thước chạy ngang X giữ vùng chết, đơn vị mm.
constexpr float LATERAL_X_HOLD_MAX_SPEED_MM_S = 140.0f; // VI: Tốc độ chạy ngang X giữ tối đa tốc độ, đơn vị mm/s.
// Absolute wall-distance hold used while strafing; encoder Mecanum odometry
// alone cannot distinguish real longitudinal drift from wheel slip.
constexpr float LATERAL_TOF_HOLD_KP = 0.90f; // VI: Hệ số P của bộ điều khiển chạy ngang ToF giữ.
constexpr float LATERAL_TOF_HOLD_DEADBAND_MM = 8.0f; // VI: Khoảng cách/kích thước chạy ngang ToF giữ vùng chết, đơn vị mm.
constexpr float LATERAL_TOF_HOLD_MAX_SPEED_MM_S = 130.0f; // VI: Tốc độ chạy ngang ToF giữ tối đa tốc độ, đơn vị mm/s.
// Lateral scan from point B to the third center-array line.
constexpr float BRIDGE_LATERAL_SCAN_SPEED_MM_S = 500.0f; // VI: Tốc độ cầu chạy ngang SCAN tốc độ, đơn vị mm/s.
constexpr float X_ALIGN_MAX_SPEED_MM_S = 80.0f; // VI: Tốc độ X ALIGN tối đa tốc độ, đơn vị mm/s.
// Loaded-wheel minimum at A/B: commands below this can remain inside static
// friction and leave the state machine waiting even though line error exists.
constexpr float X_ALIGN_MIN_SPEED_MM_S = 55.0f; // VI: Tốc độ X ALIGN tối thiểu tốc độ, đơn vị mm/s.
constexpr float X_ALIGN_SEARCH_SPEED_MM_S = 55.0f; // VI: Tốc độ X ALIGN tìm kiếm tốc độ, đơn vị mm/s.
// Station alignment uses one short breakaway kick, then keeps creeping without
// the old repeated stop/start pulses. A direction reversal still inserts a
// dead-time before issuing a new breakaway kick.
constexpr float STATION_ALIGN_BREAKAWAY_SPEED_MM_S = 55.0f; // VI: Tốc độ xung đủ thắng ma sát tĩnh khi căn A/B/C.
constexpr uint32_t STATION_ALIGN_PULSE_ON_MS = 70; // VI: Thời gian phát xung phá ma sát ban đầu khi căn line A/B/C.
constexpr float STATION_ALIGN_CREEP_SPEED_MM_S = 35.0f; // VI: Tốc độ bò liên tục sau xung phá ma sát khi căn A/B/C, đơn vị mm/s.
// Nếu lệnh bò tại A/B kéo dài, phát một xung lực ngắn để vượt bậc nhỏ trên sân.
// Cảm biến đạt line hoặc lệnh đổi chiều sẽ hủy xung ngay ở chu kỳ điều khiển kế tiếp.
constexpr float STATION_ALIGN_OBSTACLE_BOOST_SPEED_MM_S = 80.0f; // VI: Tốc độ xung tăng lực vượt bậc tại A/B, đơn vị mm/s.
constexpr uint32_t STATION_ALIGN_OBSTACLE_BOOST_MS = 60; // VI: Thời gian tối đa của một xung tăng lực vượt bậc tại A/B.
constexpr uint32_t STATION_ALIGN_OBSTACLE_BOOST_INTERVAL_MS = 250; // VI: Khoảng bò tối thiểu giữa hai xung tăng lực tại A/B.
constexpr uint32_t STATION_ALIGN_REVERSE_DEADTIME_MS = 110; // VI: Thời gian bắt buộc dừng trước khi đảo chiều căn line.
constexpr uint16_t STATION_SIDE_RELEASE_NORMALIZED = 430; // VI: Ngưỡng nhả hysteresis của mắt cụm trái/phải tại A/B/C.
constexpr uint16_t STATION_HOLD_RELEASE_NORMALIZED = 400; // VI: Ngưỡng nhả hysteresis của H1/H2 tại A/B/C.
// Point B is accepted only when the middle (0) and inside (+1000) eyes are
// simultaneously on black. Their geometric midpoint is +500.
constexpr float POINT_B_PAIR_TARGET_POSITION = 500.0f; // VI: Tâm mục tiêu giữa mắt giữa và mắt trong tại B.
// Official field after the bridge: outside (-1000) + middle (0), midpoint -500.
// Point A/B intentionally keep the original middle + inside target.
constexpr float POST_BRIDGE_OUTER_PAIR_TARGET_POSITION = -500.0f; // VI: Tâm mục tiêu giữa mắt ngoài và mắt giữa sau cầu.
// If the 200 mm A-to-B encoder move passes the line completely, search back
// first and then sweep around the arrival point instead of drifting forever.
constexpr float POINT_B_PAIR_SEARCH_RADIUS_MM = 120.0f; // VI: Khoảng cách/kích thước POINT B cặp mắt tìm kiếm bán kính, đơn vị mm.
constexpr uint32_t POINT_B_PAIR_SEARCH_TIMEOUT_MS = 10000; // VI: Thời gian POINT B cặp mắt tìm kiếm TIMEOUT, đơn vị ms.
// Reject short threshold dropouts while heading settles at B.
constexpr uint32_t POINT_B_PAIR_LOST_GRACE_MS = 250; // VI: Thời gian POINT B cặp mắt mất cho phép mất tạm, đơn vị ms.
constexpr uint32_t STATION_UART_LINE_CONFIRM_MS = 200; // VI: Thời gian bắt buộc toàn bộ mắt tại A/B giữ ổn định ngay trước khi gửi lệnh hạ cơ cấu.
// Preserve the precise 0.6 degree condition first. If BNO rate noise prevents
// 350 ms of perfect settling, accept this still-tight bounded result instead.
constexpr uint32_t POINT_B_HEADING_MAX_SETTLE_MS = 1800; // VI: Thời gian POINT B khóa hướng tối đa SETTLE, đơn vị ms.
constexpr float POINT_B_HEADING_FALLBACK_TOLERANCE_DEG = 1.0f; // VI: Góc hoặc sai số góc POINT B khóa hướng dự phòng, đơn vị độ.
constexpr float POINT_B_HEADING_FALLBACK_RATE_DEG_S = 4.0f; // VI: Tốc độ góc POINT B khóa hướng dự phòng tốc độ, đơn vị độ/s.
constexpr float TOF_ALIGN_MAX_SPEED_MM_S = 100.0f; // VI: Tốc độ ToF ALIGN tối đa tốc độ, đơn vị mm/s.
// Leave wheel-speed headroom for simultaneous line and heading correction.
constexpr float BRIDGE_FORWARD_SPEED_MM_S = 1150.0f; // VI: Tốc độ cầu tiến tốc độ, đơn vị mm/s.
constexpr float BRIDGE_APPROACH_SPEED_MM_S = 1000.0f; // VI: Tốc độ cầu tiếp cận tốc độ, đơn vị mm/s.
constexpr float BLUE_BRIDGE_FORWARD_SPEED_MM_S = 1250.0f; // VI: Tốc độ leo dốc riêng MAP xanh, đơn vị mm/s; thấp hơn giới hạn bánh 1300 mm/s.
constexpr float BLUE_BRIDGE_ASCENT_MIN_SPEED_MM_S = 900.0f; // VI: Tốc độ tiến tối thiểu MAP xanh khi đang leo và 8 mắt vẫn thấy line, tránh giảm ga quá sâu.
constexpr uint32_t BLUE_BRIDGE_LINE_GAP_HOLD_MS = 300; // VI: Thời gian MAP xanh được phép giữ lực khi 8 mắt tạm mất line tại mép nối dốc–mặt cầu.
constexpr float BLUE_BRIDGE_LINE_GAP_SPEED_MM_S = 1150.0f; // VI: Tốc độ giữ khi mất line nhưng vẫn đang ở dốc lên, đơn vị mm/s.
constexpr float BLUE_BRIDGE_PRE_CREST_SPEED_MM_S = 500.0f; // VI: Tốc độ phanh sớm ngay khi góc dốc bắt đầu giảm về mặt cầu, đơn vị mm/s.
constexpr uint32_t BLUE_BRIDGE_CREST_LINE_GAP_HOLD_MS = 1500; // VI: Cho phép mất line tối đa 1 giây tại mép/đỉnh cầu MAP xanh; không áp dụng cho dốc lên.
constexpr float BLUE_BRIDGE_CREST_LINE_GAP_SPEED_MM_S = 500.0f; // VI: Tốc độ giữ khi mất line tại chính mép đỉnh; không dùng lại 1150 mm/s để tránh lao xuống dốc.
constexpr float BLUE_BRIDGE_CREST_SPEED_MM_S = 500.0f; // VI: Tốc độ tiến cơ bản trên mặt phẳng đỉnh cầu MAP xanh, đủ thắng tải cơ cấu lệch A/B.
constexpr uint32_t BLUE_BRIDGE_CREST_TRACTION_HOLD_MS = 100; // VI: Chỉ giữ lực ngắn 100 ms sau khi xác nhận đỉnh.
constexpr float BLUE_BRIDGE_CREST_TRACTION_SPEED_MM_S = 500.0f; // VI: Tốc độ tại mép đỉnh MAP xanh trước khi giảm về tốc độ mặt cầu.
constexpr float BLUE_BRIDGE_CREST_MAX_LINE_VY_MM_S = 90.0f; // VI: Giới hạn bù ngang ở mép đỉnh để tránh lắc trái-phải khi tốc độ tiến đã giảm.
constexpr float BLUE_BRIDGE_DESCENT_MAX_LINE_VY_MM_S = 60.0f; // VI: Giới hạn bù ngang khi xuống dốc, tránh hai bánh bị giảm tốc quá sâu lúc line bắt lại.
constexpr uint32_t BLUE_BRIDGE_DESCENT_LINE_RECOVERY_RAMP_MS = 300; // VI: Thời gian tăng mềm Vy từ 0 sau khi line được bắt lại ở dốc xuống.
constexpr float BLUE_BRIDGE_CREST_MIN_FORWARD_SPEED_MM_S = 450.0f; // VI: Sàn tốc độ tiến trên đỉnh MAP xanh sau bù line, tránh bánh khựng khi cơ cấu đang lệch A/B.
constexpr float BLUE_BRIDGE_DESCENT_MIN_FORWARD_SPEED_MM_S = 220.0f; // VI: Giữ Vx không thấp hơn tốc độ xuống dốc khi line lệch hoặc vừa bắt lại.
constexpr uint32_t BLUE_BRIDGE_DESCENT_LINE_GAP_HOLD_MS = 2200; // VI: MAP xanh giữ tiến thẳng tối đa khoảng 484 mm ở 220 mm/s qua khe mất line tại mép xuống/chân cầu; quá thời gian này mới dừng lỗi.
constexpr float BLUE_BRIDGE_DESCENT_LINE_GAP_SPEED_MM_S = 220.0f; // VI: Giữ cùng tốc độ khi line tạm mất để lúc phục hồi không tạo bước phanh 300 xuống 180 mm/s.
// MAP xanh đổi từ chạy ngang sang tiến ngay tại chân dốc. Không cho thuật toán
// giảm tốc theo độ lệch line xuống dưới mức này vì robot sẽ mất lực và tụt dốc.
constexpr float BLUE_BRIDGE_ENTRY_MIN_FORWARD_SPEED_MM_S = 650.0f; // VI: Tốc độ tiến tối thiểu của MAP xanh khi chuẩn bị leo cầu.
constexpr float BRIDGE_CREST_SPEED_MM_S = 350.0f; // VI: Tốc độ cầu đỉnh cầu tốc độ, đơn vị mm/s.
constexpr float BRIDGE_DESCENT_SPEED_MM_S = 300.0f; // VI: Tốc độ cầu xuống dốc tốc độ, đơn vị mm/s.
// Chỉ dùng sau khi BNO085 xác nhận MAP đỏ đã xuống hết dốc và cho phép bắt
// vạch ngang. Tách riêng để tăng tốc tìm line mà không làm xe lao xuống dốc.
constexpr float RED_POST_BRIDGE_MARKER_SEARCH_SPEED_MM_S = 400.0f; // VI: Tốc độ tiến tìm line ngang MAP đỏ sau cầu, đơn vị mm/s.
constexpr float BLUE_BRIDGE_DESCENT_SPEED_MM_S = 220.0f; // VI: Tốc độ xuống dốc riêng MAP xanh để hạn chế lao dốc, đơn vị mm/s.
// Relative roll/pitch thresholds. The level reference is captured immediately
// after the eight-eye array is centred, before forward bridge travel begins.
constexpr float BRIDGE_INCLINE_ENTER_DEG = 5.0f; // VI: Góc hoặc sai số góc cầu lên dốc bắt đầu, đơn vị độ.
constexpr float BRIDGE_CREST_LEVEL_DEG = 4.0f; // VI: Góc hoặc sai số góc cầu đỉnh cầu mặt phẳng, đơn vị độ.
constexpr uint32_t BRIDGE_INCLINE_CONFIRM_MS = 150; // VI: Thời gian cầu lên dốc xác nhận, đơn vị ms.
constexpr uint32_t BRIDGE_CREST_CONFIRM_MS = 120; // VI: Thời gian cầu đỉnh cầu xác nhận, đơn vị ms.
constexpr uint32_t BRIDGE_MIN_ASCENT_BEFORE_CREST_MS = 300; // VI: Thời gian cầu tối thiểu ASCENT BEFORE đỉnh cầu, đơn vị ms.
constexpr uint32_t BLUE_BRIDGE_MIN_ASCENT_BEFORE_CREST_MS = 600; // VI: MAP xanh phải leo tối thiểu 600 ms trước khi được phép nhận diện đỉnh, tránh nhận nhầm mép đầu dốc.
constexpr float BLUE_BRIDGE_CREST_MIN_PEAK_DEG = 7.0f; // VI: Góc dốc cực đại tối thiểu để cho phép nhận diện đỉnh theo mức giảm góc.
constexpr float BLUE_BRIDGE_CREST_DROP_FROM_PEAK_DEG = 3.0f; // VI: Khi góc dốc giảm ít nhất 3 độ so với cực đại, coi là đang chuyển lên mặt cầu.
constexpr float BLUE_BRIDGE_CREST_DROP_MAX_TILT_DEG = 5.0f; // VI: Chỉ cho phép nhận đỉnh theo độ giảm khi góc đã về gần mặt phẳng; tránh rung/trượt trên dốc bị nhận nhầm là đỉnh.
// After the crest is level, a renewed tilt confirms that the robot is actually
// descending. Transverse-line detection is disabled until this state.
constexpr float BRIDGE_DESCENT_ENTER_DEG = 4.5f; // VI: Góc hoặc sai số góc cầu xuống dốc bắt đầu, đơn vị độ.
constexpr uint32_t BRIDGE_DESCENT_CONFIRM_MS = 100; // VI: Thời gian cầu xuống dốc xác nhận, đơn vị ms.
constexpr float BLUE_BRIDGE_DESCENT_ENTER_DEG = 3.5f; // VI: MAP xanh nhận diện xuống dốc sớm hơn khi góc đạt 3,5 độ.
constexpr uint32_t BLUE_BRIDGE_DESCENT_CONFIRM_MS = 70; // VI: Thời gian xác nhận xuống dốc MAP xanh để cắt lực đỉnh sớm, đơn vị ms.
// Do not arm transverse-line detection at the vibrating foot of the bridge.
// The robot must return close to level continuously for this settling time.
constexpr float BRIDGE_DESCENT_EXIT_LEVEL_DEG = 2.0f; // VI: Góc hoặc sai số góc cầu xuống dốc kết thúc mặt phẳng, đơn vị độ.
constexpr uint32_t BRIDGE_DESCENT_EXIT_LEVEL_CONFIRM_MS = 800; // VI: Thời gian cầu xuống dốc kết thúc mặt phẳng xác nhận, đơn vị ms.
// MAP xanh phải rời khỏi vùng dập/rung ở chân cầu bằng encoder trước khi bất
// kỳ cụm mắt nào được phép chốt vạch ngang nhiệm vụ sau cầu.
constexpr float BLUE_BRIDGE_MARKER_MIN_FORWARD_AFTER_LEVEL_MM = 150.0f; // VI: Quãng tiến tối thiểu sau khi BNO xác nhận mặt phẳng rồi mới cho phép bắt vạch ngang ở MAP xanh.


// The longitudinal guide normally covers only 1-2 center eyes. A transverse
// marker covers most of the array, so detect it directly without stacking the
// centerLine.cross debounce with another long confirmation delay.
constexpr uint16_t BRIDGE_CENTER_MARKER_ACTIVE_NORMALIZED = 400; // VI: Thông số cấu hình cho cầu giữa vạch đánh dấu kích hoạt NORMALIZED.
constexpr uint8_t BRIDGE_CENTER_MARKER_MIN_ACTIVE_SENSORS = 5; // VI: Thông số cấu hình cho cầu giữa vạch đánh dấu tối thiểu kích hoạt SENSORS.
constexpr uint32_t BRIDGE_END_MARKER_CONFIRM_MS = 30; // VI: Thời gian cầu END vạch đánh dấu xác nhận, đơn vị ms.
constexpr uint32_t BLUE_BRIDGE_END_MARKER_CONFIRM_MS = 70; // VI: MAP xanh xác nhận 8 mắt trong 70 ms; đủ lọc xung dập ngắn nhưng không bỏ vạch khi xe còn quán tính.
constexpr uint32_t BRIDGE_SIDE_MARKER_CONFIRM_MS = 80; // VI: Thời gian xác nhận vạch bằng hai cụm bên ở MAP đỏ, đơn vị ms.
constexpr uint32_t BLUE_BRIDGE_SIDE_MARKER_CONFIRM_MS = 100; // VI: MAP xanh xác nhận đồng thời hai cụm bên trong 100 ms để dự phòng khi hàng 8 mắt không chốt được vạch.
// Once an end marker is latched, continue following the longitudinal guide by
// encoder before stopping. The center array is farther forward than the sides.
constexpr float BRIDGE_CENTER_MARKER_ADVANCE_MM = 300.0f; // VI: Khoảng cách/kích thước cầu giữa vạch đánh dấu tiến thêm, đơn vị mm.
constexpr float BRIDGE_SIDE_MARKER_ADVANCE_MM = 300.0f; // VI: Khoảng cách/kích thước cầu SIDE vạch đánh dấu tiến thêm, đơn vị mm.
constexpr float BRIDGE_POST_MARKER_SPEED_MM_S = 300.0f; // VI: Tốc độ tiến thêm sau khi bắt vạch ngang cuối cầu, đơn vị mm/s.
// After the end-marker advance, move right 190 mm by encoder, then reuse
// the side group's outside (-1000) + middle (0) eye PID to pull onto the line.
// The outside+middle pair is the primary stop/advance condition. ESP32 E18 is
// auxiliary telemetry only and must never block the following 70 mm move.
// During that move, one remaining point-B eye steers laterally; losing both
// pauses forward motion until the middle+inside pair is stable again.
constexpr float POST_BRIDGE_E18_RIGHT_DISTANCE_MM = 250.0f; // VI: Khoảng cách/kích thước sau cầu E18 bên phải khoảng cách, đơn vị mm.
constexpr float POST_BRIDGE_E18_RIGHT_SPEED_MM_S = 200.0f; // VI: Tốc độ sau cầu E18 bên phải tốc độ, đơn vị mm/s.
constexpr float POST_BRIDGE_E18_ALIGN_MIN_SPEED_MM_S = 90.0f; // VI: Tốc độ sau cầu E18 ALIGN tối thiểu tốc độ, đơn vị mm/s.
constexpr float POST_BRIDGE_E18_ALIGN_MAX_SPEED_MM_S = 140.0f; // VI: Tốc độ sau cầu E18 ALIGN tối đa tốc độ, đơn vị mm/s.
constexpr uint32_t POST_BRIDGE_E18_LINE_CONFIRM_MS = 40; // VI: Thời gian sau cầu E18 line xác nhận, đơn vị ms.
constexpr uint32_t POST_BRIDGE_E18_POLL_MS = 100; // VI: Thời gian sau cầu E18 đọc định kỳ, đơn vị ms.
constexpr float POST_BRIDGE_E18_FORWARD_DISTANCE_MM = 30.0f; // VI: Khoảng cách/kích thước sau cầu E18 tiến khoảng cách, đơn vị mm.
constexpr float BLUE_POST_BRIDGE_E18_FORWARD_DISTANCE_MM = 100.0f; // VI: Khoảng tiến riêng MAP xanh sau khi cặp mắt bắt line sau cầu, trước khi chuyển sang hành trình ngang/chéo, đơn vị mm.
constexpr float POST_BRIDGE_E18_FORWARD_SPEED_MM_S = 200.0f; // VI: Tốc độ sau cầu E18 tiến tốc độ, đơn vị mm/s.
constexpr float POST_BRIDGE_E18_FORWARD_MIN_SPEED_MM_S = 110.0f; // VI: Tốc độ sau cầu E18 tiến tối thiểu tốc độ, đơn vị mm/s.
constexpr uint32_t POST_BRIDGE_FORWARD_PAIR_LOST_CONFIRM_MS = 60; // VI: Thời gian sau cầu tiến cặp mắt mất xác nhận, đơn vị ms.
constexpr uint32_t POST_BRIDGE_E18_SEARCH_TIMEOUT_MS = 13000; // VI: Thời gian sau cầu E18 tìm kiếm TIMEOUT, đơn vị ms.
// Route from completed 2A placement to the 2B drop line. Physical right is
// negative body Y on the verified drivetrain; backward is negative body X.
constexpr float POST_THA2A_RIGHT_DISTANCE_MM = 450.0f; // VI: Khoảng cách/kích thước sau THA_2A bên phải khoảng cách, đơn vị mm.
constexpr float POST_THA2A_BACKWARD_DISTANCE_MM = 200.0f; // VI: Khoảng cách/kích thước sau THA_2A lùi khoảng cách, đơn vị mm.
constexpr float POST_THA2A_DIAGONAL_SPEED_MM_S = 600.0f; // VI: Tốc độ chạy chéo từ THA_2A sang khu vực THA_2B, đơn vị mm/s.
// Watch the right outside/middle eyes during the coarse diagonal move. The
// previous pickup line must first clear, then touching either eye on the next
// vertical line immediately hands control to the slow pair aligner.
constexpr float THA2B_COARSE_LINE_ARM_DISTANCE_MM = 80.0f; // VI: Khoảng cách/kích thước THA_2B thô line cho phép bắt khoảng cách, đơn vị mm.
constexpr uint16_t THA2B_COARSE_LINE_THRESHOLD = 450; // VI: Ngưỡng phát hiện THA_2B thô line.
constexpr uint32_t THA2B_COARSE_CLEAR_CONFIRM_MS = 40; // VI: Thời gian THA_2B thô thoát xác nhận, đơn vị ms.
constexpr uint32_t THA2B_COARSE_HIT_CONFIRM_MS = 10; // VI: Thời gian THA_2B thô chạm xác nhận, đơn vị ms.
constexpr float THA2B_PAIR_SEARCH_SPEED_MM_S = 60.0f; // VI: Tốc độ THA_2B cặp mắt tìm kiếm tốc độ, đơn vị mm/s.
constexpr float THA2B_PAIR_ALIGN_MIN_SPEED_MM_S = 25.0f; // VI: Tốc độ THA_2B cặp mắt ALIGN tối thiểu tốc độ, đơn vị mm/s.
constexpr float THA2B_PAIR_ALIGN_MAX_SPEED_MM_S = 90.0f; // VI: Tốc độ THA_2B cặp mắt ALIGN tối đa tốc độ, đơn vị mm/s.
constexpr uint32_t THA2B_PAIR_CONFIRM_MS = 80; // VI: Thời gian THA_2B cặp mắt xác nhận, đơn vị ms.
constexpr float THA2B_LINE_REVERSE_SPEED_MM_S = 150.0f; // VI: Tốc độ THA_2B line lùi tốc độ, đơn vị mm/s.
// Once both side groups confirm the horizontal marker, slow down before the
// trailing edge so inertia cannot carry the chassis too far past the line.
constexpr float THA2B_LINE_CLEAR_SPEED_MM_S = 100.0f; // VI: Tốc độ THA_2B line thoát tốc độ, đơn vị mm/s.
constexpr uint8_t THA2B_LEFT_MARKER_MIN_ACTIVE = 2; // VI: Thông số cấu hình cho THA_2B bên trái vạch đánh dấu tối thiểu kích hoạt.
constexpr uint32_t THA2B_HORIZONTAL_CONFIRM_MS = 50; // VI: Thời gian THA_2B line ngang xác nhận, đơn vị ms.
// Stop on the first control sample where both horizontal references clear.
constexpr uint32_t THA2B_HORIZONTAL_CLEAR_CONFIRM_MS = 0; // VI: Thời gian THA_2B line ngang thoát xác nhận, đơn vị ms.
constexpr uint32_t THA2B_DIAGONAL_TIMEOUT_MS = 8000; // VI: Thời gian THA_2B chéo TIMEOUT, đơn vị ms.
constexpr uint32_t THA2B_LINE_SEARCH_TIMEOUT_MS = 13000; // VI: Thời gian THA_2B line tìm kiếm TIMEOUT, đơn vị ms.
constexpr uint32_t THA2B_REVERSE_TIMEOUT_MS = 15000; // VI: Thời gian THA_2B lùi TIMEOUT, đơn vị ms.
// Final route after ESP32 confirms THA_2B. Physical right is -body Y and
// reverse is -body X on the verified drivetrain.
constexpr float POST_THA2B_EXIT_RIGHT_DISTANCE_MM = 2000.0f; // VI: Khoảng cách/kích thước sau THA_2B kết thúc bên phải khoảng cách, đơn vị mm.
constexpr float POST_THA2B_EXIT_REVERSE_DISTANCE_MM = 3000.0f; // VI: Khoảng cách/kích thước sau THA_2B kết thúc lùi khoảng cách, đơn vị mm.
constexpr float POST_THA2B_EXIT_DIAGONAL_SPEED_MM_S = 1200.0f; // VI: Tốc độ sau THA_2B kết thúc chéo tốc độ, đơn vị mm/s.
constexpr uint32_t POST_THA2B_EXIT_DIAGONAL_TIMEOUT_MS = 18000; // VI: Thời gian sau THA_2B kết thúc chéo TIMEOUT, đơn vị ms.

constexpr float BNO_TILT_FILTER_ALPHA = 0.35f; // VI: Hệ số lọc của BNO TILT lọc.
constexpr float BRIDGE_LINE_SEARCH_RADIUS_MM = 120.0f; // VI: Bán kính quét tìm line lên cầu sau B; thu ngắn để giảm thời gian tìm, đơn vị mm.
constexpr float BRIDGE_LINE_SEARCH_SPEED_MM_S = 90.0f; // VI: Tốc độ quét tìm line lên cầu; đủ thắng ma sát ở tốc độ thấp, đơn vị mm/s.
constexpr float BRIDGE_ENTRY_CENTER_KP = 0.08f; // VI: Hệ số P của bộ điều khiển cầu lối vào giữa.
// Minimum command overcomes drivetrain stiction while centering on line 3.
constexpr float BRIDGE_ENTRY_CENTER_MIN_VY_MM_S = 90.0f; // VI: Tốc độ cầu lối vào giữa tối thiểu VY, đơn vị mm/s.
constexpr float BRIDGE_ENTRY_CENTER_MAX_VY_MM_S = 140.0f; // VI: Tốc độ cầu lối vào giữa tối đa VY, đơn vị mm/s.
constexpr float BRIDGE_ENTRY_CENTER_TOLERANCE = 300.0f; // VI: Sai số cho phép của cầu lối vào giữa.
constexpr uint32_t BRIDGE_ENTRY_CENTER_CONFIRM_MS = 80; // VI: Thời gian cầu lối vào giữa xác nhận, đơn vị ms.
// Stop this commissioning route after the center array has followed the
// bridge line and detected this many complete transverse markers.
constexpr uint8_t BRIDGE_STOP_CROSS_TARGET = 3; // VI: Giá trị mục tiêu của cầu dừng vạch ngang.
constexpr uint32_t BRIDGE_LINE_ACQUIRE_CONFIRM_MS = 40; // VI: Line hợp lệ liên tục 40 ms thì vào bám cầu ngay; chống nhiễu nhưng không dừng chờ lâu.
constexpr uint32_t BRIDGE_LINE_ACQUIRE_TIMEOUT_MS = 10000; // VI: Thời gian cầu line bắt TIMEOUT, đơn vị ms.
constexpr uint32_t BRIDGE_FOLLOW_TIMEOUT_MS = 30000; // VI: Thời gian tối đa toàn hành trình cầu; tăng vì pha đỉnh/xuống dốc đã giảm tốc, đơn vị ms.
// TODO: set to +1 or -1 only after low-speed tests. Zero locks AUTO START.
constexpr int8_t TOF_CONTROL_SIGN = 1; // VI: Dấu quy ước chiều của ToF điều khiển.
// The two logical arrays are ordered outside -> body. Live AUTO logging showed
// the left array oscillating with -1 while the right array converged with +1.
constexpr int8_t LEFT_STOP_CONTROL_SIGN = 1; // VI: Dấu quy ước chiều của bên trái dừng điều khiển.
constexpr int8_t RIGHT_STOP_CONTROL_SIGN = -1; // VI: Dấu quy ước chiều của bên phải dừng điều khiển.

// ---------------- Optical flow ----------------
// Temporary conflict-free choice. TODO: confirm module UART and baud.
// Disabled while FL encoder uses pins 0/1. Move optical flow to another UART
// before enabling it; Serial1 would change the pin mux and break FL encoder.
constexpr bool OPTICAL_FLOW_ENABLED = false; // VI: Bật hoặc tắt cảm biến Optical Flow.
#define FLOW_UART Serial1 // VI: Cổng UART dành cho Optical Flow.
constexpr uint32_t FLOW_BAUD = 115200; // VI: Tốc độ truyền Optical Flow, đơn vị baud.
constexpr uint8_t MAVLINK_OPTICAL_FLOW_RAD_ID = 106; // VI: Thông số cấu hình cho MAVLINK OPTICAL Optical Flow.
constexpr uint8_t MAVLINK_OPTICAL_FLOW_RAD_LEN = 44; // VI: Thông số cấu hình cho MAVLINK OPTICAL Optical Flow.
constexpr uint8_t MAVLINK_OPTICAL_FLOW_RAD_CRC_EXTRA = 138; // VI: Thông số cấu hình cho MAVLINK OPTICAL Optical Flow.
constexpr int8_t FLOW_X_FROM_SENSOR_AXIS = 0; // VI: Thông số cấu hình cho Optical Flow X lấy từ cảm biến trục.
constexpr int8_t FLOW_Y_FROM_SENSOR_AXIS = 1; // VI: Thông số cấu hình cho Optical Flow Y lấy từ cảm biến trục.
constexpr int8_t FLOW_X_SIGN = 1; // VI: Dấu quy ước chiều của Optical Flow X.
constexpr int8_t FLOW_Y_SIGN = 1; // VI: Dấu quy ước chiều của Optical Flow Y.
constexpr float FLOW_SCALE_X_MM = 1.0f; // VI: Khoảng cách/kích thước Optical Flow tỷ lệ X, đơn vị mm.
constexpr float FLOW_SCALE_Y_MM = 1.0f; // VI: Khoảng cách/kích thước Optical Flow tỷ lệ Y, đơn vị mm.
constexpr uint8_t FLOW_MIN_QUALITY = 80; // VI: Thông số cấu hình cho Optical Flow tối thiểu chất lượng.
constexpr uint32_t FLOW_TIMEOUT_MS = 250; // VI: Thời gian Optical Flow TIMEOUT, đơn vị ms.
constexpr float ODOM_ENCODER_WEIGHT = 1.0f; // VI: Trọng số của odometry encoder.
constexpr float ODOM_FLOW_WEIGHT = 0.0f; // VI: Trọng số của odometry Optical Flow.
constexpr float LATERAL_SLIP_THRESHOLD_MM_S = 180.0f; // VI: Tốc độ chạy ngang trượt, đơn vị mm/s.

// ---------------- VL53L3CX (DFRobot SEN0378) ----------------
// Shares Teensy 4.1 Wire1 (SDA17/SCL16) with BNO085.
// BNO085 uses 0x4A and VL53L3CX uses its default address 0x29.
// This competition route no longer uses ToF. Keep the driver code available
// for the standalone diagnostic build, but do not initialize or poll it here.
constexpr bool TOF_ENABLED = false; // VI: Bật hoặc tắt cảm biến ToF trong tuyến thi đấu.
constexpr uint8_t TOF_I2C_ADDRESS = 0x29; // VI: Địa chỉ giao tiếp của ToF I2C.
// Measured alignment point: H1 and H2 are centred on the black line at ~160 mm.
constexpr uint16_t TOF_TARGET_MM = 160; // VI: Khoảng cách/kích thước ToF mục tiêu, đơn vị mm.
constexpr uint16_t TOF_MIN_VALID_MM = 35; // VI: Khoảng cách/kích thước ToF tối thiểu VALID, đơn vị mm.
constexpr uint16_t TOF_MAX_VALID_MM = 800; // VI: Khoảng cách/kích thước ToF tối đa VALID, đơn vị mm.
// Reject implausible single-step jumps caused by VL53L3CX ghost targets.
constexpr uint16_t TOF_MAX_SAMPLE_JUMP_MM = 250; // VI: Khoảng cách/kích thước ToF tối đa SAMPLE JUMP, đơn vị mm.
constexpr uint32_t TOF_TIMEOUT_MS = 1500; // VI: Thời gian ToF TIMEOUT, đơn vị ms.
// Hold the robot stopped while VL53L3CX recovers from short motion/power-noise
// dropouts. The 30 cm target and tolerance remain unchanged.
constexpr uint32_t TOF_ALIGNMENT_RECOVERY_MS = 3000; // VI: Thời gian ToF căn chỉnh phục hồi, đơn vị ms.
// At the starting mark the sensor is only about 20 mm from the wall, below
// the reliable VL53L3CX range. Creep forward by encoder until ToF can take over.
// Initial launch is encoder-only; ToF is intentionally ignored here.
constexpr float RED_START_ENCODER_DISTANCE_MM = 200.0f; // VI: Quãng tiến lúc xuất phát của MAP đỏ, đơn vị mm.
constexpr float BLUE_START_ENCODER_DISTANCE_MM = 130.0f; // VI: Quãng tiến lúc xuất phát của MAP xanh, ngắn hơn MAP đỏ 70 mm.
constexpr float START_ENCODER_SPEED_MM_S = 280.0f; // VI: Tốc độ xuất phát encoder tốc độ, đơn vị mm/s.
constexpr float START_LATERAL_DISTANCE_MM = 1200.0f; // VI: Khoảng cách/kích thước xuất phát chạy ngang khoảng cách, đơn vị mm.
constexpr float START_LATERAL_SPEED_MM_S = 700.0f; // VI: Tốc độ xuất phát chạy ngang tốc độ, đơn vị mm/s.
constexpr float START_HOLD_LINE_SEARCH_SPEED_MM_S = 280.0f; // VI: Tốc độ xuất phát giữ line tìm kiếm tốc độ, đơn vị mm/s.
constexpr uint32_t START_HOLD_LINE_CONFIRM_MS = 100; // VI: Thời gian xuất phát giữ line xác nhận, đơn vị ms.
constexpr uint32_t START_HOLD_LINE_SEARCH_TIMEOUT_MS = 8000; // VI: Thời gian xuất phát giữ line tìm kiếm TIMEOUT, đơn vị ms.
// After H1/H2 acquire the longitudinal guide, creep sideways until the
// logical right array's middle and inside eyes both see the pickup line.
constexpr float POINT_A_RIGHT_ALIGN_SPEED_MM_S = 50.0f; // VI: Tốc độ POINT A bên phải ALIGN tốc độ, đơn vị mm/s.
constexpr uint16_t POINT_A_RIGHT_SENSOR_THRESHOLD = 500; // VI: Ngưỡng phát hiện POINT A bên phải cảm biến.
constexpr uint32_t POINT_A_RIGHT_CONFIRM_MS = 120; // VI: Thời gian POINT A bên phải xác nhận, đơn vị ms.
constexpr uint32_t POINT_A_RIGHT_PAIR_WINDOW_MS = 1200; // VI: Thời gian POINT A bên phải cặp mắt WINDOW, đơn vị ms.
constexpr uint32_t POINT_A_RIGHT_SEARCH_TIMEOUT_MS = 15000; // VI: Thời gian POINT A bên phải tìm kiếm TIMEOUT, đơn vị ms.
// Measured lateral distance from the start of the sideways leg to point A.
// Encoder distance defines only the search window; the line sensors still
// decide the final stop/alignment because Mecanum wheels can slip.
constexpr float POINT_A_EXPECTED_LATERAL_MM = 1175.0f; // VI: Khoảng cách/kích thước POINT A dự kiến chạy ngang, đơn vị mm.
constexpr float POINT_A_SEARCH_HALF_RANGE_MM = 100.0f; // VI: Chỉ giảm tốc trong vùng ±100 mm quanh vị trí A dự kiến; bắt đầu tại khoảng 1075 mm.
// Cho phép tiếp tục đi đúng chiều qua mép trên của vùng dự kiến khi encoder
// thiếu quãng đường do bánh trượt. Chỉ đảo chiều tìm lại sau giới hạn mở rộng.
constexpr float POINT_A_SEARCH_FORWARD_EXTENSION_MM = 200.0f; // VI: Quãng mở rộng tìm A theo chiều đang chạy trước khi cho phép đảo chiều, đơn vị mm.
constexpr float POINT_A_SEARCH_SPEED_MM_S = 90.0f; // VI: Tốc độ POINT A tìm kiếm tốc độ, đơn vị mm/s.
// Bù lực tụt dọc do mặt sân MAP đỏ bị nghiêng, chỉ dùng khi chạy ngang từ
// vị trí xuất phát đến A. +Vx là tiến lên; đặt 0 để tắt hoàn toàn phần bù này.
constexpr float RED_START_TO_A_UPHILL_BIAS_MM_S = 65.0f; // VI: Tốc độ bù tiến chống tụt của MAP đỏ từ xuất phát đến A, đơn vị mm/s.
// Point A becomes the new odometry origin. Move this measured lateral
// distance toward B, then let the B side-array perform the final alignment.
constexpr float POINT_A_TO_B_DISTANCE_MM = 270.0f; // VI: Khoảng cách/kích thước POINT A TO B khoảng cách, đơn vị mm.
constexpr float POINT_A_TO_B_SPEED_MM_S = 400.0f; // VI: Tốc độ chạy ngang từ điểm A sang điểm B, đơn vị mm/s.
// Bù lực tụt dọc riêng cho đoạn chạy ngang A -> B của MAP đỏ.
// +Vx là tiến lên; đặt 0 để tắt mà không ảnh hưởng bù xuất phát -> A.
constexpr float RED_A_TO_B_UPHILL_BIAS_MM_S = 30.0f; // VI: Tốc độ bù tiến chống tụt của MAP đỏ từ A đến B, đơn vị mm/s.
// Once point A''s middle/inside side eyes are locked on the line, keep that
// lateral position and search forward/back for H1/H2 within this safe radius.
constexpr float POINT_A_H_SEARCH_RADIUS_MM = 100.0f; // VI: Khoảng cách/kích thước POINT A H tìm kiếm bán kính, đơn vị mm.
constexpr float POINT_A_H_SEARCH_SPEED_MM_S = 35.0f; // VI: Tốc độ POINT A H tìm kiếm tốc độ, đơn vị mm/s.
constexpr uint32_t POINT_A_H_SEARCH_TIMEOUT_MS = 15000; // VI: Thời gian POINT A H tìm kiếm TIMEOUT, đơn vị ms.
// Chỉ dùng khi cả H1 và H2 đều không thấy line tại A/B. +1 là tiến, -1 là lùi.
// Sau khi chạm giới hạn bán kính, robot tự đảo chiều để quét phía còn lại.
constexpr int8_t RED_POINT_H_SEARCH_FIRST_SIGN = 1; // VI: MAP đỏ ưu tiên tiến để tìm lại line ngang tại A/B.
constexpr int8_t BLUE_POINT_H_SEARCH_FIRST_SIGN = -1; // VI: MAP xanh ưu tiên lùi để tìm lại line ngang tại A/B.
// After POINT_B, move diagonally: robot-left while advancing toward the bridge.
constexpr float RED_POST_B_FORWARD_DISTANCE_MM = 200.0f; // VI: MAP đỏ tiến thêm sau B khi chạy chéo tới line cầu, đơn vị mm.
constexpr float RED_POST_B_LATERAL_DISTANCE_MM = 1450.0f; // VI: MAP đỏ chạy ngang sau B tới line cầu, đơn vị mm.
constexpr float BLUE_POST_B_FORWARD_DISTANCE_MM = 60.0f; // VI: MAP xanh tiến thêm sau B khi chạy chéo tới line cầu, đơn vị mm.
constexpr float BLUE_POST_B_LATERAL_DISTANCE_MM = 1400.0f; // VI: MAP xanh chạy ngang sau B tới line cầu, đơn vị mm.
constexpr float POST_B_LEFT_SPEED_MM_S = 800.0f; // VI: Tốc độ sau B bên trái tốc độ, đơn vị mm/s.
constexpr float TOF_START_ESCAPE_DISTANCE_MM = 250.0f; // VI: Khoảng cách/kích thước ToF xuất phát thoát khoảng cách, đơn vị mm.
constexpr float TOF_START_ESCAPE_SPEED_MM_S = 300.0f; // VI: Tốc độ ToF xuất phát thoát tốc độ, đơn vị mm/s.
constexpr uint8_t TOF_MEDIAN_SAMPLES = 5; // VI: Số mẫu dùng cho ToF trung vị.
constexpr float TOF_COARSE_SPEED_MM_S = 180.0f; // VI: Tốc độ ToF thô tốc độ, đơn vị mm/s.
constexpr float TOF_FINE_SPEED_MM_S = 65.0f; // VI: Tốc độ ToF tinh tốc độ, đơn vị mm/s.
constexpr float TOF_TOLERANCE_MM = 30.0f; // VI: Khoảng cách/kích thước ToF, đơn vị mm.
constexpr uint32_t TOF_CONFIRM_MS = 200; // VI: Thời gian ToF xác nhận, đơn vị ms.

// ---------------- UART and safety ----------------
// RBT/1 GUI telemetry through the CP210x/air-radio link (PC COM5).
// Teensy 4.1 Serial5: RX5 pin 21, TX5 pin  20.
#define TELEMETRY_UART Serial5 // VI: Cổng UART telemetry nối giao diện COM5.
constexpr uint32_t TELEMETRY_BAUD = 57600; // VI: Tốc độ truyền telemetry, đơn vị baud.
constexpr uint8_t TELEMETRY_RX_PIN = 21; // VI: Chân GPIO dùng cho telemetry RX.
constexpr uint8_t TELEMETRY_TX_PIN = 20; // VI: Chân GPIO dùng cho telemetry TX.

// ESP32 link on Teensy 4.1 Serial6, separate from Serial5 telemetry.
// Confirmed free from the motor driver: TX6 pin 24, RX6 pin 25.
constexpr bool ESP32_UART_ENABLED = true; // VI: Bật hoặc tắt UART Teensy–ESP32 cơ cấu.
#define ESP32_UART Serial6 // VI: Cổng UART Teensy giao tiếp ESP32 cơ cấu.
constexpr uint8_t ESP32_TX_PIN = 24; // VI: Chân GPIO dùng cho ESP32 TX.
constexpr uint8_t ESP32_RX_PIN = 25; // VI: Chân GPIO dùng cho ESP32 RX.
constexpr uint32_t ESP32_BAUD = 57600; // VI: Tốc độ truyền ESP32, đơn vị baud.
// AUTO always uses the real ESP32 mechanism link.
constexpr uint32_t ESP32_REPLY_TIMEOUT_MS = 15000; // VI: Thời gian ESP32 phản hồi TIMEOUT, đơn vị ms.
constexpr uint32_t ESP32_HOME_READY_TIMEOUT_MS = 30000; // VI: Thời gian ESP32 HOME sẵn sàng TIMEOUT, đơn vị ms.
constexpr uint32_t ESP32_FIELD_SYNC_RETRY_MS = 500; // VI: Chu kỳ Teensy gửi lại FIELD,RED/BLUE cho tới khi ESP32 phản hồi đúng MAP.
// Post-bridge handshake: Teensy notifies ESP32 after the 70 mm line-held move.
// ESP32 must reply with the exact token DONE_THA2A. Retries are idempotent.
constexpr uint32_t DONE_THA2A_RETRY_INTERVAL_MS = 1000; // VI: Thời gian DONE THA_2A gửi lại chu kỳ, đơn vị ms.
constexpr uint8_t DONE_THA2A_MAX_ATTEMPTS = 6; // VI: Số lần thử tối đa cho DONE THA_2A tối đa.
constexpr uint32_t DONE_THA2A_TOTAL_TIMEOUT_MS = 7000; // VI: Thời gian DONE THA_2A tổng TIMEOUT, đơn vị ms.
constexpr uint32_t BRIDGE_B_RESUME_START_RETRY_MS = 1000; // VI: Chu kỳ gửi lại BRIDGE_STABLE khi ESP32 chưa nhận lệnh, đơn vị ms.
constexpr uint32_t BRIDGE_B_RESUME_DONE_RETRY_MS = 5000; // VI: Chu kỳ hỏi lại nếu ESP32 đã ACK nhưng chưa trả DONE_BRIDGE_B, đơn vị ms.
constexpr uint8_t BRIDGE_B_RESUME_MAX_ATTEMPTS = 10; // VI: Số lần tối đa gửi BRIDGE_STABLE trước khi báo lỗi.
constexpr uint32_t BRIDGE_B_RESUME_TOTAL_TIMEOUT_MS = 30000; // VI: Thời gian tối đa chờ B nâng nốt 2/3 quãng đường, khoảng 1133 mm, sau cầu.
// Final 2B action sends CHO_THA2B and waits for exact DONE_THA2B.
constexpr uint32_t DONE_THA2B_RETRY_INTERVAL_MS = 1000; // VI: Thời gian DONE THA_2B gửi lại chu kỳ, đơn vị ms.
constexpr uint8_t DONE_THA2B_MAX_ATTEMPTS = 6; // VI: Số lần thử tối đa cho DONE THA_2B tối đa.
constexpr uint32_t DONE_THA2B_TOTAL_TIMEOUT_MS = 7000; // VI: Thời gian DONE THA_2B tổng TIMEOUT, đơn vị ms.
constexpr uint32_t STATE_DEFAULT_TIMEOUT_MS = 12000; // VI: Thời gian trạng thái mặc định TIMEOUT, đơn vị ms.
// Normal telemetry is emitted as small staged records. Four complete updates
// per second keep the dashboard responsive without saturating the 57600-baud
// radio or delaying command/STOP processing.
constexpr uint32_t TELEMETRY_PERIOD_MS = 250; // VI: Thời gian telemetry PERIOD, đơn vị ms.
constexpr uint32_t TELEMETRY_STAGE_GAP_MS = 8; // VI: Thời gian telemetry giai đoạn khoảng nghỉ, đơn vị ms.
constexpr uint32_t TUNER_WATCHDOG_MS = 600; // VI: Thời gian tune PID watchdog, đơn vị ms.
constexpr uint32_t MANUAL_DRIVE_WATCHDOG_MS = 500; // VI: Thời gian thủ công điều khiển chạy watchdog, đơn vị ms.
constexpr uint32_t POSITION_LINK_WATCHDOG_MS = 600; // VI: Thời gian vị trí liên kết watchdog, đơn vị ms.
// Position tests use a gentler heading correction so k*wz cannot dominate
// the translational wheel targets while the robot is converging on X/Y.
constexpr float MAX_POSITION_HEADING_WZ_RAD_S = 0.35f; // VI: Tốc độ góc tối đa vị trí khóa hướng WZ, đơn vị rad/s.

// Active-low physical controls. Each input uses Teensy INPUT_PULLUP, so the
// released level is HIGH and a pressed switch connects the pin to GND.
constexpr bool PHYSICAL_CONTROL_BUTTONS_ENABLED = true; // VI: Bật hai nút vật lý START và đổi MAP.
constexpr uint8_t PHYSICAL_START_BUTTON_PIN = 34; // VI: Chân GPIO dùng cho vật lý xuất phát nút.
constexpr uint8_t PHYSICAL_FIELD_BUTTON_PIN = 35; // VI: Chân GPIO dùng cho vật lý MAP nút.
constexpr uint32_t PHYSICAL_BUTTON_DEBOUNCE_MS = 35; // VI: Thời gian vật lý nút DEBOUNCE, đơn vị ms.
constexpr uint32_t PHYSICAL_FIELD_DOUBLE_CLICK_MS = 450; // VI: Thời gian vật lý MAP nhấn đôi nhấn, đơn vị ms.

// AO3400A is a low-side switch: HIGH on IN_BUZZER enables the buzzer.
constexpr uint8_t BUZZER_PIN = 40; // VI: Chân GPIO dùng cho còi.
constexpr bool BUZZER_ACTIVE_HIGH = true; // VI: Còi hoạt động khi chân điều khiển ở mức HIGH.
constexpr uint16_t BUZZER_BEEP_ON_MS = 110; // VI: Thời gian còi tiếng còi bật, đơn vị ms.
constexpr uint16_t BUZZER_BEEP_OFF_MS = 100; // VI: Thời gian còi tiếng còi tắt, đơn vị ms.
constexpr uint16_t BUZZER_PATTERN_GAP_MS = 180; // VI: Thời gian còi mẫu khoảng nghỉ, đơn vị ms.
// Short double tone used only for command replies from the mechanism ESP32.
constexpr uint16_t BUZZER_UART_FAST_ON_MS = 55; // VI: Thời gian còi UART nhanh bật, đơn vị ms.
constexpr uint16_t BUZZER_UART_FAST_OFF_MS = 45; // VI: Thời gian còi UART nhanh tắt, đơn vị ms.
constexpr uint16_t BUZZER_UART_FAST_GAP_MS = 100; // VI: Thời gian còi UART nhanh khoảng nghỉ, đơn vị ms.
constexpr uint16_t BUZZER_ALARM_TOGGLE_MS = 120; // VI: Thời gian còi cảnh báo đảo trạng thái, đơn vị ms.
constexpr uint32_t BUZZER_ALIGNMENT_ALARM_MS = 2000; // VI: Thời gian còi căn chỉnh cảnh báo, đơn vị ms.

static_assert(PHYSICAL_START_BUTTON_PIN != PHYSICAL_FIELD_BUTTON_PIN &&
              PHYSICAL_START_BUTTON_PIN != BUZZER_PIN &&
              PHYSICAL_FIELD_BUTTON_PIN != BUZZER_PIN,
              "Physical control pins must be unique");

// ---------------- Mission constants copied from V10 ----------------
constexpr float SENSOR_TO_CENTER_MM = 20.0f; // VI: Khoảng cách từ cảm biến đến tâm robot, đơn vị mm.
constexpr float START_APPROACH_MM = 125.0f; // VI: Khoảng cách/kích thước xuất phát tiếp cận, đơn vị mm.
constexpr float C_OFFSET_MM = 150.0f; // VI: Khoảng cách/kích thước C OFFSET, đơn vị mm.
constexpr float HOME_TO_B_MM = 410.0f; // VI: Khoảng cách/kích thước HOME TO B, đơn vị mm.
constexpr float RETURN_A_MM = 330.0f; // VI: Khoảng cách/kích thước quay về A, đơn vị mm.
constexpr float RETURN_LONG_MM = 2650.0f; // VI: Khoảng cách/kích thước quay về dọc dài, đơn vị mm.
constexpr float RETURN_LATERAL_MM = 1650.0f; // VI: Khoảng cách/kích thước quay về chạy ngang, đơn vị mm.
constexpr float MOVE_SPEED_MM_S = 450.0f; // VI: Tốc độ di chuyển tốc độ, đơn vị mm/s.
// Commissioning limits: do not jump from the 180 mm/s acquisition speed to
// 700 mm/s. Ramp up only after the center line is confirmed.
constexpr float BRIDGE_SPEED_FAST_MM_S = 1150.0f; // VI: Tốc độ cầu tốc độ nhanh, đơn vị mm/s.
constexpr float BRIDGE_SPEED_SLOW_MM_S = 1000.0f; // VI: Tốc độ cầu tốc độ SLOW, đơn vị mm/s.
constexpr float BRIDGE_ACCEL_MM_S2 = 1800.0f; // VI: Gia tốc cầu tăng tốc, đơn vị mm/s².
constexpr float BRIDGE_DECEL_MM_S2 = 700.0f; // VI: Gia tốc cầu giảm tốc, đơn vị mm/s².

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

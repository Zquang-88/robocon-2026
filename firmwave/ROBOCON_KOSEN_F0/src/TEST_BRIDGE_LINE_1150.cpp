// Firmware thử riêng bám line 8 mắt khi đi lên cầu ở 1150 mm/s.
//
// Build/upload:
//   platformio run -e bridge_line_1150_test -t upload
//
// Cách đặt robot:
//   - Đặt robot tại chân cầu, hàng 8 mắt giữa đang thấy line dọc.
//   - Chuẩn bị E-STOP và vùng cầu an toàn.
//   - Nhấn START vật lý chân 34, hoặc gửi MODE,AUTO rồi START.
//
// Firmware chỉ chạy chassis, không gửi nhiệm vụ cơ cấu. COM5 (57600 baud)
// xuất NORM của CH0..CH7, EYE_DROP/EYE_FOUND, vị trí line, BNO và fault.
// Robot tự dừng khi BNO nhận diện tới đỉnh cầu hoặc khi có lỗi an toàn.

#define ROBOCON_BRIDGE_LINE_1150_TEST_BUILD 1
#define ROBOCON_BRIDGE_LINE_TEST_SPEED_MM_S 1150.0f
#define ROBOCON_AFTER_BRIDGE_TEST_CHASSIS_ONLY 1
#include "main.cpp"

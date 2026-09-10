# ROBOCON_KOSEN_F0 — Teensy 4.1 Mecanum

Firmware thi đấu Mecanum được nâng cấp trực tiếp từ trình tự nhiệm vụ của
`V10_SAN_DO_V2.cpp`. Chương trình dùng PID bánh xe 100 Hz, BNO085, hai MCP3008,
optical flow MAVLink, VL53L1X, UART ESP32 và parser Serial không chặn.

## Trạng thái an toàn ban đầu

Firmware biên dịch được nhưng **không cho motor chạy ngay sau khi nạp**.
Trong `include/RobotConfig.h`, `HARDWARE_CONFIG_VERIFIED` đang là `false`, kích
thước robot, số count encoder và phía sân chưa được đặt. Đây là khóa có chủ ý để
robot không chạy sai chiều do thông số chưa đo.

Chỉ đặt `HARDWARE_CONFIG_VERIFIED = true` sau khi hoàn thành toàn bộ mục hiệu
chỉnh bên dưới. Khi STOP pin 31 đang LOW hoặc còn fault, lệnh chạy bị từ chối.

## File chính

- `include/RobotConfig.h`: mọi chân, dấu chiều, kích thước, calibration và PID.
- `src/main.cpp`: firmware thi đấu hoàn chỉnh.
- `src/Read_Optical_Flow.cpp`: firmware chẩn đoán optical flow riêng.
- `platformio.ini`: môi trường `robot` và `optical_flow`; mặc định là `robot`.

## Bảng chân kết nối

| Chức năng | Chân Teensy 4.1 |
|---|---:|
| FL RPWM / LPWM | 36 / 37 |
| FR RPWM / LPWM | 22 / 23 |
| RL RPWM / LPWM | 18 / 19 |
| RR RPWM / LPWM | 14 / 15 |
| Encoder FL A / B | 0 / 1 |
| Encoder FR A / B | 6 / 7 |
| Encoder RL A / B | 2 / 3 |
| Encoder RR A / B | 4 / 5 |
| MCP3008 center CS | 26 |
| MCP3008 stop CS | 8 |
| SPI MOSI / MISO / SCK | 11 / 12 / 13 |
| BNO085 + VL53L1X Wire1 SDA / SCL | 17 / 16 |
| Optical flow UART | Tạm tắt; Serial1 chân 0/1 trùng encoder FL |
| Telemetry Serial5 RX / TX | RX21 / TX20; CP210x phía PC là COM5, 57600 baud |
| ESP32 UART | Tạm tắt; chưa chốt UART và chân riêng không xung đột |
| START / STOP vật lý | Không dùng; chân 32 / 31 để trống |

Hai MCP3008 dùng chung SPI0 MOSI/MISO/SCK và khác CS. MCP center đọc CH0–CH7;
do hai cáp cụm stop được đấu chéo, MCP stop đọc CH3–CH5 cho bên trái và CH0–CH2
cho bên phải. Firmware đổi chúng về thứ tự logic trái trước, phải sau. Tất cả thiết
bị phải chung GND;
Teensy dùng mức logic 3.3 V. Optical flow và VL53L1X đang tắt. Lệnh điều khiển
và telemetry RBT/1 đi qua Serial5 tới COM5; USB native COM14 không phải kênh điều
khiển trong bản này. Chân 31/32 hiện không dùng.

## Thư viện PlatformIO/Arduino

`platformio.ini` đã khai báo:

- Paul Stoffregen `Encoder`
- Adafruit `Adafruit BNO08x`
- Pololu `VL53L1X`
- `SPI` và `Wire` có sẵn trong Teensy framework

## Thông số bắt buộc phải đo

Sửa tập trung trong `include/RobotConfig.h`:

1. `ROBOT_LENGTH_MM`: khoảng cách tâm bánh trước–sau.
2. `ROBOT_WIDTH_MM`: khoảng cách tâm bánh trái–phải.
3. `ENCODER_COUNTS_PER_WHEEL_REV`: số count AB thực tế trên một vòng bánh sau hộp số.
4. `MOTOR_SIGN_*` và `ENCODER_SIGN_*` cho bốn bánh.
5. Chọn sân trên giao diện trước khi START AUTO: sân đỏ bắt đầu đi trái,
   sân xanh bắt đầu đi phải. Firmware tự đảo hướng và chọn cụm 3 mắt tương ứng.
6. Xác nhận bố trí con lăn chữ X và `BNO_YAW_SIGN`.
7. UART/baud, đổi trục, dấu trục và scale của optical flow.
8. `CENTER_SENSOR_MIN/MAX`, `STOP_SENSOR_MIN/MAX`, cực tính line/nền và thứ tự sáu mắt dừng.
9. `TOF_TARGET_MM` và giới hạn hợp lệ VL53L1X.
10. Feedforward, dead-zone và PID riêng của từng bánh sau khi test cơ khí.

`MM_PER_COUNT` được tính từ đường kính 100 mm, count/vòng và
`DISTANCE_CALIBRATION`; không còn dùng hằng số cũ `0.73`.

## Build

```text
pio run -e robot
pio run -e optical_flow
```

Chọn environment `robot` để nạp firmware thi đấu. Chỉ nạp `optical_flow` khi
muốn chạy chương trình chẩn đoán MAVLink độc lập.

## Lệnh telemetry COM5, Serial5, 57600 baud

```text
START                 hoặc RUN / AUTO
STOP                  hoặc ESTOP
RESET
CAL_LINE
TEST_MOTOR
TEST_ENCODER
TEST_BNO
TEST_LINE_CENTER
TEST_LINE_STOP
TEST_FLOW
TEST_TOF
STATUS
DRIVE,vx_mm_s,vy_mm_s,wz_rad_s
MOVE,dx_mm,dy_mm,max_speed_mm_s
POSE_RESET
YAW,degrees
SET_PID,wheel(0-3),kp,ki,kd,kff,deadzone
PID,WHEEL,FL,0.4,0.3,0.002
PID,WHEEL,ALL,0.4,0.3,0.002
PID,HEADING,0,6.0,0.1,0.08
PID,LINE,0,0.16,0.02,0.003
PING
GET_CONFIG
TUNE,WHEEL,FL,120
TUNE,STOP
SAVE_CONFIG
PROFILE_LOAD
FACTORY_RESET
```

`DRIVE` phải được gửi lại trong tối đa 500 ms; mất lệnh sẽ dừng và disarm.
Manual `DRIVE` được phép hoạt động khi BNO085 timeout vì Vx/Vy/Wz được điều khiển
trực tiếp; chỉ riêng `FAULT_IMU_TIMEOUT` được bỏ qua trong chế độ này. Mọi fault
khác vẫn khóa motor. `MOVE`, heading hold và state machine vẫn bắt buộc BNO085.
Khi BNO085 hợp lệ, manual Vx/Vy với Wz bằng 0 dùng PID heading để giữ yaw; mức
bù quay manual được giới hạn bởi `MAX_MANUAL_HEADING_WZ_RAD_S`.
`MOVE` điều khiển vị trí tương đối theo X tiến/lùi và Y phải/trái, dùng odometry,
PID vị trí và BNO085 giữ yaw. Mất heartbeat giao diện quá 600 ms cũng dừng MOVE.
Các alias `RUN`, `ESTOP`, `PID,...` và dòng `TEL,...` tương thích giao diện
ASCII RBT/1 trong thư mục `pid-tuner`. Telemetry đầy đủ có tiền tố `DBG` ở 10 Hz.

Trong lúc AUTO đang ARM, firmware không phát khối telemetry định kỳ. Chọn
`AUTO_TELEM,ON_REQUEST` để chỉ đọc một lần bằng `AUTO_TELEM,SNAPSHOT`, hoặc
`AUTO_TELEM,SILENT` để không trả snapshot. Snapshot theo yêu cầu được chia thành
`AUTO_SNAPSHOT`, `AUTO_WHEELS`, `AUTO_SENSORS` và
`AUTO_TELEM,SNAPSHOT_END`, mỗi vòng lặp chỉ phát một khung ngắn.

Các sự kiện `STATE`, `FAULT`, `SAFETY,FAULT`, `SAFETY,ESTOP`,
`SAFETY,STOP_INPUT` và `SAFETY,AUTO_COMPLETE` luôn được gửi trong cả hai chế độ;
giao diện không thể tắt các bản tin an toàn này.

Giao diện Web Serial gửi heartbeat mỗi 100 ms. Chế độ `TUNE,WHEEL` chỉ cho
phép một bánh chạy; nếu mất heartbeat quá 600 ms, firmware đặt bốn target về 0,
xóa PID và disarm. PID, feedforward, deadzone và các giới hạn mặc định nằm trong
`include/RobotConfig.h`; firmware không đọc hoặc ghi EEPROM. Tune từ giao diện chỉ
thay đổi RAM cho tới khi reset. `SAVE_CONFIG` xác nhận RAM-only và `PROFILE_LOAD`
nạp lại các giá trị đã biên dịch từ `RobotConfig.h`. Hãy mở `pid-tuner`, chọn cổng
CP210x COM5 ở 57600 baud, chọn một
bánh và bắt đầu ở RPM thấp khi robot đang được kê khỏi mặt đất.

Telemetry dùng Serial5 RX21/TX20. ESP32 dùng UART6 riêng: Teensy TX24 → ESP32
RX16 và ESP32 TX15 → Teensy RX25, kèm GND chung.

## Trình tự test bắt buộc

1. Kê bốn bánh khỏi mặt đất, tháo cơ cấu tải và giữ STOP ở trạng thái có thể bấm ngay.
2. Để khóa motion, chạy `TEST_BNO`, `TEST_LINE_CENTER`, `TEST_LINE_STOP`,
   `TEST_FLOW`, `TEST_TOF`; xem `DBG` và sửa chân/cực tính/calibration.
3. Gửi `CAL_LINE`, đưa từng mắt qua cả nền và line, rồi gửi `CAL_LINE` lần nữa.
   Chép bốn mảng min/max được in ra vào `RobotConfig.h`.
4. Đo kích thước, encoder count/vòng, chọn phía sân; đặt
   `HARDWARE_CONFIG_VERIFIED = true`.
5. Kê bánh, chạy `TEST_MOTOR`. Bánh 0/1/2/3 lần lượt là FL/FR/RL/RR, mỗi bánh 2 s.
6. Khi target dương, mỗi bánh phải tạo lực đẩy robot về trước. Nếu sai, đổi
   `MOTOR_SIGN_*`, không đảo dây ngẫu nhiên trong lúc cấp nguồn.
7. Quan sát `ENC`. Khi bánh quay theo chiều dương, count phải tăng; nếu giảm, đổi
   `ENCODER_SIGN_*`. Đo đủ một vòng để xác nhận count/vòng.
8. Dùng `DRIVE,100,0,0`: bốn bánh phải làm robot tiến.
9. Dùng `DRIVE,0,100,0`: robot phải sang phải. Nếu cả hướng bị đối xứng, kiểm tra
   vị trí FL/FR/RL/RR và bố trí con lăn trước khi sửa công thức.
10. Dùng `DRIVE,0,0,0.2`: robot phải quay theo chiều kim đồng hồ. Đồng thời yaw
    BNO phải tăng; nếu không, đổi `BNO_YAW_SIGN`.
11. Đẩy robot bằng tay: chuyển động tiến phải cho flow X dương, sang phải cho
    flow Y dương. Sửa `FLOW_*_FROM_SENSOR_AXIS` và `FLOW_*_SIGN`, rồi đo scale.
12. Tune feedforward/dead-zone trước, sau đó P, I, D từng bánh; cuối cùng mới tune
    heading, line, stop-line và position.
13. Test riêng từng state ở tốc độ thấp, luôn kiểm timeout/fault, rồi mới chạy toàn tuyến.

## Tuyến AUTO commissioning đến cầu

Tuyến hiện tại dừng có chủ ý sau giai đoạn 8 mắt giữa bám line lên cầu. Sau khi
căn ToF và hoàn thành các điểm A/B bằng hai cụm 3 mắt, robot tìm line cầu, yêu
cầu line hợp lệ liên tục, sau đó bám line bằng PID và BNO085. Vạch ngang được
đếm có debounce; đủ `BRIDGE_STOP_CROSS_TARGET` thì firmware vào
`BRIDGE_ENTRY_STOP`, đặt bốn target/PWM về 0 và DISARM. Các nhiệm vụ sau cầu
không được chạy trong bản commissioning này.

## Giới hạn chính đã sửa từ V10

Code V10 dùng động học Omni, `mmPerCount = 0.73`, BNO055, hai line digital và
đọc ESP bằng `String/readStringUntil()`. Heading integral không nhân `dt`; có
`delay()`/`while(1)` chặn, đọc ToF thiếu kiểm soát timeout theo state, và nhiều
lệnh khởi tạo chuyển động nằm trực tiếp trong vòng state. Những điểm này đã được
thay bằng Mecanum X, PID có `dt`/anti-windup, parser không chặn, cảm biến có
validity/timeout, state `onEnter` và fault latch. Thứ tự nhiệm vụ V10 và các lệnh
ESP32 `POINT_A`, `POINT_B`, `THA_2B`, `THA_2A`/phản hồi `DONE` vẫn được giữ.

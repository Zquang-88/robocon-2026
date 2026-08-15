# Kiểm kê hệ thống hiện tại

## Firmware đang hoạt động

Dự án hiện hành: `firmwave/ROBOCON_KOSEN_F0` cho Teensy 4.1.

Các module logic đang có trong `src/main.cpp`:

| Nhóm | Hiện trạng |
|---|---|
| Motor | 4 motor FL/FR/RL/RR, encoder riêng, mixer mecanum |
| PID motor | PID riêng từng bánh, 100 Hz; chưa có open-loop mode rõ ràng |
| Heading | BNO055 + PID heading |
| Line | 1 cụm 8 mắt MCP3008 + 1 PID dùng chung |
| ToF | 1 VL53L1X, mới đọc sensor; chưa có PID khoảng cách |
| Battery | ADC điện áp, chưa có dòng điện và fault threshold hoàn chỉnh |
| Safety | E-stop vật lý, khóa motor khi boot, timeout DRIVE 500 ms |
| Auto | State machine kiểm tra 3 bước, chưa phải kịch bản thi đấu chính thức |
| ESP32 | Serial4, gửi chuỗi `POINT_A`, chờ chuỗi chứa `DONE` |
| PC telemetry | Serial5, RBT/1 ASCII, không CRC/sequence/heartbeat/config sync |
| Config | Gain khởi tạo trong code, chưa lưu bền vững |
| Logging | Chưa có logger/ring buffer/SD logging |

## Dashboard hiện tại

`pid-tuner` là dashboard React/Web Serial. Hiện có:

- Kết nối cổng COM bằng Web Serial.
- PID riêng từng bánh, PID heading và một PID line.
- Hiển thị RPM, heading, line error, điện áp.
- Gửi `RUN`, `ESTOP`, `PID`, `YAW` theo RBT/1.

Thiếu: handshake, heartbeat 10 Hz, config sync, packet CRC/sequence, reconnect state machine,
fault model, module tabs đầy đủ, ring buffer graph và CSV logger.

## Rủi ro cần giữ nguyên trong lúc refactor

1. Không thay pin, chiều motor/encoder, CPR hoặc kinematics nếu chưa test trên giá kê.
2. Không xóa RBT/1 cho đến khi RBT/2 đã chạy ổn định trên PC và Teensy.
3. Không chuyển PID ra GUI; tất cả control loop vẫn nằm trên Teensy.
4. Không ghi flash/EEPROM mỗi lần kéo slider.
5. Mọi actuator mới mặc định disabled/disarmed/off.

## Phần cứng chưa đủ thông tin để hoàn thiện

- Địa chỉ/XSHUT của 4 VL53L1X.
- Model và protocol của 4 optical-flow sensor.
- Chân, feedback và giới hạn của 4 servo.
- BLDC dùng PWM, DShot hay protocol khác.
- Mapping 8 van và ma trận interlock.
- Sensor dòng điện và ngưỡng pin thấp.
- Command thật của ESP32 cơ cấu.


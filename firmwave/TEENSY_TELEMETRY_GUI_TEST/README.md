# Teensy 4.1 — Telemetry GUI Test

Chương trình độc lập để kiểm tra truyền/nhận giữa dashboard ROBOCON Control Lab,
hai module radio telemetry kiểu SiK/3DR và Teensy 4.1. Chương trình không điều
khiển motor thật; nó mô phỏng RPM, yaw, line error và pin để kiểm tra giao diện
an toàn trên bàn.

## Đấu dây

| Telemetry phía robot | Teensy 4.1 |
|---|---|
| TX | RX8, chân 34 |
| RX | TX8, chân 35 |
| GND | GND |

Không cấp tín hiệu UART 5 V vào Teensy 4.1. Hai radio và Teensy dùng baud
`57600`, định dạng `8N1`.

## Nạp bằng PlatformIO

Mở thư mục `TEENSY_TELEMETRY_GUI_TEST`, cắm Teensy bằng USB rồi chạy:

```powershell
platformio run --target upload
```

USB Serial Monitor dùng `115200` baud và chỉ in lệnh nhận được để debug. Radio
telemetry dùng `Serial8` ở `57600` baud.

## Kiểm tra với dashboard

1. Cắm telemetry mặt đất vào PC.
2. Mở dashboard bằng Chrome hoặc Edge.
3. Chọn `57 600 baud` và nhấn **KẾT NỐI RADIO**.
4. Chọn đúng cổng COM của telemetry.
5. Chờ `TELEMETRY ONLINE` và `CONFIG SYNCED`.
6. Thử thay PID riêng cho FL/FR/BL/BR, gửi góc heading hoặc lệnh chạy.
7. Mở USB Serial Monitor để xem Teensy in các dòng `RX: ...`.

Dashboard sẽ tự gửi `PING`, `GET_CONFIG` và `HEARTBEAT`. Teensy trả dữ liệu
`TEL` ở 20 Hz. Nút **WRITE TO ROBOT** lưu PID vào EEPROM; rút nguồn, cắm lại và
kết nối lại để kiểm tra cấu hình vẫn được đồng bộ lên giao diện.

Sau khi nhấn E-STOP, gửi `RUN` hoặc `CLEAR_ESTOP` từ terminal UART, hoặc reset
Teensy để chạy lại phần mô phỏng.

## Lệnh hỗ trợ

```text
PING
HEARTBEAT
GET_CONFIG
PID,WHEEL,FL,0.82,0.031,0.012
PID,HEADING,0,3.2,0.018,0.24
PID,LINE,0,1.45,0.012,0.18
YAW,90
DRIVE,FORWARD,1200
DRIVE,0.5,0.0,0.0
ESTOP
RUN
SAVE_CONFIG
FACTORY_RESET
```

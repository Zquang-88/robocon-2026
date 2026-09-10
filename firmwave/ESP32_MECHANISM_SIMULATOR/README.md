# ESP32 mechanism simulator

Firmware này giả lập cơ cấu chưa hoàn thiện để Teensy 4.1 có thể tiếp tục chạy
state machine tự động. ESP32 nhận một lệnh cơ cấu hợp lệ, chờ không chặn 4 giây,
sau đó gửi đúng một dòng `DONE` về robot.

## Giao thức khớp với firmware chính

| Teensy gửi | ESP32 trả sau 4 giây |
|---|---|
| `POINT_A` | `DONE` |
| `POINT_B` | `DONE` |
| `THA_2B` | `DONE` |
| `THA_2A` | `DONE` |
| `DONE_THA_2A` | `DONE_THA2A` |
| `DONE_THA_2B` | `DONE_THA2B` |

- UART giữa hai board: `57600`, `8N1`.
- Mỗi bản tin kết thúc bằng CR/LF; `println()` ở hai phía đáp ứng yêu cầu này.
- Chuỗi `DONE` phải viết hoa vì Teensy so sánh chính xác, phân biệt hoa/thường.
- Lệnh không thuộc bảng trên sẽ bị bỏ qua và chỉ được báo trên USB Serial của
  ESP32. Lệnh lặp trong lúc đang chờ cũng bị bỏ qua để không khởi động lại bộ đếm.

## Đấu dây

| ESP32 DevKit V1 | Teensy 4.1 |
|---|---|
| GPIO16 / RX2 | pin 24 / TX6 |
| GPIO15 / TX2 | pin 25 / RX6 |
| GND | GND |

Hai board đều dùng logic 3.3 V. Phải nối chung GND và nối chéo TX-RX. Không nối
chân 5 V vào đường RX/TX.

## Nạp bằng PlatformIO

Mở thư mục này bằng PlatformIO rồi chạy:

```powershell
platformio run
platformio run --target upload
platformio device monitor
```

USB monitor chạy ở 115200 baud và hiển thị lệnh đã nhận, thời điểm bắt đầu mô
phỏng và lúc gửi `DONE`. Kênh USB debug độc lập với UART2 nối sang Teensy.

Firmware chính đã đặt `ESP32_REPLY_TIMEOUT_MS` thành 15 giây để đủ thời gian chờ
4 giây và có thêm biên an toàn cho scheduler/UART.

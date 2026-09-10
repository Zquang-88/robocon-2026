# ESP32-S3 mechanism controller

Firmware điều khiển hai động cơ bước, hai cảm biến E18 và bốn van điện từ hai đầu qua PCF8574. ESP32 nhận lệnh từ Teensy 4.1 bằng UART2; USB Serial 115200 dùng chung bộ parser để thử và chẩn đoán.

## Kiến trúc

```text
Teensy Serial6 (TX24, RX25)
  <-> 57600 baud
ESP32-S3 UART2 (RX16, TX15)
  |- UartProtocol: buffer char cố định, lọc CR/LF/khoảng trắng/byte rác đầu dòng
  |- MechanismController: state machine động cơ, HOME, E18 và chuỗi van
  |- ValveController: PCF8574 active-low, interlock cặp và deadtime 50 ms
  |- AccelStepper.run(): phát xung động cơ không chặn
  `- PersistentConfig: cấu hình profile trong NVS có CRC
```

Chuỗi van dùng `millis()`, không dùng `delay(700)`. Trong thời gian chờ, vòng lặp vẫn đọc UART, chạy động cơ bước, cập nhật E18 và nhận `STOP`.

## Chân phần cứng

| Chức năng | ESP32-S3 | Nối với |
| --- | ---: | --- |
| UART2 RX | GPIO16 | Teensy TX6 chân 24 |
| UART2 TX | GPIO15 | Teensy RX6 chân 25 |
| I2C SDA | GPIO8 | PCF8574 SDA |
| I2C SCL | GPIO3 | PCF8574 SCL |
| PCF8574 | `0x20` | P0-P7, active LOW |
| Step/Dir A | GPIO1 / GPIO2 | Driver stepper A |
| Step/Dir B | GPIO45 / GPIO48 | Driver stepper B |
| HOME A/B | GPIO36 / GPIO37 | Công tắc active LOW |
| E18 A/B | GPIO11 / GPIO18 | Cảm biến active LOW |

Firmware chỉ chấp nhận PCF8574 đúng địa chỉ `0x20`. Khi khởi động, `pcfState = 0xFF`, tức P0-P7 đều HIGH và mọi cuộn van đều mất điện. Nếu không tìm thấy PCF8574, cơ cấu không bắt đầu HOME/AUTO và UART báo `ERROR_PCF8574`.

## Ghép bốn van hai đầu

| Van | Đầu thứ nhất | Đầu đối diện |
| --- | --- | --- |
| 1 | P0 | P7 |
| 2 | P1 | P6 |
| 3 | P2 | P5 |
| 4 | P3 | P4 |

Mức LOW bật cuộn; mức HIGH tắt cuộn. Trước mỗi lần đổi đầu kích, `ValveController` đưa cả hai đầu của cặp lên HIGH, chờ 50 ms, rồi mới kéo đầu mới xuống LOW. Mọi byte ghi ra PCF đều được kiểm tra để không có hai đầu cùng một cặp đồng thời ở LOW.

Các hàm chính trong `ValveController`:

- `writePCF()`: ghi toàn bộ `pcfState` và kiểm tra giao tiếp.
- `setOutput(pin, enabled)` và `switchValveOutput(pin, enabled)`: chọn một đầu của đúng cặp; `false` chọn đầu đối diện.
- `allValveCoilsOff()`: hủy chuyển mạch đang chờ và ghi `0xFF` ngay.
- `update()`: hoàn tất pha make sau deadtime 50 ms mà không chặn vòng lặp.

## Chuỗi AUTO của van

| Lệnh | Bước 1 | Chờ | Bước 2 | Chờ | Phản hồi |
| --- | --- | ---: | --- | ---: | --- |
| `POINT_A` | P2 ON, P5 OFF | 700 ms | P3 ON, P4 OFF | 700 ms | `DONE_POINT_A` |
| `POINT_B` | P0 ON, P7 OFF | 700 ms | P1 ON, P6 OFF | 700 ms | `DONE_POINT_B` |
| `THA_2A` | P1 OFF, P6 ON | 700 ms | P3 OFF, P4 ON | 700 ms | `DONE_THA_2A` |
| `THA_2B` | P2 OFF, P5 ON | 350 ms | P0 OFF, P7 ON | 700 ms | `DONE_THA_2B` |

`MechanismController::updateValveSequence()` thực hiện lần lượt các trạng thái switch, chờ deadtime, chờ 700 ms, switch đầu thứ hai và hoàn tất. Lệnh chuyển động khác trong lúc cơ cấu bận nhận phản hồi `BUSY`. `STOP` luôn hủy chuỗi, dừng stepper, ghi `0xFF` và phản hồi `STOPPED`.

Sau chuỗi van `THA_2B`, ESP32 trả `DONE_THA_2B` ngay để Teensy bắt đầu chạy robot. ESP32 cho riêng động cơ B hạ trước 400 mm, sau đó A và B cùng chạy với tốc độ 8000 step/s và gia tốc 5000 step/s² cho tới khi HOME37 ổn định 25 ms. Lúc đó B được đặt zero và riêng A đổi chiều kéo về HOME36. Nếu HOME37 tác động sớm trong 400 mm đầu, firmware dừng B ngay và bỏ qua pha hạ đồng bộ để bảo vệ cơ cấu. Toàn bộ quá trình không chặn UART/E18/STOP và có timeout 45 giây.

Phía Teensy, sau `DONE_THA_2B`, robot giữ heading và thực hiện một vector chéo duy nhất: lùi 3000 mm (`-X`) đồng thời đi phải 2000 mm (`-Y`), giới hạn tốc độ tổng 500 mm/s, rồi chuyển sang `FINISH`.

Log kiểm tra ví dụ:

```text
[POINT_A] P2 ON, P5 OFF
[POINT_A] Wait 700 ms
[POINT_A] P3 ON, P4 OFF
[POINT_A] Wait 700 ms
[POINT_A] DONE
DONE_POINT_A
```

## Lệnh chẩn đoán PCF8574

| Lệnh | Chức năng |
| --- | --- |
| `PCF,P0,ON` ... `PCF,P7,ON` | Chọn đầu được chỉ định, tự tắt đầu đối diện |
| `PCF,P0,OFF` ... `PCF,P7,OFF` | Tắt đầu chỉ định và chọn đầu đối diện |
| `PCF,ALL,OFF` | Tắt điện toàn bộ P0-P7 (`0xFF`) |
| `PCF,STATUS` | Đọc status hiện tại |
| `STOP` | Dừng ưu tiên cao, safe-off toàn bộ van |

Chân ngoài P0-P7 bị từ chối. Nếu ghi I2C thất bại, chuỗi bị hủy, cơ cấu vào fault và UART phát `ERROR_PCF8574`.

## Build và kiểm thử

```powershell
C:\Users\Acer\.platformio\penv\Scripts\platformio.exe run -e esp32-s3-devkitm-1
.\test\run_jog_host.ps1
```

Bộ test host kiểm tra địa chỉ/chân I2C, trạng thái khởi động `0xFF`, active-low, bốn cặp interlock, deadtime 50 ms, thứ tự cả bốn chuỗi, `BUSY`, `STOP` và lỗi ghi PCF8574.
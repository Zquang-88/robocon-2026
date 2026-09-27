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
| E18 A/B | GPIO17 / GPIO18 | Cảm biến active LOW |
| Nút van | GPIO11 | Nút thường hở nối GND, INPUT_PULLUP |
| Nút HOME B rồi A | GPIO12 | Nút thường hở nối GND, INPUT_PULLUP |

Firmware chỉ chấp nhận PCF8574 đúng địa chỉ `0x20`. Khi khởi động, `pcfState = 0xFF`, tức P0-P7 đều HIGH và mọi cuộn van đều mất điện. Nếu không tìm thấy PCF8574, cơ cấu không bắt đầu HOME/AUTO và UART báo `ERROR_PCF8574`.

## Hai nút điều khiển tại ESP32

Các nút vẫn hoạt động khi USB/telemetry không kết nối. Một đầu nút nối GPIO,
đầu còn lại nối GND: bình thường HIGH, nhấn LOW. Chống dội 30 ms; nút bị giữ
khi bật nguồn hoặc lúc STOP phải được nhả ra rồi nhấn lại mới có lệnh mới.

- **GPIO11 nhấn một lần:** đóng cả bốn van bằng cách bật P4/P5/P6/P7 và tắt
  P0/P1/P2/P3 (`enabledMask = 0xF0`, trạng thái PCF sau chuyển mạch là `0x0F`).
  Firmware vẫn mở cửa sổ 400 ms để nhận lần nhấn thứ hai. Nếu cơ cấu đang chạy,
  dừng cả chu trình và motor để bước kế tiếp không ghi đè trạng thái đóng.
  UART/USB báo `ACK,BUTTON11,P4_P7_ON_CLOSED`.
- **GPIO11 nhấn nhanh hai lần trong 400 ms:** mở cả bốn van bằng cách bật
  P0/P1/P2/P3 và tắt P4/P5/P6/P7
  trong cùng một mask (`pcfState = 0xF0`), sau deadtime 50 ms. Khi cơ cấu bận
  hoặc có fault, từ chối bật van. UART/USB báo
  `ACK,BUTTON11,P0_P3_ON_REQUESTED` khi nhận yêu cầu chuyển van.
- **GPIO12 nhấn một lần:** khi cơ cấu rảnh và không có fault, giữ A đứng yên,
  cho B chạy về HOME37. B chạm công tắc thì dừng xung ngay, xác nhận ổn định
  25 ms rồi đặt B=0; sau đó mới kéo A về HOME36 và đặt A=0. Cả hai trục dùng
  8000 step/s, gia tốc 5500 step/s²; timeout toàn chu trình 45 giây. Trục đã
  chạm HOME không chạy thêm. Báo `ACK,BUTTON12,HOME_B_THEN_A,8000,5500` khi
  bắt đầu và `HOME,STATUS,...` khi kết thúc. Nhấn khi bận bị từ chối, không xếp
  hàng để tự chạy sau. Lệnh UART `STOP` vẫn được xử lý trong khi HOME.

Chỉnh chân nút, thời gian nhấn đôi và tốc độ HOME ở `include/HardwareConfig.h`:
`VALVE_BUTTON_PIN`, `HOME_BUTTON_PIN`, `BUTTON_DOUBLE_CLICK_MS`,
`BUTTON_HOME_SPEED_STEPS_S`, `BUTTON_HOME_ACCEL_STEPS_S2`, `BUTTON_HOME_TIMEOUT_MS`.
HOME khi bật nguồn và lệnh UART HOME vẫn dùng cấu hình HOME thông thường;
chuỗi B trước A tốc độ 8000/5500 áp dụng riêng cho GPIO12.

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

Tại cả hai MAP, `POINT_A` hạ sâu hơn `POINT_B`: A/B hạ `610/545 mm` tại
`POINT_A` và `520/480 mm` tại `POINT_B`, tính từ `READY_HOME_A`. MAP đỏ và MAP
xanh có bộ hằng số riêng trong `include/HardwareConfig.h`: tiền tố
`RED_POINT_*` và `BLUE_POINT_*`, nên có thể hiệu chỉnh độc lập về sau.

| Lệnh | Bước 1 | Chờ | Bước 2 | Chờ | Phản hồi |
| --- | --- | ---: | --- | ---: | --- |
| `POINT_A` | P2 và P3 ON đồng thời trong một byte PCF | 50 ms deadtime an toàn | Nâng A/B về HOME_A ngay | - | `DONE_POINT_A` |
| `POINT_B` | P0 và P1 ON đồng thời trong một byte PCF | 50 ms deadtime an toàn | A/B cùng về READY_HOME_A; A nâng 1700 mm, B nâng 850 mm rồi giữ | - | `DONE_POINT_B` khi pha nâng tầng 1 bắt đầu |
| `BRIDGE_STABLE` | Không đổi van | - | A giữ ở 1700 mm, chỉ B nâng nốt 850 mm | - | `DONE_BRIDGE_B` |
| `THA_2A` | P1 OFF, P6 ON | 700 ms | P3 OFF, P4 ON | 700 ms | `DONE_THA_2A` |
| `THA_2B` MAP đỏ | P2 OFF, P5 ON | 350 ms | P0 OFF, P7 ON | 700 ms | `DONE_THA_2B` |
| `THA_2B` MAP xanh | P0 OFF, P7 ON | 350 ms | P2 OFF, P5 ON | 700 ms | `DONE_THA_2B` |

Tại `POINT_A` và `POINT_B`, hai cuộn yêu cầu được chuyển bằng một mask PCF8574 nên cùng kích sau deadtime an toàn 50 ms; không còn khoảng chờ 700 ms giữa hai van. Tại `POINT_B`, A và B trước tiên cùng nâng từ vị trí đang hạ về đúng `READY_HOME_A`. Sau khi cả hai tới READY, A/B cùng tốc độ nâng 850 mm; B dừng giữ, còn A nâng tiếp 850 mm để đạt tổng 1700 mm. ESP32 gửi `DONE_POINT_B` ngay khi pha 850 mm đầu bắt đầu để Teensy chạy qua line lên cầu song song. Khi BNO085 trên Teensy xác nhận robot đã xuống hết dốc và mặt phẳng ổn định 500 ms, Teensy gửi `BRIDGE_STABLE`; ESP32 giữ A ở 1700 mm, nâng riêng B nốt 850 mm và trả `DONE_BRIDGE_B`. Teensy vẫn được căn line sau cầu trong lúc B nâng, nhưng bắt buộc chờ `DONE_BRIDGE_B` trước khi gửi `THA_2A`. Các pha nâng dùng giới hạn 9000 step/s và gia tốc 6000 step/s². `MechanismController::updateValveSequence()` vẫn xử lý không chặn cho `THA_2A` và `THA_2B`. Lệnh chuyển động khác trong lúc cơ cấu bận nhận phản hồi `BUSY`. `STOP` luôn hủy chuỗi, dừng stepper, ghi `0xFF` và phản hồi `STOPPED`.
Sau chuỗi van `THA_2B`, ESP32 trả `DONE_THA_2B` ngay để Teensy bắt đầu chạy robot. ESP32 cho riêng động cơ B hạ trước 400 mm, sau đó A và B cùng chạy với tốc độ 8000 step/s và gia tốc 5000 step/s² cho tới khi HOME37 ổn định 25 ms. Lúc đó B được đặt zero và riêng A đổi chiều kéo về HOME36. Nếu HOME37 tác động sớm trong 400 mm đầu, firmware dừng B ngay và bỏ qua pha hạ đồng bộ để bảo vệ cơ cấu. Toàn bộ quá trình không chặn UART/E18/STOP và có timeout 45 giây.

Phía Teensy, sau `DONE_THA_2B`, robot giữ heading và thực hiện một vector chéo duy nhất: lùi 3000 mm (`-X`) đồng thời đi phải 2000 mm (`-Y`), giới hạn tốc độ tổng 1200 mm/s, rồi chuyển sang `FINISH`.

Log kiểm tra ví dụ:

```text
[POINT_A] P2 + P3 ON TOGETHER
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

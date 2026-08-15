# ROBOCON_KOSEN_F0

Firmware nền cho robot mecanum dùng Teensy 4.1. Project tích hợp giao diện Web Serial tại `pid-tuner` theo giao thức RBT/1.

## An toàn trước khi chạy

1. Kê bốn bánh khỏi mặt đất.
2. Kiểm tra lại toàn bộ chân trong khối `Config` của `src/main.cpp`.
3. Sửa `ENCODER_CPR`, đường kính bánh, chiều `MOTOR_SIGN` và `ENCODER_SIGN` theo robot thật.
4. Chân ESTOP 31 phải ở HIGH mới cho phép `RUN`; kéo LOW sẽ dừng ngay.
5. Firmware khởi động ở trạng thái khóa motor. Phải gửi `RUN` mới arm robot.

## Build và nạp

Mở chính thư mục này bằng VS Code + PlatformIO, sau đó chọn Build và Upload. Serial Monitor dùng 115200 baud.

## Kết nối PID Tuner

- Telemetry TX -> Teensy RX5 pin 34.
- Telemetry RX -> Teensy TX5 pin 33.
- Nối chung GND, logic 3.3 V.
- Giao diện chọn 57600 baud.

Lệnh hỗ trợ:

```text
RUN
ESTOP
PID,WHEEL,FL,0.82,0.035,0.014
PID,WHEEL,ALL,0.82,0.035,0.014
PID,HEADING,0,3.2,0.018,0.24
PID,LINE,0,1.45,0.012,0.18
YAW,90
DRIVE,60,0,0
AUTO
```

`DRIVE,vx,vy,wz` dùng đơn vị RPM bánh: `vx` tiến, `vy` sang phải, `wz` quay. Nếu không nhận lệnh DRIVE mới trong 500 ms, robot tự đưa vận tốc về 0.

## Kịch bản tự động

Máy trạng thái trong `updateAutonomous()` hiện chỉ là kịch bản kiểm tra an toàn:

1. Đi thẳng 1000 mm.
2. Gửi `POINT_A` sang cơ cấu bằng Serial4.
3. Chờ cơ cấu trả `DONE`.
4. Lùi về 1000 mm và kết thúc.

Phải thay phần này bằng kích thước và trình tự chính thức của sân Kosen trước khi thi đấu. Mỗi bước có timeout 8 giây; quá thời gian robot khóa motor và gửi `ERR,AUTO_TIMEOUT`.

## Thứ tự commissioning

Motor từng bánh -> chiều encoder -> CPR -> mecanum -> PID từng bánh -> BNO055 -> line MCP3008 -> ToF -> UART cơ cấu -> từng trạng thái auto.

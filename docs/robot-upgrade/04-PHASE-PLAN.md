# Kế hoạch triển khai theo phase

## Phase 1 — Protocol + Dashboard foundation

- RBT/2 binary framing, CRC16, sequence, timestamp.
- HELLO/HELLO_ACK, heartbeat 10 Hz, connection timeout 500 ms.
- CONFIG_READ handshake; giữ RBT/1 tương thích trong giai đoạn migration.
- Data model và diagnostics counters.
- Test parser với packet chia nhỏ, dính liền, CRC sai và noise.

## Phase 2 — Motor PID Tune

- Tách motor/encoder/PID khỏi main.
- OPEN_LOOP_PWM và CLOSED_LOOP_SPEED_PID loại trừ nhau.
- Điều khiển từng bánh và mixer 4 bánh.
- Ring-buffer graph target/actual/error/PWM.

## Phase 3 — Heading PID

- Heading hold enable/disable/reset/set-current-target.
- Runtime target/actual/error/omega và graph.

## Phase 4 — Line PID

- 3 cụm left/right/rear, mỗi cụm 3 mắt và PID riêng.
- Enable/valid/fault độc lập.

## Phase 5 — ToF PID

- 4 VL53L1X, XSHUT/address management.
- PID không chạy khi invalid/timeout/range error.

## Phase 6 — Optical Flow

- 4 module, quality/flow/velocity/timestamp/valid.

## Phase 7 — Actuators + ESP32 mechanism

- Servo, BLDC protocol abstraction, pneumatic interlock.
- ESP32 command ID + ACK/DONE/ERROR/timeout.

## Phase 8 — Config storage

- EEPROM/Flash record A/B, version, size, CRC32, factory defaults.
- Save/Load/Factory reset; rate-limit writes.

## Phase 9 — Safety + Fault

- Safety priority override, fault latching/clearing, physical E-stop policy.

## Phase 10 — Logging + Diagnostics

- CSV browser logging, fault log, optional SD buffered logger.
- Loop rates, drops, CRC errors, timeouts, CPU load estimate.

## Gate sau mỗi phase

1. Build firmware và dashboard.
2. Unit test protocol/module.
3. Bench test bánh kê khỏi mặt đất.
4. Ghi lại pin/config/threshold đã xác nhận.
5. Chỉ merge sang phase tiếp theo khi gate đạt.


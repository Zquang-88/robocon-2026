# Kết quả Phase 1A

Ngày kiểm tra: 2026-08-08.

## Đã triển khai

- Audit code hiện hành, không phục hồi cây firmware cũ đã bị xóa.
- Kiến trúc module, data model và kế hoạch migration 10 phase.
- Lõi RBT/2 binary:
  - header 16 byte;
  - CRC16-CCITT-FALSE;
  - sequence/timestamp;
  - parser resync sau noise;
  - giới hạn payload 1024 byte;
  - diagnostics counters.
- RBT/1 compatibility handshake dùng được ngay:
  - `PING` -> firmware/config version;
  - `GET_CONFIG` -> PID 4 bánh, heading, line, yaw target;
  - `HEARTBEAT` 10 Hz;
  - GUI báo mất link sau 500 ms;
  - GUI không cho gửi PID trước khi nhận `CFG_END`.

## Kết quả test

| Test | Kết quả |
|---|---|
| Native RBT/2 encode/decode | PASS |
| Packet bị chia nhỏ | PASS |
| Packet dính liền | PASS |
| Noise trước magic | PASS |
| CRC corruption + recovery | PASS |
| CRC vector `123456789` = `0x29B1` | PASS |
| Teensy 4.1 PlatformIO release build | PASS |
| Dashboard production build | PASS |

Firmware build sử dụng 72,352 byte code; còn dư lớn trên Teensy 4.1.

## Chưa được tuyên bố hoàn thành

- RBT/2 chưa thay RBT/1 trên robot thật; cần bench test telemetry radio trước.
- Config mới đồng bộ RAM, chưa lưu EEPROM/Flash (thuộc Phase 8).
- Chưa test motor/sensor vật lý; tuyệt đối kê bánh khi commissioning.
- Dashboard mở rộng đầy đủ các tab thuộc các phase sau.

## Gate trước Phase 2

1. Nạp firmware lên Teensy và xác nhận `PING/GET_CONFIG/HEARTBEAT` qua radio.
2. Rút/cắm lại telemetry và xác nhận GUI đọc đúng PID RAM hiện tại.
3. Xác nhận pin, chiều motor/encoder và CPR.
4. Chốt việc dùng tên `RL/RR` hay `BL/BR` trên toàn hệ thống.


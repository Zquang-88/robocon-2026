# RBT/2 protocol

## Framing

Little-endian, không padding:

```cpp
struct PacketHeader {
  uint16_t magic;         // 0x4252 = "RB" trên dây
  uint8_t version;        // 2
  uint8_t type;
  uint16_t payloadLength; // tối đa 1024
  uint32_t sequence;
  uint32_t timestampMs;
  uint16_t crc16;
};
```

CRC16-CCITT-FALSE tính trên header với `crc16 = 0`, sau đó payload.

## Packet types Phase 1

| Type | Hướng | Mục đích |
|---|---|---|
| HELLO | PC -> Teensy | protocol range + client nonce |
| HELLO_ACK | Teensy -> PC | firmware/config version + capabilities |
| HEARTBEAT | hai chiều | 10 Hz, mode/fault/link health |
| CONFIG_READ | PC -> Teensy | yêu cầu config hiện tại |
| CONFIG_DATA | Teensy -> PC | chunk config versioned |
| CONFIG_WRITE | PC -> Teensy | thay đổi RAM, chưa ghi flash |
| CONFIG_SAVE | PC -> Teensy | yêu cầu lưu bền vững |
| TELEMETRY | Teensy -> PC | snapshot/module samples |
| COMMAND | PC -> Teensy | mode/setpoint/actuator command |
| ACK | hai chiều | ACK theo sequence/command ID |
| ERROR | hai chiều | mã lỗi có cấu trúc |

## Reconnect

```text
open port
HELLO
HELLO_ACK
CONFIG_READ
CONFIG_DATA chunk(s)
ACK config complete
start HEARTBEAT + TELEMETRY
```

Không có bước nào tự ghi giá trị mặc định GUI xuống robot.

## Tương thích

RBT/1 ASCII tiếp tục là protocol active của robot trong Phase 1A. RBT/2 được đưa vào
song song và chỉ trở thành mặc định sau bench test. Không trộn byte ASCII và binary
trên cùng link sau khi đã negotiate RBT/2.


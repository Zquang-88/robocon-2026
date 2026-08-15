# Data model RBT/2

## Trạng thái chung

- `RobotMode`: IDLE, MANUAL, AUTO, TUNE, SAFE, FAULT.
- `ModuleState`: enabled, valid, fault, lastUpdateMs.
- `FaultFlags`: bitmask 32-bit; fault latching do `FaultManager` quản lý.

## PID

Mọi PID dùng chung cấu trúc:

```cpp
struct PIDConfig {
  float kp, ki, kd;
  float integralLimit;
  float outputLimit;
  bool enabled;
};

struct PIDRuntime {
  float target, actual, error;
  float integral, derivative, output;
};
```

`PIDConfig` là dữ liệu lưu bền vững. `PIDRuntime` chỉ là telemetry.

## RobotConfig

Một record versioned gồm:

- 4 motor config.
- Heading PID.
- 3 line PID (left/right/rear).
- 4 distance PID (front/rear/left/right).
- 4 servo config.
- 2 BLDC config.
- 8 valve config + interlock mask.
- Safety thresholds và communication settings.
- `version`, `size`, `crc32`.

Phase 1 mới truyền schema/config hiện có. Field chưa có phần cứng phải mang trạng thái disabled.

## Telemetry

Telemetry snapshot chứa runtime hiện tại. Dữ liệu graph tần số cao có thể tách thành
packet `STREAM_SAMPLE` để không phải gửi toàn bộ snapshot lớn ở 50 Hz.


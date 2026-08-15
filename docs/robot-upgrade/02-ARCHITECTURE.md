# Kiến trúc đích

## Firmware Teensy

```text
ROBOCON_KOSEN_F0/
  src/
    main.cpp                  # setup, scheduler, wiring dependency
    app/ModeManager.*         # IDLE/MANUAL/AUTO/TUNE/SAFE/FAULT
    control/PidController.*
    control/MotorControl.*
    control/HeadingControl.*
    control/LineControl.*
    control/DistanceControl.*
    sensors/EncoderBank.*
    sensors/ImuSensor.*
    sensors/LineSensors.*
    sensors/TofSensors.*
    sensors/OpticalFlow.*
    sensors/BatteryMonitor.*
    actuators/ServoBank.*
    actuators/BldcBank.*
    actuators/PneumaticBank.*
    communication/Protocol.*
    communication/TelemetryService.*
    communication/MechanismLink.*
    config/RobotConfig.*
    config/ConfigStore.*
    safety/FaultManager.*
    safety/SafetyManager.*
    logging/Logger.*
```

`main.cpp` chỉ điều phối scheduler và gọi module. Module không tự dùng `delay()`.

## Scheduler đề xuất

| Task | Tần số | Ưu tiên |
|---|---:|---:|
| E-stop + Safety | mỗi vòng loop | cao nhất |
| Motor speed PID | 200 Hz | cao |
| Heading PID | 100 Hz | cao |
| Line PID | 100 Hz | cao |
| Distance PID | 50 Hz | trung bình |
| Sensor update | theo sensor | trung bình |
| PC heartbeat | 10 Hz | trung bình |
| Telemetry | 25–50 Hz | thấp |
| Buffered logging | 20–100 Hz | thấp |

## GUI

```text
Web Serial worker
  -> RBT/2 stream parser
  -> Robot data store
  -> ring buffers (10–30 s)
  -> tab UI
  -> CSV/profile export
```

State kết nối:

```text
DISCONNECTED -> PORT_OPEN -> HANDSHAKE -> SYNC_CONFIG -> STREAMING
                                             |              |
                                             +----ERROR-----+
```

GUI không gửi PID mặc định khi reconnect. Robot là nguồn cấu hình chính.


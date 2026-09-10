import csv
import statistics
import time
from pathlib import Path

import serial

PORT = "COM5"
BAUD = 57600
WHEELS = ("FL", "FR", "BL", "BR")
PID_GAINS = {
    "FL": (0.15, 0.08, 0.0),
    "FR": (0.15, 0.10, 0.0),
    "BL": (0.15, 0.08, 0.0),
    "BR": (0.15, 0.08, 0.0),
}
TARGETS = (60.0, -60.0, 100.0, -100.0)
RUN_S = 4.2
SETTLE_S = 2.6
REST_S = 0.35
OUT = Path(__file__).with_name("wheel_lifted_baseline.csv")


def send(port, command):
    port.write((command + "\n").encode("ascii"))
    port.flush()


def read_lines(port, buffer):
    buffer.extend(port.read(2048))
    lines = []
    while b"\n" in buffer:
        raw, _, remainder = buffer.partition(b"\n")
        buffer[:] = remainder
        lines.append(raw.rstrip(b"\r").decode("ascii", errors="replace"))
    return lines


def run_target(port, wheel, target, writer):
    samples = []
    buffer = bytearray()
    started = time.monotonic()
    deadline = started + RUN_S
    send(port, f"TUNE,WHEEL,{wheel},{target:.1f}")
    next_heartbeat = 0.0
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_heartbeat:
            send(port, "HEARTBEAT")
            next_heartbeat = now + 0.10
        for line in read_lines(port, buffer):
            if line.startswith(("ACK,", "ERR,", "FAULT,")):
                print(f"ROBOT:{line}", flush=True)
            if line.startswith("FAULT,") or line.startswith("STATE,FAULT_STOP"):
                raise RuntimeError(line)
            if line.startswith("ERR,"):
                raise RuntimeError(line)
            if not line.startswith(f"WHEEL,{wheel},"):
                continue
            fields = line.split(",")
            if len(fields) < 6:
                continue
            row = (wheel, target, now - started, float(fields[2]),
                   float(fields[3]), int(fields[4]), int(fields[5]))
            writer.writerow(row)
            if now - started >= SETTLE_S:
                samples.append(row)
        time.sleep(0.004)
    if len(samples) < 3:
        raise RuntimeError(f"INSUFFICIENT_DATA_{wheel}_{target}")
    actual = statistics.mean(s[4] for s in samples)
    pwm = statistics.mean(s[5] for s in samples)
    ripple = statistics.pstdev(s[4] for s in samples)
    error = target - actual
    print(f"{wheel:2s} target={target:6.1f} actual={actual:7.2f} "
          f"error={error:7.2f} pwm={pwm:7.1f} ripple={ripple:6.2f}", flush=True)
    return wheel, target, actual, error, pwm, ripple


def main():
    summaries = []
    result = "COMPLETE"
    port = serial.Serial(PORT, BAUD, timeout=0.02)
    print(f"{PORT}_OPEN_DIRECT_WIRED", flush=True)
    try:
        time.sleep(0.8)
        port.reset_input_buffer()
        send(port, "RESET")
        time.sleep(0.25)
        send(port, "MODE,MANUAL")
        time.sleep(0.25)
        for wheel, gains in PID_GAINS.items():
            send(port, f"PID,WHEEL,{wheel},{gains[0]},{gains[1]},{gains[2]}")
            time.sleep(0.10)
        send(port, "GET_CONFIG")
        time.sleep(0.25)
        with OUT.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(("wheel", "requested_rpm", "elapsed_s", "target_rpm",
                             "actual_rpm", "pwm", "encoder"))
            for wheel in WHEELS:
                for target in TARGETS:
                    summaries.append(run_target(port, wheel, target, writer))
                    send(port, "TUNE,STOP")
                    time.sleep(REST_S)
    except Exception as exc:
        result = f"ABORTED:{exc}"
    finally:
        try:
            for _ in range(3):
                send(port, "TUNE,STOP")
                time.sleep(0.05)
            send(port, "STOP")
        finally:
            port.close()
    print(f"RESULT={result}", flush=True)
    print("STOP_SENT_COM5_CLOSED", flush=True)
    print(f"LOG={OUT.resolve()}", flush=True)


if __name__ == "__main__":
    main()

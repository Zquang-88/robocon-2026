import csv
import time
from datetime import datetime, timezone
from pathlib import Path

import serial


PORT = "COM5"
BAUD = 57600
DRIVE_SECONDS = 2.0
REST_SECONDS = 0.8
SPEEDS_MM_S = (100, -100, 200, -200, 300, -300, 400, -400)
LOG_PATH = Path(__file__).with_name("longitudinal_loaded_baseline.csv")
PID_COMMANDS = ()


def send(port: serial.Serial, command: str) -> None:
    port.write((command + "\n").encode("ascii"))
    port.flush()


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def run_phase(port, writer, log_file, label, vx, duration):
    deadline = time.monotonic() + duration
    next_command = 0.0
    receive_buffer = bytearray()
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_command:
            send(port, f"DRIVE,{vx},0,0")
            send(port, "HEARTBEAT")
            next_command = now + 0.10

        data = port.read(1024)
        if data:
            receive_buffer.extend(data)
            while b"\n" in receive_buffer:
                raw, _, receive_buffer = receive_buffer.partition(b"\n")
                line = raw.rstrip(b"\r").decode("ascii", errors="replace")
                writer.writerow((utc_now(), label, vx, line))
                log_file.flush()
                if line.startswith(("FAULT,", "STATE,FAULT_STOP")):
                    raise RuntimeError(line)
        else:
            time.sleep(0.004)


def main() -> None:
    result = "COMPLETE"
    port = serial.Serial(PORT, BAUD, timeout=0.02)
    print(f"{PORT}_OPEN")
    try:
        time.sleep(1.0)
        port.reset_input_buffer()
        for command in ("RESET", "MODE,MANUAL"):
            send(port, command)
            time.sleep(0.25)

        for command in PID_COMMANDS:
            send(port, command)
            time.sleep(0.15)

        with LOG_PATH.open("w", newline="", encoding="utf-8") as log_file:
            writer = csv.writer(log_file)
            writer.writerow(("pc_time_iso", "phase", "command_vx_mm_s", "telemetry"))
            for speed in SPEEDS_MM_S:
                label = ("FORWARD" if speed > 0 else "BACKWARD") + f"_{abs(speed)}"
                print(f"RUN {label}")
                run_phase(port, writer, log_file, label, speed, DRIVE_SECONDS)
                run_phase(port, writer, log_file, "REST", 0, REST_SECONDS)
    except Exception as exc:
        result = f"ABORTED:{exc}"
    finally:
        try:
            for _ in range(3):
                send(port, "DRIVE,0,0,0")
                time.sleep(0.05)
            send(port, "STOP")
            time.sleep(0.2)
        finally:
            port.close()
        print(f"RESULT={result}")
        print("STOP_SENT_COM5_CLOSED")
        print(f"LOG={LOG_PATH.resolve()}")


if __name__ == "__main__":
    main()

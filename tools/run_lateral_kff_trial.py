from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

import serial


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "firmwave" / "ROBOCON_KOSEN_F0" / "tools"))
import run_loaded_lateral_suite as suite  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM5")
    parser.add_argument("--seconds", type=float, default=1.2)
    parser.add_argument("--speeds", default="500,700")
    parser.add_argument("--output", required=True)
    parser.add_argument("--fr-negative", type=float, required=True)
    parser.add_argument("--bl-negative", type=float, required=True)
    parser.add_argument("--bl-ki", type=float)
    args = parser.parse_args()

    speeds = [int(value) for value in args.speeds.split(",") if value.strip()]
    rows = []
    started = time.monotonic()
    with serial.Serial(args.port, 57600, timeout=0.03) as port:
        time.sleep(0.5)
        port.reset_input_buffer()
        for command, wait_s in (
            ("STOP", 0.2),
            ("RESET", 0.2),
            ("MODE,MANUAL", 0.2),
            ("MANUAL,HEADING,OFF", 0.3),
            (f"SET_FF_DIR,1,0.0115,{args.fr_negative:.6f},48,45", 0.2),
            (f"SET_FF_DIR,2,0.0382,{args.bl_negative:.6f},38,48", 0.3),
        ):
            suite.send(port, command)
            time.sleep(wait_s)
            suite.read_available(port, rows, -1, "IDLE", 0, started)

        if args.bl_ki is not None:
            suite.send(port, f"PID,WHEEL,BL,0.25,{args.bl_ki:.6f},0")
            time.sleep(0.3)
            suite.read_available(port, rows, -1, "IDLE", 0, started)

        segment = 0
        try:
            for speed in speeds:
                for direction in ("LEFT", "RIGHT"):
                    print(f"RUN {direction} {speed} mm/s", flush=True)
                    suite.run_segment(
                        port, rows, segment, direction, speed, args.seconds, started
                    )
                    result, max_heading = suite.summarize(rows, segment, speed)
                    print(
                        f"RESULT {result} MAX_HEADING_ERROR={max_heading:.2f}",
                        flush=True,
                    )
                    segment += 1
        finally:
            suite.send(port, "DRIVE,0,0,0")
            time.sleep(0.2)
            suite.send(port, "STOP")
            time.sleep(0.3)
            suite.read_available(port, rows, segment, "STOP", 0, started)

    fieldnames = [
        "host_s",
        "segment",
        "direction",
        "command_mm_s",
        "record",
        "wheel",
        "target_rpm",
        "actual_rpm",
        "pwm",
        "encoder",
        "yaw_deg",
        "yaw_target_deg",
        "yaw_error_deg",
    ]
    with open(args.output, "w", newline="", encoding="utf-8-sig") as output:
        writer = csv.DictWriter(output, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    print(f"SAVED {args.output}", flush=True)


if __name__ == "__main__":
    main()

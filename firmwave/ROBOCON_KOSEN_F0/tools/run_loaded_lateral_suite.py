import argparse
import csv
import time

import serial


WHEELS = ("FL", "FR", "BL", "BR")


def send(port, command):
    port.write((command + "\r\n").encode("ascii"))
    port.flush()


def read_available(port, rows, segment, direction, speed, started):
    while port.in_waiting:
        line = port.readline().decode("ascii", errors="ignore").strip()
        if not line:
            continue
        fields = line.split(",")
        now = time.monotonic() - started
        if len(fields) >= 6 and fields[0] == "WHEEL" and fields[1] in WHEELS:
            rows.append({
                "host_s": now,
                "segment": segment,
                "direction": direction,
                "command_mm_s": speed,
                "record": "WHEEL",
                "wheel": fields[1],
                "target_rpm": fields[2],
                "actual_rpm": fields[3],
                "pwm": fields[4],
                "encoder": fields[5],
                "yaw_deg": "",
                "yaw_target_deg": "",
                "yaw_error_deg": "",
            })
        elif len(fields) >= 6 and fields[0] == "HDG":
            rows.append({
                "host_s": now,
                "segment": segment,
                "direction": direction,
                "command_mm_s": speed,
                "record": "HDG",
                "wheel": "",
                "target_rpm": "",
                "actual_rpm": "",
                "pwm": "",
                "encoder": "",
                "yaw_deg": fields[1],
                "yaw_target_deg": fields[2],
                "yaw_error_deg": fields[3],
            })
        elif fields[0] in ("FAULT", "SAFETY", "ERR"):
            print(line, flush=True)


def run_segment(port, rows, segment, direction, speed, duration, started):
    vy = speed if direction == "LEFT" else -speed
    command = f"DRIVE,0,{vy},0"
    segment_started = time.monotonic()
    next_command = 0.0
    while time.monotonic() - segment_started < duration:
        elapsed = time.monotonic() - segment_started
        if elapsed >= next_command:
            send(port, command)
            next_command += 0.20
        read_available(port, rows, segment, direction, speed, started)
        time.sleep(0.01)
    send(port, "DRIVE,0,0,0")
    stop_started = time.monotonic()
    while time.monotonic() - stop_started < 0.7:
        read_available(port, rows, segment, direction, speed, started)
        time.sleep(0.01)


def summarize(rows, segment, command_mm_s):
    selected = [row for row in rows if row["segment"] == segment]
    result = {}
    nominal_rpm = abs(command_mm_s) * 60.0 / (3.141592653589793 * 100.0)
    for wheel in WHEELS:
        samples = [row for row in selected if row["record"] == "WHEEL" and
                   row["wheel"] == wheel and
                   abs(float(row["target_rpm"])) >= nominal_rpm * 0.90]
        if samples:
            actual = [float(row["actual_rpm"]) for row in samples]
            target = [float(row["target_rpm"]) for row in samples]
            pwm = [float(row["pwm"]) for row in samples]
            result[wheel] = (
                sum(target) / len(target),
                sum(actual) / len(actual),
                sum(pwm) / len(pwm),
            )
    headings = [abs(float(row["yaw_error_deg"])) for row in selected
                if row["record"] == "HDG"]
    max_heading = max(headings) if headings else 0.0
    return result, max_heading


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM5")
    parser.add_argument("--seconds", type=float, default=1.5)
    parser.add_argument("--speeds", default="200,300,400,500,800")
    parser.add_argument("--heading", choices=("on", "off"), default="off")
    parser.add_argument("--output", default="loaded_lateral_tune.csv")
    args = parser.parse_args()

    speeds = [int(value) for value in args.speeds.split(",") if value.strip()]
    rows = []
    started = time.monotonic()
    with serial.Serial(args.port, 57600, timeout=0.03) as port:
        time.sleep(0.5)
        port.reset_input_buffer()
        send(port, "STOP")
        time.sleep(0.2)
        send(port, "RESET")
        time.sleep(0.2)
        send(port, "MODE,MANUAL")
        time.sleep(0.2)
        send(port, "MANUAL,HEADING," + args.heading.upper())
        time.sleep(0.4)
        read_available(port, rows, -1, "IDLE", 0, started)

        segment = 0
        for speed in speeds:
            for direction in ("LEFT", "RIGHT"):
                print(f"RUN {direction} {speed} mm/s", flush=True)
                run_segment(port, rows, segment, direction, speed,
                            args.seconds, started)
                summary, max_heading = summarize(rows, segment, speed)
                print(f"RESULT {summary} MAX_HEADING_ERROR={max_heading:.2f}",
                      flush=True)
                segment += 1
        send(port, "STOP")
        time.sleep(0.3)
        read_available(port, rows, segment, "STOP", 0, started)

    fieldnames = [
        "host_s", "segment", "direction", "command_mm_s", "record",
        "wheel", "target_rpm", "actual_rpm", "pwm", "encoder",
        "yaw_deg", "yaw_target_deg", "yaw_error_deg",
    ]
    with open(args.output, "w", newline="", encoding="utf-8-sig") as output:
        writer = csv.DictWriter(output, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    print(f"SAVED {args.output}", flush=True)


if __name__ == "__main__":
    main()

import argparse
import json
import re
import statistics
import time

import serial


TARGET_RE = re.compile(r"\|\s+(FL|FR|BL|BR)\s+TARGET:\s+([-+0-9.]+)\s+RPM")
ACTUAL_RE = re.compile(r"\|\s+ACTUAL:\s+([-+0-9.]+)\s+ERROR:\s+([-+0-9.]+)")
PWM_RE = re.compile(r"\|\s+PWM:\s+(-?\d+)\s+ENC:")


def send(port, command):
    port.write((command + "\r\n").encode("ascii"))
    port.flush()


def run_test(port, command, duration_s=5.0):
    port.reset_input_buffer()
    send(port, command)
    end = time.monotonic() + duration_s + 0.8
    current = None
    pending_actual = None
    rows = []
    while time.monotonic() < end:
        raw = port.readline()
        if not raw:
            continue
        line = raw.decode("ascii", errors="ignore").strip()
        target_match = TARGET_RE.search(line)
        if target_match:
            current = {
                "wheel": target_match.group(1),
                "target": float(target_match.group(2)),
                "time": time.monotonic(),
            }
            pending_actual = None
            continue
        actual_match = ACTUAL_RE.search(line)
        if current is not None and actual_match:
            pending_actual = (float(actual_match.group(1)), float(actual_match.group(2)))
            continue
        pwm_match = PWM_RE.search(line)
        if current is not None and pending_actual is not None and pwm_match:
            current["actual"] = pending_actual[0]
            current["error"] = pending_actual[1]
            current["pwm"] = int(pwm_match.group(1))
            rows.append(current)
            current = None
            pending_actual = None
    send(port, "STOP")
    time.sleep(0.15)
    return rows


def summarize(rows, settle_s=2.0):
    if not rows:
        return {}
    start = min(row["time"] for row in rows)
    steady = [row for row in rows if row["time"] - start >= settle_s]
    result = {}
    for wheel in ("FL", "FR", "BL", "BR"):
        samples = [row for row in steady if row["wheel"] == wheel]
        if not samples:
            continue
        actual = [row["actual"] for row in samples]
        error = [row["error"] for row in samples]
        pwm = [row["pwm"] for row in samples]
        target = statistics.fmean(row["target"] for row in samples)
        result[wheel] = {
            "target_rpm": round(target, 3),
            "actual_mean_rpm": round(statistics.fmean(actual), 3),
            "actual_std_rpm": round(statistics.pstdev(actual), 3),
            "mean_error_rpm": round(statistics.fmean(error), 3),
            "mae_rpm": round(statistics.fmean(abs(value) for value in error), 3),
            "mean_pwm": round(statistics.fmean(pwm), 2),
            "samples": len(samples),
        }
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM14")
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--output", default="wheel_tune_results.json")
    args = parser.parse_args()

    speeds = [13, 38, 57, 76, 95, 191, 220]
    results = []
    with serial.Serial(args.port, 115200, timeout=0.15) as port:
        time.sleep(1.0)
        send(port, "STOP")
        for direction_name, signs in (
            ("LEFT", (-1, 1, 1, -1)),
            ("RIGHT", (1, -1, -1, 1)),
        ):
            for speed in speeds:
                values = [sign * speed for sign in signs]
                command = "RUN4_RPM " + " ".join(str(value) for value in values)
                command += f" {max(1, int(args.seconds))}"
                print(f"RUN {direction_name} {speed} RPM", flush=True)
                rows = run_test(port, command, args.seconds)
                summary = summarize(rows)
                results.append({
                    "direction": direction_name,
                    "speed_rpm": speed,
                    "command": command,
                    "summary": summary,
                })
                print(json.dumps(summary, ensure_ascii=False), flush=True)
                time.sleep(0.35)

    with open(args.output, "w", encoding="utf-8") as output:
        json.dump(results, output, indent=2, ensure_ascii=False)
    print(f"SAVED {args.output}")


if __name__ == "__main__":
    main()

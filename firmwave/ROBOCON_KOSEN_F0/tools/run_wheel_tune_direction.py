import argparse
import json
import time

import serial

from run_wheel_tune_suite import run_test, send, summarize


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM14")
    parser.add_argument("--direction", choices=("LEFT", "RIGHT"), required=True)
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--speeds", default="13,38,57,76,95,191,220")
    parser.add_argument("--config", action="append", default=[])
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    signs = (-1, 1, 1, -1) if args.direction == "LEFT" else (1, -1, -1, 1)
    speeds = [int(value) for value in args.speeds.split(",") if value.strip()]
    results = []
    with serial.Serial(args.port, 115200, timeout=0.15) as port:
        time.sleep(0.8)
        send(port, "STOP")
        for command in args.config:
            send(port, command)
            time.sleep(0.08)
        for speed in speeds:
            values = [sign * speed for sign in signs]
            command = "RUN4_RPM " + " ".join(str(value) for value in values)
            command += f" {max(1, int(args.seconds))}"
            print(f"RUN {args.direction} {speed} RPM", flush=True)
            rows = run_test(port, command, args.seconds)
            summary = summarize(rows)
            results.append({
                "direction": args.direction,
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

from __future__ import annotations

import argparse
import csv
import math
from collections import defaultdict


WHEELS = ("FL", "FR", "BL", "BR")
WHEEL_CIRCUMFERENCE_MM = math.pi * 100.0


def rpm_to_mm_s(value: float) -> float:
    return value * WHEEL_CIRCUMFERENCE_MM / 60.0


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path")
    args = parser.parse_args()
    with open(args.csv_path, newline="", encoding="utf-8-sig") as handle:
        rows = list(csv.DictReader(handle))

    wheel_rows = defaultdict(list)
    heading_rows = defaultdict(list)
    for row in rows:
        segment = int(row["segment"])
        if segment < 0:
            continue
        if row["record"] == "WHEEL":
            wheel_rows[segment].append(row)
        elif row["record"] == "HDG":
            heading_rows[segment].append(row)

    for segment in sorted(wheel_rows):
        source = wheel_rows[segment]
        sets = []
        current = {}
        for row in source:
            wheel = row["wheel"]
            if wheel == "FL" and current:
                current = {}
            current[wheel] = row
            if all(name in current for name in WHEELS):
                sets.append(current)
                current = {}
        command = float(source[0]["command_mm_s"])
        direction = source[0]["direction"]
        steady = []
        for item in sets:
            targets = [rpm_to_mm_s(float(item[name]["target_rpm"])) for name in WHEELS]
            actual = [rpm_to_mm_s(float(item[name]["actual_rpm"])) for name in WHEELS]
            if max(abs(value) for value in targets) < 0.90 * command:
                continue
            fl, fr, bl, br = actual
            vx = (fl + fr + bl + br) / 4.0
            vy = (-fl + fr + bl - br) / 4.0
            steady.append((vx, vy, actual, targets))
        if not steady:
            continue
        vx_values = [item[0] for item in steady]
        vy_values = [item[1] for item in steady]
        headings = [abs(float(row["yaw_error_deg"])) for row in heading_rows[segment]]
        print(
            f"segment={segment} {direction:5s} command={command:4.0f} "
            f"samples={len(steady)} mean_vx={sum(vx_values)/len(vx_values):7.1f} "
            f"peak_abs_vx={max(abs(v) for v in vx_values):7.1f} "
            f"mean_abs_vy={sum(abs(v) for v in vy_values)/len(vy_values):7.1f} "
            f"max_heading_error={max(headings) if headings else 0.0:5.2f}"
        )
        for vx, vy, actual, _ in steady:
            print(
                f"  vx={vx:7.1f} vy={vy:7.1f} actual_mm_s="
                + ",".join(f"{name}:{value:7.1f}" for name, value in zip(WHEELS, actual))
            )


if __name__ == "__main__":
    main()

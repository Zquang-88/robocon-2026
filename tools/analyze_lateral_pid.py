import csv
import statistics
import sys
from collections import defaultdict


WHEELS = ("FL", "FR", "BL", "BR")


def analyze(path):
    samples = defaultdict(lambda: defaultdict(list))
    with open(path, newline="", encoding="utf-8-sig") as handle:
        rows = list(csv.DictReader(handle))
    phase_counts = defaultdict(int)
    for row in rows:
        phase_counts[row["phase"]] += 1
    seen = defaultdict(int)
    for row in rows:
        phase = row["phase"]
        seen[phase] += 1
        if phase == "REST" or seen[phase] <= phase_counts[phase] // 2:
            continue
        fields = row["telemetry"].split(",")
        if len(fields) >= 5 and fields[0] == "WHEEL" and fields[1] in WHEELS:
            try:
                samples[phase][fields[1]].append((float(fields[2]), float(fields[3]), float(fields[4])))
            except ValueError:
                pass
    print(f"FILE {path}")
    for phase, wheels in samples.items():
        errors = []
        parts = []
        for wheel in WHEELS:
            values = wheels[wheel]
            if not values:
                parts.append(f"{wheel} --")
                continue
            target = statistics.mean(v[0] for v in values)
            speed = statistics.mean(v[1] for v in values)
            pwm = statistics.mean(v[2] for v in values)
            errors.extend(abs(v[0] - v[1]) for v in values)
            parts.append(f"{wheel} {target:.1f}/{speed:.1f} pwm={pwm:.1f}")
        mae = statistics.mean(errors) if errors else float("nan")
        print(f"{phase}: {' | '.join(parts)} | MAE={mae:.2f}")


for filename in sys.argv[1:]:
    analyze(filename)

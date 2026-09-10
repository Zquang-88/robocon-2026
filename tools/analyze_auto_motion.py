from collections import defaultdict
from pathlib import Path
import math
import sys


path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).with_name("auto_test_last.log")
lines = path.read_bytes().decode("ascii", errors="replace").splitlines()
wheels = {name: [0.0, 0.0, 0.0, 0.0] for name in ("FL", "FR", "BL", "BR")}
state = "UNKNOWN"
pose = [0.0, 0.0, 0.0]
yaw = [0.0, 0.0, 0.0]
tof = [0.0, 0]
records = []
aliases = {"RL": "BL", "RR": "BR"}

for line in lines:
    f = line.strip().split(",")
    try:
        if f[0] == "WHEEL" and len(f) >= 6:
            name = aliases.get(f[1], f[1])
            if name in wheels:
                wheels[name] = [float(f[2]), float(f[3]), float(f[4]), float(f[5])]
        elif f[0] == "POSE" and len(f) >= 4:
            pose = [float(f[1]), float(f[2]), float(f[3])]
        elif f[0] == "SYS" and len(f) >= 2:
            state = f[1]
        elif f[0] == "HDG" and len(f) >= 4:
            yaw = [float(f[1]), float(f[2]), float(f[3])]
        elif f[0] == "TOF" and len(f) >= 3:
            tof = [float(f[1]), int(f[2])]
        elif f[0] == "MOTION" and len(f) >= 8:
            robot_ms = int(f[1]); active = int(f[2]); vx, vy, wz = map(float, f[3:6])
            mm_s = {name: values[1] * math.pi * 100.0 / 60.0 for name, values in wheels.items()}
            actual_vx = sum(mm_s.values()) / 4.0
            actual_vy = (-mm_s["FL"] + mm_s["FR"] + mm_s["BL"] - mm_s["BR"]) / 4.0
            records.append({"ms": robot_ms, "state": state, "active": active, "vx": vx, "vy": vy,
                            "actual_vx": actual_vx, "actual_vy": actual_vy, "pose": tuple(pose),
                            "yaw": tuple(yaw), "tof": tuple(tof),
                            "wheels": {k: tuple(v) for k, v in wheels.items()}})
    except (ValueError, IndexError):
        pass

groups = defaultdict(list)
for record in records:
    if record["active"]:
        groups[record["state"]].append(record)

for name, group in groups.items():
    first, last = group[0], group[-1]
    mean = lambda key: sum(item[key] for item in group) / len(group)
    yaw_abs = [abs(item["yaw"][2]) for item in group]
    print(f"STATE={name} samples={len(group)} duration={(last['ms']-first['ms'])/1000:.2f}s")
    print(f"  command vx/vy={mean('vx'):.1f}/{mean('vy'):.1f} actual vx/vy={mean('actual_vx'):.1f}/{mean('actual_vy'):.1f}")
    print(f"  pose delta x/y={last['pose'][0]-first['pose'][0]:.1f}/{last['pose'][1]-first['pose'][1]:.1f} yawErr mean/max={sum(yaw_abs)/len(yaw_abs):.2f}/{max(yaw_abs):.2f}")
    print(f"  yaw actual first/last={first['yaw'][0]:.2f}/{last['yaw'][0]:.2f} target first/last={first['yaw'][1]:.2f}/{last['yaw'][1]:.2f}")
    valid_tof = [item["tof"][0] for item in group if item["tof"][1]]
    if valid_tof:
        print(f"  tof first/last/min/max={valid_tof[0]:.1f}/{valid_tof[-1]:.1f}/{min(valid_tof):.1f}/{max(valid_tof):.1f}")
    for wheel in ("FL", "FR", "BL", "BR"):
        errors = [item["wheels"][wheel][1] - item["wheels"][wheel][0] for item in group]
        targets = [item["wheels"][wheel][0] for item in group]
        actuals = [item["wheels"][wheel][1] for item in group]
        pwms = [item["wheels"][wheel][2] for item in group]
        saturation = sum(abs(value) >= 250 for value in pwms) * 100.0 / len(pwms)
        print(f"  {wheel}: target={sum(targets)/len(targets):.1f} actual={sum(actuals)/len(actuals):.1f} err={sum(errors)/len(errors):+.1f} abs={sum(map(abs,errors))/len(errors):.1f} pwm_mean/max={sum(map(abs,pwms))/len(pwms):.0f}/{max(map(abs,pwms)):.0f} sat={saturation:.0f}%")

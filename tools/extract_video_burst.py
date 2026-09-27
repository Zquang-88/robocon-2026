from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("video", type=Path)
    parser.add_argument("--start", type=float, required=True)
    parser.add_argument("--end", type=float, required=True)
    parser.add_argument("--count", type=int, default=12)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    cap = cv2.VideoCapture(str(args.video))
    frames = []
    for timestamp in np.linspace(args.start, args.end, args.count):
        cap.set(cv2.CAP_PROP_POS_MSEC, float(timestamp * 1000.0))
        ok, frame = cap.read()
        if not ok:
            continue
        frame = cv2.resize(frame, (180, 320), interpolation=cv2.INTER_AREA)
        cv2.putText(frame, f"{timestamp:.2f}s", (5, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1, cv2.LINE_AA)
        frames.append(frame)
    cap.release()
    if not frames:
        raise RuntimeError("No frames decoded")
    columns = 6
    while len(frames) % columns:
        frames.append(np.zeros_like(frames[0]))
    sheet = np.vstack([np.hstack(frames[i : i + columns]) for i in range(0, len(frames), columns)])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.output), sheet, [cv2.IMWRITE_JPEG_QUALITY, 72])


if __name__ == "__main__":
    main()

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np


def rolling_median(values: np.ndarray, radius: int) -> np.ndarray:
    result = np.empty_like(values, dtype=float)
    for i in range(len(values)):
        lo = max(0, i - radius)
        hi = min(len(values), i + radius + 1)
        result[i] = np.nanmedian(values[lo:hi])
    return result


def analyze(video_path: Path, output_dir: Path, scale: float = 0.5) -> None:
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open {video_path}")
    fps = float(cap.get(cv2.CAP_PROP_FPS))
    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    sample_indexes = np.linspace(0, frame_count - 1, min(61, frame_count), dtype=int)
    samples: list[np.ndarray] = []
    for index in sample_indexes:
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(index))
        ok, frame = cap.read()
        if ok:
            samples.append(cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA))
    if not samples:
        raise RuntimeError("No video frames decoded")
    background = np.median(np.stack(samples), axis=0).astype(np.uint8)

    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
    records: list[dict[str, float]] = []
    overlays: list[np.ndarray] = []
    previous: tuple[float, float] | None = None
    kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    kernel_join = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (13, 13))
    index = 0
    while True:
        ok, frame_full = cap.read()
        if not ok:
            break
        frame = cv2.resize(frame_full, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        diff = cv2.absdiff(frame, background)
        score = np.max(diff, axis=2)
        mask = (score >= 32).astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_join)
        mask = cv2.dilate(mask, kernel_join, iterations=1)
        count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)

        candidates: list[tuple[float, int]] = []
        for label in range(1, count):
            x, y, w, h, area = stats[label]
            if area < 500 or area > frame.shape[0] * frame.shape[1] * 0.32:
                continue
            cx, cy = centroids[label]
            compactness = area / max(float(w * h), 1.0)
            score_value = float(area) * (0.5 + compactness)
            if previous is not None:
                distance = np.hypot(cx - previous[0], cy - previous[1])
                score_value /= 1.0 + 0.05 * distance
            candidates.append((score_value, label))

        if candidates:
            label = max(candidates)[1]
            x, y, w, h, area = stats[label]
            cx, cy = map(float, centroids[label])
            previous = (cx, cy)
            records.append({"frame": index, "time_s": index / fps, "x_px": cx, "y_px": cy, "area": area})
            if index % max(1, round(fps)) == 0:
                preview = frame.copy()
                cv2.rectangle(preview, (x, y), (x + w, y + h), (0, 255, 255), 2)
                cv2.circle(preview, (round(cx), round(cy)), 5, (0, 0, 255), -1)
                cv2.putText(preview, f"{index/fps:.2f}s", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)
                overlays.append(preview)
        else:
            records.append({"frame": index, "time_s": index / fps, "x_px": np.nan, "y_px": np.nan, "area": 0})
        index += 1
    cap.release()

    output_dir.mkdir(parents=True, exist_ok=True)
    x = np.array([r["x_px"] for r in records], dtype=float)
    y = np.array([r["y_px"] for r in records], dtype=float)
    valid = np.isfinite(x) & np.isfinite(y)
    if valid.sum() < 20:
        raise RuntimeError(f"Too few tracked frames in {video_path.name}")
    frame_axis = np.arange(len(records))
    x = np.interp(frame_axis, frame_axis[valid], x[valid])
    y = np.interp(frame_axis, frame_axis[valid], y[valid])

    points = np.column_stack((x, y))
    centered = points - np.mean(points, axis=0)
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    along_axis = vh[0]
    cross_axis = vh[1]
    along = centered @ along_axis
    cross = centered @ cross_axis
    cross_smooth = rolling_median(cross, max(2, round(fps * 0.20)))
    jump_window = max(1, round(fps * 0.15))
    cross_step = np.full_like(cross_smooth, np.nan)
    cross_step[jump_window:] = cross_smooth[jump_window:] - cross_smooth[:-jump_window]

    csv_path = output_dir / f"{video_path.stem}_trajectory.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["frame", "time_s", "x_px", "y_px", "along_px", "cross_px", "cross_step_150ms_px"])
        for i, record in enumerate(records):
            writer.writerow([record["frame"], f"{record['time_s']:.4f}", f"{x[i]:.3f}", f"{y[i]:.3f}", f"{along[i]:.3f}", f"{cross_smooth[i]:.3f}", f"{cross_step[i]:.3f}"])

    times = frame_axis / fps
    figure, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    axes[0].plot(x, y, linewidth=1.3)
    axes[0].scatter(x[:: max(1, round(fps))], y[:: max(1, round(fps))], c=times[:: max(1, round(fps))], s=18, cmap="viridis")
    axes[0].invert_yaxis()
    axes[0].set_aspect("equal", adjustable="datalim")
    axes[0].set_title("Tracked robot path in video")
    axes[0].set_xlabel("image x (px)")
    axes[0].set_ylabel("image y (px)")
    axes[0].grid(True, alpha=0.25)
    axes[1].plot(times, cross_smooth, label="cross-track")
    axes[1].plot(times, cross_step, label="150 ms step", alpha=0.75)
    axes[1].axhline(0, color="black", linewidth=0.7)
    axes[1].set_title("Perpendicular motion and abrupt steps")
    axes[1].set_xlabel("time (s)")
    axes[1].set_ylabel("pixels")
    axes[1].grid(True, alpha=0.25)
    axes[1].legend()
    figure.tight_layout()
    plot_path = output_dir / f"{video_path.stem}_trajectory.png"
    figure.savefig(plot_path, dpi=150)
    plt.close(figure)

    if overlays:
        tile_w, tile_h = 240, 426
        tiles = []
        for overlay in overlays:
            tile = cv2.resize(overlay, (tile_w, tile_h), interpolation=cv2.INTER_AREA)
            tiles.append(tile)
        columns = 6
        while len(tiles) % columns:
            tiles.append(np.zeros_like(tiles[0]))
        sheet = np.vstack([np.hstack(tiles[i : i + columns]) for i in range(0, len(tiles), columns)])
        cv2.imwrite(str(output_dir / f"{video_path.stem}_tracking.jpg"), sheet, [cv2.IMWRITE_JPEG_QUALITY, 85])

    finite_steps = np.flatnonzero(np.isfinite(cross_step))
    strongest = finite_steps[np.argsort(np.abs(cross_step[finite_steps]))[-8:]][::-1]
    print(f"\n{video_path.name}")
    print(f"tracked={valid.sum()}/{len(records)}, path_axis={along_axis}, cross_axis={cross_axis}")
    for i in strongest:
        print(f"  t={times[i]:6.3f}s cross_step_150ms={cross_step[i]:8.3f}px cross={cross_smooth[i]:8.3f}px")
    print(plot_path.resolve())
    print(csv_path.resolve())


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("videos", nargs="+", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    for video in args.videos:
        analyze(video, args.output_dir)


if __name__ == "__main__":
    main()

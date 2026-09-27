from __future__ import annotations

import argparse
import csv
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np


REFERENCE_POLYGONS = {
    "1789266678601": np.array(
        [[58, 245], [175, 225], [255, 285], [315, 455], [270, 555], [105, 565], [45, 470], [45, 320]],
        dtype=np.int32,
    ),
    "1789266678602": np.array(
        [[38, 8], [125, 5], [165, 48], [158, 155], [115, 190], [42, 170], [28, 55]],
        dtype=np.int32,
    ),
}


def rolling_median(values: np.ndarray, radius: int) -> np.ndarray:
    result = np.empty_like(values, dtype=float)
    for i in range(len(values)):
        lo = max(0, i - radius)
        hi = min(len(values), i + radius + 1)
        result[i] = np.nanmedian(values[lo:hi])
    return result


def analyze(video_path: Path, output_dir: Path) -> None:
    key = next((prefix for prefix in REFERENCE_POLYGONS if video_path.stem.startswith(prefix)), None)
    if key is None:
        raise RuntimeError(f"No reference polygon configured for {video_path.name}")
    polygon = REFERENCE_POLYGONS[key]
    cap = cv2.VideoCapture(str(video_path))
    fps = float(cap.get(cv2.CAP_PROP_FPS))
    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    ok, reference_full = cap.read()
    if not ok:
        raise RuntimeError(f"Cannot read {video_path}")
    reference = cv2.resize(reference_full, (360, 640), interpolation=cv2.INTER_AREA)
    mask = np.zeros(reference.shape[:2], dtype=np.uint8)
    cv2.fillPoly(mask, [polygon], 255)
    sift = cv2.SIFT_create(nfeatures=1200, contrastThreshold=0.025)
    ref_keypoints, ref_descriptors = sift.detectAndCompute(reference, mask)
    if ref_descriptors is None or len(ref_keypoints) < 12:
        raise RuntimeError("Not enough reference features")
    matcher = cv2.BFMatcher(cv2.NORM_L2)
    center_ref = np.mean(polygon, axis=0).astype(np.float32).reshape(1, 1, 2)
    axis_ref = np.array([[[center_ref[0, 0, 0] - 30, center_ref[0, 0, 1]], [center_ref[0, 0, 0] + 30, center_ref[0, 0, 1]]]], dtype=np.float32)

    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
    step = 2
    observations: list[tuple[int, float, float, float, int]] = []
    previous_center: np.ndarray | None = None
    frame_index = 0
    while frame_index < frame_count:
        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
        ok, frame_full = cap.read()
        if not ok:
            break
        frame = cv2.resize(frame_full, (360, 640), interpolation=cv2.INTER_AREA)
        keypoints, descriptors = sift.detectAndCompute(frame, None)
        accepted = False
        if descriptors is not None:
            pairs = matcher.knnMatch(ref_descriptors, descriptors, k=2)
            good = [first for first, second in pairs if first.distance < 0.70 * second.distance]
            if len(good) >= 8:
                src = np.float32([ref_keypoints[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
                dst = np.float32([keypoints[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
                transform, inlier_mask = cv2.findHomography(src, dst, cv2.RANSAC, 4.0)
                inliers = int(inlier_mask.sum()) if inlier_mask is not None else 0
                if transform is not None and inliers >= 7:
                    center = cv2.perspectiveTransform(center_ref, transform)[0, 0]
                    axis = cv2.perspectiveTransform(axis_ref, transform)[0]
                    heading = float(np.degrees(np.arctan2(axis[1, 1] - axis[0, 1], axis[1, 0] - axis[0, 0])))
                    if (
                        -40 <= center[0] <= 400
                        and -60 <= center[1] <= 700
                        and (previous_center is None or np.linalg.norm(center - previous_center) < 55)
                    ):
                        observations.append((frame_index, float(center[0]), float(center[1]), heading, inliers))
                        previous_center = center
                        accepted = True
        if not accepted:
            observations.append((frame_index, np.nan, np.nan, np.nan, 0))
        frame_index += step
    cap.release()

    frames = np.array([item[0] for item in observations])
    x = np.array([item[1] for item in observations])
    y = np.array([item[2] for item in observations])
    heading = np.array([item[3] for item in observations])
    inliers = np.array([item[4] for item in observations])
    valid = np.isfinite(x) & np.isfinite(y)
    if valid.sum() < 20:
        raise RuntimeError(f"Only {valid.sum()} valid SIFT positions for {video_path.name}")
    x = np.interp(frames, frames[valid], x[valid])
    y = np.interp(frames, frames[valid], y[valid])
    heading_valid = np.isfinite(heading)
    heading = np.interp(frames, frames[heading_valid], heading[heading_valid])

    points = np.column_stack((x, y))
    centered = points - np.mean(points, axis=0)
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    along_axis, cross_axis = vh[0], vh[1]
    along = centered @ along_axis
    cross = centered @ cross_axis
    cross = rolling_median(cross, 2)
    heading = rolling_median(heading, 2)
    window = max(1, round(fps * 0.15 / step))
    cross_step = np.full_like(cross, np.nan)
    heading_step = np.full_like(heading, np.nan)
    cross_step[window:] = cross[window:] - cross[:-window]
    heading_step[window:] = heading[window:] - heading[:-window]
    times = frames / fps

    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / f"{video_path.stem}_sift.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["frame", "time_s", "x_px", "y_px", "along_px", "cross_px", "cross_step_150ms_px", "heading_deg", "heading_step_150ms_deg", "inliers"])
        for row in zip(frames, times, x, y, along, cross, cross_step, heading, heading_step, inliers):
            writer.writerow(row)

    figure, axes = plt.subplots(1, 3, figsize=(15, 4.5))
    axes[0].plot(x, y, linewidth=1.2)
    axes[0].scatter(x[::15], y[::15], c=times[::15], cmap="viridis", s=14)
    axes[0].invert_yaxis()
    axes[0].set_aspect("equal", adjustable="datalim")
    axes[0].set_title("SIFT robot-center trajectory")
    axes[0].grid(True, alpha=0.25)
    axes[1].plot(times, cross, label="cross-track")
    axes[1].plot(times, cross_step, label="150 ms step", alpha=0.75)
    axes[1].set_title("Perpendicular motion")
    axes[1].grid(True, alpha=0.25)
    axes[1].legend()
    axes[2].plot(times, heading, label="image heading")
    axes[2].plot(times, heading_step, label="150 ms step", alpha=0.75)
    axes[2].set_title("Body angle from visual features")
    axes[2].grid(True, alpha=0.25)
    axes[2].legend()
    for axis in axes[1:]:
        axis.set_xlabel("time (s)")
    figure.tight_layout()
    plot_path = output_dir / f"{video_path.stem}_sift.png"
    figure.savefig(plot_path, dpi=150)
    plt.close(figure)

    finite = np.flatnonzero(np.isfinite(cross_step))
    strongest = finite[np.argsort(np.abs(cross_step[finite]))[-10:]][::-1]
    print(f"\n{video_path.name}: reference_features={len(ref_keypoints)}, valid={valid.sum()}/{len(valid)}")
    print(f"path_axis={along_axis}, cross_axis={cross_axis}")
    for i in strongest:
        print(
            f"t={times[i]:6.3f}s cross_step={cross_step[i]:7.2f}px "
            f"heading_step={heading_step[i]:7.2f}deg inliers={inliers[i]}"
        )
    print(plot_path.resolve())


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("videos", nargs="+", type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()
    for video in args.videos:
        analyze(video, args.output_dir)


if __name__ == "__main__":
    main()

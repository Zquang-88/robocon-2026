from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


def make_contact_sheet(video_path: Path, output_dir: Path, samples: int = 12) -> None:
    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        raise RuntimeError(f"Cannot open {video_path}")

    fps = float(capture.get(cv2.CAP_PROP_FPS))
    frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    duration = frame_count / fps if fps > 0 else 0.0
    print(
        f"{video_path.name}: {width}x{height}, {fps:.3f} fps, "
        f"{frame_count} frames, {duration:.3f} s"
    )

    indexes = np.linspace(0, max(frame_count - 1, 0), samples, dtype=int)
    tiles: list[np.ndarray] = []
    tile_width = 360
    tile_height = 240
    for index in indexes:
        capture.set(cv2.CAP_PROP_POS_FRAMES, int(index))
        ok, frame = capture.read()
        if not ok:
            continue
        scale = min(tile_width / frame.shape[1], tile_height / frame.shape[0])
        resized = cv2.resize(
            frame,
            (max(1, int(frame.shape[1] * scale)), max(1, int(frame.shape[0] * scale))),
            interpolation=cv2.INTER_AREA,
        )
        tile = np.full((tile_height, tile_width, 3), 20, dtype=np.uint8)
        y = (tile_height - resized.shape[0]) // 2
        x = (tile_width - resized.shape[1]) // 2
        tile[y : y + resized.shape[0], x : x + resized.shape[1]] = resized
        timestamp = index / fps if fps > 0 else 0.0
        cv2.putText(
            tile,
            f"{timestamp:.2f}s  frame {index}",
            (8, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )
        tiles.append(tile)

    capture.release()
    if not tiles:
        raise RuntimeError(f"No frames decoded from {video_path}")
    while len(tiles) % 4:
        tiles.append(np.zeros_like(tiles[0]))
    rows = [np.hstack(tiles[i : i + 4]) for i in range(0, len(tiles), 4)]
    sheet = np.vstack(rows)
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / f"{video_path.stem}_contact.jpg"
    if not cv2.imwrite(str(output_path), sheet, [cv2.IMWRITE_JPEG_QUALITY, 92]):
        raise RuntimeError(f"Cannot write {output_path}")
    print(output_path.resolve())


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("videos", nargs="+", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    for video in args.videos:
        make_contact_sheet(video, args.output_dir)


if __name__ == "__main__":
    main()

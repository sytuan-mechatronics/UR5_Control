"""Rebuild multi-sample tray reference from annotated capture images.

Assumptions:
- Each annotated image contains 5 green slot markers + green numbers
- Existing `tray_slot_reference.json` contains at least one valid sample
  that defines canonical slot names/order
- We recover marker centers by choosing the 5 green components whose geometry
  best matches the canonical slot layout
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import List

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from vision.tray_slot_reference import append_tray_slot_reference_sample, load_tray_slot_reference


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Rebuild tray slot reference from annotated images")
    parser.add_argument(
        "--captures-dir",
        default="captures/tray_reference",
        help="Directory containing tray_reference_annotated_*.jpg",
    )
    parser.add_argument(
        "--output",
        default="tray_slot_reference.json",
        help="Output tray slot reference JSON path",
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=5,
        help="How many latest annotated images to use",
    )
    return parser.parse_args()


def detect_green_candidates(image_path: Path) -> List[np.ndarray]:
    image = cv2.imread(str(image_path))
    if image is None:
        raise RuntimeError(f"Khong doc duoc anh: {image_path}")

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower = np.array([35, 80, 80], dtype=np.uint8)
    upper = np.array([90, 255, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower, upper)
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)
    candidates: List[np.ndarray] = []
    for idx in range(1, num_labels):
        area = int(stats[idx, cv2.CC_STAT_AREA])
        if area < 120:
            continue
        candidates.append(np.array([float(centroids[idx][0]), float(centroids[idx][1])], dtype=np.float64))
    return candidates


def estimate_similarity_transform(src_pts: np.ndarray, dst_pts: np.ndarray):
    src_mean = src_pts.mean(axis=0)
    dst_mean = dst_pts.mean(axis=0)
    src_centered = src_pts - src_mean
    dst_centered = dst_pts - dst_mean
    src_var = np.sum(src_centered ** 2) / src_pts.shape[0]
    if src_var <= 1e-9:
        return None
    cov = (dst_centered.T @ src_centered) / src_pts.shape[0]
    u, singular_values, vh = np.linalg.svd(cov)
    s_mat = np.eye(2)
    if np.linalg.det(u) * np.linalg.det(vh) < 0:
        s_mat[-1, -1] = -1.0
    rotation = u @ s_mat @ vh
    scale = float(np.trace(np.diag(singular_values) @ s_mat) / src_var)
    translation = dst_mean - scale * (rotation @ src_mean)
    return scale, rotation, translation


def apply_similarity(points: np.ndarray, scale: float, rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    return (scale * (rotation @ points.T)).T + translation


def best_match_components(canonical_pts: np.ndarray, candidates: List[np.ndarray]):
    import itertools

    best = None
    cand_pts = np.array(candidates, dtype=np.float64)
    n = canonical_pts.shape[0]
    for chosen_idx in itertools.combinations(range(len(candidates)), n):
        subset = cand_pts[list(chosen_idx)]
        for perm in itertools.permutations(range(n)):
            ordered_subset = subset[list(perm)]
            estimate = estimate_similarity_transform(canonical_pts, ordered_subset)
            if estimate is None:
                continue
            scale, rotation, translation = estimate
            projected = apply_similarity(canonical_pts, scale, rotation, translation)
            errors = np.linalg.norm(projected - ordered_subset, axis=1)
            score = float(np.mean(errors))
            candidate = {
                "chosen_idx": chosen_idx,
                "perm": perm,
                "ordered_subset": ordered_subset,
                "mean_error": score,
                "max_error": float(np.max(errors)),
            }
            if best is None or score < best["mean_error"]:
                best = candidate
    return best


def main() -> int:
    args = parse_args()
    reference = load_tray_slot_reference()
    if not reference.get("slots"):
        raise RuntimeError("tray_slot_reference.json hien tai khong co mau chuan de suy ra ten slot")

    canonical_slots = reference["slots"]
    canonical_pts = np.array([[float(s["u"]), float(s["v"])] for s in canonical_slots], dtype=np.float64)

    captures_dir = ROOT / args.captures_dir
    annotated_paths = sorted(captures_dir.glob("tray_reference_annotated_*.jpg"))[-args.limit:]
    if len(annotated_paths) < 1:
        raise RuntimeError(f"Khong tim thay anh annotated trong {captures_dir}")

    output_path = ROOT / args.output
    if output_path.exists():
        output_path.unlink()

    sample_count = 0
    for sample_index, annotated_path in enumerate(annotated_paths, start=1):
        candidates = detect_green_candidates(annotated_path)
        if len(candidates) < len(canonical_slots):
            print(f"Bo qua {annotated_path.name}: chi co {len(candidates)} candidates")
            continue

        best = best_match_components(canonical_pts, candidates)
        if best is None:
            print(f"Bo qua {annotated_path.name}: khong match duoc")
            continue

        raw_name = annotated_path.name.replace("tray_reference_annotated_", "tray_reference_")
        raw_path = annotated_path.with_name(raw_name)
        slots = []
        for slot_idx, canonical in enumerate(canonical_slots):
            point = best["ordered_subset"][slot_idx]
            slots.append({"name": canonical["name"], "u": float(point[0]), "v": float(point[1])})

        append_tray_slot_reference_sample(
            slots=slots,
            image_path=str(raw_path if raw_path.exists() else annotated_path),
            image_width=1920,
            image_height=1080,
            sample_name=f"sample_{sample_index}",
            path=output_path,
        )
        sample_count += 1
        print(
            f"{annotated_path.name} -> sample_{sample_index} "
            f"(mean_err={best['mean_error']:.2f}px, max_err={best['max_error']:.2f}px)"
        )

    print(f"Da tao {sample_count} mau -> {output_path}")
    print(json.dumps(load_tray_slot_reference(), ensure_ascii=True, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

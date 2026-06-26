"""Reference-image based tray slot matching with multi-sample support.

Design goals:
- Support one or many tray reference samples
- Keep backward compatibility with the older single-sample JSON format
- Compare runtime detections only against samples captured at SCAN_POSE
- Choose the best-matching sample by geometric error
"""

from __future__ import annotations

import itertools
import json
import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

import config
from vision.single_slot_reference import match_single_slot_target
from vision.calibration import normalize_slot_name


_CACHE = {
    "path": None,
    "mtime_ns": None,
    "data": None,
}


def _reference_file() -> Path:
    raw_path = str(getattr(config, "TRAY_SLOT_REFERENCE_PATH", "tray_slot_reference.json")).strip()
    path = Path(raw_path)
    if not path.is_absolute():
        path = Path(__file__).resolve().parents[1] / raw_path
    return path


def _normalize_slots(raw_slots: List[Dict]) -> List[Dict]:
    slots = []
    for idx, slot in enumerate(raw_slots or [], start=1):
        try:
            slots.append(
                {
                    "name": normalize_slot_name(slot.get("name", f"slot_{idx}")),
                    "u": float(slot["u"]),
                    "v": float(slot["v"]),
                }
            )
        except Exception:
            continue
    return slots


def _normalize_sample(raw_sample: Dict, fallback_name: str) -> Optional[Dict]:
    slots = _normalize_slots(raw_sample.get("slots", []))
    if not slots:
        return None

    scan_pose_joints = None
    raw_joints = raw_sample.get("scan_pose_joints")
    if isinstance(raw_joints, list) and len(raw_joints) == 6:
        try:
            scan_pose_joints = [float(v) for v in raw_joints]
        except Exception:
            scan_pose_joints = None

    scan_pose_tcp = None
    raw_tcp = raw_sample.get("scan_pose_tcp")
    if isinstance(raw_tcp, list) and len(raw_tcp) == 6:
        try:
            scan_pose_tcp = [float(v) for v in raw_tcp]
        except Exception:
            scan_pose_tcp = None

    return {
        "name": str(raw_sample.get("name") or fallback_name),
        "image_path": str(raw_sample.get("image_path", "")),
        "image_width": int(raw_sample.get("image_width", 0) or 0),
        "image_height": int(raw_sample.get("image_height", 0) or 0),
        "slots": slots,
        "scan_pose_joints": scan_pose_joints,
        "scan_pose_tcp": scan_pose_tcp,
        "meta": raw_sample.get("meta", {}),
    }


def _load_reference_from_path(path: Path, use_cache: bool) -> Dict:
    if not path.exists():
        return {"enabled": False, "reason": f"missing:{path.name}", "samples": [], "slots": []}

    stat = path.stat()
    cache_key = str(path)
    if use_cache:
        if (
            _CACHE["path"] == cache_key
            and _CACHE["mtime_ns"] == stat.st_mtime_ns
            and _CACHE["data"] is not None
        ):
            return _CACHE["data"]

    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        data = {"enabled": False, "reason": f"json_error:{exc}", "samples": [], "slots": []}
        if use_cache:
            _CACHE.update({"path": cache_key, "mtime_ns": stat.st_mtime_ns, "data": data})
        return data

    samples = []
    raw_samples = raw.get("samples")
    if isinstance(raw_samples, list) and raw_samples:
        for idx, sample in enumerate(raw_samples, start=1):
            normalized = _normalize_sample(sample, fallback_name=f"sample_{idx}")
            if normalized is not None:
                samples.append(normalized)
    else:
        # Backward compatibility: old format stored a single top-level sample.
        legacy = _normalize_sample(
            {
                "name": raw.get("name", "sample_1"),
                "image_path": raw.get("image_path", ""),
                "image_width": raw.get("image_width", 0),
                "image_height": raw.get("image_height", 0),
                "slots": raw.get("slots", []),
                "scan_pose_joints": raw.get("scan_pose_joints"),
                "scan_pose_tcp": raw.get("scan_pose_tcp"),
                "meta": raw.get("meta", {}),
            },
            fallback_name="sample_1",
        )
        if legacy is not None:
            samples.append(legacy)

    data = {
        "enabled": bool(samples) and bool(getattr(config, "TRAY_SLOT_REFERENCE_ENABLED", False)),
        "reason": "ok" if samples else "no_samples",
        "path": str(path),
        "samples": samples,
        # Legacy convenience fields retained for callers that only inspect one sample.
        "slots": samples[0]["slots"] if samples else [],
        "image_path": samples[0]["image_path"] if samples else "",
        "image_width": samples[0]["image_width"] if samples else 0,
        "image_height": samples[0]["image_height"] if samples else 0,
        "meta": raw.get("meta", {}),
    }
    if use_cache:
        _CACHE.update({"path": cache_key, "mtime_ns": stat.st_mtime_ns, "data": data})
    return data


def load_tray_slot_reference() -> Dict:
    return _load_reference_from_path(_reference_file(), use_cache=True)


def append_tray_slot_reference_sample(
    slots: List[Dict[str, float]],
    image_path: str,
    image_width: int,
    image_height: int,
    sample_name: str = "",
    scan_pose_joints: Optional[List[float]] = None,
    scan_pose_tcp: Optional[List[float]] = None,
    path: Optional[Path] = None,
) -> Path:
    out_path = path or _reference_file()
    out_path.parent.mkdir(parents=True, exist_ok=True)

    existing = _load_reference_from_path(out_path, use_cache=False) if out_path.exists() else {"samples": []}
    samples = list(existing.get("samples", []))
    sample_index = len(samples) + 1
    sample_name = (sample_name or f"sample_{sample_index}").strip()

    samples.append(
        {
            "name": sample_name,
            "image_path": image_path,
            "image_width": int(image_width),
            "image_height": int(image_height),
            "slots": [
                {
                    "name": normalize_slot_name(slot["name"]),
                    "u": round(float(slot["u"]), 3),
                    "v": round(float(slot["v"]), 3),
                }
                for slot in slots
            ],
            "scan_pose_joints": [round(float(v), 6) for v in (scan_pose_joints or [])] if scan_pose_joints else None,
            "scan_pose_tcp": [round(float(v), 6) for v in (scan_pose_tcp or [])] if scan_pose_tcp else None,
            "meta": {
                "slot_count": len(slots),
                "source": "register_tray_reference",
            },
        }
    )

    payload = {
        "meta": {
            "sample_count": len(samples),
            "format_version": 2,
            "source": "register_tray_reference",
        },
        "samples": samples,
    }
    out_path.write_text(json.dumps(payload, ensure_ascii=True, indent=2), encoding="utf-8")
    return out_path


def _estimate_similarity_transform(src_pts: np.ndarray, dst_pts: np.ndarray) -> Optional[Tuple[float, np.ndarray, np.ndarray]]:
    """Estimate 2D similarity transform dst ~= s * R * src + t."""
    if src_pts.shape != dst_pts.shape or src_pts.shape[0] < 2:
        return None

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


def _apply_similarity(points: np.ndarray, scale: float, rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    return (scale * (rotation @ points.T)).T + translation


def _rotation_angle_deg(rotation: np.ndarray) -> float:
    return float(math.degrees(math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))))


def _build_match_meta(reason: str) -> Dict[str, object]:
    return {
        "enabled": bool(getattr(config, "TRAY_SLOT_REFERENCE_ENABLED", False)),
        "reason": reason,
        "assignments": {},
        "best_mean_error_px": None,
        "best_max_error_px": None,
        "used_scale": None,
        "sample_name": "",
        "sample_count": 0,
    }


def match_reference_slots(
    detections: List[object],
    preferred_sample_name: str = "",
    strict_sample: bool = False,
) -> Dict[str, object]:
    """Match runtime detections to slot IDs from the best available reference sample."""
    meta = _build_match_meta("disabled")

    if not getattr(config, "TRAY_SLOT_REFERENCE_ENABLED", False):
        return meta

    reference = load_tray_slot_reference()
    samples = reference.get("samples", [])
    meta["sample_count"] = len(samples)
    if not reference.get("enabled", False):
        meta["reason"] = reference.get("reason", "reference_unavailable")
        return meta

    preferred_sample_name = str(preferred_sample_name or "").strip()
    if preferred_sample_name:
        preferred = [sample for sample in samples if str(sample.get("name", "")).strip() == preferred_sample_name]
        if preferred:
            if strict_sample:
                samples = preferred
            else:
                sample_ids = {id(sample) for sample in preferred}
                samples = preferred + [sample for sample in samples if id(sample) not in sample_ids]
            meta["preferred_sample_name"] = preferred_sample_name
            meta["strict_sample"] = bool(strict_sample)
        else:
            meta["preferred_sample_name"] = preferred_sample_name
            meta["strict_sample"] = bool(strict_sample)
            meta["preferred_sample_missing"] = True

    detection_count = len(detections)
    min_detections = max(2, int(getattr(config, "TRAY_SLOT_MATCH_MIN_DETECTIONS", 2)))
    if detection_count == 1:
        det_u = float(detections[0].center[0])
        det_v = float(detections[0].center[1])
        if preferred_sample_name and samples:
            sample = samples[0]
            sample_candidates = []
            for slot in sample.get("slots", []):
                dist_px = float(np.hypot(det_u - float(slot["u"]), det_v - float(slot["v"])))
                sample_candidates.append(
                    {
                        "sample_name": sample["name"],
                        "slot_name": slot["name"],
                        "dist_px": dist_px,
                    }
                )
            sample_candidates.sort(key=lambda item: item["dist_px"])
            if sample_candidates:
                best_single = sample_candidates[0]
                second_best = sample_candidates[1] if len(sample_candidates) > 1 else None
                max_allowed_single = float(getattr(config, "TRAY_SLOT_SINGLE_MATCH_MAX_ERROR_PX", 120.0))
                if best_single["dist_px"] <= max_allowed_single:
                    assignments = {
                        0: {
                            "slot_name": best_single["slot_name"],
                            "error_px": round(best_single["dist_px"], 3),
                        }
                    }
                    meta.update(
                        {
                            "reason": "single_detection_preferred_sample",
                            "assignments": assignments,
                            "best_mean_error_px": round(best_single["dist_px"], 3),
                            "best_max_error_px": round(best_single["dist_px"], 3),
                            "sample_name": best_single["sample_name"],
                            "used_scale": None,
                        }
                    )
                    if second_best is not None:
                        meta["second_best_slot"] = second_best["slot_name"]
                        meta["second_best_error_px"] = round(second_best["dist_px"], 3)
                    return meta
                if strict_sample:
                    meta["reason"] = "single_detection_preferred_sample_outside_radius"
                    meta["best_mean_error_px"] = round(best_single["dist_px"], 3)
                    meta["best_max_error_px"] = round(best_single["dist_px"], 3)
                    meta["sample_name"] = best_single["sample_name"]
                    if second_best is not None:
                        meta["second_best_slot"] = second_best["slot_name"]
                        meta["second_best_error_px"] = round(second_best["dist_px"], 3)
                    return meta

        if not preferred_sample_name:
            single_slot_meta = match_single_slot_target(det_u, det_v)
            if single_slot_meta.get("reason") == "ok":
                assignments = {
                    0: {
                        "slot_name": single_slot_meta["slot_name"],
                        "error_px": single_slot_meta["error_px"],
                    }
                }
                meta.update(
                    {
                        "reason": "single_slot_reference",
                        "assignments": assignments,
                        "best_mean_error_px": single_slot_meta["error_px"],
                        "best_max_error_px": single_slot_meta["error_px"],
                        "sample_name": single_slot_meta["sample_name"],
                        "used_scale": None,
                        "second_best_slot": single_slot_meta.get("second_best_slot", ""),
                        "second_best_error_px": single_slot_meta.get("second_best_error_px"),
                    }
                )
                return meta

        single_candidates = []
        for sample in samples:
            for slot in sample.get("slots", []):
                dist_px = float(np.hypot(det_u - float(slot["u"]), det_v - float(slot["v"])))
                single_candidates.append(
                    {
                        "sample_name": sample["name"],
                        "slot_name": slot["name"],
                        "dist_px": dist_px,
                    }
                )

        if not single_candidates:
            meta["reason"] = "no_single_detection_candidates"
            return meta

        # For each slot, keep its best distance over all reference samples.
        best_per_slot = {}
        for candidate in single_candidates:
            slot_name = candidate["slot_name"]
            existing = best_per_slot.get(slot_name)
            if existing is None or candidate["dist_px"] < existing["dist_px"]:
                best_per_slot[slot_name] = candidate

        ranked = sorted(best_per_slot.values(), key=lambda item: item["dist_px"])
        best_single = ranked[0]
        second_best = ranked[1] if len(ranked) > 1 else None
        max_allowed_single = float(getattr(config, "TRAY_SLOT_SINGLE_MATCH_MAX_ERROR_PX", 120.0))
        if best_single["dist_px"] > max_allowed_single:
            meta["reason"] = "single_detection_outside_radius"
            meta["best_mean_error_px"] = round(best_single["dist_px"], 3)
            meta["best_max_error_px"] = round(best_single["dist_px"], 3)
            meta["sample_name"] = best_single["sample_name"]
            return meta

        assignments = {
            0: {
                "slot_name": best_single["slot_name"],
                "error_px": round(best_single["dist_px"], 3),
            }
        }
        meta.update(
            {
                "reason": "single_detection_reference",
                "assignments": assignments,
                "best_mean_error_px": round(best_single["dist_px"], 3),
                "best_max_error_px": round(best_single["dist_px"], 3),
                "sample_name": best_single["sample_name"],
                "used_scale": None,
            }
        )
        if second_best is not None:
            meta["second_best_slot"] = second_best["slot_name"]
            meta["second_best_error_px"] = round(second_best["dist_px"], 3)
        return meta

    if detection_count < min_detections:
        meta["reason"] = f"need_at_least_{min_detections}_detections"
        return meta

    det_pts = np.array([[float(det.center[0]), float(det.center[1])] for det in detections], dtype=np.float64)
    scale_min = float(getattr(config, "TRAY_SLOT_MATCH_SCALE_MIN", 0.7))
    scale_max = float(getattr(config, "TRAY_SLOT_MATCH_SCALE_MAX", 1.3))
    max_rot_deg = max(0.0, float(getattr(config, "TRAY_SLOT_MATCH_MAX_ROT_DEG", 35.0)))
    best = None

    for sample in samples:
        slots = sample.get("slots", [])
        if detection_count > len(slots):
            continue

        slot_names = [slot["name"] for slot in slots]
        slot_pts = np.array([[float(slot["u"]), float(slot["v"])] for slot in slots], dtype=np.float64)

        for ref_indexes in itertools.combinations(range(len(slots)), detection_count):
            for perm_indexes in itertools.permutations(ref_indexes):
                ref_subset = slot_pts[list(perm_indexes)]
                estimate = _estimate_similarity_transform(ref_subset, det_pts)
                if estimate is None:
                    continue
                scale, rotation, translation = estimate
                if not (scale_min <= scale <= scale_max):
                    continue
                rotation_deg = _rotation_angle_deg(rotation)
                if abs(rotation_deg) > max_rot_deg:
                    continue

                projected = _apply_similarity(ref_subset, scale, rotation, translation)
                errors = np.linalg.norm(projected - det_pts, axis=1)
                mean_error = float(np.mean(errors))
                max_error = float(np.max(errors))
                candidate = {
                    "sample_name": sample["name"],
                    "slot_names": slot_names,
                    "perm_indexes": perm_indexes,
                    "scale": scale,
                    "rotation_deg": rotation_deg,
                    "mean_error": mean_error,
                    "max_error": max_error,
                    "errors": errors,
                }
                if best is None or mean_error < best["mean_error"]:
                    best = candidate

    if best is None:
        meta["reason"] = "no_valid_transform"
        return meta

    max_allowed = float(getattr(config, "TRAY_SLOT_MATCH_MAX_ERROR_PX", 80.0))
    if best["max_error"] > max_allowed:
        meta["reason"] = "match_error_too_large"
        meta["best_mean_error_px"] = round(best["mean_error"], 3)
        meta["best_max_error_px"] = round(best["max_error"], 3)
        meta["used_scale"] = round(best["scale"], 5)
        meta["rotation_deg"] = round(best["rotation_deg"], 3)
        meta["sample_name"] = best["sample_name"]
        return meta

    assignments = {}
    for det_index, ref_index in enumerate(best["perm_indexes"]):
        assignments[det_index] = {
            "slot_name": best["slot_names"][ref_index],
            "error_px": round(float(best["errors"][det_index]), 3),
        }

    meta.update(
        {
            "reason": "ok",
            "assignments": assignments,
            "best_mean_error_px": round(best["mean_error"], 3),
            "best_max_error_px": round(best["max_error"], 3),
            "used_scale": round(best["scale"], 5),
            "rotation_deg": round(best["rotation_deg"], 3),
            "sample_name": best["sample_name"],
        }
    )
    return meta


def resolve_selected_slot_for_target(
    detections: List[object],
    target: object,
    preferred_sample_name: str = "",
    strict_sample: bool = False,
) -> Tuple[str, Dict[str, object]]:
    """Return matched slot name for the selected target detection if available."""
    match_meta = match_reference_slots(
        detections,
        preferred_sample_name=preferred_sample_name,
        strict_sample=strict_sample,
    )
    if not match_meta.get("assignments"):
        return "", match_meta

    target_index = -1
    for idx, det in enumerate(detections):
        if det is target:
            target_index = idx
            break
    if target_index < 0:
        return "", match_meta

    slot_info = match_meta["assignments"].get(target_index) or {}
    tray_slot_name = str(slot_info.get("slot_name") or "")

    # Cross-check the chosen target against single-slot references using the
    # refined pick point. This is especially valuable when the multi-detection
    # geometric fit is plausible but swaps nearby slots due to annotation drift
    # or symmetric tray layouts.
    pick_point = getattr(target, "pick_point", None)
    if isinstance(pick_point, (list, tuple)) and len(pick_point) >= 2:
        single_meta = match_single_slot_target(float(pick_point[0]), float(pick_point[1]))
        match_meta["single_slot_meta"] = single_meta
        single_slot_name = str(single_meta.get("slot_name") or "")
        if single_meta.get("reason") == "ok" and single_slot_name:
            if not tray_slot_name:
                match_meta["reason"] = "single_slot_target_override_no_tray_match"
                match_meta["sample_name"] = single_meta.get("sample_name", match_meta.get("sample_name", ""))
                match_meta["best_mean_error_px"] = single_meta.get("error_px")
                match_meta["best_max_error_px"] = single_meta.get("error_px")
                match_meta["used_scale"] = None
                match_meta["rotation_deg"] = None
                match_meta["override_source"] = "single_slot_reference"
                return single_slot_name, match_meta

            if single_slot_name != tray_slot_name:
                match_meta["tray_slot_name"] = tray_slot_name
                match_meta["override_slot_name"] = single_slot_name
                match_meta["override_source"] = "single_slot_reference"
                match_meta["reason"] = "single_slot_target_override_conflict"
                return single_slot_name, match_meta

    return tray_slot_name, match_meta

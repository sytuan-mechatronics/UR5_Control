"""Single-part reference matching for per-slot identification.

Used when runtime sees only one detected part. Each sample belongs to a known
slot and stores the clicked/observed image position of that part at SCAN_POSE.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, List, Optional

import config
from vision.calibration import normalize_slot_name


_CACHE = {
    "path": None,
    "mtime_ns": None,
    "data": None,
}


def _reference_file() -> Path:
    raw_path = str(getattr(config, "SINGLE_SLOT_REFERENCE_PATH", "single_slot_reference.json")).strip()
    path = Path(raw_path)
    if not path.is_absolute():
        path = Path(__file__).resolve().parents[1] / raw_path
    return path


def _load_from_path(path: Path, use_cache: bool) -> Dict:
    if not path.exists():
        return {"enabled": False, "reason": f"missing:{path.name}", "samples": []}

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
        data = {"enabled": False, "reason": f"json_error:{exc}", "samples": []}
        if use_cache:
            _CACHE.update({"path": cache_key, "mtime_ns": stat.st_mtime_ns, "data": data})
        return data

    parsed = []
    for idx, sample in enumerate(raw.get("samples", []), start=1):
        try:
            parsed.append(
                {
                    "name": str(sample.get("name", f"sample_{idx}")),
                    "slot_name": normalize_slot_name(sample["slot_name"]),
                    "u": float(sample["u"]),
                    "v": float(sample["v"]),
                    "image_path": str(sample.get("image_path", "")),
                    "scan_pose_joints": sample.get("scan_pose_joints"),
                    "scan_pose_tcp": sample.get("scan_pose_tcp"),
                    "meta": sample.get("meta", {}),
                }
            )
        except Exception:
            continue

    data = {
        "enabled": bool(parsed) and bool(getattr(config, "SINGLE_SLOT_REFERENCE_ENABLED", False)),
        "reason": "ok" if parsed else "no_samples",
        "path": str(path),
        "samples": parsed,
        "meta": raw.get("meta", {}),
    }
    if use_cache:
        _CACHE.update({"path": cache_key, "mtime_ns": stat.st_mtime_ns, "data": data})
    return data


def load_single_slot_reference() -> Dict:
    return _load_from_path(_reference_file(), use_cache=True)


def append_single_slot_reference_sample(
    slot_name: str,
    u: float,
    v: float,
    image_path: str,
    sample_name: str = "",
    scan_pose_joints: Optional[List[float]] = None,
    scan_pose_tcp: Optional[List[float]] = None,
    path: Optional[Path] = None,
) -> Path:
    out_path = path or _reference_file()
    out_path.parent.mkdir(parents=True, exist_ok=True)

    existing = _load_from_path(out_path, use_cache=False) if out_path.exists() else {"samples": []}
    samples = list(existing.get("samples", []))
    sample_name = (sample_name or f"{slot_name}_{len(samples)+1}").strip()
    slot_name = normalize_slot_name(slot_name)
    samples.append(
        {
            "name": sample_name,
            "slot_name": slot_name,
            "u": round(float(u), 3),
            "v": round(float(v), 3),
            "image_path": image_path,
            "scan_pose_joints": [round(float(x), 6) for x in (scan_pose_joints or [])] if scan_pose_joints else None,
            "scan_pose_tcp": [round(float(x), 6) for x in (scan_pose_tcp or [])] if scan_pose_tcp else None,
            "meta": {"source": "register_single_slot_reference"},
        }
    )

    payload = {
        "meta": {
            "sample_count": len(samples),
            "format_version": 1,
            "source": "register_single_slot_reference",
        },
        "samples": samples,
    }
    out_path.write_text(json.dumps(payload, ensure_ascii=True, indent=2), encoding="utf-8")
    return out_path


def match_single_slot_target(u: float, v: float) -> Dict[str, object]:
    meta = {
        "enabled": bool(getattr(config, "SINGLE_SLOT_REFERENCE_ENABLED", False)),
        "reason": "disabled",
        "slot_name": "",
        "sample_name": "",
        "error_px": None,
        "second_best_slot": "",
        "second_best_error_px": None,
        "sample_count": 0,
    }
    if not getattr(config, "SINGLE_SLOT_REFERENCE_ENABLED", False):
        return meta

    reference = load_single_slot_reference()
    samples = reference.get("samples", [])
    meta["sample_count"] = len(samples)
    if not reference.get("enabled", False):
        meta["reason"] = reference.get("reason", "reference_unavailable")
        return meta

    by_slot: Dict[str, Dict[str, object]] = {}
    for sample in samples:
        dist_px = ((float(u) - sample["u"]) ** 2 + (float(v) - sample["v"]) ** 2) ** 0.5
        existing = by_slot.get(sample["slot_name"])
        candidate = {
            "slot_name": sample["slot_name"],
            "sample_name": sample["name"],
            "error_px": dist_px,
        }
        if existing is None or dist_px < existing["error_px"]:
            by_slot[sample["slot_name"]] = candidate

    if not by_slot:
        meta["reason"] = "no_slot_candidates"
        return meta

    ranked = sorted(by_slot.values(), key=lambda item: item["error_px"])
    best = ranked[0]
    second = ranked[1] if len(ranked) > 1 else None
    max_allowed = float(getattr(config, "SINGLE_SLOT_MATCH_MAX_ERROR_PX", 90.0))
    if best["error_px"] > max_allowed:
        meta["reason"] = "single_slot_outside_radius"
        meta["sample_name"] = best["sample_name"]
        meta["error_px"] = round(best["error_px"], 3)
        return meta

    meta.update(
        {
            "reason": "ok",
            "slot_name": best["slot_name"],
            "sample_name": best["sample_name"],
            "error_px": round(best["error_px"], 3),
        }
    )
    if second is not None:
        meta["second_best_slot"] = second["slot_name"]
        meta["second_best_error_px"] = round(second["error_px"], 3)
    return meta

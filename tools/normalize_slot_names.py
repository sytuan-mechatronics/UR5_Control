"""Normalize slot naming across JSON files to slot_1..slot_5."""

from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from vision.calibration import normalize_slot_name


def normalize_pick_correction_map(path: Path) -> bool:
    if not path.exists():
        return False
    raw = json.loads(path.read_text(encoding="utf-8"))
    changed = False
    for point in raw.get("points", []):
        name = point.get("name")
        normalized = normalize_slot_name(name)
        if normalized and normalized != name:
            point["name"] = normalized
            changed = True
    if changed:
        path.write_text(json.dumps(raw, ensure_ascii=True, indent=2), encoding="utf-8")
    return changed


def normalize_single_slot_reference(path: Path) -> bool:
    if not path.exists():
        return False
    raw = json.loads(path.read_text(encoding="utf-8"))
    changed = False
    for sample in raw.get("samples", []):
        slot_name = sample.get("slot_name")
        normalized = normalize_slot_name(slot_name)
        if normalized and normalized != slot_name:
            sample["slot_name"] = normalized
            changed = True
    if changed:
        path.write_text(json.dumps(raw, ensure_ascii=True, indent=2), encoding="utf-8")
    return changed


def main() -> int:
    changed_any = False
    changed_any |= normalize_pick_correction_map(ROOT / "pick_correction_map.json")
    changed_any |= normalize_single_slot_reference(ROOT / "single_slot_reference.json")
    print("normalized=" + ("true" if changed_any else "false"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

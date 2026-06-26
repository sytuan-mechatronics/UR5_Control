"""Migrate experiment CSV files to the latest headers.

This tool:
1. Backs up old CSV files to `.bak`.
2. Rewrites them with the newest headers from `experiment_report_logger`.
3. Preserves as much existing data as possible.
4. Injects UR5 speed columns from current config defaults.

It is intentionally tolerant of malformed historical rows.
"""

from __future__ import annotations

import csv
import shutil
import sys
from pathlib import Path
from typing import Dict, List, Tuple

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import config
from core.experiment_report_logger import (
    SCENARIO1_FIELDS,
    SCENARIO2_FIELDS,
    SCENARIO3_FIELDS,
)


def _speed_defaults() -> Dict[str, float]:
    return {
        "ur5_joint_speed_rad_s": config.JOINT_VEL,
        "ur5_linear_speed_m_s": config.LINEAR_VEL,
        "ur5_pick_approach_speed_m_s": config.PICK_APPROACH_VEL,
    }


def _backup(path: Path) -> Path:
    backup = path.with_suffix(path.suffix + ".bak")
    shutil.copy2(path, backup)
    return backup


def _read_loose_csv(path: Path) -> Tuple[List[str], List[List[str]]]:
    with path.open("r", encoding="utf-8", newline="") as fh:
        rows = list(csv.reader(fh))
    if not rows:
        return [], []
    return rows[0], rows[1:]


def _normalize_row(header: List[str], row: List[str], fields: List[str]) -> Dict[str, str]:
    data = {field: "" for field in fields}
    data.update({k: v for k, v in _speed_defaults().items() if k in data})

    for index, key in enumerate(header):
        if key in data and index < len(row):
            data[key] = row[index]

    return data


def _migrate(path: Path, fields: List[str]) -> str:
    if not path.exists():
        with path.open("w", encoding="utf-8", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=fields)
            writer.writeheader()
        return f"created empty {path.name}"

    backup = _backup(path)
    header, rows = _read_loose_csv(path)

    normalized: List[Dict[str, str]] = []
    for row in rows:
        if not any(cell.strip() for cell in row):
            continue
        normalized.append(_normalize_row(header, row, fields))

    with path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        writer.writerows(normalized)

    return f"migrated {path.name} ({len(normalized)} rows, backup={backup.name})"


def main() -> int:
    results_dir = ROOT / config.RESULTS_DIR
    results_dir.mkdir(parents=True, exist_ok=True)

    tasks = [
        (results_dir / Path(config.RESULTS_SCENARIO1_CSV).name, SCENARIO1_FIELDS),
        (results_dir / Path(config.RESULTS_SCENARIO2_CSV).name, SCENARIO2_FIELDS),
        (results_dir / Path(config.RESULTS_SCENARIO3_CSV).name, SCENARIO3_FIELDS),
    ]

    for path, fields in tasks:
        print(_migrate(path, fields))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

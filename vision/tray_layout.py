"""Tray layout utilities for fixed hole geometry on the tray."""

import json
from pathlib import Path
from typing import List, Dict, Optional


def save_tray_layout(
    output_path: Path,
    image_size: List[int],
    tray_corners_uv: Optional[List[List[float]]],
    holes_uv: List[List[float]],
    notes: str = "",
) -> None:
    """Save clicked tray model to JSON."""
    payload: Dict[str, object] = {
        "version": 1,
        "image_size": {
            "width": int(image_size[0]),
            "height": int(image_size[1]),
        },
        "tray_corners_uv": [
            {"id": idx + 1, "u": float(uv[0]), "v": float(uv[1])}
            for idx, uv in enumerate(tray_corners_uv or [])
        ],
        "holes_uv": [
            {"id": idx + 1, "u": float(uv[0]), "v": float(uv[1])}
            for idx, uv in enumerate(holes_uv)
        ],
        "notes": notes,
    }
    output_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def load_tray_layout(layout_path: Path) -> Dict[str, object]:
    """Load tray layout JSON."""
    return json.loads(layout_path.read_text(encoding="utf-8"))

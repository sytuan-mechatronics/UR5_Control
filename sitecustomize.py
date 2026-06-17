"""
Local Python startup hook for this repository.

Purpose:
- Add bundled Orbbec runtime directory to `sys.path`
- Preload `libOrbbecSDK` so direct `import pyorbbecsdk` works from repo root
"""

from __future__ import annotations

import ctypes
import os
import sys
from pathlib import Path


def _setup_orbbec_runtime():
    repo_root = Path(__file__).resolve().parent
    runtime_dir = repo_root / "vendor" / "orbbec_runtime" / "linux-x86_64"
    sdk_lib = runtime_dir / "libOrbbecSDK.so.2.7.6"

    if not runtime_dir.exists():
        return

    runtime_path = str(runtime_dir)
    if runtime_path not in sys.path:
        sys.path.insert(0, runtime_path)

    lib_dirs = [
        runtime_dir,
        runtime_dir / "extensions" / "depthengine",
        runtime_dir / "extensions" / "filters",
        runtime_dir / "extensions" / "frameprocessor",
        runtime_dir / "extensions" / "firmwareupdater",
    ]
    current_ld = os.environ.get("LD_LIBRARY_PATH", "")
    prepend = ":".join(str(p) for p in lib_dirs if p.exists())
    os.environ["LD_LIBRARY_PATH"] = f"{prepend}:{current_ld}" if current_ld else prepend

    if sdk_lib.exists():
        try:
            ctypes.CDLL(str(sdk_lib), mode=ctypes.RTLD_GLOBAL)
        except Exception:
            pass


_setup_orbbec_runtime()


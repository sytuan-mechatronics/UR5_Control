"""
Compatibility shim so project code and docs can consistently use `import ob`.

Load order:
1. Installed `pyorbbecsdk`
2. Bundled runtime from `vendor/orbbec_runtime/linux-x86_64`
"""

from __future__ import annotations

import ctypes
import importlib
import os
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent
RUNTIME_DIR = ROOT / "vendor" / "orbbec_runtime" / "linux-x86_64"
RUNTIME_EXT_DIRS = [
    RUNTIME_DIR / "extensions" / "depthengine",
    RUNTIME_DIR / "extensions" / "filters",
    RUNTIME_DIR / "extensions" / "frameprocessor",
    RUNTIME_DIR / "extensions" / "firmwareupdater",
]


def _prepend_env_path(var_name: str, paths: list[Path]) -> None:
    values = [str(path) for path in paths if path.exists()]
    if not values:
        return

    current = os.environ.get(var_name, "")
    current_parts = [part for part in current.split(os.pathsep) if part]
    merged = values + [part for part in current_parts if part not in values]
    os.environ[var_name] = os.pathsep.join(merged)


def _prepare_bundled_runtime() -> None:
    if not RUNTIME_DIR.exists():
        return

    if str(RUNTIME_DIR) not in sys.path:
        sys.path.insert(0, str(RUNTIME_DIR))

    _prepend_env_path("LD_LIBRARY_PATH", [RUNTIME_DIR, *RUNTIME_EXT_DIRS])

    shared_lib = RUNTIME_DIR / "libOrbbecSDK.so.2.7.6"
    if shared_lib.exists():
        try:
            ctypes.CDLL(str(shared_lib), mode=ctypes.RTLD_GLOBAL)
        except OSError:
            pass


def _load_binding():
    try:
        return importlib.import_module("pyorbbecsdk")
    except ImportError:
        _prepare_bundled_runtime()
        return importlib.import_module("pyorbbecsdk")


_binding = _load_binding()

for name in dir(_binding):
    if not name.startswith("_"):
        globals()[name] = getattr(_binding, name)

__all__ = [name for name in globals() if not name.startswith("_")]


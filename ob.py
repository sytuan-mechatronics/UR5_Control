"""
Compatibility shim for Orbbec Python binding.

Import order:
1. Try system-installed `ob`
2. Try system-installed `pyorbbecsdk`
3. Fallback to bundled runtime in `vendor/orbbec_runtime/linux-x86_64`
"""

from __future__ import annotations

import ctypes
import importlib
import importlib.util
import os
import sys
from pathlib import Path


def _load_from_system():
    try:
        return importlib.import_module("pyorbbecsdk")
    except Exception:
        return None


def _load_from_vendor():
    repo_root = Path(__file__).resolve().parent
    runtime_dir = repo_root / "vendor" / "orbbec_runtime" / "linux-x86_64"
    module_path = runtime_dir / "pyorbbecsdk.cpython-38-x86_64-linux-gnu.so"
    sdk_lib = runtime_dir / "libOrbbecSDK.so.2.7.6"

    if not module_path.exists():
        raise ImportError(f"Bundled Orbbec binding not found: {module_path}")

    # Best-effort: make dynamic loader aware of runtime tree.
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
        ctypes.CDLL(str(sdk_lib), mode=ctypes.RTLD_GLOBAL)

    spec = importlib.util.spec_from_file_location("pyorbbecsdk", module_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Cannot create spec for {module_path}")

    module = importlib.util.module_from_spec(spec)
    sys.modules["pyorbbecsdk"] = module
    spec.loader.exec_module(module)
    return module


_mod = _load_from_system()
if _mod is None:
    _mod = _load_from_vendor()

# Re-export everything from the real binding so `import ob` behaves like the SDK module.
globals().update(_mod.__dict__)


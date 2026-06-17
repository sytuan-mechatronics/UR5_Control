"""
Compatibility wrapper for bundled Orbbec Python extension.

This allows scripts that directly do `import pyorbbecsdk as ob`
to work from the repository root without separately installing the SDK.
"""

from __future__ import annotations

import ctypes
import importlib.util
import os
import sys
from pathlib import Path


repo_root = Path(__file__).resolve().parent
runtime_dir = repo_root / "vendor" / "orbbec_runtime" / "linux-x86_64"
module_path = runtime_dir / "pyorbbecsdk.cpython-38-x86_64-linux-gnu.so"
sdk_lib = runtime_dir / "libOrbbecSDK.so.2.7.6"

if not module_path.exists():
    raise ImportError(f"Bundled pyorbbecsdk not found: {module_path}")

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

spec = importlib.util.spec_from_file_location(__name__, module_path)
if spec is None or spec.loader is None:
    raise ImportError(f"Cannot load extension spec: {module_path}")

module = importlib.util.module_from_spec(spec)
sys.modules[__name__] = module
spec.loader.exec_module(module)


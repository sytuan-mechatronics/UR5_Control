# Orbbec Runtime Bundle (Linux x86_64)

This folder contains runtime binaries copied from local `pyorbbecsdk_repo` so teammates can clone the main repository and run without rebuilding Orbbec SDK.

## Included

- `linux-x86_64/pyorbbecsdk.cpython-38-x86_64-linux-gnu.so`
- `linux-x86_64/libOrbbecSDK.so*`
- `linux-x86_64/OrbbecSDKConfig.xml`
- `linux-x86_64/extensions/*`

## Usage

1. Keep this folder in the repository.
2. Set `LD_LIBRARY_PATH` to include:
   - `vendor/orbbec_runtime/linux-x86_64`
   - `vendor/orbbec_runtime/linux-x86_64/extensions/depthengine`
   - `vendor/orbbec_runtime/linux-x86_64/extensions/filters`
   - `vendor/orbbec_runtime/linux-x86_64/extensions/frameprocessor`
   - `vendor/orbbec_runtime/linux-x86_64/extensions/firmwareupdater`
3. Ensure Python version is compatible with `pyorbbecsdk.cpython-38-...so`.

Example:

```bash
export LD_LIBRARY_PATH="$PWD/vendor/orbbec_runtime/linux-x86_64:$PWD/vendor/orbbec_runtime/linux-x86_64/extensions/depthengine:$PWD/vendor/orbbec_runtime/linux-x86_64/extensions/filters:$PWD/vendor/orbbec_runtime/linux-x86_64/extensions/frameprocessor:$PWD/vendor/orbbec_runtime/linux-x86_64/extensions/firmwareupdater:$LD_LIBRARY_PATH"
```

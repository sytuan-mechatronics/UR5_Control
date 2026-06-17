# YOLO model for part detection

The production model file is required for vision phases.

Required path (default):
- models/phoi.pt

Important:
- The `.pt` model is not committed by default (`.gitignore` ignores `models/*.pt`).
- After cloning this repository, copy your model file into this folder.

Minimum check:

```bash
ls -lh models/phoi.pt
```

If the model is missing:
- Phase 1 can still run.
- Phase 2/3 will not detect parts correctly.

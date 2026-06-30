# Isaac-Tactile-LIBERO

This repository currently contains a Phase 1 mock/stub skeleton for the
Isaac-Tactile-LIBERO benchmark contract.

It is not a Lightwheel fork, does not connect to real Lightwheel hardware, and
does not run Isaac Sim physics yet. The current code is a lightweight Python API
for registry, schema, tactile-mode, task, and smoke-test wiring.

Quick checks:

```bash
python scripts/list_tasks.py
python scripts/smoke_test.py --task PegInsert --tactile force_wrench --episodes 1
pytest tests/
```

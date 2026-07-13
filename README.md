# Isaac-Tactile-LIBERO

This repository contains the Isaac-Tactile-LIBERO benchmark reconstruction
library. Its development simulator baseline is Isaac Sim 6.0.1 with Python
3.12. Mock, diagnostic, and runtime-smoke results remain distinct from formal
physical or benchmark evidence.

The explicit `isaacsim_fr3_press_button` backend loads the retained FR3 asset,
uses Isaac Sim 6 experimental Articulation, Contact Sensor, and RTX Camera APIs,
and preserves the public 7D action/observation contract. On the current driver,
RTX rendering uses GPU 0 while Contact uses CPU physics. Force vectors and
wrenches remain masked because only scalar force magnitude/raw impulses were
validated.

Quick checks:

```bash
python scripts/list_tasks.py
python scripts/smoke_test.py --task PegInsert --tactile force_wrench --episodes 1
python -m pytest -q
python scripts/check_isaacsim6_imports.py --deprecated-as-error
```

Installation and asset setup are documented in
[`docs/installation.md`](docs/installation.md) and
[`docs/asset_setup.md`](docs/asset_setup.md). Isaac Sim 5.1/Python 3.11 is
archived as a reference baseline under `requirements/archive/`.

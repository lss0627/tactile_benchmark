# Installation

## Active development baseline

```text
Python 3.12.x
Isaac Sim 6.0.1
isaacsim[all,extscache] == 6.0.1.0
Driver 550.144.03 / UNVALIDATED
CPU physics + MBP
GPU dynamics disabled
RTX rendering on GPU
```

Accept the NVIDIA EULA for local runs:

```bash
export OMNI_KIT_ACCEPT_EULA=YES
```

Use the pinned candidate/released files under `requirements/` to create the `isaac6` environment.

## Repository-only checks

```bash
python -m pip install -e '.[test]'
python -m pytest -q
python scripts/check_deprecated_isaac_imports.py
```

Use the repository clean-checkout runner for authoritative intentional future-RED classification.

## Runtime preflight

```bash
conda activate isaac6
python --version
python -c 'import isaacsim; print("Isaac Sim import OK")'
```

Follow `specs/001-benchmark-reconstruction/quickstart.md` for current G1 commands. Old G-1B commands are migration-history tools, not the active benchmark acceptance.

## Physics/rendering boundary

- Keep Contact validation on CPU physics/MBP.
- Keep GPU dynamics/native GPU Contact disabled.
- RTX rendering may use the installed RTX 4090.
- Do not change physics mode inside an evidence series.

## Archived reference

Python 3.11, Isaac Sim 5.1, Isaac Lab v2.3.2, retained assets, and historical evidence remain under reference/archive documentation. They are not a second maintained benchmark backend.

## Final release

The local driver is permitted for development but not NVIDIA reference validation. G6 requires a current reference/validated-driver rerun.

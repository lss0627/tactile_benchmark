# Installation

## Development baseline

- Python 3.12.x
- Isaac Sim 6.0.1 (`isaacsim[all,extscache]==6.0.1.0`)
- NVIDIA driver 550.144.03, recorded as `UNVALIDATED`
- GPU rendering on `cuda:0`; Contact Sensor physics on CPU

Accept the NVIDIA EULA non-interactively for local runs:

```bash
export OMNI_KIT_ACCEPT_EULA=YES
```

Create the candidate environment from
`requirements/candidates/isaac-sim-6.0.1-candidate.md` and verify it against
`requirements/candidates/lock-py312-isaacsim-6.0.1.txt`.

For repository-only tests:

```bash
python -m pip install -e '.[test]'
python -m pytest -q
python scripts/check_isaacsim6_imports.py --deprecated-as-error
```

For the real FR3 compatibility backend:

```bash
python scripts/run_isaacsim6_g1b.py \
  --config configs/backend/isaacsim_fr3_press_button.yaml \
  --cycles 100 --steps 500 \
  --output outputs/evidence/G-1B/repository-integration/report.json
```

The runner rejects GPU physics before native initialization with
`GPU_CONTACT_NATIVE_INSTABILITY`. This does not disable GPU RTX rendering.

## Archived reference baseline

Python 3.11, Isaac Sim 5.1, Isaac Lab v2.3.2, the retained assets, and historical
evidence remain reference-only inputs. They are documented under
`requirements/archive/` and are not a second maintained benchmark backend.

See `docs/asset_setup.md` for portable asset resolution.

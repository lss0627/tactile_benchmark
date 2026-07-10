# Quickstart: Isaac Sim 6.0.1 Development Baseline

G0 and the Isaac Sim 6.0.1 migration checkpoints are implemented. Commands below reproduce the
development/runtime-smoke baseline; they do **not** satisfy G1-G6 physical benchmark gates.

## 1. Select the feature

From the repository root:

```bash
export SPECIFY_FEATURE=001-benchmark-reconstruction
bash .specify/scripts/bash/check-prerequisites.sh --json --require-tasks --include-tasks
```

Expected: the command resolves `spec.md`, `plan.md`, `tasks.md`, and the available design documents
under `specs/001-benchmark-reconstruction/`.

## 2. Validate documentation and machine-readable contracts

```bash
rg -n "NEEDS CLARIFICATION|\[FEATURE\]|\[###-feature|TODO|TBD" \
  specs/001-benchmark-reconstruction
git diff --check -- .specify specs/001-benchmark-reconstruction
python -m json.tool \
  specs/001-benchmark-reconstruction/contracts/evidence-manifest.schema.json >/dev/null
python -m json.tool \
  specs/001-benchmark-reconstruction/contracts/gate-status.schema.json >/dev/null
```

Expected: the placeholder search returns no matches and every other command exits zero.

## 3. Review the implementation handoff

Read in this order:

1. [spec.md](./spec.md) — scope, user stories, FRs, and measurable success.
2. [research.md](./research.md) — audited deficiencies and chosen design boundaries.
3. [plan.md](./plan.md) — gate architecture and implementation phases.
4. [data-model.md](./data-model.md) and [contracts/](./contracts/) — normative state and API rules.
5. [tasks.md](./tasks.md) — dependency-ordered implementation units.
6. [implementation.md](./implementation.md) — execution protocol and stop rules.
7. [acceptance.md](./acceptance.md) — command/evidence matrix and current status.

## 4. Verify the promoted Python 3.12 baseline

```bash
python -m pip install --extra-index-url https://pypi.nvidia.com \
  -r requirements/lock-py312.txt
python -m pip install -e '.[test]'
python -m pytest -q
python scripts/check_isaacsim6_imports.py --deprecated-as-error
```

Set `OMNI_KIT_ACCEPT_EULA=YES` and configure assets as described in `docs/asset_setup.md`.

## 5. Reproduce migration checks

```bash
python scripts/check_clean_checkout.py --output outputs/evidence/G0/clean-checkout
python scripts/review_gate.py --gate G0 \
  --evidence outputs/evidence/G0/clean-checkout/manifest.json
python scripts/run_isaacsim6_g1b.py --cycles 100 --steps 500 \
  --output outputs/evidence/G-1B/repository-integration/report.json
```

The runtime config forces CPU physics for Contact and GPU rendering on `cuda:0`. A request for GPU
physics fails before native initialization with `GPU_CONTACT_NATIVE_INSTABILITY`.

## 6. Evidence handling

Generated results belong under an immutable run directory such as:

```text
outputs/evidence/<gate-id>/<run-id>/
├── manifest.json
├── command.log
├── report.json
└── artifacts/
```

Validate `manifest.json` against `contracts/evidence-manifest.schema.json`, store artifact hashes,
and update the canonical gate status only after review. Changing semantic code/config/assets makes
older evidence stale and returns the gate to `IN_PROGRESS`.

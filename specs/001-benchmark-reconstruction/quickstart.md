# Quickstart

## 1. Select the isolated worktree

```bash
cd /mnt/data/home/lss/.config/superpowers/worktrees/VLA-Adapter-Lightwheel/g1-press-button-safety
git status --short --branch
```

Preserve unrelated local changes. Evidence-producing runs require a clean committed tree.

## 2. Activate Isaac Sim 6

```bash
conda activate isaac6
export OMNI_KIT_ACCEPT_EULA=YES
python --version
```

Expected Python: `3.12.x`.

## 3. Run no-simulator preflight

```bash
python -m pytest -q
python scripts/check_deprecated_isaac_imports.py
python scripts/check_clean_checkout.py --help
```

The repository intentionally tracks a future-RED inventory; use the repository’s clean-checkout runner for the authoritative classification instead of interpreting raw failure count alone.

## 4. Inspect active acceptance

```bash
sed -n '1,260p' specs/001-benchmark-reconstruction/acceptance.md
sed -n '1,260p' specs/001-benchmark-reconstruction/tasks.md
```

Historical `g1-*` investigation files are reference material. The active G1 requirements are in `acceptance.md`.

## 5. Run a G1 pilot

After T016–T031 are GREEN:

```bash
python scripts/run_g1_press_button_benchmark.py \
  --config configs/tasks/press_button_physical.yaml \
  --robot-config configs/robots/fr3_press_button_safe.yaml \
  --mode pilot \
  --episodes 1 \
  --headless \
  --output outputs/evidence/G1/press-button-pilot-<commit>
```

The runner must use CPU physics/MBP and record `driver_validation=UNVALIDATED` on driver 550.144.03.

## 6. Run reset acceptance

```bash
python scripts/run_g1_press_button_benchmark.py \
  --config configs/tasks/press_button_physical.yaml \
  --robot-config configs/robots/fr3_press_button_safe.yaml \
  --mode resets \
  --reset-cycles 100 \
  --headless \
  --output outputs/evidence/G1/press-button-resets-<commit>
```

## 7. Run bounded rollout

```bash
python scripts/run_g1_press_button_benchmark.py \
  --config configs/tasks/press_button_physical.yaml \
  --robot-config configs/robots/fr3_press_button_safe.yaml \
  --mode rollout \
  --steps 500 \
  --capture-media \
  --headless \
  --output outputs/evidence/G1/press-button-rollout-<commit>
```

## 8. Run formal G1 episodes

Only after the pilot, reset, and rollout checks pass:

```bash
python scripts/run_g1_press_button_benchmark.py \
  --config configs/tasks/press_button_physical.yaml \
  --robot-config configs/robots/fr3_press_button_safe.yaml \
  --mode episodes \
  --episodes 10 \
  --capture-media \
  --headless \
  --output outputs/evidence/G1/press-button-final-<commit>
```

Do not discard failures or rerun into the same output directory.

## 9. Review evidence

```bash
sha256sum -c outputs/evidence/G1/press-button-final-<commit>/checksums.sha256
python scripts/review_gate.py \
  --gate G1 \
  --evidence outputs/evidence/G1/press-button-final-<commit>
```

G1 passes only when all G1-01 through G1-09 in `acceptance.md` pass.

## 10. Optional formal diagnostics

Full-robot sweep/GJK/cooked-shape runners are optional. Run them only with an explicit bounded diagnostic budget and a separate `runtime_smoke` output. Their failure does not block the benchmark runner and their success does not pass G1.

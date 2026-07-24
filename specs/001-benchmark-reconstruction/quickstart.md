# Quickstart

Commands below describe the target public workflow. A command becomes formal only after its Gate tasks are implemented and accepted.

## Repository preflight

```bash
conda activate isaac6
export OMNI_KIT_ACCEPT_EULA=YES
python --version
python -m pytest -q
python scripts/check_deprecated_isaac_imports.py
```

## List benchmark content

```bash
python scripts/list_tasks.py
python scripts/list_protocols.py
python scripts/list_policies.py
```

Paper-v1 expects four suites and 16 accepted tasks.

## Collect official or custom data

```bash
python scripts/collect_data.py \
  --suite precision \
  --task peg_insert \
  --num-episodes 1000 \
  --policy scripted \
  --num-envs 8 \
  --resume \
  --output datasets/runs/precision-peg-scripted
```

Validate and replay:

```bash
python scripts/validate_dataset.py \
  --dataset datasets/runs/precision-peg-scripted

python scripts/replay_demos.py \
  --dataset datasets/runs/precision-peg-scripted \
  --num-episodes 20
```

## Train offline

```bash
python scripts/train.py \
  --algo diffusion_policy \
  --suite precision \
  --tasks peg_insert usb_insert key_turn pin_socket \
  --modalities vision tactile proprio \
  --dataset configs/datasets/tactilibero_v1.yaml \
  --seed 1701 \
  --output outputs/training/dp-precision-s1701
```

Replace `diffusion_policy` with `bc`, `act`, `transformer`, or `univtac` under the same shared contract.

## Train or collect online

```bash
python scripts/train.py \
  --algo act \
  --data-regime online \
  --suite articulation \
  --modalities vision tactile proprio \
  --environment-steps 100000 \
  --seed 1701 \
  --output outputs/training/act-articulation-online-s1701
```

Online outputs remain a separate evaluation track.

## Evaluate generalization

```bash
python scripts/evaluate.py \
  --checkpoint outputs/training/dp-precision-s1701/best.ckpt \
  --benchmark configs/benchmark/tactilibero_v1.yaml \
  --protocol GP-01 \
  --seeds 1701 1702 1703 \
  --output outputs/evaluation/dp-precision-gp01
```

Use:

- `GP-01` for object/geometry;
- `GP-02` for contact/material/physics;
- `GP-03` for sensor/observation.

## Build leaderboard

```bash
python scripts/validate_submission.py \
  --bundle outputs/evaluation/dp-precision-gp01/submission

python scripts/build_leaderboard.py \
  --submissions outputs/evaluation/*/submission \
  --output release/leaderboard
```

Expected outputs include CSV, HTML, radar data, manifest, and checksums.

## Immediate repository milestone

Before formal multi-task collection, complete active G1:

```text
100 PressButton resets
500 rendered steps
10 consecutive task-state episodes
```

See `acceptance.md` and `tasks.md`.

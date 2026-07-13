# PressButton Runtime Dataset Evaluation

This stage verifies that the three-episode PressButton runtime-smoke HDF5
dataset can enter the existing offline evaluation, policy, and training sanity
chain. It does not start Isaac Sim, does not replay physics, and does not
produce benchmark or paper results.

## Scope

- Input dataset:
  `outputs/press_button_runtime_dataset_smoke/press_button_runtime_smoke_gpu1.hdf5`
- Dataset kind: `runtime_smoke`
- Backend metadata: `isaacsim_press_button`
- Task: `PressButton`
- Default policy sanity mode: `replay`
- Force source: `unavailable`
- Contact force availability: `false`

## Offline Evaluation

Run:

```bash
python scripts/evaluate_runtime_dataset.py \
  --dataset outputs/press_button_runtime_dataset_smoke/press_button_runtime_smoke_gpu1.hdf5 \
  --policy replay \
  --max-episodes 3 \
  --output outputs/press_button_runtime_dataset_eval/gpu1_replay_eval
```

The script writes:

- `metrics.json`
- `summary.csv`
- `dataset_eval_report.json`

The report checks action shape, observation schema, tactile schema, success-label
consistency, force/wrench zero-safety, `mask.has_force=false`, and
`no_fake_force_from_displacement=true`.

## StateBC Sanity

The runtime-smoke dataset can be inspected through the existing StateBC batch
builder:

```bash
python scripts/inspect_baseline_batch.py \
  --dataset outputs/press_button_runtime_dataset_smoke/press_button_runtime_smoke_gpu1.hdf5 \
  --policy state_bc \
  --max-episodes 3 \
  --output outputs/press_button_runtime_dataset_eval/state_bc_batch_inspect.json
```

Dry-run training may be used only as protocol validation:

```bash
python scripts/train_bc.py \
  --config configs/train/bc_mock.yaml \
  --dataset outputs/press_button_runtime_dataset_smoke/press_button_runtime_smoke_gpu1.hdf5 \
  --policy state_bc \
  --output outputs/press_button_runtime_dataset_eval/state_bc_dry_run_train \
  --dry-run
```

With only three runtime-smoke episodes, this remains insufficient for real
training claims.

## Non-Claims

- This is not a formal benchmark dataset.
- This is not a paper result.
- This is not physical replay.
- Force and wrench are unavailable; `mask.has_force=false` and
  `mask.has_wrench=false`.
- Success comes from the PressButton displacement hook, not a force-aware
  tactile sensor.
- Button displacement is never written into force or wrench arrays.

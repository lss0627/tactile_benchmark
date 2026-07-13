# PressButton Runtime Stability Gate 50 Episodes

This gate checks whether the single-task PressButton Isaac Sim runtime smoke can
collect 50 episodes on GPU1, close resources cleanly, write the existing HDF5
schema, and pass validation, replay consistency, offline dataset evaluation, and
StateBC dry-run protocol checks.

It is not a formal benchmark dataset, not a paper-results dataset, and not a
force-aware tactile benchmark.

## Scope

- Task: `PressButton`
- Backend: `isaacsim_press_button`
- Dataset kind: `runtime_smoke`
- Policy: `scripted`
- Tactile mode: `force_wrench`
- Episode count: 50
- Seeds: `0..49`
- Max steps: 80
- Runtime config: `outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml`
- Output: `outputs/press_button_runtime_dataset_smoke/press_button_runtime_smoke_50ep_gpu1.hdf5`

## Runtime Collection

Run after confirming GPU1 is available:

```bash
CUDA_VISIBLE_DEVICES=1 conda run -n isaac python scripts/collect_press_button_runtime_demos.py \
  --runtime-config outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml \
  --output outputs/press_button_runtime_dataset_smoke/press_button_runtime_smoke_50ep_gpu1.hdf5 \
  --num-episodes 50 \
  --seeds 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49 \
  --policy scripted \
  --tactile force_wrench \
  --max-steps 80 \
  --headless \
  --webrtc
```

If an episode fails, preserve the partial dataset and report it as partial. Do
not relabel a partial run as the complete 50-episode gate.

## Required Checks

Validate the dataset:

```bash
python scripts/validate_dataset.py \
  --dataset outputs/press_button_runtime_dataset_smoke/press_button_runtime_smoke_50ep_gpu1.hdf5 \
  --output outputs/press_button_runtime_dataset_smoke/validation_report_50ep_gpu1.json
```

Replay consistency without physics replay:

```bash
python scripts/replay_runtime_dataset.py \
  --dataset outputs/press_button_runtime_dataset_smoke/press_button_runtime_smoke_50ep_gpu1.hdf5 \
  --max-episodes 50 \
  --output outputs/press_button_runtime_dataset_smoke/replay_report_50ep_gpu1.json
```

Offline dataset evaluation:

```bash
python scripts/evaluate_runtime_dataset.py \
  --dataset outputs/press_button_runtime_dataset_smoke/press_button_runtime_smoke_50ep_gpu1.hdf5 \
  --policy replay \
  --max-episodes 50 \
  --output outputs/press_button_runtime_dataset_eval/gpu1_50ep_replay_eval
```

StateBC sanity checks remain batch inspection and dry-run only:

```bash
python scripts/inspect_baseline_batch.py \
  --dataset outputs/press_button_runtime_dataset_smoke/press_button_runtime_smoke_50ep_gpu1.hdf5 \
  --policy state_bc \
  --max-episodes 50 \
  --output outputs/press_button_runtime_dataset_eval/state_bc_50ep_batch_inspect.json

python scripts/train_bc.py \
  --config configs/train/bc_mock.yaml \
  --dataset outputs/press_button_runtime_dataset_smoke/press_button_runtime_smoke_50ep_gpu1.hdf5 \
  --policy state_bc \
  --output outputs/press_button_runtime_dataset_eval/state_bc_50ep_dry_run_train \
  --dry-run
```

## Non-Claims

- `benchmark_result=false`
- `not_for_paper_claims=true`
- Force and wrench remain unavailable.
- `mask.has_force=false` and `mask.has_wrench=false`.
- Success comes from the PressButton `button_displacement` hook.
- The pusher is still a placeholder, not a validated FR3/end-effector control
  stack.
- The optional `ee_placeholder` runtime path is a later smoke transition and
  does not change the 50-episode pusher stability dataset.
- Button displacement must never be written into force or wrench arrays.
- This gate checks runtime stability and schema plumbing only.

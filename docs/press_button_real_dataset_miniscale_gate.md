# PressButton Real Dataset Mini-Scale Gate

This gate expands the PressButton runtime-smoke collection from 3 episodes to
10 episodes to check multi-episode Isaac Sim runtime stability, GPU cleanup,
HDF5 writing, validation, replay consistency, offline evaluation, and StateBC
sanity plumbing.

It is not a formal benchmark dataset, not a released demonstration dataset, and
not a paper-results dataset.

## Scope

- Task: `PressButton`
- Backend: `isaacsim_press_button`
- Dataset kind: `runtime_smoke`
- Episodes: 10
- Seeds: `0 1 2 3 4 5 6 7 8 9`
- Policy: `scripted`
- Tactile mode: `force_wrench`
- Runtime config: `outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml`
- Output:
  `outputs/press_button_runtime_dataset_smoke/press_button_runtime_smoke_10ep_gpu1.hdf5`

The matching config file is
`configs/dataset/press_button_runtime_smoke_10ep.yaml`.

## Collection

```bash
CUDA_VISIBLE_DEVICES=1 conda run -n isaac python scripts/collect_press_button_runtime_demos.py \
  --runtime-config outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml \
  --output outputs/press_button_runtime_dataset_smoke/press_button_runtime_smoke_10ep_gpu1.hdf5 \
  --num-episodes 10 \
  --seeds 0 1 2 3 4 5 6 7 8 9 \
  --policy scripted \
  --tactile force_wrench \
  --max-steps 80 \
  --headless \
  --webrtc \
  --save-screenshots
```

GPU cleanup remains manual and conservative: inspect GPU1 first, list unknown
processes with `ps -fp <PID>`, and do not kill unconfirmed processes.

## Validation and Replay

```bash
python scripts/validate_dataset.py \
  --dataset outputs/press_button_runtime_dataset_smoke/press_button_runtime_smoke_10ep_gpu1.hdf5 \
  --output outputs/press_button_runtime_dataset_smoke/validation_report_10ep_gpu1.json

python scripts/replay_runtime_dataset.py \
  --dataset outputs/press_button_runtime_dataset_smoke/press_button_runtime_smoke_10ep_gpu1.hdf5 \
  --max-episodes 10 \
  --output outputs/press_button_runtime_dataset_smoke/replay_report_10ep_gpu1.json
```

Offline evaluation:

```bash
python scripts/evaluate_runtime_dataset.py \
  --dataset outputs/press_button_runtime_dataset_smoke/press_button_runtime_smoke_10ep_gpu1.hdf5 \
  --policy replay \
  --max-episodes 10 \
  --output outputs/press_button_runtime_dataset_eval/gpu1_10ep_replay_eval
```

StateBC sanity:

```bash
python scripts/inspect_baseline_batch.py \
  --dataset outputs/press_button_runtime_dataset_smoke/press_button_runtime_smoke_10ep_gpu1.hdf5 \
  --policy state_bc \
  --max-episodes 10 \
  --output outputs/press_button_runtime_dataset_eval/state_bc_10ep_batch_inspect.json

python scripts/train_bc.py \
  --config configs/train/bc_mock.yaml \
  --dataset outputs/press_button_runtime_dataset_smoke/press_button_runtime_smoke_10ep_gpu1.hdf5 \
  --policy state_bc \
  --output outputs/press_button_runtime_dataset_eval/state_bc_10ep_dry_run_train \
  --dry-run
```

StateBC dry-run outputs must keep `dry_run=true`, `is_trained=false`,
`mock_or_stub=true`, `dataset_kind=runtime_smoke`,
`insufficient_real_episodes=true`, `benchmark_result=false`, and
`not_for_paper_claims=true`.

## Tactile and Success Boundary

Force and wrench remain unavailable:

- `mask.has_force=false`
- `mask.has_wrench=false`
- `force_source=unavailable`
- force/wrench arrays are finite zeros
- `no_fake_force_from_displacement=true`

Success comes from the PressButton displacement hook. The pusher is still a
placeholder, and button displacement must not be reported as tactile force.

## Non-Claims

- The 10 episodes are a runtime-smoke stability gate, not a formal benchmark
  dataset.
- The dataset is not suitable for paper main results.
- It is not a force-aware tactile benchmark.
- It does not include full Lightwheel runtime integration.
- It does not expand the benchmark to 30 tasks.

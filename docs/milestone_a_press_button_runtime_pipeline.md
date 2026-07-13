# Milestone A: PressButton Single-Task Isaac Sim Runtime Pipeline

Status: PASS

Date: 2026-06-30

This document closes Milestone A for the Isaac-Tactile-LIBERO project. It is a
stage report only. It does not introduce new runtime behavior, does not connect
full FR3 articulation, does not expand the task suite, does not collect a formal
benchmark dataset, and does not claim paper-ready results.

## Completed Scope

Milestone A established the first end-to-end single-task runtime pipeline around
`PressButton`:

- Mock benchmark skeleton for 5 tasks and 4 tactile modes.
- Stable 7D action schema.
- Observation and tactile schema contracts, including tactile masks.
- HDF5 dataset writer, reader, validator, metadata, and replay helpers.
- ReplayPolicy and unified evaluation paths for mock and runtime-smoke data.
- Baseline policy skeletons and StateBC minimal training protocol.
- Isaac Sim WebRTC visual smoke preparation and runtime path.
- PressButton pusher runtime loop with `button_displacement` success hook.
- Unified `evaluate.py --backend isaacsim_press_button` entry point.
- PressButton runtime-smoke dataset collection, validation, replay, and offline
  dataset evaluation.
- 50-episode pusher runtime-smoke dataset on GPU1.
- 10-episode EE placeholder runtime-smoke dataset on GPU1.
- Runtime tactile schema mapping that keeps force and wrench unavailable when
  no real force source exists.
- No-fake-force validation: button displacement is not encoded as force or
  wrench.

## Key Reusable Assets

- Unified 7D action schema: `isaac_tactile_libero/schemas/action.py`
- Runtime tactile schema mapping:
  `isaac_tactile_libero/sensors/runtime_tactile_adapter.py`
- HDF5 runtime-smoke dataset schema:
  `isaac_tactile_libero/datasets/` and
  `isaac_tactile_libero/schemas/dataset.py`
- PressButton runtime loop:
  `isaac_tactile_libero/envs/isaacsim_press_button_env.py`
- Standalone runtime script:
  `scripts/run_press_button_runtime_loop.py`
- Unified eval backend:
  `scripts/evaluate.py --backend isaacsim_press_button`
- Runtime dataset collection:
  `scripts/collect_press_button_runtime_demos.py`
- Runtime dataset replay/eval:
  `scripts/replay_runtime_dataset.py` and
  `scripts/evaluate_runtime_dataset.py`
- ReplayPolicy and StateBC sanity path:
  `isaac_tactile_libero/policies/replay.py`,
  `isaac_tactile_libero/policies/batch_builder.py`, and
  `scripts/train_bc.py --dry-run`
- Asset and Lightwheel license/provenance gate:
  `assets/asset_manifest.csv`,
  `isaac_tactile_libero/assets/manifest.py`, and
  `isaac_tactile_libero/assets/provenance_gate.py`
- Isaac Sim WebRTC runtime path:
  `configs/backend/isaacsim_visual_smoke.yaml`,
  `scripts/check_isaacsim_webrtc_ready.py`, and
  `scripts/launch_isaacsim_webrtc_smoke.sh`

## Key Artifacts

### Runtime Configs

- Isaac Sim runtime config:
  `outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml`
- Mock eval config: `configs/eval/mock_default.yaml`
- PressButton runtime dataset eval config:
  `configs/eval/press_button_runtime_dataset_eval.yaml`
- Pusher 10-episode config:
  `configs/dataset/press_button_runtime_smoke_10ep.yaml`
- Pusher 50-episode config:
  `configs/dataset/press_button_runtime_smoke_50ep.yaml`
- EE placeholder 10-episode config:
  `configs/dataset/press_button_ee_placeholder_smoke_10ep.yaml`
- EE placeholder robot config:
  `configs/robots/fr3_ee_placeholder.yaml`

### Visual And Runtime Smoke

- Isaac Sim visual smoke screenshot:
  `outputs/isaacsim_visual_smoke/press_button_visual_smoke.png`
- Unified PressButton runtime screenshots and metrics examples:
  `outputs/eval_press_button_isaacsim_runtime/`,
  `outputs/eval_press_button_ee_placeholder_runtime/`
- Standalone runtime loop screenshot:
  `outputs/press_button_runtime_loop/press_button_runtime_loop.png`

### Pusher Runtime-Smoke Dataset

- HDF5:
  `outputs/press_button_runtime_dataset_smoke/press_button_runtime_smoke_50ep_gpu1.hdf5`
- Size: 132,221,208 bytes, about 126.10 MiB
- Episodes: 50
- Success rate: 1.0
- Mean steps: 41.0
- Failed episodes: none
- Force source: `unavailable`
- `mask.has_force=false` for all episodes
- `mask.has_wrench=false` for all episodes
- Success source: `button_displacement`

Validation:

- `outputs/press_button_runtime_dataset_smoke/validation_report_50ep_gpu1.json`
- `ok=true`
- `num_episodes=50`
- `runtime_smoke=true`
- `benchmark_result=false`
- `not_for_paper_claims=true`
- `force_unavailable_mask_ok=true`
- `force_wrench_zero_safe_ok=true`
- `no_fake_force_from_displacement=true`

Replay:

- `outputs/press_button_runtime_dataset_smoke/replay_report_50ep_gpu1.json`
- `ok=true`
- `num_replayed=50`
- `runtime_smoke=true`
- `benchmark_result=false`

Offline dataset evaluation:

- `outputs/press_button_runtime_dataset_eval/gpu1_50ep_replay_eval/metrics.json`
- `num_episodes=50`
- `success_rate=1.0`
- `mean_steps=41.0`
- `force_source=unavailable`
- `contact_force_available=false`
- `no_fake_force_from_displacement=true`
- `benchmark_result=false`

### EE Placeholder Runtime-Smoke Dataset

- HDF5:
  `outputs/press_button_ee_placeholder_dataset_smoke/press_button_ee_placeholder_10ep_gpu1.hdf5`
- Size: 26,466,768 bytes, about 25.24 MiB
- Episodes: 10
- Success rate: 1.0
- Mean steps: 41.0
- Failed episodes: none
- Robot mode: `ee_placeholder`
- `placeholder_robot=true`
- `real_fr3_articulation=false`
- Force source: `unavailable`
- `mask.has_force=false` for all episodes
- `mask.has_wrench=false` for all episodes
- Success source: `button_displacement`
- Screenshots:
  `outputs/press_button_ee_placeholder_dataset_smoke/runtime-smoke-PressButton-force_wrench-seed*-ep*.png`

Validation:

- `outputs/press_button_ee_placeholder_dataset_smoke/validation_report_10ep_gpu1.json`
- `ok=true`
- `num_episodes=10`
- `runtime_smoke=true`
- `robot_mode=ee_placeholder`
- `placeholder_robot=true`
- `real_fr3_articulation=false`
- `benchmark_result=false`
- `not_for_paper_claims=true`
- `force_unavailable_mask_ok=true`
- `force_wrench_zero_safe_ok=true`
- `no_fake_force_from_displacement=true`

Replay:

- `outputs/press_button_ee_placeholder_dataset_smoke/replay_report_10ep_gpu1.json`
- `ok=true`
- `num_replayed=10`
- all episode checks passed
- all actions are 7D

Offline dataset evaluation:

- `outputs/press_button_ee_placeholder_dataset_eval/gpu1_10ep_replay_eval/metrics.json`
- `num_episodes=10`
- `success_rate=1.0`
- `mean_steps=41.0`
- `robot_mode=ee_placeholder`
- `placeholder_robot=true`
- `real_fr3_articulation=false`
- `force_source=unavailable`
- `contact_force_available=false`
- `no_fake_force_from_displacement=true`
- `benchmark_result=false`

StateBC sanity:

- Batch inspect:
  `outputs/press_button_ee_placeholder_dataset_eval/state_bc_10ep_batch_inspect.json`
- Dry-run train summary:
  `outputs/press_button_ee_placeholder_dataset_eval/state_bc_10ep_dry_run_train/train_summary.json`
- `dataset_kind=runtime_smoke`
- `robot_mode=ee_placeholder`
- `dry_run=true`
- `is_trained=false`
- `mock_or_stub=true`
- `benchmark_result=false`

## Key Commands

Mock and regression:

```bash
python scripts/smoke_test.py --seeds 0 1 2
python scripts/evaluate.py --config configs/eval/mock_default.yaml --backend mock --policy random --output outputs/eval_mock_regression
```

Isaac Sim readiness and visual smoke:

```bash
python scripts/check_isaacsim_webrtc_ready.py \
  --config outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml \
  --output outputs/eval_press_button_isaacsim_runtime/readiness.json

CUDA_VISIBLE_DEVICES=1 conda run -n isaac python scripts/evaluate.py \
  --backend isaacsim_press_button \
  --task PressButton \
  --policy scripted \
  --runtime-config outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml \
  --robot-mode ee_placeholder \
  --robot-config configs/robots/fr3_ee_placeholder.yaml \
  --max-steps 80 \
  --headless \
  --webrtc \
  --save-screenshot \
  --save-rollout-json \
  --output outputs/eval_press_button_ee_placeholder_runtime
```

Pusher 50-episode runtime-smoke dataset:

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
  --webrtc \
  --save-screenshots
```

EE placeholder 10-episode runtime-smoke dataset:

```bash
CUDA_VISIBLE_DEVICES=1 conda run -n isaac python scripts/collect_press_button_runtime_demos.py \
  --runtime-config outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml \
  --output outputs/press_button_ee_placeholder_dataset_smoke/press_button_ee_placeholder_10ep_gpu1.hdf5 \
  --num-episodes 10 \
  --seeds 0 1 2 3 4 5 6 7 8 9 \
  --policy scripted \
  --tactile force_wrench \
  --robot-mode ee_placeholder \
  --robot-config configs/robots/fr3_ee_placeholder.yaml \
  --max-steps 80 \
  --headless \
  --webrtc \
  --save-screenshots
```

Validation, replay, and offline eval:

```bash
python scripts/validate_dataset.py \
  --dataset outputs/press_button_ee_placeholder_dataset_smoke/press_button_ee_placeholder_10ep_gpu1.hdf5 \
  --output outputs/press_button_ee_placeholder_dataset_smoke/validation_report_10ep_gpu1.json

python scripts/replay_runtime_dataset.py \
  --dataset outputs/press_button_ee_placeholder_dataset_smoke/press_button_ee_placeholder_10ep_gpu1.hdf5 \
  --max-episodes 10 \
  --output outputs/press_button_ee_placeholder_dataset_smoke/replay_report_10ep_gpu1.json

python scripts/evaluate_runtime_dataset.py \
  --dataset outputs/press_button_ee_placeholder_dataset_smoke/press_button_ee_placeholder_10ep_gpu1.hdf5 \
  --policy replay \
  --max-episodes 10 \
  --output outputs/press_button_ee_placeholder_dataset_eval/gpu1_10ep_replay_eval
```

StateBC sanity:

```bash
python scripts/inspect_baseline_batch.py \
  --dataset outputs/press_button_ee_placeholder_dataset_smoke/press_button_ee_placeholder_10ep_gpu1.hdf5 \
  --policy state_bc \
  --max-episodes 10 \
  --output outputs/press_button_ee_placeholder_dataset_eval/state_bc_10ep_batch_inspect.json

python scripts/train_bc.py \
  --config configs/train/bc_mock.yaml \
  --dataset outputs/press_button_ee_placeholder_dataset_smoke/press_button_ee_placeholder_10ep_gpu1.hdf5 \
  --policy state_bc \
  --output outputs/press_button_ee_placeholder_dataset_eval/state_bc_10ep_dry_run_train \
  --dry-run
```

Final regression:

```bash
pytest tests/
```

Last verified result:

- `187 passed, 1 warning in 52.54s`
- Warning: existing PyTorch/CUDA driver warning in the StateBC real-training
  test path. It did not affect this milestone.

## Current Boundaries And Non-Claims

Milestone A cannot be claimed as any of the following:

- It is not a real FR3 embodiment.
- It is not a force-aware tactile benchmark.
- It is not a formal benchmark dataset.
- It is not a paper main-result dataset or score.
- It has no real force or wrench signal.
- It has no 30-task suite.
- It has no formal baseline comparison.
- It has no full Lightwheel runtime integration.

Current placeholder and proxy elements:

- Pusher mode uses kinematic primitive geometry.
- EE placeholder mode uses simple wrist/finger primitives and records EE pose,
  but does not load or control real FR3 articulation.
- Success is derived from `button_displacement`.
- Contact remains a geometric/runtime proxy unless a real force backend becomes
  available.
- Runtime tactile mapping keeps force and wrench arrays zero and masks false
  when `contact_force_available=false`.
- StateBC checks on runtime-smoke datasets are sanity/dry-run paths, not formal
  training or evaluation.

## No-Fake-Force Boundary

Milestone A keeps a strict no-fake-force rule:

- `contact_force_available=false`
- `force_source=unavailable`
- `mask.has_force=false`
- `mask.has_wrench=false`
- force and wrench arrays are finite zeros
- `button_displacement` is not copied into force or wrench fields
- validation and offline eval record `no_fake_force_from_displacement=true`

## Readiness For Real FR3 Articulation Planning

Milestone A is ready to feed Real FR3 Articulation Planning because it already
provides:

- A working single-task Isaac Sim runtime scene.
- A stable PressButton reset/step/read smoke path.
- A unified eval backend boundary.
- HDF5 runtime-smoke dataset IO with validation and replay.
- Runtime tactile mapping and no-fake-force checks.
- Asset/license provenance gates.
- WebRTC/runtime launch and screenshot evidence paths.
- EE placeholder metadata fields for the transition to real articulation.

Required preconditions for the next stage:

- Select the FR3 USD/articulation source and confirm license/provenance.
- Define real FR3 prim paths, joint names, end-effector frame, and gripper or
  tactile mount frames.
- Decide the minimum control layer: joint position, Cartesian controller, or
  scripted IK wrapper.
- Preserve the existing 7D action schema or explicitly version any change.
- Keep pusher and EE placeholder runtime paths as regressions.
- Keep runtime-smoke datasets clearly marked as non-benchmark artifacts.
- Add real-contact/force hooks only when simulator APIs provide real signals.

Largest risk for Real FR3 Articulation Planning:

- The main risk is control and frame alignment. A real FR3 articulation needs
  validated USD assets, stable joint/frame names, reachable scripted motion,
  collision settings, and consistent end-effector transforms before it can
  safely replace the kinematic placeholder without breaking the existing schema
  and runtime dataset pipeline.

## Go/No-Go

Milestone A: PASS

Go to next stage: Real FR3 Articulation Planning.

The next stage should remain planning-first and single-task. It should not
expand to 30 tasks, should not claim force-aware tactile results, and should not
turn runtime-smoke datasets into formal benchmark datasets.

## Post-Milestone A FR3 Planning Handoff

Real FR3 articulation work starts after Milestone A as a separate planning
stage. The handoff document is `docs/real_fr3_articulation_plan.md`.

That stage must keep the existing `pusher` and `ee_placeholder` paths as
regressions, preserve the 7D action schema, and avoid loading USD or starting
Isaac Sim until the follow-up FR3 load-only visual smoke gate. Planning probes
may check asset paths and license/provenance metadata, but they must not be
described as a connected real FR3 runtime.

# PressButton Dataset Collection Smoke

This stage verifies that the single-task PressButton runtime rollout can be
written into the existing HDF5 dataset schema and read back by the normal
reader, validator, and replay-consistency tools.

It is not a formal benchmark dataset, not a paper dataset, and not evidence for
force-aware tactile manipulation. The default dry-run path does not start Isaac
Sim. The optional real runtime path still uses the current single-task
PressButton scene and placeholder pusher.

## Scope

- Task: `PressButton`
- Suite: `tactile_contact`
- Backend metadata: `isaacsim_press_button`
- Dataset kind: `runtime_smoke`
- Default policy: `scripted`
- Default tactile mode: `force_wrench`
- Dataset schema: existing HDF5 schema version, with runtime-specific flags in
  dataset and episode metadata

The writer does not create a new incompatible dataset layout. Episodes still
store:

- `/episodes/{episode_id}/observations/...`
- `/episodes/{episode_id}/actions`
- `/episodes/{episode_id}/rewards`
- `/episodes/{episode_id}/success`
- `/episodes/{episode_id}/contact_metrics`
- `/episodes/{episode_id}/metadata`
- `/metadata/dataset_info`
- `/metadata/schema_version`
- `/metadata/creation_config`

## Tactile Mapping

Current runtime force is unavailable:

- `contact_force_available=false`
- `force_source=unavailable`
- `mask.has_force=false`
- `mask.has_wrench=false`
- force arrays are finite zeros
- wrench arrays are finite zeros

Button displacement may set `contact_flag_left/right` through
`contact_flag_source=button_displacement`, and success may be labeled with
`success_source=button_displacement`. Button displacement is never written into
force or wrench arrays.

## Commands

Dry-run collection:

```bash
python scripts/collect_press_button_runtime_demos.py \
  --dry-run \
  --runtime-config configs/backend/isaacsim_visual_smoke.yaml \
  --output outputs/press_button_runtime_dataset_smoke/dry_run_dataset.hdf5 \
  --num-episodes 2 \
  --seeds 0 1 \
  --policy scripted \
  --tactile force_wrench \
  --max-steps 20
```

Validation:

```bash
python scripts/validate_dataset.py \
  --dataset outputs/press_button_runtime_dataset_smoke/dry_run_dataset.hdf5 \
  --output outputs/press_button_runtime_dataset_smoke/dry_run_validation_report.json
```

Replay consistency:

```bash
python scripts/replay_runtime_dataset.py \
  --dataset outputs/press_button_runtime_dataset_smoke/dry_run_dataset.hdf5 \
  --max-episodes 2 \
  --output outputs/press_button_runtime_dataset_smoke/dry_run_replay_report.json
```

Offline runtime-dataset evaluation:

```bash
python scripts/evaluate_runtime_dataset.py \
  --dataset outputs/press_button_runtime_dataset_smoke/press_button_runtime_smoke_gpu1.hdf5 \
  --policy replay \
  --max-episodes 3 \
  --output outputs/press_button_runtime_dataset_eval/gpu1_replay_eval
```

This evaluation reads HDF5 only. It does not start Isaac Sim and does not
physically replay the episode. It checks schema consistency, success-label
consistency, zero-safe force/wrench arrays, and the force-unavailable masks.

Optional real runtime collection, only after Isaac Sim is configured:

```bash
conda run -n isaac python scripts/collect_press_button_runtime_demos.py \
  --runtime-config outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml \
  --output outputs/press_button_runtime_dataset_smoke/press_button_runtime_smoke.hdf5 \
  --num-episodes 3 \
  --seeds 0 1 2 \
  --policy scripted \
  --tactile force_wrench \
  --max-steps 80 \
  --headless \
  --webrtc \
  --save-screenshots
```

## Required Flags

All runtime-smoke artifacts must carry:

- `runtime_smoke=true`
- `benchmark_result=false`
- `not_for_paper_claims=true`
- `lightwheel_assets_used=false`
- `contact_force_available=false` while force remains unavailable

## Non-Claims

- This is not a formal demonstration dataset.
- This is not a force-based tactile benchmark.
- Dry-run collection is a schema proxy and does not start Isaac Sim.
- Real runtime collection is still a single-task smoke with a placeholder
  pusher, not full FR3 control.
- Future real force/contact integration may reuse the same HDF5 schema, but it
  must set force masks only when real force values are available.

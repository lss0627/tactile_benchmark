# PressButton EE Placeholder Runtime Smoke 10 Episodes

This stage checks whether the optional single-task Isaac Sim PressButton runtime
can collect 10 schema-compatible episodes with `robot_mode=ee_placeholder`.
It is a runtime smoke gate only.

## Scope

- Task: `PressButton`
- Backend: `isaacsim_press_button`
- Robot mode: `ee_placeholder`
- Robot config: `configs/robots/fr3_ee_placeholder.yaml`
- Dataset kind: `runtime_smoke`
- Episode count: 10
- Policy: `scripted`
- Tactile mode: `force_wrench`

The end effector is a kinematic primitive placeholder. It is not a real FR3
articulation, not a tactile end-effector mount, and not a formal robot
embodiment.

## Collection Command

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

## Required Metadata

Dataset-level and episode-level metadata must keep these fields:

- `dataset_kind=runtime_smoke`
- `backend=isaacsim_press_button`
- `robot_mode=ee_placeholder`
- `placeholder_robot=true`
- `real_fr3_articulation=false`
- `force_source=unavailable`
- `contact_force_available=false`
- `benchmark_result=false`
- `not_for_paper_claims=true`

The tactile force and wrench masks must remain false while force sensing is
unavailable:

- `mask.has_force=false`
- `mask.has_wrench=false`

Button displacement may label success, but it must not be encoded as force or
wrench. The validator and replay consistency checks must keep
`no_fake_force_from_displacement=true`.

## Validation And Sanity Checks

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

StateBC may be inspected or dry-run trained for plumbing checks only. The
outputs must keep `benchmark_result=false` and `not_for_paper_claims=true`.

## Non-Claims

This dataset is not a formal benchmark dataset and must not be used for paper
claims. The runtime still uses a geometric contact proxy, success comes from
`button_displacement`, and force/wrench data are unavailable. Real FR3
articulation control and tactile force sensing remain future work.

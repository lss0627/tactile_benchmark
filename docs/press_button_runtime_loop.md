# PressButton Runtime Loop

This page documents the first single-task real Isaac Sim runtime loop for
Isaac-Tactile-LIBERO. It is intentionally narrow: one `PressButton` scene, one
kinematic pusher placeholder by default, one success check, and runtime status artifacts.
It is not the unified benchmark evaluator and not a paper result.

## Scope

The runtime loop lives outside the existing mock `evaluate.py` path:

- env class: `isaac_tactile_libero/envs/isaacsim_press_button_env.py`
- script: `scripts/run_press_button_runtime_loop.py`
- task: `PressButton`
- action schema: existing 7D action schema
- contact/displacement check: a best-effort PressButton displacement hook is
  preferred, with explicit fallback to the geometric proxy when the hook is not
  available.

The visual smoke script remains separate. `scripts/run_press_button_visual_smoke.py`
only checks static scene rendering, WebRTC, and screenshots.

## Current Scene

The default runtime scene uses primitives:

- ground plane;
- table;
- red button cylinder;
- button housing;
- blue kinematic pusher placeholder;
- camera;
- lights.

The pusher is not a real FR3. It exists to verify a minimal `reset / step /
read_observation / success` loop while preserving the public 7D action schema.

An optional `robot_mode=ee_placeholder` path can replace the blue sphere with a
simple wrist block and two-finger gripper primitive. This is still a kinematic
placeholder, not a real FR3 articulation or tactile embodiment.

## Action Loop

The 7D action contract is unchanged:

- `action[0:3]`: pusher or EE-placeholder delta xyz in meters;
- `action[3:6]`: retained rotation delta fields, currently ignored;
- `action[6]`: retained gripper command, currently recorded only.

All actions pass through `clip_action()` before use.

The scripted policy moves above the button, moves down, holds, and moves up.
`random` and `zero` are sanity policies only.

## Observation

`read_observation()` returns an observation compatible with the existing public
observation schema and includes additional runtime metadata:

- `task_name`;
- `seed`;
- `timestep`;
- `runtime.pusher_pose`;
- `runtime.ee_pose`;
- `runtime.robot_mode`;
- `runtime.placeholder_robot`;
- `runtime.real_fr3_articulation`;
- `runtime.gripper_command`;
- `runtime.action_schema_version`;
- `runtime.button_pose`;
- `runtime.button_pressed`;
- `runtime.contact_proxy`;
- `runtime.geometric_contact_proxy`;
- `runtime.physics_contact_available`;
- `runtime.contact_signal_seen`;
- `runtime.contact_force_available`;
- `runtime.button_displacement_available`;
- `runtime.button_press_depth`;
- `runtime.max_button_press_depth`;
- `runtime.using_geometric_fallback`;
- `runtime.success_source`;
- tactile mask compatible with the existing tactile schema.

RGB capture remains optional. If viewport screenshot APIs are unavailable, the
loop records a warning and continues.

## Success and Metrics

Success source priority is:

- button displacement exceeds the configured threshold;
- physics contact signal plus downward motion, when a real contact hook is
  available;
- geometric fallback if contact/displacement hooks are unavailable.

Metrics include:

- `success`;
- `success_source`;
- `num_steps`;
- `first_contact_step`;
- `first_success_step`;
- `contact_step_count`;
- `completion_time`;
- `min_distance_to_button`;
- `max_press_depth`;
- `button_press_depth`;
- `max_button_press_depth`;
- `physics_contact_available`;
- `contact_signal_seen`;
- `contact_force_available`;
- `button_displacement_available`;
- `using_geometric_fallback`;
- `contact_proxy_triggered`;
- `geometric_contact_proxy`.

The current pusher is still a kinematic placeholder. If `success_source` is
`geometric_fallback`, the rollout must not be described as physics-contact
success. Even when `success_source=button_displacement`, this remains a
single-task runtime smoke rather than a final tactile benchmark.

## Dry Run

Dry-run does not import or start Isaac Sim:

```bash
python scripts/run_press_button_runtime_loop.py \
  --config configs/backend/isaacsim_visual_smoke.yaml \
  --dry-run \
  --policy scripted \
  --robot-mode ee_placeholder \
  --robot-config configs/robots/fr3_ee_placeholder.yaml \
  --max-steps 20 \
  --output outputs/press_button_runtime_loop/dry_run_status.json
```

## Real Runtime

Run only from an Isaac Sim-capable Python environment after readiness passes:

```bash
python scripts/run_press_button_runtime_loop.py \
  --config outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml \
  --headless \
  --webrtc \
  --policy scripted \
  --robot-mode pusher \
  --max-steps 80 \
  --save-screenshot \
  --save-rollout-json \
  --output outputs/press_button_runtime_loop/runtime_status.json
```

Artifacts:

- `runtime_status.json`;
- `rollout.json` when `--save-rollout-json` is set;
- screenshot when `--save-screenshot` is set.

## Non-Goals

- no complete Lightwheel runtime;
- no Lightwheel asset download or redistribution;
- no 30-task expansion;
- no dataset collection;
- no training;
- no full benchmark evaluation;
- no claim that the pusher or EE placeholder is a real FR3;
- no claim that geometric contact proxy is tactile/contact sensing;
- no claim that contact force is available unless `contact_force_available=true`.

The pusher path remains the default for regression and 50-episode stability
checks. The `ee_placeholder` path is a transition toward later real FR3
articulation planning while still keeping the task count fixed.

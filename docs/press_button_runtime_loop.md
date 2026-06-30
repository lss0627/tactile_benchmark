# PressButton Runtime Loop

This page documents the first single-task real Isaac Sim runtime loop for
Isaac-Tactile-LIBERO. It is intentionally narrow: one `PressButton` scene, one
kinematic pusher placeholder, one success check, and runtime status artifacts.
It is not the unified benchmark evaluator and not a paper result.

## Scope

The runtime loop lives outside the existing mock `evaluate.py` path:

- env class: `isaac_tactile_libero/envs/isaacsim_press_button_env.py`
- script: `scripts/run_press_button_runtime_loop.py`
- task: `PressButton`
- action schema: existing 7D action schema
- contact check: geometric proxy unless a later phase replaces it with real
  Isaac Sim contact/tactile sensing

The visual smoke script remains separate. `scripts/run_press_button_visual_smoke.py`
only checks static scene rendering, WebRTC, and screenshots.

## Current Scene

The first runtime scene uses primitives:

- ground plane;
- table;
- red button cylinder;
- button housing;
- blue kinematic pusher placeholder;
- camera;
- lights.

The pusher is not a real FR3. It exists to verify a minimal `reset / step /
read_observation / success` loop while preserving the public 7D action schema.

## Action Loop

The 7D action contract is unchanged:

- `action[0:3]`: pusher delta xyz in meters;
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
- `runtime.button_pose`;
- `runtime.button_pressed`;
- `runtime.contact_proxy`;
- `runtime.geometric_contact_proxy`;
- tactile mask compatible with the existing tactile schema.

RGB capture remains optional. If viewport screenshot APIs are unavailable, the
loop records a warning and continues.

## Success and Metrics

Success is currently:

- `button_pressed=true`; or
- button press depth exceeds the configured geometric threshold.

Metrics include:

- `success`;
- `num_steps`;
- `completion_time`;
- `min_distance_to_button`;
- `max_press_depth`;
- `contact_proxy_triggered`;
- `geometric_contact_proxy`.

These are single-task runtime metrics, not final benchmark metrics and not
paper evidence.

## Dry Run

Dry-run does not import or start Isaac Sim:

```bash
python scripts/run_press_button_runtime_loop.py \
  --config configs/backend/isaacsim_visual_smoke.yaml \
  --dry-run \
  --policy scripted \
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
- no claim that the pusher placeholder is a real FR3;
- no claim that geometric contact proxy is tactile/contact sensing.

The next phase can integrate this single-task runtime into unified
`make_env()` / `evaluate.py` while still keeping the task count fixed.

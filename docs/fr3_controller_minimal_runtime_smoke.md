# FR3 Controller Minimal Runtime Smoke

This document closes the FR3 controller minimal runtime smoke gate. The goal of
this gate is only to prove that the loaded FR3 articulation can be initialized,
joint state can be read, a hold-position command can be issued, and a single
tiny safe joint nudge can be observed. It is not PressButton control, not
dataset collection, and not a benchmark result.

## Runtime Inputs

- Robot config: `configs/robots/fr3_real_articulation.yaml`
- Runtime config: `outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml`
- Safety config: `configs/robots/fr3_controller_safety.yaml`
- FR3 USD: `/mnt/data/home/lss/isaacsim_assets/Assets/Isaac/5.1/Isaac/Robots/FrankaRobotics/FrankaFR3/fr3.usd`
- Articulation root: `/World/FR3`
- Runtime command environment: `CUDA_VISIBLE_DEVICES=1 conda run -n isaac python ...`

## Gate 1: init_only

Command:

```bash
CUDA_VISIBLE_DEVICES=1 conda run -n isaac python scripts/run_fr3_controller_smoke.py \
  --robot-config configs/robots/fr3_real_articulation.yaml \
  --runtime-config outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml \
  --mode init_only \
  --headless \
  --webrtc \
  --save-screenshot \
  --max-steps 20 \
  --output outputs/fr3_controller_smoke/init_only_status.json
```

Result summary:

- `ok=true`
- `runtime_started=true`
- `simulation_app_created=true`
- `fr3_prim_loaded=true`
- `articulation_found=true`
- `articulation_root_path=/World/FR3`
- `controller_initialized=true`
- `joint_state_read=true`
- `controller_api=dynamic_control`
- `num_joints=9`
- `dof_count=9`
- `sends_joint_commands=false`
- `screenshot_saved=true`

Joint names read from the controller:

- `fr3_joint1`
- `fr3_joint2`
- `fr3_joint3`
- `fr3_joint4`
- `fr3_joint5`
- `fr3_joint6`
- `fr3_joint7`
- `fr3_finger_joint1`
- `fr3_finger_joint2`

## Gate 2: hold_position

Command:

```bash
CUDA_VISIBLE_DEVICES=1 conda run -n isaac python scripts/run_fr3_controller_smoke.py \
  --robot-config configs/robots/fr3_real_articulation.yaml \
  --runtime-config outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml \
  --mode hold_position \
  --headless \
  --webrtc \
  --save-screenshot \
  --max-steps 50 \
  --output outputs/fr3_controller_smoke/hold_position_status.json
```

Result summary:

- `ok=true`
- `controller_api=dynamic_control`
- `hold_position_available=true`
- `hold_position_commanded=true`
- `sends_joint_commands=true`
- `num_steps=50`
- `max_joint_position_drift=0.0003741830587387085`
- `max_joint_velocity_norm=0.004961511172840641`
- `stable_hold=true`
- `safety_limits_enabled=true`
- `benchmark_result=false`

This command is a hold-position smoke only. It sends the current joint position
targets to keep the robot still; it does not execute a task trajectory.

## Gate 3: tiny_joint_nudge

Command:

```bash
CUDA_VISIBLE_DEVICES=1 conda run -n isaac python scripts/run_fr3_controller_smoke.py \
  --robot-config configs/robots/fr3_real_articulation.yaml \
  --runtime-config outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml \
  --mode tiny_joint_nudge \
  --headless \
  --webrtc \
  --save-screenshot \
  --max-steps 50 \
  --output outputs/fr3_controller_smoke/tiny_joint_nudge_status.json
```

Result summary:

- `ok=true`
- `controller_api=dynamic_control`
- `selected_joint=fr3_joint1`
- `commanded_delta=0.02`
- `observed_delta=0.02007075693381921`
- `joint_command_sent=true`
- `safety_abort=false`
- `safety_abort_reason=null`
- `nan_detected=false`
- `max_joint_position_drift=0.02007075693381921`
- `benchmark_result=false`
- `not_for_paper_claims=true`

The tiny nudge verifies that one small bounded joint-position command can be
sent and observed. It does not control the end-effector pose, gripper task
state, or PressButton.

## Controller API Method

The runtime wrapper uses an import-safe boundary:

- base Python import does not import `isaacsim`, `omni`, `pxr`, or `carb`;
- real runtime imports occur only after `SimulationApp` is created;
- the script loads the FR3 USD at `/World/FR3`;
- the timeline is started before articulation initialization;
- `omni.isaac.dynamic_control` is used for this smoke;
- Core `SingleArticulation` remains a fallback path.

## Safety Limits

The safety config is `configs/robots/fr3_controller_safety.yaml`:

- `max_joint_delta_rad=0.02`
- `max_steps=50`
- `max_velocity_norm=2.0`
- `max_joint_position_drift_rad=0.1`
- `abort_on_nan=true`
- `abort_on_large_drift=true`
- `benchmark_result=false`

## Known Warnings

Isaac Sim logs may include headless/GLFW warnings, `CUDA_VISIBLE_DEVICES`
warnings, dynamic-control deprecation warnings, and FR3 inertia warnings for
some hand/tcp bodies. These are recorded as runtime caveats, not benchmark
failures. The status JSON remains the source of truth for gate pass/fail.

## Current Non-Claims

- This is not PressButton control.
- This is not a Cartesian or EE controller.
- This is not IK or motion generation.
- This is not tactile or force-aware benchmarking.
- This is not dataset collection.
- This is not a paper result.
- It does not change the 7D action schema.
- It does not replace pusher or `ee_placeholder` regression paths.

## Next Stage

The next stage can be FR3 EE Controller Planning. That stage should design a
safe mapping from the existing 7D action contract to an EE target/controller,
using the verified articulation root and joint-state read path from this smoke.
It must still avoid PressButton task execution until the EE controller contract
and safety checks pass.

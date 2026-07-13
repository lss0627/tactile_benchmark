# FR3 EE Controller Minimal Runtime Smoke

This document closes the FR3 EE Controller Minimal Runtime Smoke gate. The goal
of this gate is narrow: load the real FR3 articulation, read the current
end-effector transform, pass a zero 7D action through the EE mapping path as a
hold/no-op, and send one tiny bounded free-space EE delta through the currently
available safe fallback. It is not PressButton control, not dataset collection,
not tactile sensing, and not a benchmark result.

## Runtime Inputs

- Robot config: `configs/robots/fr3_real_articulation.yaml`
- Controller config: `configs/robots/fr3_ee_controller_contract.yaml`
- Safety config: `configs/robots/fr3_ee_controller_safety.yaml`
- Runtime config: `outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml`
- FR3 USD: `/mnt/data/home/lss/isaacsim_assets/Assets/Isaac/5.1/Isaac/Robots/FrankaRobotics/FrankaFR3/fr3.usd`
- Articulation root: `/World/FR3`
- EE frame: `/World/FR3/fr3_hand_tcp`
- Runtime command environment: `CUDA_VISIBLE_DEVICES=1 conda run -n isaac python ...`

## Gate 1: read_ee

Command:

```bash
CUDA_VISIBLE_DEVICES=1 conda run -n isaac python scripts/run_fr3_ee_controller_smoke.py \
  --robot-config configs/robots/fr3_real_articulation.yaml \
  --controller-config configs/robots/fr3_ee_controller_contract.yaml \
  --safety-config configs/robots/fr3_ee_controller_safety.yaml \
  --runtime-config outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml \
  --mode read_ee \
  --headless \
  --webrtc \
  --save-screenshot \
  --max-steps 20 \
  --output outputs/fr3_ee_controller_smoke/read_ee_status.json
```

Result summary:

- `ok=true`
- `runtime_started=true`
- `simulation_app_created=true`
- `fr3_loaded=true`
- `articulation_found=true`
- `articulation_root_path=/World/FR3`
- `controller_initialized=true`
- `controller_api=dynamic_control`
- `ee_transform_read=true`
- `joint_state_read=true`
- `num_joints=9`
- `dof_count=9`
- `sends_joint_commands=false`
- `joint_command_sent=false`
- `screenshot_saved=true`

Controller joint names:

- `fr3_joint1`
- `fr3_joint2`
- `fr3_joint3`
- `fr3_joint4`
- `fr3_joint5`
- `fr3_joint6`
- `fr3_joint7`
- `fr3_finger_joint1`
- `fr3_finger_joint2`

The current EE position reported by the runtime was approximately
`[0.220728, -0.000037, 0.880394]`.

## Gate 2: zero_action

Command:

```bash
CUDA_VISIBLE_DEVICES=1 conda run -n isaac python scripts/run_fr3_ee_controller_smoke.py \
  --robot-config configs/robots/fr3_real_articulation.yaml \
  --controller-config configs/robots/fr3_ee_controller_contract.yaml \
  --safety-config configs/robots/fr3_ee_controller_safety.yaml \
  --runtime-config outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml \
  --mode zero_action \
  --headless \
  --webrtc \
  --save-screenshot \
  --max-steps 50 \
  --output outputs/fr3_ee_controller_smoke/zero_action_status.json
```

Result summary:

- `ok=true`
- `zero_action=true`
- `target_equals_current=true`
- `controller_method_used=hold_position`
- `hold_commanded=true`
- `sends_joint_commands=true`
- `joint_command_sent=true`
- `ee_motion_commanded=false`
- `stable_noop=true`
- `num_steps=50`
- `ee_displacement_norm=0.00025789921440881673`
- `max_joint_position_drift=0.0003741830587387085`
- `max_joint_velocity_norm=0.004961511172840641`
- `safety_abort=false`
- `nan_detected=false`
- `benchmark_result=false`

This mode intentionally sends the current joint targets as a hold/no-op command
after the 7D zero action maps to the current EE target. It verifies the
controller path can hold without visible drift. It does not execute a task.

## Gate 3: tiny_ee_delta

Command:

```bash
CUDA_VISIBLE_DEVICES=1 conda run -n isaac python scripts/run_fr3_ee_controller_smoke.py \
  --robot-config configs/robots/fr3_real_articulation.yaml \
  --controller-config configs/robots/fr3_ee_controller_contract.yaml \
  --safety-config configs/robots/fr3_ee_controller_safety.yaml \
  --runtime-config outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml \
  --mode tiny_ee_delta \
  --headless \
  --webrtc \
  --save-screenshot \
  --max-steps 80 \
  --output outputs/fr3_ee_controller_smoke/tiny_ee_delta_status.json
```

Result summary:

- `ok=true`
- `commanded_7d_action=[0.005, 0, 0, 0, 0, 0, 0]`
- `commanded_ee_delta=[0.004999995231628418, 0, 0]`
- `observed_ee_delta=[0.005720362067222595, -0.00002303440123796463, -0.0023578405380249023]`
- `ee_displacement_norm=0.006187284118780478`
- `controller_method_used=joint_space_fallback`
- `ik_success=false`
- `direction_alignment_ok=true`
- `sends_joint_commands=true`
- `joint_command_sent=true`
- `max_joint_position_drift=0.010315145133063197`
- `max_joint_velocity_norm=0.08843057663695739`
- `safety_abort=false`
- `nan_detected=false`
- `benchmark_result=false`
- `not_for_paper_claims=true`

The runtime discovery gate reported kinematics and IK routes as available, but
this minimal smoke still uses a deliberately bounded joint-space fallback for
the tiny motion. That fallback is a smoke-only bridge: it validates that the
existing 7D action can pass through the EE mapping boundary and produce a small
safe physical change. It is not a production Cartesian controller.

## Controller Method

The runtime wrapper is import-safe:

- importing project modules does not import `isaacsim`, `omni`, `pxr`, or
  `carb`;
- real runtime imports occur only after `SimulationApp` is created;
- dry-run writes the same status schema without starting Isaac Sim;
- the real runs use `omni.isaac.dynamic_control` for articulation state and
  joint-position commands;
- EE transforms are read from dynamic control when possible, with USD xform as
  fallback.

## Safety Limits

The safety config is `configs/robots/fr3_ee_controller_safety.yaml`:

- `max_delta_xyz_per_step=0.01`
- `max_delta_rot_per_step=0.05`
- `max_joint_velocity_norm=1.0`
- `max_joint_position_drift=0.05`
- `abort_on_nan=true`
- `abort_on_workspace_violation=true`
- `abort_on_large_joint_motion=true`
- `benchmark_result=false`

The tiny fallback joint delta is capped to 0.01 rad for this smoke. This keeps
the observed EE motion small while avoiding a claim that an operational IK
controller is already connected.

## Known Warnings

Isaac Sim may emit headless/GLFW warnings, `CUDA_VISIBLE_DEVICES` warnings,
dynamic-control deprecation warnings, requests dependency warnings, and FR3
inertia warnings for hand/TCP bodies. These are runtime caveats. The status
JSON files remain the source of truth for gate pass/fail.

## Current Non-Claims

- This is not PressButton control.
- This is not a benchmark result.
- This is not dataset collection.
- This is not tactile or force-aware sensing.
- This is not a complete Cartesian controller.
- This does not control the gripper as a task actuator.
- This does not change the stable 7D action schema.
- This does not replace the pusher or `ee_placeholder` regression paths.

## Follow-Up: Differential IK Diagnostic

The follow-up FR3 Local Differential IK / Jacobian Diagnostic has passed and
is documented in `docs/fr3_differential_ik_diagnostic.md`.

That diagnostic replaces the smoke-only joint-space fallback for tiny Cartesian
motions with:

- FK finite-difference translation Jacobian;
- damped least-squares local solve;
- bounded `dq` checks;
- FK validation before command;
- one tiny 0.25 mm free-space motion smoke.

The successful tiny motion status is
`outputs/fr3_differential_ik/tiny_diffik_motion_status.json`. It records
`controller_method_used=differential_ik`, `uses_lula_global_ik=false`,
`uses_joint_space_fallback=false`, `joint_command_sent=true`,
`safety_abort=false`, and `benchmark_result=false`.

## Next Stage

The next stage can be FR3 EE-to-PressButton Planning. Before connecting a task,
the project should preserve the local differential IK safety envelope, keep
pusher and `ee_placeholder` paths passing, and keep all status artifacts marked
`benchmark_result=false` until a formal benchmark gate exists.

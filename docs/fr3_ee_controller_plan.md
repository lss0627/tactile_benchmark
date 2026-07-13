# FR3 EE Controller Planning

This document closes the FR3 EE Controller Planning gate. It prepares the
contract for a future FR3 end-effector controller, but it does not execute EE
motion, PressButton control, dataset collection, or benchmark evaluation.

## Inputs

- Robot config: `configs/robots/fr3_real_articulation.yaml`
- Introspection report: `outputs/fr3_articulation_introspection/report.json`
- Controller smoke report: `outputs/fr3_controller_smoke/init_only_status.json`
- API discovery report: `outputs/fr3_ee_controller_plan/api_discovery.json`
- Action mapping config: `configs/robots/fr3_ee_controller_contract.yaml`
- Safety config: `configs/robots/fr3_ee_controller_safety.yaml`

## Gate 1: EE Frame And Kinematic Readiness

Command:

```bash
python scripts/check_fr3_ee_controller_readiness.py \
  --robot-config configs/robots/fr3_real_articulation.yaml \
  --introspection-report outputs/fr3_articulation_introspection/report.json \
  --controller-smoke-report outputs/fr3_controller_smoke/init_only_status.json \
  --output outputs/fr3_ee_controller_plan/readiness.json
```

Result summary:

- `ok=true`
- `ready_for_ee_controller_design=true`
- `articulation_root_path=/World/FR3`
- `ee_frame_candidate=/World/FR3/fr3_hand_tcp`
- `gripper_frame_candidate=/World/FR3/fr3_hand`
- controller API: `dynamic_control`
- controller DOF count: `9`
- controller joint names: `fr3_joint1` through `fr3_joint7`,
  `fr3_finger_joint1`, `fr3_finger_joint2`

The USD introspection report sees 13 joint prims, while the runtime controller
reports 9 controllable DOFs. The EE controller plan uses the controller DOF
view for control planning and keeps the USD traversal count as diagnostic
context.

## Gate 2: EE Controller API Discovery

Dry-run command:

```bash
python scripts/probe_fr3_ee_controller_api.py \
  --robot-config configs/robots/fr3_real_articulation.yaml \
  --runtime-config outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml \
  --dry-run \
  --output outputs/fr3_ee_controller_plan/api_discovery_dry_run.json
```

Runtime discovery command:

```bash
CUDA_VISIBLE_DEVICES=1 conda run -n isaac python scripts/probe_fr3_ee_controller_api.py \
  --robot-config configs/robots/fr3_real_articulation.yaml \
  --runtime-config outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml \
  --headless \
  --webrtc \
  --output outputs/fr3_ee_controller_plan/api_discovery.json
```

Result summary:

- `ok=true`
- `runtime_started=true`
- `fr3_loaded=true`
- `articulation_found=true`
- `controller_api=dynamic_control`
- `joint_state_read=true`
- `sends_joint_commands=false`
- `ee_motion_executed=false`
- `kinematics_solver_available=true`
- `ik_solver_available=true`
- `joint_space_fallback_available=true`
- `recommended_method=kinematics_solver`

This gate starts Isaac Sim only to discover available APIs and verify the FR3
articulation can be read. It does not send EE or joint commands.

## Gate 3: 7D Action Mapping

Command:

```bash
python scripts/check_fr3_ee_action_mapping.py \
  --config configs/robots/fr3_ee_controller_contract.yaml \
  --output outputs/fr3_ee_controller_plan/action_mapping_report.json
```

The stable action schema remains:

- `[0:3]`: `dx, dy, dz` in meters;
- `[3:6]`: `droll, dpitch, dyaw` in radians;
- `[6]`: normalized gripper command.

The mapping currently produces target descriptions only:

- `zero`
- `small_plus_x`
- `small_minus_z`
- `small_yaw`

All mapped targets stay within the configured workspace bounds and record
`sends_commands=false`.

## Gate 4: Runtime Readiness

Command:

```bash
python scripts/check_fr3_ee_runtime_readiness.py \
  --readiness-report outputs/fr3_ee_controller_plan/readiness.json \
  --api-discovery-report outputs/fr3_ee_controller_plan/api_discovery.json \
  --action-mapping-report outputs/fr3_ee_controller_plan/action_mapping_report.json \
  --safety-config configs/robots/fr3_ee_controller_safety.yaml \
  --output outputs/fr3_ee_controller_plan/runtime_readiness.json
```

Result summary:

- `ready_for_minimal_ee_runtime_smoke=true`
- `recommended_controller_method=kinematics_solver`
- `joint_space_fallback_available=true`
- `ee_frame=fr3_hand_tcp`
- `workspace_bounds_valid=true`
- `safety_config_valid=true`
- `sends_joint_commands=false`
- `ee_motion_executed=false`

## Safety Envelope

`configs/robots/fr3_ee_controller_safety.yaml` defines the next runtime smoke
envelope:

- `max_delta_xyz_per_step=0.01`
- `max_delta_rot_per_step=0.05`
- `max_joint_velocity_norm=1.0`
- `max_joint_position_drift=0.05`
- `abort_on_nan=true`
- `abort_on_workspace_violation=true`
- `abort_on_large_joint_motion=true`
- `benchmark_result=false`

## Next Runtime Gates

The next stage, FR3 EE Controller Minimal Runtime Smoke, has passed in small
gates:

1. Load FR3 and read current EE/TCP transform.
2. Map zero action to a no-op target and verify stable hold.
3. Apply one clipped, tiny target in free space through the current
   joint-space fallback.
4. Abort on NaN, workspace violation, large joint motion, or unexpected command
   direction.

The runtime status files are in `outputs/fr3_ee_controller_smoke/`, and the
report is `docs/fr3_ee_controller_minimal_runtime_smoke.md`.

The follow-up FR3 Local Differential IK / Jacobian Diagnostic has also passed.
It uses FK finite differences and damped least-squares local IK for tiny EE
motions, not Lula global target IK and not the earlier joint-space fallback.
The diagnostic report is `docs/fr3_differential_ik_diagnostic.md`, and its
runtime artifacts are in `outputs/fr3_differential_ik/`.

PressButton remains disconnected after these gates; the next controller work
should plan task interaction around the differential IK method and the existing
safety envelope.

## Current Non-Claims

- Minimal EE smoke motion has been executed only in free space; no task control
  has been executed.
- Differential IK has been validated only for tiny free-space EE deltas.
- No PressButton control is connected.
- No dataset has been collected.
- No training has been run.
- No tactile force/wrench backend is connected.
- No benchmark result or paper claim is produced.
- The 7D action schema is unchanged.

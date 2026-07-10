# FR3 Control Contract Skeleton

The FR3 control contract defines how the existing benchmark action schema will
map to a future FR3 end-effector controller. It does not implement that
controller.

## 7D Action Interpretation

The schema remains unchanged:

- `[0:3]`: `dx, dy, dz` in meters;
- `[3:6]`: `droll, dpitch, dyaw` in radians;
- `[6]`: normalized gripper command.

`isaac_tactile_libero/robots/fr3_control_contract.py` maps those fields into a
target EE delta object and records `sends_joint_commands=false`.

## Contract Boundaries

- `controller_connected=false`
- `sends_joint_commands=false`
- `articulation_control_enabled=false`
- `benchmark_result=false`
- `not_for_paper_claims=true`

Future controller work must satisfy this contract or explicitly bump the action
schema version. The pusher and `ee_placeholder` paths remain valid regression
paths and are not replaced by this planning artifact.

## Runtime Smoke Follow-Up

`scripts/run_fr3_controller_smoke.py` now provides a minimal runtime smoke that
validates articulation initialization and very small joint-level control in
Isaac Sim. This does not replace the control contract skeleton above:

- the contract module still maps the 7D action schema to a planned EE delta and
  sends no commands;
- the runtime smoke uses direct joint-position targets only for
  `hold_position` and `tiny_joint_nudge`;
- no EE controller, IK, PressButton task logic, tactile mount, or dataset path
  is connected.

The runtime smoke is therefore a prerequisite for FR3 EE Controller Planning,
not an implementation of the final controller.

## EE Controller Planning Follow-Up

`isaac_tactile_libero/robots/fr3_ee_action_mapping.py` now defines the
planning-only mapping from the stable 7D action to a future EE target:

- xyz deltas are meters;
- rotation deltas are radians;
- gripper is normalized;
- workspace bounds and clipping are checked;
- `sends_commands=false`.

`scripts/probe_fr3_ee_controller_api.py` can start Isaac Sim to discover
available kinematics/IK/fallback APIs, but it does not execute EE motion and
does not send joint commands. The current recommended method from the discovery
report is `kinematics_solver`, with `joint_space_fallback` also available.

## EE Controller Minimal Runtime Smoke Follow-Up

`scripts/run_fr3_ee_controller_smoke.py` now provides the first guarded EE
runtime smoke. It keeps the same 7D action schema and adds these runtime-only
modes:

- `read_ee`: load FR3 and read `/World/FR3/fr3_hand_tcp` without commands.
- `zero_action`: map `[0, 0, 0, 0, 0, 0, 0]` through the EE boundary and hold
  the current joint targets as a no-op.
- `tiny_ee_delta`: map `[0.005, 0, 0, 0, 0, 0, 0]` to a tiny free-space EE
  target and execute the current bounded joint-space fallback.

The passing status reports are in `outputs/fr3_ee_controller_smoke/`. This is
still not the final controller contract: `ik_success=false` for the tiny smoke
because the operational IK/Cartesian controller is still deferred. PressButton,
dataset collection, tactile mounts, and benchmark scoring remain disconnected.

## Differential IK PressButton Approach Boundary

The FR3 PressButton approach-only smoke uses the local differential IK path,
not Lula global target IK and not a joint-space fallback. It may send tiny
joint targets only to approach `pre_press` or `near_contact`; it does not
execute `press_target`, does not press the button, does not claim task success,
does not collect datasets, and keeps the stable 7D action schema unchanged.

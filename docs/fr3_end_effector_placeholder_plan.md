# FR3 / End-Effector Placeholder Plan

This plan defines the narrow transition from the current primitive PressButton
pusher toward a more robot-shaped runtime smoke path. It does not introduce full
FR3 articulation control, tactile mounts, Lightwheel assets, formal benchmark
datasets, or training.

## Current Primitive Pusher Limitations

The current PressButton runtime uses a blue kinematic primitive as the active
body. It is useful for proving that Isaac Sim can start, step, update the button
displacement hook, write HDF5 runtime-smoke datasets, and cleanly release GPU
resources. It is not a robot embodiment:

- no FR3 USD or articulation;
- no joint state, dynamics, or IK;
- no validated end-effector frame;
- no gripper geometry beyond a generic pusher contact point;
- no real tactile/contact force stream.

## Transition Stages

Stage 1: visual-only FR3 placeholder

- Add robot-shaped visual primitives only.
- Keep the action schema and success hook unchanged.
- Use this only to make screenshots and WebRTC visual smoke less abstract.

Stage 2: kinematic end-effector placeholder

- Use `robot_mode=ee_placeholder`.
- Create a wrist block plus two simple finger primitives.
- Apply the existing 7D action schema as kinematic delta end-effector motion:
  xyz is applied, rotation and gripper fields are recorded but not yet used for
  real control.
- Record `placeholder_robot=true` and `real_fr3_articulation=false`.

Stage 3: later real FR3 articulation control

- Introduce real FR3 USD/articulation loading, validated frames, reset poses,
  controller boundaries, and eventually tactile mount definitions.
- This must be a separate gate with its own readiness checks and tests.
- The current planning handoff is documented in
  `docs/real_fr3_articulation_plan.md`. It starts with asset/config discovery
  and does not load USD or attach a controller.

## Contract

The existing action schema remains unchanged:

- `action[0:3]`: end-effector delta xyz in meters;
- `action[3:6]`: retained delta rotation fields, recorded but not applied by
  the placeholder;
- `action[6]`: normalized gripper command, recorded by the placeholder.

The placeholder config is `configs/robots/fr3_ee_placeholder.yaml`. It must
declare:

- `robot_mode=ee_placeholder`;
- `placeholder_robot=true`;
- `real_fr3_articulation=false`;
- `use_real_fr3_usd=false`;
- `use_lightwheel_assets=false`;
- `benchmark_result=false`;
- `not_for_paper_claims=true`.

## Metadata

Runtime status, rollout, metrics, and future runtime-smoke dataset episodes must
record:

- `robot_mode`;
- `robot_config_path`;
- `placeholder_robot`;
- `real_fr3_articulation`;
- `ee_pose`;
- `gripper_command`;
- `action_schema_version`.

Older pusher runtime-smoke datasets are still valid. Validators may check these
fields when present, but must not require them for previous datasets.

## Non-Claims

- `ee_placeholder` is not a real FR3.
- The pusher path remains available and is still the default.
- Real FR3 articulation is only planned; it is not connected in the current
  repository state.
- No Lightwheel assets are used.
- No force/wrench stream is fabricated.
- PressButton success still comes from button displacement unless a later gate
  connects a real contact/force hook.
- Results from this stage are visual/runtime smoke artifacts only, not paper
  benchmark results.

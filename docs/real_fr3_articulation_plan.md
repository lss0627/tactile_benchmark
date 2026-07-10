# Real FR3 Articulation Planning

This document defines the staged plan for moving from the current
`ee_placeholder` PressButton runtime smoke to a real FR3 articulation path. The
FR3 asset path is now locally bound, but real FR3 remains gated: load-only
visual smoke, introspection, controller contract, and PressButton readiness must
all pass before any controller runtime work begins. The stable 7D action schema
does not change.

## Current Starting Point

Milestone A provides:

- `pusher` runtime path: kinematic primitive pusher, still retained as the
  default regression path.
- `ee_placeholder` runtime path: kinematic wrist/finger primitives with EE pose
  metadata, still retained as the transition smoke path.
- Stable 7D action schema.
- PressButton runtime loop, unified evaluation backend, runtime tactile schema
  mapping, HDF5 runtime-smoke dataset schema, replay checks, and no-fake-force
  validation.

The next stage must preserve pusher and EE placeholder paths while adding real
FR3 planning artifacts.

## Phase Plan

### Phase FR3-A: FR3 Asset / USD Discovery

Goal: locate and validate candidate FR3, gripper, and tactile mount assets
without launching Isaac Sim.

Deliverables:

- `configs/robots/fr3_real_articulation.yaml`
- `isaac_tactile_libero/robots/fr3_articulation_spec.py`
- `scripts/probe_fr3_assets.py`
- Asset manifest/license gate checks

This phase checks whether paths are configured and whether license/provenance
metadata exist. It does not load USD files. The local Isaac Assets FR3 path is
now bound in config and recorded in the asset manifest.

### Phase FR3-B: FR3 Load-Only Visual Smoke

Goal: launch Isaac Sim and load a configured FR3 USD into a minimal scene
without controlling it.

Expected outputs:

- runtime status JSON;
- screenshot;
- stage/prim path report;
- no reset/step benchmark result.

The script for this phase is `scripts/run_fr3_load_only_visual_smoke.py`. It
loads only FR3 at `/World/FR3`; it does not connect PressButton or a controller.

### Phase FR3-C: Joint / Articulation Introspection

Goal: inspect the real articulation after load.

Required checks:

- joint count and names match the planned config;
- base, EE, gripper, and tactile frames are discoverable;
- joint limits and default joint pose are readable;
- no controller is attached yet.

The script for this phase is `scripts/introspect_fr3_articulation.py`.

### Phase FR3-D: 7D Action To EE Controller Mapping

Goal: design a controller boundary that maps the existing 7D action contract to
FR3 motion.

The existing schema remains unchanged:

- `action[0:3]`: delta xyz;
- `action[3:6]`: delta orientation fields;
- `action[6]`: gripper command.

Controller implementation choices may include a scripted IK wrapper, a
Cartesian controller, or a joint-position bridge, but any real controller must
be introduced behind the same action schema or through an explicit schema
version bump in a later phase.

The contract module is `isaac_tactile_libero/robots/fr3_control_contract.py`,
and the checker is `scripts/check_fr3_control_contract.py`. They do not send
joint commands.

### Phase FR3-E: PressButton Scripted Action

Goal: adapt the current scripted PressButton motion to the real FR3 controller
while preserving the same success and metadata boundaries.

Required checks:

- pusher and EE placeholder regression paths still pass;
- FR3 scripted action reaches the button without pretending to use tactile
  force;
- success source remains explicit;
- no paper/benchmark claim is made.

The planning checker is `scripts/check_fr3_press_button_readiness.py`. It reads
the load/introspection/control reports and stays offline.

### Phase FR3-F: Runtime Dataset Smoke

Goal: collect a small PressButton runtime-smoke dataset from the real FR3 path
only after load, introspection, and scripted action gates pass.

The dataset must still record:

- `dataset_kind=runtime_smoke`;
- `benchmark_result=false`;
- `not_for_paper_claims=true`;
- force availability and tactile masks truthfully;
- asset provenance and robot config path.

## Config And Contract

The planning config is:

`configs/robots/fr3_real_articulation.yaml`

The schema-only contract module is:

`isaac_tactile_libero/robots/fr3_articulation_spec.py`

The config currently declares:

- `robot_name=fr3_tactile`;
- `robot_mode=real_fr3_articulation_planned`;
- `use_real_fr3_usd=true`;
- `fr3_usd_path=/mnt/data/home/lss/isaacsim_assets/Assets/Isaac/5.1/Isaac/Robots/FrankaRobotics/FrankaFR3/fr3.usd`;
- `gripper_usd_path=null`;
- `gripper_embedded_in_fr3_usd=true`;
- `tactile_mount_usd_path=null`;
- `tactile_mounts_planned=true`;
- planned base, EE, gripper, and tactile frames;
- planned FR3 arm and finger joint names;
- `action_schema_version=0.1.0`;
- `control_mode=planned_delta_ee`;
- `benchmark_result=false`;
- `not_for_paper_claims=true`.

After local Isaac asset discovery, the FR3 USD path is bound in
`configs/robots/fr3_real_articulation.yaml`. The gripper is marked as embedded
in the FR3 USD for load-only smoke readiness, while tactile mounts remain
planned. See `docs/fr3_asset_discovery.md`.

## Asset And License Gate

Any FR3, gripper, tactile mount, or Lightwheel-derived asset must pass the asset
manifest/provenance gate before use:

- record the asset in `assets/asset_manifest.csv`;
- keep `source`, `original_url`, `license`, and `attribution` complete;
- do not relicense Lightwheel or other third-party assets as this project's
  license;
- record whether the asset is modified and redistributed;
- do not download, copy, or bundle assets in the probe script.

If an asset comes from Isaac Sim built-ins or an internally authored local USD,
it should still have a manifest row or an explicit source note before a runtime
gate uses it.

## Non-Claims

- Real FR3 is not connected in the current stage.
- No Isaac Sim runtime is started by the planning probe.
- No USD is loaded by the planning probe.
- No controller or IK is implemented.
- No force-aware tactile benchmark is implemented.
- No 30-task expansion is included.
- No formal dataset or paper result is produced.

## Go Criteria For FR3 Load-Only Visual Smoke

Before entering FR3-B:

- configure a valid `fr3_usd_path`;
- configure or intentionally defer `gripper_usd_path`;
- configure or intentionally defer tactile mount assets;
- verify asset manifest/license metadata;
- keep pusher and EE placeholder regression paths green;
- keep `action_schema_version=0.1.0`;
- keep HDF5 schema unchanged.

The largest anticipated risk is frame and joint-name mismatch between the
selected FR3 USD and the planned control contract. This should be resolved in
load-only and introspection gates before any controller is attached.

## Phase FR3-G: Controller Minimal Runtime Smoke

Status: completed as a smoke gate.

`scripts/run_fr3_controller_smoke.py` now provides three guarded modes:

- `init_only`: load FR3, initialize a runtime articulation/controller wrapper,
  and read joint state without sending commands.
- `hold_position`: read the current joint positions and command a bounded
  hold-position target for a short step window.
- `tiny_joint_nudge`: send one tiny bounded joint-position delta to
  `fr3_joint1` and verify the observed delta is safe.

The passing runtime reports are:

- `outputs/fr3_controller_smoke/init_only_status.json`
- `outputs/fr3_controller_smoke/hold_position_status.json`
- `outputs/fr3_controller_smoke/tiny_joint_nudge_status.json`

This phase still does not connect PressButton, does not implement an
end-effector controller, does not collect data, and does not create benchmark
results. Pusher and `ee_placeholder` paths remain active regression paths.

## Phase FR3-H: EE Controller Planning

Status: completed as a planning/API-discovery gate.

The planning artifacts are:

- `isaac_tactile_libero/robots/fr3_ee_controller_plan.py`
- `isaac_tactile_libero/robots/fr3_ee_action_mapping.py`
- `configs/robots/fr3_ee_controller_contract.yaml`
- `configs/robots/fr3_ee_controller_safety.yaml`
- `scripts/check_fr3_ee_controller_readiness.py`
- `scripts/probe_fr3_ee_controller_api.py`
- `scripts/check_fr3_ee_action_mapping.py`
- `scripts/check_fr3_ee_runtime_readiness.py`
- `docs/fr3_ee_controller_plan.md`

The real API discovery gate found `kinematics_solver`, `ik_solver`, and
`joint_space_fallback` routes, with `recommended_method=kinematics_solver`.
This does not execute EE motion and does not send joint commands. It only
establishes that the next stage can attempt a minimal FR3 EE controller runtime
smoke under the configured safety envelope.

## Phase FR3-I: EE Controller Minimal Runtime Smoke

Status: completed as a guarded runtime smoke gate.

The runtime artifacts are:

- `isaac_tactile_libero/robots/fr3_ee_runtime_controller.py`
- `scripts/run_fr3_ee_controller_smoke.py`
- `outputs/fr3_ee_controller_smoke/read_ee_status.json`
- `outputs/fr3_ee_controller_smoke/zero_action_status.json`
- `outputs/fr3_ee_controller_smoke/tiny_ee_delta_status.json`
- `docs/fr3_ee_controller_minimal_runtime_smoke.md`

The passing gates loaded `/World/FR3`, read `/World/FR3/fr3_hand_tcp`, held the
current joint state after mapping a zero 7D action, and sent one tiny bounded
free-space EE delta through the current joint-space fallback. This validates the
minimal runtime control boundary, but it still does not connect PressButton,
does not collect data, does not connect tactile mounts, does not fabricate
force, and does not produce benchmark results.

Before any PressButton control gate, the controller path should be refined from
the smoke-only fallback toward a clearer IK/Cartesian method while preserving
the stable 7D action schema and the existing pusher / `ee_placeholder`
regression paths.

## Phase FR3-J: Local Differential IK Diagnostic

Status: completed.

The diagnostic found that local damped least-squares differential IK over a
finite-difference translation Jacobian can produce bounded tiny EE deltas. It
also confirmed that Lula global target IK should not be the default PressButton
control path because earlier 1-5 mm targets produced nonlocal unsafe joint
solutions.

Artifacts:

- `outputs/fr3_differential_ik/jacobian_fk_probe.json`
- `outputs/fr3_differential_ik/target_report.json`
- `outputs/fr3_differential_ik/fk_validation_report.json`
- `outputs/fr3_differential_ik/tiny_diffik_motion_status.json`
- `docs/fr3_differential_ik_diagnostic.md`

## Phase FR3-K: EE-to-PressButton Planning

Status: completed as a planning/no-command gate.

The new planning artifacts are:

- `configs/tasks/press_button_fr3_planned.yaml`
- `isaac_tactile_libero/tasks/press_button_geometry.py`
- `isaac_tactile_libero/tasks/fr3_press_button_planner.py`
- `scripts/check_press_button_geometry.py`
- `scripts/run_fr3_press_button_load_only_smoke.py`
- `scripts/plan_fr3_press_button_waypoints.py`
- `scripts/check_fr3_press_button_approach_readiness.py`
- `docs/fr3_ee_to_press_button_planning.md`

The gate records PressButton geometry, verifies FR3/button co-scene load-only
readiness, generates home/pre-press/near-contact/press-target/hold/retract
waypoints, and checks no-command differential IK feasibility. It does not send
joint commands, does not press the button, does not collect a dataset, and does
not fabricate force.

Next phase: FR3 PressButton Approach-Only Runtime Smoke. That phase should
move only toward pre-press or near-contact and must still leave
`press_motion_allowed=false` until a separate press-runtime gate is approved.
The canonical differential IK report for this transition is
`outputs/fr3_differential_ik/target_report.json`.

Status update: the approach-only runtime smoke passed for `micro_approach`,
`short_approach`, `pre_press`, and optional `near_contact`. The results are
documented in `docs/fr3_press_button_approach_only_runtime_smoke.md`.

## Phase FR3-L: PressButton Press Runtime Smoke

Status: partially passed, blocked at press-and-retract.

The press-depth smoke now has guarded artifacts for:

- `outputs/fr3_press_button_press_runtime/preflight.json`
- `outputs/fr3_press_button_press_runtime/dry_run_status.json`
- `outputs/fr3_press_button_press_runtime/partial_press_2mm_status.json`
- `outputs/fr3_press_button_press_runtime/partial_press_10mm_status.json`
- `outputs/fr3_press_button_press_runtime/full_press_status.json`
- `outputs/fr3_press_button_press_runtime/press_and_retract_status.json`
- `docs/fr3_press_button_press_runtime_smoke.md`

Partial 2mm, partial 10mm, and full press pass through local differential IK.
The press-and-retract gate is blocked because retract from the pressed pose did
not safely release the button. Dataset collection and benchmark evaluation must
remain disabled until the retract gate passes.

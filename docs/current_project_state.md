# Current Project State

Date: 2026-07-10

This document summarizes the current progress of Isaac-Tactile-LIBERO. It is a
working status note, not a benchmark result report.

## Overall Status

The project has progressed from a pure mock benchmark skeleton to a
single-task Isaac Sim runtime pipeline for `PressButton`, plus FR3 load-only,
controller smoke, EE controller planning, local differential IK diagnostics,
and FR3 EE-to-PressButton planning gates. The current next stage is
the post-migration physical PressButton mechanism and formal G1 safety gate.

Current high-level status:

- Mock benchmark loop: complete.
- Dataset writer/reader/validator/replay: complete.
- Tactile schema and runtime tactile mapping: complete.
- ReplayPolicy, baseline skeletons, and minimal StateBC training protocol:
  complete.
- PressButton Isaac Sim runtime smoke: complete for pusher and EE placeholder
  paths.
- Runtime-smoke HDF5 datasets: collected and validated for pusher and EE
  placeholder paths.
- Development baseline: Isaac Sim 6.0.1 and Python 3.12.
- Driver 550.144.03: retained and recorded as `UNVALIDATED`.
- Experimental Contact: scalar force/raw contacts validated on CPU physics; GPU physics blocked.
- RTX RGB/depth: validated on GPU 0.
- Real force-vector/wrench tactile sensing: not available; public masks remain false.
- Real FR3 USD asset binding: complete.
- FR3 load-only visual smoke: complete.
- FR3 articulation introspection: complete.
- FR3 7D action control contract skeleton: complete.
- FR3-to-PressButton planning readiness: complete.
- FR3 controller minimal runtime smoke: complete.
- FR3 EE controller planning/API discovery: complete.
- FR3 EE controller minimal runtime smoke: complete.
- FR3 local differential IK / Jacobian diagnostic: complete.
- FR3 EE-to-PressButton geometry/load-only/waypoint planning: complete.
- Real FR3 PressButton compatibility backend: implemented through `make_env`.
- G0 clean repository integrity: `PASS_BENCHMARK`.
- G-1B integration: `PASS_SMOKE` with 100 resets and 500 steps.
- Formal G1 physical PressButton mechanism/task truth: not implemented yet.

## Isaac Sim 6.0.1 Migration

The project now uses Python `>=3.12,<3.13` and Isaac Sim 6.0.1 as its sole development simulator
baseline. Python 3.11/Isaac Sim 5.1 is archived under `requirements/archive/`; assets and historical
evidence remain unchanged and reference-only.

Verified migration results:

- P0 Kit/headless startup: 100 steps passed.
- G-1A FR3 asset/API: exact 9-DOF contract, joint limits/frames, micro-motion, 500-step stability.
- Contact lifecycle: 100/100 ready/onset/release cycles, 0 stale handles, finite scalar/raw data.
- Camera: RGB/depth dtype, shapes, update, finite values and synchronization passed.
- G0: clean `git archive` export, wheel, isolated venv and 340 tests passed.
- G-1B: 100/100 resets, 500/500 bounded steps, 0 invalid/stale handles, maximum observed button
  penetration `1.54e-7 m` with zero persistent-penetration steps.
- A/B: DOF order, limits, direction and zero-action drift passed. The corrected zero-drift bound was
  `min(max(2 * 0.257899 mm, 0.05 mm), 1.0 mm) = 0.515798 mm`; 6.0.1 measured 0.

Native GPU physics Contact is not claimed. The backend rejects it with
`GPU_CONTACT_NATIVE_INSTABILITY` before initialization and keeps GPU rendering enabled.

## Completed Milestones

### Mock Benchmark And Dataset Foundation

The repo has a stable mock/stub benchmark base:

- 5 minimal tasks and 4 tactile modes.
- Stable 7D action schema.
- Observation and tactile schema contracts.
- HDF5 dataset writer, reader, validator, replay helpers, and metadata.
- `scripts/evaluate.py` for mock, replay, baseline skeleton, and runtime-smoke
  paths.
- `ReplayPolicy`, untrained BC skeletons, and minimal StateBC training/dry-run
  protocols.

### Tactile Contract

The tactile contract is explicit:

- `force_wrench`, `visuotactile`, and combined modes have schema/spec coverage.
- Runtime tactile mapping does not fake force from displacement.
- When contact force is unavailable, `mask.has_force=false` and
  `mask.has_wrench=false`.

### PressButton Runtime Pipeline

Milestone A, **PressButton Single-Task Isaac Sim Runtime Pipeline**, is complete.

Reusable runtime pieces:

- `scripts/run_press_button_visual_smoke.py`
- `scripts/run_press_button_runtime_loop.py`
- `scripts/collect_press_button_runtime_demos.py`
- `scripts/evaluate_runtime_dataset.py`
- `isaac_tactile_libero/envs/isaacsim_press_button_env.py`
- `isaac_tactile_libero/sensors/runtime_tactile_adapter.py`

Known runtime-smoke datasets:

- Pusher 50 episodes:
  `outputs/press_button_runtime_dataset_smoke/press_button_runtime_smoke_50ep_gpu1.hdf5`
- EE placeholder 10 episodes:
  `outputs/press_button_ee_placeholder_dataset_smoke/press_button_ee_placeholder_10ep_gpu1.hdf5`

Both remain runtime-smoke datasets only. They are not formal benchmark datasets.

## FR3 Status

### Asset Discovery And Binding

FR3 local asset discovery found and bound the Isaac Assets FR3 USD:

`/mnt/data/home/lss/isaacsim_assets/Assets/Isaac/5.1/Isaac/Robots/FrankaRobotics/FrankaFR3/fr3.usd`

Config:

`configs/robots/fr3_real_articulation.yaml`

Current FR3 asset assumptions:

- `fr3_usd_path` is configured and exists.
- Gripper is marked as embedded in the FR3 USD.
- `tactile_mount_usd_path` remains null.
- Tactile mounts are still planned.
- Asset manifest/provenance gate records the Isaac Sim Assets source and does
  not relicense or redistribute the asset.

### FR3 Integration Sprint A

FR3 Integration Sprint A has passed all four gates.

Gate 1: FR3 Load-Only Visual Smoke

- Script: `scripts/run_fr3_load_only_visual_smoke.py`
- Status: `outputs/fr3_load_only_visual_smoke/status.json`
- Screenshot: `outputs/fr3_load_only_visual_smoke/fr3_load_only_visual_smoke.png`
- Result: `fr3_prim_loaded=true`
- Controller: not connected.
- PressButton: not connected.

Gate 2: FR3 Articulation Introspection

- Script: `scripts/introspect_fr3_articulation.py`
- Report: `outputs/fr3_articulation_introspection/report.json`
- Result summary:
  - `articulation_found=true`
  - `articulation_root_path=/World/FR3`
  - `num_joints=13`
  - `num_links=103`
  - frame candidates include FR3 hand, TCP, fingers, and base paths.
- No joint control is executed.

Gate 3: FR3 Control Contract Skeleton

- Module: `isaac_tactile_libero/robots/fr3_control_contract.py`
- Script: `scripts/check_fr3_control_contract.py`
- Report: `outputs/fr3_control_contract/report.json`
- Result summary:
  - `action_dim=7`
  - `action_schema_version=0.1.0`
  - `control_mode=planned_delta_ee`
  - `controller_connected=false`
  - `sends_joint_commands=false`

Gate 4: FR3-to-PressButton Integration Planning

- Script: `scripts/check_fr3_press_button_readiness.py`
- Report: `outputs/fr3_press_button_readiness/report.json`
- Result summary:
  - `ready_for_real_fr3_press_button=true`
  - readiness is planning-only;
  - no runtime is started;
  - no USD is loaded by the readiness checker;
  - no joint command is sent.

### FR3 Controller Minimal Runtime Smoke

This gate has passed:

- `init_only` can load FR3, initialize the articulation wrapper, and read joint
  state.
- `hold_position` can command current joint targets for a bounded step window.
- `tiny_joint_nudge` can send one small bounded joint command.
- The gate remains non-benchmark and does not connect PressButton.

Reports:

- `outputs/fr3_controller_smoke/init_only_status.json`
- `outputs/fr3_controller_smoke/hold_position_status.json`
- `outputs/fr3_controller_smoke/tiny_joint_nudge_status.json`

### FR3 EE Controller Planning

This gate has passed:

- EE/TCP readiness identifies `/World/FR3/fr3_hand_tcp`.
- API discovery found kinematics, IK, and joint-space fallback routes.
- Recommended method is `kinematics_solver`.
- 7D action to EE target mapping is defined but sends no commands.
- Runtime readiness reports `ready_for_minimal_ee_runtime_smoke=true`.

Reports:

- `outputs/fr3_ee_controller_plan/readiness.json`
- `outputs/fr3_ee_controller_plan/api_discovery.json`
- `outputs/fr3_ee_controller_plan/action_mapping_report.json`
- `outputs/fr3_ee_controller_plan/runtime_readiness.json`

### FR3 Local Differential IK Diagnostic

This gate has passed:

- FK is available for the FR3 kinematics solver.
- A finite-difference translation Jacobian is available with shape `[3, 7]`.
- Damped least-squares target checks are safe for 0.25 mm, 0.5 mm, and 1 mm
  +/-X and +/-Z actions.
- FK validation passed for all checked tiny actions.
- One tiny 0.25 mm +X free-space motion was executed with
  `controller_method_used=differential_ik`.
- The status records `uses_lula_global_ik=false`,
  `uses_joint_space_fallback=false`, `safety_abort=false`, and
  `nan_detected=false`.

Reports:

- `outputs/fr3_differential_ik/jacobian_fk_probe.json`
- `outputs/fr3_differential_ik/target_report.json`
- `outputs/fr3_differential_ik/fk_validation_report.json`
- `outputs/fr3_differential_ik/tiny_diffik_motion_status.json`
- `docs/fr3_differential_ik_diagnostic.md`

## Current Boundaries

The following are still not implemented or not claimable:

- No real FR3 PressButton task rollout.
- No FR3 press-depth execution yet; the next permitted runtime mode is
  approach-only.
- No force-aware tactile benchmark.
- No real force/wrench observations.
- No tactile mount USD loaded.
- No 30-task expansion.
- No formal benchmark dataset.
- No paper-ready performance claim.
- No full Lightwheel runtime integration.

Existing PressButton success is still based on button displacement. That is a
geometric/runtime smoke signal, not tactile force evidence.

### FR3 EE-to-PressButton Planning

This planning gate connects the local differential IK envelope to the
PressButton geometry without commanding the robot to press:

- Geometry report:
  `outputs/fr3_press_button_planning/press_button_geometry_report.json`
- Co-scene load-only status:
  `outputs/fr3_press_button_planning/load_only_status.json`
- No-command waypoint plan:
  `outputs/fr3_press_button_planning/waypoint_plan.json`
- Approach readiness:
  `outputs/fr3_press_button_planning/approach_readiness.json`

The plan records `uses_differential_ik=true`,
`uses_lula_global_ik=false`, `uses_joint_space_fallback=false`,
`joint_command_sent=false`, `button_pressed=false`, and
`dataset_collection_allowed=false`.

The readiness checker now treats
`outputs/fr3_differential_ik/target_report.json` as the canonical differential
IK artifact. Its output includes `diffik_report_exists` and
`diffik_report_canonical` to avoid stale warnings from the older
`differential_ik_target_report.json` naming.

### FR3 PressButton Approach-Only Runtime Smoke

Status: completed.

The new approach-only runtime entrypoint is:

`scripts/run_fr3_press_button_approach_only_smoke.py`

It supports dry-run and guarded real Isaac Sim modes for
`micro_approach`, `short_approach`, `pre_press`, and `near_contact`. The stage
is strictly pre-press / near-contact only:

- no `press_target`;
- no press-depth execution;
- no button success requirement;
- no dataset collection;
- no fake force;
- no Lula global IK;
- no joint-space fallback.

Press runtime readiness is checked separately by
`scripts/check_fr3_press_button_press_readiness.py` after approach-only statuses
exist.

Passing artifacts:

- `outputs/fr3_press_button_approach_only/dry_run_status.json`
- `outputs/fr3_press_button_approach_only/micro_approach_status.json`
- `outputs/fr3_press_button_approach_only/short_approach_status.json`
- `outputs/fr3_press_button_approach_only/pre_press_status.json`
- `outputs/fr3_press_button_approach_only/near_contact_status.json`
- `outputs/fr3_press_button_approach_only/press_readiness.json`
- `docs/fr3_press_button_approach_only_runtime_smoke.md`

The real approach-only statuses record `joint_command_sent=true`,
`uses_differential_ik=true`, `uses_lula_global_ik=false`,
`uses_joint_space_fallback=false`, `button_pressed=false`,
`press_target_executed=false`, and `dataset_collection_allowed=false`.

## Current Test Baseline

Most recent full regression result from the FR3 Differential IK Diagnostic gate:

`pytest tests/` -> `255 passed, 1 warning`

The warning is an existing PyTorch/CUDA driver warning from StateBC training
tests, not an FR3 gate failure.

## Recommended Next Stage

Next stage: **FR3 PressButton Press Runtime Smoke**.

Recommended entry conditions:

- Keep pusher and EE placeholder paths passing.
- Keep 7D action schema unchanged.
- Use the introspection report to confirm the final EE/gripper/base frame
  choices.
- Introduce task interaction only behind the local differential IK controller
  path and `fr3_ee_controller_safety.yaml`.
- Move only to pre-press / near-contact in the next stage; do not execute press
  depth or collect datasets yet.
- Keep all outputs marked `benchmark_result=false` and
  `not_for_paper_claims=true`.
- Stop immediately if controller setup cannot identify safe frames or would
  require schema changes.

The approach gate should test runtime motion without changing the 7D action
schema, faking force, pressing the button, collecting a dataset, or turning the
diagnostic motion smoke into a benchmark result.

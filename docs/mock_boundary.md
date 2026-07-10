# Mock Boundary for Phase 1.5 / Phase 2C Sensor Contract

This repository state is a mock/stub benchmark contract. It is intended to
exercise APIs, schemas, registries, tactile contracts, metric aggregation,
dataset IO, replay policies, replay checks, and CI smoke tests without claiming
physical realism.

## Current Mock/Stub Modules

- `isaac_tactile_libero/envs/mock_env.py`: no Isaac Sim scene, no physics step,
  no contacts, no articulated dynamics. It only advances deterministic counters
  and mock robot arrays.
- `isaac_tactile_libero/tasks/`: task success is a deterministic mock/stub
  condition based on step count. Task metrics such as insertion depth and
  jamming count are placeholders.
- `isaac_tactile_libero/sensors/`: tactile modes return schema-compatible
  arrays and masks. Force, wrench, tactile RGB, and tactile depth are mock/stub
  values, not Lightwheel data and not physically rendered Isaac Sim outputs.
  The sensor contract is now hardened through `observation_spec()`,
  `metric_spec()`, normalization helpers, temporal history buffers, and
  calibration snapshots.
- `isaac_tactile_libero/datasets/`: HDF5 writer/reader/validation/replay uses a
  stable schema, but the stored episodes are generated from mock/stub env data.
- `isaac_tactile_libero/robots/fr3_tactile/`: robot metadata is static config.
  USD paths, collision settings, and articulation properties are placeholders.
- `isaac_tactile_libero/policies/random.py`: random actions are only for smoke
  and CI checks.
- `isaac_tactile_libero/policies/replay.py`: `ReplayPolicy` replays 7D actions
  from an HDF5 mock/stub dataset episode. It is a dataset-driven consistency
  baseline, not behavior cloning, not imitation learning, and not a trained
  model.
- `isaac_tactile_libero/policies/bc.py`: `StateBC`, `VisionBC`,
  `VisionStateBC`, `VisionForceBC`, `VisionVisuoTactileBC`,
  `VisionForceVisuoTactileBC`, and `OracleStateBC` are untrained mock/stub BC
  skeletons. They return schema-valid zero actions and record
  `is_trained=false`, `mock_or_stub=true`, and `untrained_mock_policy=true`.
  They are not behavior cloning results.
- `isaac_tactile_libero/policies/observation_filter.py` and
  `isaac_tactile_libero/policies/batch_builder.py`: these enforce and inspect
  modality contracts for future training. They do not optimize or fit a model.
- `isaac_tactile_libero/training/` and `scripts/train_bc.py`: two training
  paths are present. Dry-run training for BC skeletons writes
  `checkpoint_mock.json` metadata and does not optimize. The minimal
  `state_bc` path can run real PyTorch MSE optimization and write
  `checkpoint.pt` plus `checkpoint.json`, but it trains only on mock HDF5 data.
  It records `runtime_env=mock_dataset`, `dataset_is_mock=true`, and
  `not_for_paper_claims=true`.
- `isaac_tactile_libero/envs/lightwheel_wrapper.py`: planned optional
  Lightwheel adapter skeleton and capability probe only. By default it does not
  import Lightwheel. It checks configured paths and optional import
  availability only when requested, and it still does not download assets or
  implement real reset/step/read/evaluate runtime behavior.
- `isaac_tactile_libero/envs/isaacsim_backend_status.py`: Isaac Sim WebRTC
  visual-smoke readiness and runtime-status JSON only. It checks configuration
  and path placeholders for the `PressButton` smoke, but does not import
  `isaacsim`, `omni`, or `carb` by itself and does not run
  reset/step/read/evaluate.
- `isaac_tactile_libero/envs/isaacsim_press_button_env.py`: single-task
  `PressButton` runtime-loop boundary only. It can run a primitive Isaac Sim
  scene with a kinematic pusher placeholder, but it is not a real FR3
  controller. It can also run `robot_mode=ee_placeholder`, which replaces the
  sphere with simple wrist/finger primitives and records EE pose, but this is
  still kinematic placeholder geometry, not FR3 articulation. It now reports a
  best-effort PressButton displacement hook and explicit contact/fallback
  fields. It also exposes a PressButton-only PhysX
  contact-force probe, but `contact_force_available=false` and
  `contact_force_source=unavailable` unless a real contact-force API returns a
  parseable pusher-button force report. `geometric_contact_proxy=true` remains
  a fallback and is not tactile/contact sensing.
- `isaac_tactile_libero/robots/fr3_placeholder.py`: import-safe
  FR3/end-effector placeholder config and 7D action-to-EE-pose helper only. It
  does not import Isaac Sim, load a real FR3 USD, solve IK, control joints, or
  claim a real FR3-Tactile embodiment.
- `isaac_tactile_libero/robots/fr3_articulation_spec.py`: import-safe
  planning-only FR3 articulation contract. It validates config, frame, joint,
  asset-path, and action-schema metadata for a future real FR3 gate. It does
  not import Isaac Sim, load USD, create articulations, solve IK, control
  joints, or claim a connected real FR3 runtime.
- `isaac_tactile_libero/robots/fr3_runtime_controller.py` and
  `scripts/run_fr3_controller_smoke.py`: optional real Isaac Sim FR3
  controller smoke only. Base imports remain Isaac-safe. Non-dry-run can load
  `/World/FR3`, initialize an articulation wrapper, read joint state, command
  hold-position, and send one tiny bounded joint nudge. It does not connect
  PressButton, does not implement EE control/IK, does not collect datasets, and
  always records `benchmark_result=false` plus `not_for_paper_claims=true`.
- `isaac_tactile_libero/robots/fr3_ee_controller_plan.py`,
  `isaac_tactile_libero/robots/fr3_ee_action_mapping.py`,
  `scripts/check_fr3_ee_controller_readiness.py`,
  `scripts/probe_fr3_ee_controller_api.py`,
  `scripts/check_fr3_ee_action_mapping.py`, and
  `scripts/check_fr3_ee_runtime_readiness.py`: FR3 EE controller planning and
  API-discovery artifacts only. Dry-run paths do not import Isaac Sim.
  Non-dry-run API discovery may start Isaac Sim and load FR3 to inspect
  available kinematics/IK/fallback APIs, but it does not send joint commands,
  execute EE motion, connect PressButton, collect datasets, or create benchmark
  results.
- `isaac_tactile_libero/robots/fr3_ee_runtime_controller.py` and
  `scripts/run_fr3_ee_controller_smoke.py`: optional real Isaac Sim FR3 EE
  controller smoke only. Base imports remain Isaac-safe. Non-dry-run can load
  `/World/FR3`, read `/World/FR3/fr3_hand_tcp`, map a 7D zero action to a
  hold/no-op command, and send one tiny bounded free-space EE delta through the
  current joint-space fallback. It does not connect PressButton, does not
  collect datasets, does not use tactile mounts, does not fabricate force, and
  always records `benchmark_result=false` plus `not_for_paper_claims=true`.
- `isaac_tactile_libero/robots/fr3_differential_ik.py`,
  `scripts/probe_fr3_jacobian_fk.py`,
  `scripts/check_fr3_differential_ik_targets.py`,
  `scripts/validate_fr3_differential_ik_fk.py`, and
  `scripts/run_fr3_differential_ik_motion_smoke.py`: optional real Isaac Sim
  FR3 local differential IK diagnostic only. Dry-run paths do not import Isaac
  Sim. Non-dry-run can load `/World/FR3`, compute an FK finite-difference
  translation Jacobian, solve damped least-squares tiny EE deltas, validate
  with FK, and send one tiny free-space joint target after safety checks. It
  does not connect PressButton, collect datasets, use tactile mounts, fabricate
  force, or produce benchmark results. The diagnostic explicitly records
  `uses_lula_global_ik=false` and `uses_joint_space_fallback=false`.
- `isaac_tactile_libero/tasks/press_button_geometry.py`,
  `isaac_tactile_libero/tasks/fr3_press_button_planner.py`,
  `scripts/check_press_button_geometry.py`,
  `scripts/run_fr3_press_button_load_only_smoke.py`,
  `scripts/plan_fr3_press_button_waypoints.py`, and
  `scripts/check_fr3_press_button_approach_readiness.py`: FR3
  EE-to-PressButton planning only. Dry-run paths do not import Isaac Sim.
  Optional load-only/no-command runtime paths may start Isaac Sim to place FR3
  and the button in one scene or read the FR3 Jacobian, but they do not send
  joint commands, do not execute EE motion toward contact, do not press the
  button, do not collect datasets, and do not fabricate force. The waypoint
  plan explicitly records `uses_differential_ik=true`,
  `uses_lula_global_ik=false`, and `uses_joint_space_fallback=false`.
- `scripts/run_fr3_press_button_approach_only_smoke.py` and
  `scripts/check_fr3_press_button_press_readiness.py`: guarded FR3
  PressButton approach-only smoke. Dry-run paths do not import Isaac Sim.
  Non-dry-run may send tiny differential IK joint targets only toward
  `micro_approach`, `short_approach`, `pre_press`, or `near_contact`. It must
  not execute `press_target`, must not press the button, must not collect a
  dataset, must not fabricate force, and must keep `benchmark_result=false`.
- `isaac_tactile_libero/envs/isaacsim_contact.py`: import-safe helper for
  PressButton contact/displacement state and contact-force probing. It does not
  import Isaac Sim at base import time and does not fabricate contact-force
  availability from geometry or button displacement.
- `isaac_tactile_libero/envs/isaacsim_contact_force.py`: second-stage
  contact-force backend abstraction for `contact_sensor`,
  `physx_contact_report`, and `rigid_contact_view` candidates. It is
  import-safe and reports `unavailable` with method errors instead of
  fabricating force.
- `isaac_tactile_libero/sensors/runtime_tactile_adapter.py`: import-safe schema
  adapter that maps PressButton runtime contact state into `obs["tactile"]`.
  It keeps force/wrench arrays zero and `mask.has_force=false` whenever real
  contact force is unavailable. It does not turn displacement or geometric
  contact proxy into tactile force.
- `isaac_tactile_libero/envs/backend_status.py` and
  `scripts/probe_lightwheel_backend.py`: JSON-serializable probe reports only.
  They do not create Isaac Sim environments, run physics, or produce benchmark
  results.
- `scripts/check_isaacsim_webrtc_ready.py`,
  `scripts/run_press_button_visual_smoke.py`, and
  `scripts/launch_isaacsim_webrtc_smoke.sh`: visual-smoke helpers only.
  `check_isaacsim_webrtc_ready.py` and `run_press_button_visual_smoke.py
  --dry-run` do not import or launch Isaac Sim. A non-dry-run
  `run_press_button_visual_smoke.py` can attempt a minimal static PressButton
  scene only after local Isaac Sim paths are configured. The shell script is a
  user-run template and does not modify firewall, Docker, driver, or system
  network settings.
- `scripts/run_press_button_runtime_loop.py`: single-task runtime-loop helper.
  Dry-run does not import Isaac Sim. Non-dry-run starts Isaac Sim only after
  readiness passes and produces runtime status, optional rollout JSON, and
  optional screenshot.
- `scripts/collect_press_button_runtime_demos.py`: PressButton-only
  runtime-smoke dataset collector. Dry-run writes schema-compatible proxy
  episodes without starting Isaac Sim. Non-dry-run may run the current
  single-task Isaac Sim PressButton loop, but still writes
  `runtime_smoke=true`, `benchmark_result=false`, and
  `not_for_paper_claims=true`. It uses the existing HDF5 schema and records
  force as unavailable unless real force data exists; button displacement is
  never written into force or wrench arrays.
- `scripts/replay_runtime_dataset.py`: runtime-smoke HDF5 consistency checker.
  It verifies readback, action shape, observation/tactile schema, and success
  label consistency. It does not reproduce real physics and does not replay
  Isaac Sim.
- `scripts/evaluate_runtime_dataset.py`: offline runtime-smoke dataset
  evaluation/sanity report. It reads HDF5 only, writes `metrics.json`,
  `summary.csv`, and `dataset_eval_report.json`, and records
  `benchmark_result=false` plus `not_for_paper_claims=true`. It does not start
  Isaac Sim, does not replay physics, and does not convert button displacement
  into force or wrench.
- `scripts/probe_press_button_contact_force.py`: PressButton-only contact-force
  probe helper. Dry-run does not import Isaac Sim. Non-dry-run attempts to read
  PhysX contact/force information and records `contact_api_error` when the API
  is unavailable. It is not a force-based tactile benchmark.
- `scripts/probe_isaac_contact_force_second.py` and
  `scripts/probe_isaac_contact_force_scene.py`: second-stage contact-force
  probes. Dry-run does not start Isaac Sim. Non-dry-run may create a minimal
  diagnostic scene or run PressButton, but reports are still probe artifacts and
  not benchmark results. Minimal-scene force, if found, must not be copied into
  PressButton force fields.
- `scripts/probe_fr3_assets.py`: planning-only FR3 asset/config probe. It
  checks YAML fields, asset paths, and manifest/provenance metadata. It does
  not import `isaacsim`, `omni`, or `carb`, does not load USD, does not create
  an articulation, and does not connect a controller.
- `scripts/discover_isaac_assets.py`: filesystem-only Isaac asset discovery. It
  searches configured roots for local asset candidates and recommends a path,
  but does not import Isaac Sim, load USD, copy assets, or create runtime state.
- `scripts/evaluate.py --backend isaacsim_press_button`: unified-eval entry
  point for the same single-task `PressButton` runtime smoke. The default
  backend remains `mock` for CI and benchmark-contract checks. The
  `isaacsim_press_button` backend is optional, PressButton-only, records
  `single_task_runtime_smoke=true`, `benchmark_result=false`, and
  `not_for_paper_claims=true`. It records `success_source` so fallback success
  cannot be confused with physics/contact success.
- `isaac_tactile_libero/assets/manifest.py` and `assets/asset_manifest.csv`:
  asset provenance metadata only. They validate source/license/attribution
  fields but do not download or redistribute assets.
- `isaac_tactile_libero/metrics/`: metrics are computed from mock episode
  records. They are useful for validating output schema and aggregation code,
  but must not be used as paper conclusions.
- `scripts/smoke_test.py` and `scripts/evaluate.py`: both run mock/stub episodes
  through the public API. `scripts/evaluate.py --policy replay` reads dataset
  actions and runs a mock runtime consistency check. Their outputs are CI gates,
  not benchmark results.

## Configs

- `configs/tactile/calibration_default.yaml` documents tactile mode defaults for
  software plumbing only. It includes units, threshold, noise, latency, dropout,
  normalization, history, and sensor/schema version. It is not real Lightwheel
  calibration.
- `configs/eval/mock_default.yaml` defines a lightweight mock evaluation grid
  for 5 tasks x 4 tactile modes x seeds 0, 1, 2.
- `configs/eval/replay_mock.yaml` defines a lightweight replay-evaluation
  config over `outputs/mock_dataset/mock_v0.hdf5`.
- `configs/eval/press_button_runtime_smoke.yaml` defines the unified-eval
  PressButton runtime-smoke entry point. It defaults to
  `backend=isaacsim_press_button`, `dry_run_runtime=true`, and
  `benchmark_result=false`.
- `configs/eval/press_button_runtime_dataset_eval.yaml` defines the offline
  HDF5-only evaluation sanity pass for the three-episode PressButton
  runtime-smoke dataset. It is not a benchmark or paper-results config.
- `configs/dataset/press_button_runtime_smoke_10ep.yaml` defines a
  10-episode PressButton runtime-smoke mini-scale gate for GPU/runtime/HDF5
  stability checks. It is not a formal benchmark dataset configuration.
- `configs/dataset/press_button_ee_placeholder_smoke_10ep.yaml` defines the
  10-episode `robot_mode=ee_placeholder` runtime-smoke gate. It uses kinematic
  wrist/finger placeholder geometry, keeps `real_fr3_articulation=false`, and
  is not a formal FR3 benchmark dataset configuration.
- `configs/policies/baselines_mock.yaml` defines the seven untrained BC
  skeleton contracts and modality declarations.
- `configs/train/bc_mock.yaml` defines a dry-run training protocol config. It
  defaults to `dry_run=true` and is not a real training recipe.
- `configs/train/state_bc_minimal.yaml` defines the narrow real StateBC training
  slice on mock data. It defaults to `dry_run=false`, uses only robot-state
  features, and requires the optional `train` extra for PyTorch.
- `configs/backend/lightwheel_optional.yaml` declares a planned optional
  Lightwheel backend probe. It defaults to `enabled=false`,
  `allow_runtime_import=false`, `probe_only=true`, and
  `runtime_status=probe_only_not_connected`.
- `configs/backend/isaacsim_visual_smoke.yaml` declares the planned
  `PressButton` Isaac Sim WebRTC smoke. It defaults to
  `runtime_status=planned_not_connected`, `use_lightwheel_assets=false`,
  `allow_lightwheel_assets=false`, `webrtc_enabled=true`, and
  `headless_streaming=true`; local Isaac Sim paths remain user-configured.
- `configs/robots/fr3_real_articulation.yaml` declares the planning contract
  for a future real FR3 articulation. It keeps `robot_mode` as
  `real_fr3_articulation_planned`, preserves `action_schema_version=0.1.0`,
  leaves real asset paths unset by default, and records
  `benchmark_result=false` plus `not_for_paper_claims=true`.
- `configs/dataset/mock_dataset.yaml` defines a lightweight mock dataset grid
  and writes the tactile calibration snapshot into HDF5 metadata.

## Future Replacement Points

- Replace `MockIsaacTactileLiberoEnv` with an Isaac Sim / Isaac Lab backend that
  owns scene loading, physics stepping, reset distributions, and robot control.
- Replace task placeholder success and metrics with task-specific success
  checks, failure checks, and contact-aware metric computation from simulator
  state.
- Replace tactile sensor mocks with Isaac-rendered tactile approximations,
  Lightwheel-compatible adapter interfaces, or recorded sensor streams. Future
  backends should replace `build()` and `read()` while preserving observation
  keys, units, normalization config, history semantics, masks, and dataset
  snapshot paths unless the schema version is intentionally bumped.
- Replace static FR3 metadata with validated USD/articulation configs and frame
  transforms.
- Keep the public observation schema, action schema, registry API, evaluation
  output files, and dataset contracts stable unless the benchmark version is
  intentionally bumped.
- Keep the policy/evaluate API stable so future real Isaac Sim / Lightwheel
  backends can reuse `RandomPolicy`, `ReplayPolicy`, and dataset-driven
  evaluation entry points with real data sources.

## Explicit Non-Claims

- No real physics simulation is implemented here.
- No real Lightwheel integration is implemented here.
- Lightwheel is a planned optional backend and compatibility target, not a
  connected runtime in the current repository state.
- The Lightwheel probe is not a runtime. It has no physics, no reset/step, no
  real sensor read, no task rollout, and no benchmark result.
- Isaac Sim WebRTC visual smoke is only a single-task static scene smoke. Even
  if it starts a `SimulationApp`, it is not benchmark evaluation, has no
  reset/step/read loop, and must not be used as a paper performance claim.
- The `PressButton` runtime loop is the first real Isaac Sim single-task loop,
  but it still uses a primitive scene, a kinematic pusher placeholder, and a
  geometric fallback. The displacement hook and contact-force probe are steps
  toward physics/contact integration, not final tactile benchmark results.
- Runtime tactile schema mapping is only a compatibility bridge. With
  `contact_force_available=false`, `force_wrench` mode still does not contain
  real force/wrench measurements and must not be reported as a force-aware
  benchmark result.
- Contact-force second-probe outputs are diagnostic artifacts. A successful
  minimal-scene force read is not a PressButton tactile result. PressButton can
  be described as force-aware only after its own runtime observations carry
  `contact_force_available=true`, `force_source=<actual_method>`, and
  `mask.has_force=true`.
- PressButton runtime-smoke datasets are HDF5 schema/readback checks only. They
  are not official demonstrations, not a released benchmark dataset, and not a
  paper-result dataset. Dry-run episodes are schema proxies. Real runtime
  episodes still use the current placeholder pusher and displacement success
  hook while contact force remains unavailable.
- PressButton runtime-smoke dataset evaluation is an offline software
  compatibility check. It can report replay-policy consistency, StateBC batch
  sanity, and dry-run training protocol compatibility, but those reports are not
  physical replay results and are not benchmark scores.
- The PressButton 10-episode mini-scale gate remains a runtime-smoke stability
  check. Even when collected from real Isaac Sim, it still uses the placeholder
  pusher, displacement success hook, unavailable force/wrench masks, and
  `benchmark_result=false`.
- The PressButton 50-episode stability gate is still a runtime-smoke scaling
  check, not a formal benchmark dataset. It must preserve
  `benchmark_result=false`, `not_for_paper_claims=true`,
  `force_source=unavailable`, `mask.has_force=false`, and
  `no_fake_force_from_displacement=true`.
- The FR3/end-effector placeholder transition is not a real robot benchmark.
  `robot_mode=ee_placeholder` must be reported with `placeholder_robot=true`
  and `real_fr3_articulation=false`.
- FR3 local asset discovery and config binding only records paths and
  provenance. It does not load USD, create an articulation, or connect a
  controller.
- FR3 load-only visual smoke may start Isaac Sim and load `/World/FR3`, but it
  is visual inspection only: `controller_connected=false`,
  `articulation_control_enabled=false`, `press_button_connected=false`, and
  `benchmark_result=false`.
- FR3 articulation introspection is diagnostic. It may traverse stage prims and
  report joint/link/frame candidates, but it does not send commands or attach a
  controller.
- The FR3 control contract maps the existing 7D action schema to a future EE
  delta interface. It keeps `sends_joint_commands=false` and is not a
  controller implementation.
- The FR3 differential IK diagnostic is a tiny free-space controller smoke. It
  demonstrates bounded local Jacobian-based motion, but it is not a task
  controller, not tactile sensing, and not a benchmark result.
- The FR3 PressButton press runtime smoke is still a gate, not a benchmark.
  Partial 2mm, partial 10mm, and full press can exercise local differential IK,
  but success is a geometric `button_displacement` proxy and force/wrench remain
  unavailable. The press-and-retract gate is currently blocked, so dataset
  collection and benchmark evaluation remain disabled.
- FR3-to-PressButton readiness is planning-only. It can summarize missing
  reports, frames, and controller prerequisites, but it does not run PressButton
  with a real FR3.
- Unified eval with `--backend isaacsim_press_button` is a single-task runtime
  smoke wrapper, not a formal benchmark score. Dry-run runtime does not start
  Isaac Sim or execute a step loop, and geometric fallback success is not
  tactile or physical contact evidence.
- The repository is not a Lightwheel fork.
- No VisionBC, ACT, Diffusion Policy, VLA, or other large-model training
  pipeline is implemented. A minimal StateBC MLP trainer exists only for mock
  dataset software validation.
- ReplayPolicy is not a learned policy and does not estimate missing actions.
- Untrained BC skeleton evaluation is not a BC benchmark score and must not be
  used for paper performance conclusions.
- `checkpoint_mock.json` is metadata only and is not a trained model checkpoint.
- `checkpoint.json` plus `checkpoint.pt` can represent a real StateBC training
  run on mock data. It is not a real Isaac Sim / Lightwheel benchmark model and
  must not be used for paper performance conclusions.
- Mock metrics are not evidence for paper claims about tactile manipulation.
- Mock tactile arrays and mock datasets are not real demonstrations.

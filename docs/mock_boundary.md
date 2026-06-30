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
  controller and currently uses `geometric_contact_proxy=true` rather than real
  tactile/contact sensing.
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
  optional screenshot. It is intentionally not wired into the main mock
  `evaluate.py` benchmark path yet.
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
  geometric contact proxy. It is not a final tactile benchmark result.
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

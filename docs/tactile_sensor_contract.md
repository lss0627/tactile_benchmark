# Tactile Sensor Contract

## Scope

This document describes the current mock/stub tactile sensor contract. It is
designed so a future Isaac Sim or Lightwheel backend can replace sensor
construction and reading while keeping schemas stable.

The current code is not real tactile simulation and not real Lightwheel data.
Mock values must not be used for paper experiment conclusions.

## Public API

Every tactile mode keeps the same API:

- `build(robot, scene, cfg)`
- `reset(env_ids)`
- `read()`
- `observation_spec()`
- `metric_spec()`

Future real backends should primarily replace `build()` and `read()`.

## Modes

- `none`: no tactile modality, masks are false.
- `force_wrench`: force, wrench, and contact flags.
- `visuotactile`: tactile RGB and depth-style image tensors.
- `force_plus_visuotactile`: force/wrench plus tactile RGB/depth.

## Observation Schema

`observation_spec()` declares each tactile field with:

- field name
- shape
- dtype
- unit
- required flag for the mode
- mock flag

The fields match the public observation schema:

- `valid`
- `contact_flag_left`, `contact_flag_right`
- `force_left`, `force_right`
- `wrench_left`, `wrench_right`
- `vt_rgb_left`, `vt_rgb_right`
- `vt_depth_left`, `vt_depth_right`
- `force_field_left`, `force_field_right`
- `mask`

Units are Newton for force, Newton-meter for torque components, meters for
depth, and `uint8 RGB` for tactile RGB. Missing modalities are represented by
zero-filled dataset arrays plus boolean masks.

## Metric Schema

`metric_spec()` declares:

- `contact_flag`
- `max_contact_force`
- `mean_contact_force`
- `force_violation_rate`
- `contact_duration`
- `contact_loss_count`
- `jamming_count`
- `insertion_depth`

The spec records whether a metric is sensor-provided, task-derived, or
evaluator-derived. In the mock runtime, force modes provide contact flags.
Aggregate force/contact metrics are evaluator-derived from observations.
Assembly metrics are task/evaluator-derived placeholders.

## Calibration Config

`configs/tactile/calibration_default.yaml` includes:

- `sensor_version`
- `schema_version`
- `units`
- `normalization`
- `history`
- `latency`
- `dropout`
- `noise`
- `threshold`
- per-mode configs

This is a mock/stub software contract, not real calibration.

## Normalization

`isaac_tactile_libero.sensors.normalization.SensorNormalization` supports:

- force normalization
- wrench normalization
- tactile image normalization
- force-field normalization

Parameters come from the calibration config. The formula is `(value - bias) /
scale`.

## Temporal History

`isaac_tactile_libero.sensors.history.SensorHistory` provides runtime-agnostic
ring buffers for:

- force history
- wrench history
- visuotactile image history with shape checks

The current history code is only a contract helper. It does not simulate sensor
latency or physical sensor dynamics.

## Dataset Snapshot

Mock dataset collection writes the tactile calibration snapshot into:

- `/metadata/dataset_info`
- `/metadata/creation_config`
- `/episodes/{episode_id}/metadata`

`validate_dataset.py` checks that tactile masks are consistent with each saved
episode's `tactile_mode`.

## Non-Claims

- No real Isaac Sim tactile physics is implemented.
- No real Lightwheel integration is implemented.
- No tactile data here is suitable for paper result claims.
- The repository is not a Lightwheel fork.

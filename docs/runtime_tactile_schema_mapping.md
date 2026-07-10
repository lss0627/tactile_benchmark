# Runtime Tactile Schema Mapping

This stage maps the optional single-task PressButton runtime contact state into
the existing tactile observation schema. It is a schema compatibility layer, not
a new tactile sensor and not a force-aware benchmark result.

## Adapter

`isaac_tactile_libero/sensors/runtime_tactile_adapter.py` converts PressButton
runtime fields into `obs["tactile"]` without importing Isaac Sim. It supports:

- `none`
- `force_wrench`

Runtime `visuotactile` and `force_plus_visuotactile` are not implemented. The
PressButton runtime backend should reject unsupported runtime tactile modes
instead of pretending to provide image tactile streams.

## Force Unavailable Path

When `contact_force_available=false`, the adapter must report:

- `force_left = [0, 0, 0]`
- `force_right = [0, 0, 0]`
- `wrench_left = [0, 0, 0, 0, 0, 0]`
- `wrench_right = [0, 0, 0, 0, 0, 0]`
- `mask.has_force=false`
- `mask.has_wrench=false`
- `force_source=unavailable`

Button displacement, pusher/button distance, and geometric contact proxy are
never converted into force or wrench values.

## Contact Flags

`contact_flag_left` and `contact_flag_right` may come from non-force runtime
signals, but the source is explicit:

- `button_displacement`
- `contact_signal_proxy`
- `physics_contact_force`
- `contact_sensor`
- `physx_contact_report`
- `rigid_contact_view`
- `none`

The source must not be `force_threshold` unless real force is available and the
threshold logic has actually been implemented.

## Metadata

Runtime tactile observations add top-level metadata while keeping the core
tactile dictionary compatible with the existing observation schema:

- `tactile_mode`
- `tactile_schema_version`
- `contact_flag_source`
- `force_source`
- `contact_force_available`
- `physics_contact_available`
- `button_displacement_available`
- `using_geometric_fallback`

The existing `mask` keys are unchanged to avoid breaking mock datasets,
baseline policies, replay, and validators.

Runtime-smoke HDF5 datasets store per-episode runtime tactile metadata inside
`/episodes/{episode_id}/metadata/json` and dataset-level summaries inside
`/metadata/dataset_info`. This keeps the observation and dataset schema stable:
the per-timestep tactile arrays remain the existing `valid`, contact flags,
force/wrench arrays, visuotactile placeholders, force-field placeholders, and
mask fields.

When a runtime-smoke dataset declares `force_source=unavailable`, validators
require `mask.has_force=false`, `mask.has_wrench=false`, and zero-safe
force/wrench arrays for every timestep. Button displacement may explain
`contact_flag_source=button_displacement` and `success_source=button_displacement`,
but it must never be encoded as force or wrench.

## Non-Claims

Current PressButton runtime can expose button displacement and contact-signal
metadata, but `contact_force_available=false` in the current real run. This is
therefore not a force-based tactile benchmark. Future PhysX contact force or
Isaac contact sensor integration can set `mask.has_force=true` only after real
force values are available.

When real PressButton force is available, `force_source` and
`contact_flag_source` use the actual runtime method label such as
`contact_sensor`, `physx_contact_report`, or `rigid_contact_view`. If force is
available only in an independent minimal probe scene, PressButton observations
must still report `force_source=unavailable` and `mask.has_force=false`.

The current second probe did not obtain a real force vector from the minimal
scene, so PressButton runtime observations remain on the force-unavailable path.

# PressButton Contact Force Probe

This stage adds a PressButton-only Isaac Sim / PhysX contact-force probe. It is
an optional runtime diagnostic path, not a benchmark result and not a tactile
sensor implementation.

The second probe stage adds `scripts/probe_isaac_contact_force_second.py` and
the import-safe backend abstraction
`isaac_tactile_libero/envs/isaacsim_contact_force.py`. It tries
`contact_sensor`, `physx_contact_report`, and `rigid_contact_view` candidate
methods and records `unavailable` with a clear `contact_api_error` when a method
cannot provide force.

## Scope

- `scripts/probe_press_button_contact_force.py` can run in dry-run mode without
  starting Isaac Sim.
- Non-dry-run runs the existing scripted PressButton runtime loop and records
  contact/force probe fields.
- The helper in `isaac_tactile_libero/envs/isaacsim_contact.py` stays
  import-safe. It imports `omni.physx` only inside the real runtime read path.
- The probe records prim paths:
  - `/World/KinematicPusher_Placeholder`
  - `/World/PressButton_RedPrimitive`
  - `/World/PressButton_RedPrimitive` as the current button-top probe path

## Force Contract

The probe must not fabricate force from geometry or displacement. If a parseable
PhysX contact report is unavailable, outputs must say:

- `physics_contact_available=false`
- `contact_force_available=false`
- `contact_force_source=unavailable`
- `contact_api_error=<clear message>`

If a future Isaac Sim API returns a parseable pusher-button contact record, the
runtime may report:

- `physics_contact_available=true`
- `contact_signal_seen=true`
- `contact_force_available=true`
- `contact_force_norm`
- `max_contact_force_norm`
- `mean_contact_force_norm`
- `contact_force_unit=N`
- `contact_force_confirmed=true` when force norm is positive
- `contact_force_source=<actual_method>`, for example
  `physx_contact_report` or `rigid_contact_view`

The runtime tactile adapter maps these values into `obs["tactile"]`. Until
`contact_force_available=true`, force and wrench arrays remain zero-safe and
`mask.has_force=false`.

The PressButton runtime dataset smoke reuses the same contract. Its HDF5
metadata records `force_source=unavailable`,
`contact_force_available=false`, and `benchmark_result=false`. Validators fail
the dataset if button displacement is written into force or wrench arrays while
force is unavailable.

The independent minimal second-probe scene must not be treated as PressButton
force. PressButton `force_wrench` mapping changes to `mask.has_force=true` only
after the PressButton runtime itself reports a real force vector.

Current second-probe result: the minimal scene ran but did not return a
parseable contact-force vector. Therefore PressButton remains
`contact_force_available=false`, `force_source=unavailable`, and
`mask.has_force=false`.

## Success Source

Success priority remains:

1. `button_displacement`
2. `physics_contact`
3. `geometric_fallback`

Contact force is currently an additional metric/hook. It is not the only success
condition, and it is not a replacement for tactile sensing.

## Commands

Dry-run:

```bash
python scripts/probe_press_button_contact_force.py \
  --dry-run \
  --runtime-config configs/backend/isaacsim_visual_smoke.yaml \
  --max-steps 20 \
  --output outputs/press_button_contact_force_probe/dry_run_report.json
```

Real runtime, only after Isaac Sim is configured:

```bash
conda run -n isaac python scripts/probe_press_button_contact_force.py \
  --runtime-config outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml \
  --headless \
  --webrtc \
  --max-steps 80 \
  --save-rollout-json \
  --output outputs/press_button_contact_force_probe/report.json
```

## Non-Claims

- The pusher is still a placeholder, not FR3 control.
- `button_displacement_available=true` is not force sensing.
- `contact_force_available=false` means the current run is not a force-based
  tactile benchmark.
- Runtime tactile schema mapping does not change that: unavailable force remains
  unavailable in the tactile observation mask and metadata.
- Runtime-smoke HDF5 collection is a schema/readback check, not a formal
  demonstration dataset and not a force-aware benchmark dataset.
- Even if force probing succeeds, this still does not connect a real tactile
  sensor, full robot controller, Lightwheel runtime, or official benchmark
  dataset.

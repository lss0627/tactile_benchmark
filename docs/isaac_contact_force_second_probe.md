# Isaac Contact Force Second Probe

This stage probes Isaac Sim contact-force options before making any force-aware
benchmark claim. It is a diagnostic path only: no dataset collection, no model
training, no 30-task expansion, no full Lightwheel runtime, and no full FR3
controller.

## API Discovery

The discovery artifact is:

```text
outputs/contact_force_second_probe/api_discovery.json
```

Discovery is import-spec only and does not create a `SimulationApp`. In the
current Isaac conda environment, base discovery found `isaacsim` and `omni`, but
the concrete candidate modules were not importable before SimulationApp startup:

- `contact_sensor`: no importable `isaacsim.sensors.physics` or
  `omni.isaac.sensor` module at discovery time
- `physx_contact_report`: no importable `omni.physx` module at discovery time
- `rigid_contact_view`: no importable `isaacsim.core` or `omni.isaac.core`
  rigid contact view module at discovery time

This does not prove the APIs are absent at runtime. Isaac warns that many
Omni/Isaac imports must occur after `SimulationApp` is created, so the runtime
probe repeats method checks inside the real runtime.

## Backend Abstraction

`isaac_tactile_libero/envs/isaacsim_contact_force.py` is import-safe and exposes
these method labels:

- `contact_sensor`
- `physx_contact_report`
- `rigid_contact_view`
- `unavailable`

`ContactForceBackend(method="auto")` tries the candidate methods in order and
returns a JSON-safe `ContactForceReport`. If a method is missing or has an
unsupported signature, the report records `contact_api_error` and falls through
without crashing.

## Current Runtime Result

On the GPU1 Isaac conda run, the minimal scene executed and wrote:

```text
outputs/contact_force_second_probe/minimal_force_report_gpu1.json
```

The run did not read a contact-force vector:

- `contact_force_available=false`
- `contact_probe_method=unavailable`
- `contact_force_source=unavailable`
- `max_contact_force_norm=0.0`
- `mean_contact_force_norm=0.0`

Runtime method notes from `contact_api_error`:

- `contact_sensor`: module was importable after SimulationApp startup, but no
  configured ContactSensor prim/read API is wired in this probe.
- `physx_contact_report`: the runtime exposed contact-like methods such as
  `get_contact_report`, `get_full_contact_report`,
  `subscribe_contact_report_events`, and
  `subscribe_full_contact_report_events`, but no supported method returned a
  parseable pusher-target force record.
- `rigid_contact_view`: `RigidContactView` was importable, but this minimal
  probe has no initialized view binding.

Because the minimal scene did not produce force, PressButton force remains
unavailable and the PressButton scene is not claimed as force-aware.

## CLI

Dry-run:

```bash
python scripts/probe_isaac_contact_force_second.py \
  --dry-run \
  --runtime-config configs/backend/isaacsim_visual_smoke.yaml \
  --method auto \
  --scene minimal \
  --max-steps 20 \
  --output outputs/contact_force_second_probe/dry_run_minimal_report.json
```

Minimal real scene:

```bash
CUDA_VISIBLE_DEVICES=1 conda run -n isaac python scripts/probe_isaac_contact_force_second.py \
  --runtime-config outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml \
  --method auto \
  --scene minimal \
  --headless \
  --webrtc \
  --max-steps 120 \
  --save-rollout-json \
  --save-screenshot \
  --output outputs/contact_force_second_probe/minimal_force_report_gpu1.json
```

PressButton real scene, only if the minimal scene exposes a usable force method:

```bash
CUDA_VISIBLE_DEVICES=1 conda run -n isaac python scripts/probe_isaac_contact_force_second.py \
  --runtime-config outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml \
  --method auto \
  --scene press_button \
  --headless \
  --webrtc \
  --max-steps 120 \
  --save-rollout-json \
  --save-screenshot \
  --output outputs/contact_force_second_probe/press_button_force_report_gpu1.json
```

## Tactile Mapping

If PressButton force becomes available in the actual PressButton scene:

- `contact_force_available=true`
- `force_source=<actual_method>`
- `mask.has_force=true`
- `force_left` and `force_right` contain the reported force vector
- `contact_flag_source=<actual_method>`

If force is available only in the independent minimal scene, PressButton remains
`force_source=unavailable` and `mask.has_force=false`. Minimal-scene force must
not be copied into PressButton rollout metrics or datasets.

If all methods remain unavailable, force/wrench arrays stay zero-safe and no
force-aware benchmark claim is allowed.

## Non-Claims

- This stage is not a benchmark result.
- It is not a force-aware tactile benchmark unless PressButton itself reports
  real force values and the tactile mask switches to `mask.has_force=true`.
- Button displacement and geometric proximity are never written as force.
- Minimal-scene force, if found, is only an API discovery result until the same
  method is wired into PressButton.

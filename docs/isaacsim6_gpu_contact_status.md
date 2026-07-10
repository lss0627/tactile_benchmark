# Isaac Sim 6.0.1 GPU Contact Status

The development baseline intentionally uses GPU 0 for RTX rendering and CPU
physics for the experimental Contact Sensor.

Observed on 2026-07-10 with driver 550.144.03:

- CPU physics: 100/100 ready/contact/release cycles passed, zero stale handles,
  finite scalar force and raw position/normal/impulse data.
- Requested CUDA device with GPU dynamics disabled: effective physics device
  remained CPU and the Contact probe passed; this is not GPU Contact evidence.
- Native GPU dynamics with the multi-GPU configuration crashed or hung in the
  PhysX/Fabric initialization path.
- Native single-GPU dynamics also failed to reach a usable Contact result.

Therefore `validate_contact_physics_policy()` returns
`GPU_CONTACT_NATIVE_INSTABILITY` for any non-CPU device before creating
`SimulationApp`. This prevents a native crash while preserving GPU rendering.

The driver is called `UNVALIDATED`, not absolutely unsupported. NVIDIA's
tested/reference 6.0.1 driver recorded by this migration is 595.58.03. Native
GPU Contact and release-level physical/data/replay/evaluation evidence must be
rerun on a validated/reference driver before this blocker can be removed.

No scalar force magnitude or raw impulse is promoted into the public 3D force
or 6D wrench fields. Their validity masks remain false.

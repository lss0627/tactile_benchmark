# Lightwheel Integration Plan

This repository treats Lightwheel / LW-BenchHub as a reference implementation,
optional backend, compatibility target, and possible asset source for
non-commercial research. It is not a Lightwheel fork.

## Role

- Reference implementation: compare task organization, asset conventions, and
  LIBERO-style manipulation patterns.
- Optional backend: future code may wrap a user-provided Lightwheel checkout
  through an adapter layer.
- Compatibility target: keep observation/action/dataset/evaluation contracts
  suitable for a future Lightwheel-compatible runtime.
- Asset source: Lightwheel or LW-BenchHub assets may be referenced for
  non-commercial research only when their upstream license and attribution are
  preserved.

## Project Boundary

The main contribution remains Isaac-Tactile-LIBERO's tactile sensor interface,
contact-rich task definitions, schema-stable HDF5 dataset format,
contact-aware metrics, replay/evaluation protocol, and baseline policy
protocol. Lightwheel integration is a planned optional backend path, not the
identity of this project.

## Current Status

Current tests, training, dataset collection, and evaluation still use the
mock/stub runtime. No Lightwheel runtime is imported, no Lightwheel assets are
downloaded, and no real Isaac Sim scene is executed.

## Future Adapter Direction

A future backend should replace only environment build/reset/step/read/evaluate
implementation details while preserving public schemas and dataset contracts.
The adapter should accept local paths from `configs/backend/lightwheel_optional.yaml`
and should fail clearly when Lightwheel is absent.

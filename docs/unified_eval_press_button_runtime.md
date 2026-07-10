# Unified Eval PressButton Runtime Smoke

This path connects the single-task PressButton runtime loop to `scripts/evaluate.py`.
It is an optional runtime smoke entry point, not a formal Isaac-Tactile-LIBERO
benchmark result.

## Scope

- Default evaluation remains `--backend mock`.
- `--backend isaacsim_press_button` is restricted to `PressButton`.
- The runtime backend writes `metrics.json`, `summary.csv`, `runtime_status.json`,
  and `rollout.json`.
- Every artifact is marked with `benchmark_result=false`,
  `single_task_runtime_smoke=true`, and `not_for_paper_claims=true`.

## Dry-Run Runtime

Dry-run is the CI-safe mode:

```bash
python scripts/evaluate.py \
  --task PressButton \
  --backend isaacsim_press_button \
  --policy scripted \
  --dry-run-runtime \
  --runtime-config configs/backend/isaacsim_visual_smoke.yaml \
  --max-steps 20 \
  --output outputs/eval_press_button_runtime_smoke
```

Dry-run does not start Isaac Sim, does not create a SimulationApp, does not execute
the step loop, and does not prove real PressButton success. Its rollout is an
explicit empty runtime-smoke record.

## Real Runtime

Real runtime is optional and must only be attempted after the WebRTC visual smoke
gate has been checked on a machine with a configured Isaac Sim Python/app path and
compatible NVIDIA GPU:

```bash
python scripts/evaluate.py \
  --task PressButton \
  --backend isaacsim_press_button \
  --policy scripted \
  --runtime-config configs/backend/isaacsim_visual_smoke.yaml \
  --max-steps 80 \
  --output outputs/eval_press_button_runtime_real
```

The current runtime loop uses a primitive button scene, a kinematic placeholder
pusher, a best-effort button displacement hook, an optional PhysX contact-force
probe, and a geometric fallback. It does not provide real FR3 control, real
tactile sensing, or paper-grade contact-rich evaluation.

Runtime artifacts include `success_source`. Treat it as follows:

- `button_displacement`: success came from the PressButton displacement hook;
- `physics_contact`: success came from a real contact signal when available;
- `geometric_fallback`: success came from the geometric proxy and is not a
  physics-contact result;
- `none`: no success source fired.

If `physics_contact_available=false` or `contact_force_available=false`, do not
describe the rollout as force/contact-sensor based.

Unified eval writes contact-force probe fields to `metrics.json`,
`summary.csv`, `runtime_status.json`, and `rollout.json`, including:

- `contact_force_available`
- `max_contact_force_norm`
- `mean_contact_force_norm`
- `contact_force_source`
- `contact_probe_method`
- `contact_api_error`

If `contact_force_source=unavailable`, the run remains displacement/proxy based
and must not be described as a force-based tactile benchmark.

For `--tactile force_wrench`, unified eval also writes runtime tactile schema
metadata:

- `tactile_mode`
- `tactile_schema_version`
- `force_source`
- `contact_flag_source`
- `mask.has_force`
- `mask.has_wrench`

When force is unavailable, `force_left`, `force_right`, `wrench_left`, and
`wrench_right` stay zero-safe and the mask remains false. Button displacement is
allowed to set contact flags only with `contact_flag_source=button_displacement`
or `contact_signal_proxy`; it is not force.

## Non-Goals

This stage does not connect full Lightwheel, does not download or redistribute
Lightwheel assets, does not collect official datasets, does not train models, and
does not expand the benchmark task set.

The next gate is Real PressButton Runtime Evaluation Gate, where the team can
decide whether the single-task runtime loop is ready for controlled real-runtime
checks beyond dry-run.

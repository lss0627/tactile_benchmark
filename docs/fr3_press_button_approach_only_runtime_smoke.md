# FR3 PressButton Approach-Only Runtime Smoke

This gate verified that the real FR3 articulation can approach the configured
PressButton geometry through the local differential IK path. It is not a button
press, not a dataset collection gate, and not a benchmark result.

## Scope

Allowed:

- load the FR3 articulation and primitive PressButton co-scene;
- send tiny local differential IK joint targets;
- approach `micro_approach`, `short_approach`, `pre_press`, and
  `near_contact`;
- save status JSON and optional screenshots.

Forbidden:

- execute `press_target`;
- execute press depth;
- require task success;
- collect any dataset;
- fabricate force or wrench;
- use Lula global target IK;
- use joint-space fallback as PressButton control.

## Commands

Gate 0:

```bash
python scripts/check_fr3_press_button_approach_readiness.py \
  --geometry-report outputs/fr3_press_button_planning/press_button_geometry_report.json \
  --load-only-status outputs/fr3_press_button_planning/load_only_status.json \
  --waypoint-plan outputs/fr3_press_button_planning/waypoint_plan.json \
  --diffik-report outputs/fr3_differential_ik/target_report.json \
  --safety-config configs/robots/fr3_ee_controller_safety.yaml \
  --output outputs/fr3_press_button_planning/approach_readiness.json
```

Gate 1 dry-run:

```bash
python scripts/run_fr3_press_button_approach_only_smoke.py \
  --robot-config configs/robots/fr3_real_articulation.yaml \
  --controller-config configs/robots/fr3_ee_controller_contract.yaml \
  --safety-config configs/robots/fr3_ee_controller_safety.yaml \
  --task-config configs/tasks/press_button_fr3_planned.yaml \
  --runtime-config outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml \
  --geometry-report outputs/fr3_press_button_planning/press_button_geometry_report.json \
  --waypoint-plan outputs/fr3_press_button_planning/waypoint_plan.json \
  --mode micro_approach \
  --max-substeps 20 \
  --dry-run \
  --output outputs/fr3_press_button_approach_only/dry_run_status.json
```

Real approach modes were run with `CUDA_VISIBLE_DEVICES=1 conda run -n isaac`
and the same config paths.

## Results

| Mode | Status | Substeps | Initial distance | Final distance | Reached waypoint | Button displacement | Screenshot |
| --- | --- | ---: | ---: | ---: | --- | ---: | --- |
| `micro_approach` | PASS | 20 | 0.526025 m | 0.523319 m | no | 0.0 m | `outputs/fr3_press_button_approach_only/fr3_press_button_micro_approach.png` |
| `short_approach` | PASS | 100 | 0.526025 m | 0.511361 m | no | 0.0 m | `outputs/fr3_press_button_approach_only/fr3_press_button_short_approach.png` |
| `pre_press` | PASS | 1096 | 0.526025 m | 0.082547 m | `pre_press` | 0.0 m | `outputs/fr3_press_button_approach_only/fr3_press_button_pre_press.png` |
| `near_contact` | PASS | 1161 | 0.526025 m | 0.022640 m | `near_contact` | 0.0 m | `outputs/fr3_press_button_approach_only/fr3_press_button_near_contact.png` |

All runtime statuses recorded:

- `uses_differential_ik=true`;
- `uses_lula_global_ik=false`;
- `uses_joint_space_fallback=false`;
- `joint_command_sent=true`;
- `distance_to_button_decreased=true`;
- `button_pressed=false`;
- `press_depth_executed=false`;
- `press_target_executed=false`;
- `dataset_collection_allowed=false`;
- `contact_force_available=false`;
- `force_source=unavailable`;
- `uses_fake_force=false`;
- `benchmark_result=false`;
- `not_for_paper_claims=true`.

The button displacement remained below the configured success threshold
`button_press_depth=0.03 m` in every approach-only mode.

## Press Readiness

`scripts/check_fr3_press_button_press_readiness.py` consumed the approach
statuses and wrote:

`outputs/fr3_press_button_approach_only/press_readiness.json`

Key fields:

- `ready_for_press_runtime_smoke=true`;
- `approach_only_passed=true`;
- `pre_press_reached=true`;
- `near_contact_reached=true`;
- `button_not_pressed_during_approach=true`;
- `press_depth_still_disabled=true`;
- `dataset_collection_allowed=false`.

This readiness only authorizes planning the next press-runtime smoke gate. It
does not authorize dataset collection, benchmark scoring, or paper claims.

## Known Warnings

The Isaac Sim terminal output included standard headless/windowing and
deprecation warnings. The approach status JSONs did not record safety errors,
NaN, or approach-specific warnings.

## Current Non-Claims

- FR3 has not pressed the button.
- No `press_target` or press depth was executed.
- No force/wrench tactile signal is available.
- No tactile mounts are connected.
- No dataset was collected.
- No benchmark score or paper result is produced.

## Next Stage

The next stage may be **FR3 PressButton Press Runtime Smoke**, using the same
local differential IK boundary and keeping `benchmark_result=false` until a
separate formal benchmark gate exists.

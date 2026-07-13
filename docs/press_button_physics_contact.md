# PressButton Physics Contact Hook

This stage adds a narrow contact/displacement hook for the single-task
`PressButton` Isaac Sim runtime path. It does not expand the benchmark and does
not turn the result into a paper-grade tactile manipulation score.

## What Changed

- `isaac_tactile_libero/envs/isaacsim_contact.py` provides an import-safe
  helper for reading PressButton contact/displacement state.
- `IsaacSimPressButtonEnv` now reports contact/displacement fields in
  observations, metrics, runtime status, and rollout records.
- A PressButton-only contact-force probe attempts to inspect Isaac Sim / PhysX
  contact reports in real runtime. It is optional and reports unavailable
  clearly when the API cannot provide pusher-button force.
- Success source is explicit:
  - `button_displacement`
  - `physics_contact`
  - `geometric_fallback`
  - `none`

## Current Contact Sources

The preferred signal is button displacement. In the current primitive scene the
button top is still moved by the placeholder runtime loop, so this is a runtime
displacement hook rather than a complete physics-driven button mechanism.

Contact force is not claimed unless a real Isaac Sim contact-force API is wired
successfully. Current outputs may therefore include:

- `physics_contact_available=false`
- `contact_force_available=false`
- `contact_force_source=unavailable`
- `contact_api_error=<clear message>`
- `button_displacement_available=true`
- `success_source=button_displacement`

If displacement/contact hooks are unavailable, the runtime can fall back to the
geometric proxy and must report:

- `using_geometric_fallback=true`
- `success_source=geometric_fallback` for fallback success

## Metrics

PressButton runtime metrics include:

- `success`
- `success_source`
- `num_steps`
- `first_contact_step`
- `first_success_step`
- `contact_step_count`
- `button_press_depth`
- `max_button_press_depth`
- `physics_contact_available`
- `contact_signal_seen`
- `contact_force_available`
- `contact_force_norm`
- `max_contact_force_norm`
- `mean_contact_force_norm`
- `contact_force_source`
- `contact_probe_method`
- `contact_api_error`
- `button_displacement_available`
- `using_geometric_fallback`

All artifacts continue to report `benchmark_result=false`,
`single_task_runtime_smoke=true`, and `not_for_paper_claims=true`.

## Non-Claims

- The pusher is still a kinematic placeholder, not a real FR3 controller.
- The tactile sensor is not real.
- Contact force is not available unless explicitly reported true.
- Geometry and displacement are never converted into fake contact force.
- A fallback success is not a physics-contact success.
- This is still a PressButton-only runtime smoke path.

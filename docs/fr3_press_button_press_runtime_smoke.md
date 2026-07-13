# FR3 PressButton Press Runtime Smoke

This note records the first real FR3 PressButton press-depth runtime smoke.
It is not a benchmark result, not a dataset collection gate, and not a
force-aware tactile result.

## Scope

- Task: PressButton only.
- Runtime: real Isaac Sim FR3 articulation loaded from the configured FR3 USD.
- Controller path: local differential IK only.
- Disallowed paths: Lula global IK, joint-space fallback, dataset collection,
  and fake force.
- Success source: geometric `button_displacement` proxy only.
- Force status: `force_source=unavailable`, `contact_force_available=false`.

## Gate Results

- Gate 0 preflight: PASS.
  - Artifact: `outputs/fr3_press_button_press_runtime/preflight.json`.
  - `ready_for_press_runtime_smoke=true`.
  - Approach-only near-contact passed and did not press the button.
- Gate 1 dry-run: PASS.
  - Artifact: `outputs/fr3_press_button_press_runtime/dry_run_status.json`.
  - Isaac Sim was not started and no joint command was sent.
- Gate 2 partial 2mm press: PASS.
  - Artifact: `outputs/fr3_press_button_press_runtime/partial_press_2mm_status.json`.
  - `press_depth_executed` is approximately 2mm.
  - No NaN, no safety abort, no dataset write, no fake force.
- Gate 3 partial 10mm press: PASS.
  - Artifact: `outputs/fr3_press_button_press_runtime/partial_press_10mm_status.json`.
  - `press_depth_executed` is approximately 10mm.
  - No NaN, no safety abort, no dataset write, no fake force.
- Gate 4 full press: PASS.
  - Artifact: `outputs/fr3_press_button_press_runtime/full_press_status.json`.
  - Full press reached button-displacement success.
  - `success_source=button_displacement`.
  - No NaN, no safety abort, no dataset write, no fake force.
- Gate 5 press and retract: FAIL / BLOCKED.
  - Artifact: `outputs/fr3_press_button_press_runtime/press_and_retract_status.json`.
  - Press phase reached button-displacement success, but retract did not safely
    release or reach the retract target.
  - The observed final button-displacement proxy increased during the failed
    retract run.
  - A safety guard was added so future retract attempts abort if displacement
    increases during retract.

## Current Blocking Issue

The real FR3 can approach and press the button through the local differential
IK path, but post-press retract is not safe enough to pass this gate. The
observed behavior indicates that a simple upward retract target can move the
TCP deeper instead of releasing the button at this configuration.

Minimum next fixes:

- Add a dedicated retract diagnostic that exercises very small reverse-axis
  differential IK deltas from the pressed pose.
- Verify the local Jacobian sign and conditioning at the pressed pose.
- Keep the displacement-increase abort guard enabled for all retract attempts.
- Do not enter dataset collection or benchmark evaluation until
  press-and-retract passes.

## Non-Claims

- This is not a formal PressButton benchmark result.
- This is not a dataset collection result.
- This is not force-aware tactile sensing.
- The button success label is not based on force or wrench.
- No Lightwheel runtime or assets are used.
- No 30-task expansion or model training is involved.

## Readiness

- Ready for single-run full-press smoke debugging: yes.
- Ready for single-episode real FR3 PressButton evaluation: no, blocked by
  retract.
- Ready for dataset collection: no.
- Ready for paper claims: no.

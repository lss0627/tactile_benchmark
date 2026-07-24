# PressButton Runtime Loop

## State machine

```text
RESET
→ READY
→ APPROACH
→ PRESS
→ RELEASE
→ RETRACT
→ SUCCESS
```

Any state may transition to `ABORT`, followed by no further actuation except an explicitly validated safe-retract policy if the abort class permits it.

## RESET

- restore FR3/task state;
- restore released button;
- validate articulation, Contact, camera, and optional tactile handles;
- wait within the readiness window;
- record seed and reset provenance.

## APPROACH

- move using bounded public 7D actions;
- enforce finite, joint, workspace, per-step, collision, penetration, and budget guards;
- task success is impossible in this phase.

## PRESS

- continue bounded motion;
- retain Contact/raw-contact observations;
- declare press only from button mechanism state;
- abort on invalid runtime evidence or safety guard.

## RELEASE

- command release motion;
- require the button to return to the released predicate;
- retain Contact disappearance/transition truth without inventing force.

## RETRACT

- move to the declared safe retract region within budgets;
- record retract success/failure;
- zero post-abort actuation remains mandatory.

## SUCCESS

```text
pressed
and released
and safe_retract
and runtime_valid
```

No TCP-distance, geometric, timer, or action-count fallback is permitted for formal benchmark episodes.

## Evidence per step

- requested/executed action;
- robot and task state;
- RGB/depth references;
- Contact/raw Contact and masks;
- physics/render timestamps;
- safety checks;
- state-machine phase;
- failure codes.

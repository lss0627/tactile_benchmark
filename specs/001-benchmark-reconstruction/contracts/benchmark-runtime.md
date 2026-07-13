# Benchmark Runtime Contract

**Contract version**: `0.1.0` (preserved unless implementation proves a breaking change necessary)

## Development runtime baseline

The active development runtime is Isaac Sim 6.0.1 on Python 3.12. Driver 550.144.03 remains
unchanged and is reported as `UNVALIDATED`; release-level physical/data/replay/evaluation evidence
must be rerun on a current reference/validated driver. Isaac Sim 5.1/Python 3.11 is archived as a
reference baseline only.

First-party runtime code MUST NOT import `omni.isaac.*`, `omni.isaac.dynamic_control`, or deprecated
`isaacsim.core.api`, `isaacsim.core.prims`, and `isaacsim.core.utils` after cutover. Contact uses CPU
physics while RTX Camera may use GPU rendering. Native GPU-physics Contact fails explicitly with
`GPU_CONTACT_NATIVE_INSTABILITY`.

## Factory

`make_env(config)` MUST create mock, diagnostic, and accepted real backends through one entry point.
It MUST reject unknown backends, incompatible task/backend pairs, unresolved assets, and failed
joint/frame introspection. It MUST NOT silently fall back from real to mock or placeholder.

## Lifecycle

- `reset(seed=...) -> (observation, info)` initializes robot/task/sensors and reports capability.
- `step(action) -> (observation, reward, terminated, truncated, info)` executes one bounded control
  interval and reports the executed action, task state, safety state, and termination reason.
- `close()` is idempotent and leaves no active actuation.
- Calls after `close()` or `step()` before `reset()` fail explicitly.

## Action

The public action has seven finite components:

```text
[dx, dy, dz, dRx, dRy, dRz, gripper]
```

Units, reference frame, rotation representation, scaling, clipping, control interval, and gripper
semantics MUST be declared by configuration and returned in reset metadata. The environment records
both requested and executed actions. Unsupported components cause a structured error or require an
explicit diagnostic capability profile; they are never silently discarded by a benchmark backend.

## Observation

All modes return versioned, stable-shape fields for robot state, task state, vision references, and
tactile modalities. Every optional modality has distinct capability and timestep validity masks.
Missing, dropped, delayed, saturated, and invalid are separate states.

Force/wrench values are valid only when metadata declares source, units, coordinate frame,
calibration version, timestamp/clock, and transformation. TCP pose, distance, displacement, success,
or commanded motion MUST NOT populate force/wrench fields.

Contact scalar magnitude and raw position/normal/impulse do not establish a validated
three-dimensional force or six-dimensional wrench. The sensor may report a valid no-contact sample,
but `in_contact` must be false, scalar magnitude must be at most `1.0e-4`, and public force/wrench
masks must remain false. Sensor readiness and release use the windows defined by FR-035.

RGB is `uint8`; depth is `float32` and aligned to RGB. Frames must update on real rendering ticks,
valid pixels must be finite and inside the clipping range, background behavior must be declared,
and capture skew must not exceed one camera tick.

## Task outcome

PressButton success is based on observed movable-button travel satisfying the task-card threshold
for the required duration. Completion also requires safe release/retract and declared reset/release
state. Elapsed steps may truncate an episode but may not create success.

## Safety and termination

Every real motion interval checks finite values, workspace, joint position/velocity, direction,
collision/penetration, per-step and cumulative motion, operator step budget, and wall time. A failed
check stops actuation and terminates with `safety_abort`. Other canonical reasons are `success`,
`task_failure`, `step_budget`, `wall_time_budget`, `sensor_invalid`, `controller_failure`, and
`operator_abort`.

## Info requirements

`info` MUST include contract/backend/task/robot/sensor versions, capability, requested/executed
action, task-state source, success source, safety status/events, claim class, and evidence run ID.
Diagnostic backends MUST identify why they cannot support a benchmark claim.

## Compatibility

Patch/minor additions preserve readers. Removing fields, changing shape/unit/frame/action semantics,
or changing task success requires the appropriate schema/task version, migration note, fixtures, and
contract tests.

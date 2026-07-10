# Benchmark Runtime Contract

**Contract version**: `0.1.0` (preserved unless implementation proves a breaking change necessary)

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

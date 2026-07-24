# Dataset Protocol

## Goal

Release a dataset that is trainable, replayable, auditable, and bound to the eight-task benchmark.

## Default scale

```text
tasks: 8
accepted demonstrations per task: >= 50
default total accepted demonstrations: >= 400
```

A G4 quality review may approve another count; raw count alone cannot pass G4.

## Episode contents

Each episode stores:

- task/version/instruction/seed/split;
- simulator/runtime/source/config/asset hashes;
- requested and executed 7D actions;
- robot and task state;
- RGB/depth references;
- Contact/raw Contact and validity/freshness;
- optional tactile references;
- force magnitude, vector force, wrench, and raw impulse as distinct masked fields;
- physics/render/sensor timestamps;
- task-state press/release/success;
- termination/truncation/failure codes;
- media references where configured.

Privileged object/task state used for replay or metrics must be separated from policy observations.

## Units and frames

All fields declare:

- units;
- coordinate frame;
- shape;
- dtype;
- source;
- validity mask.

Default units:

```text
position: m
rotation: rad
time: s
depth: m
force magnitude/vector: N when valid
wrench torque: N m when valid
```

## Synchronization

Required timestamps:

- control/action;
- physics;
- robot state;
- task state;
- RGB/depth;
- Contact/tactile.

Validation:

- monotonic;
- no missing control step;
- arrays have the same declared step count;
- sensor skew within the contract;
- background/missing frames explicitly masked.

## Truth rules

- Invalid vector force/wrench is stored as unavailable with false mask.
- Raw impulse is not converted to force for this release.
- Runtime-invalid episodes are rejected from training/evaluation releases but retained in rejection logs.
- Geometric fallback success is forbidden.

## Validation

The validator reports:

```json
{
  "task_count": 8,
  "accepted_episode_count": 400,
  "rejected_episode_count": 0,
  "duplicate_count": 0,
  "missing_key_rate": 0.0,
  "shape_error_rate": 0.0,
  "nonfinite_rate": 0.0,
  "timestamp_error_rate": 0.0,
  "invalid_mask_encoding_rate": 0.0,
  "replay_outcome_agreement": 1.0
}
```

The numeric values above are target examples, not predeclared results.

## Splits

- `train`
- `val`
- `test_seen`
- declared robustness/OOD splits only after their generation rules are frozen.

Test episodes and language/object/randomization identities must not leak into training beyond the declared split policy.

## Replay

Replay executes the recorded reset and actions in Isaac Sim and records:

- outcome match;
- task-state trajectory difference;
- timing skew;
- Contact-event alignment;
- first divergence;
- failure codes.

Replay tolerance is declared per task; failed replay blocks dataset acceptance until reviewed.

## Dataset card

Required:

- summary and intended use;
- benchmark/schema/runtime versions;
- tasks and modalities;
- collection and rejection procedure;
- splits;
- quality/replay reports;
- licenses and asset provenance;
- limitations;
- citation and contact.

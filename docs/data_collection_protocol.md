# Data Collection Protocol

## Purpose

Data collection is a first-class benchmark service. It must produce official
offline data and support reproducible online research without coupling the
dataset to one expert implementation.

## Expert sources

The expert registry supports:

- scripted oracle;
- traditional controller;
- teleoperation;
- trained-policy rollout;
- human demonstration;
- community-defined expert adapter.

Every source declares supported tasks, required observations, action schema,
determinism, privileged inputs, version, and source digest.

## Batch interface

Target interface:

```bash
python scripts/collect_data.py \
  --suite precision \
  --task peg_insert \
  --num-episodes 10000 \
  --policy scripted \
  --split train \
  --output outputs/datasets/precision-peg-v1
```

The runner must support deterministic seeds, multiple environments, bounded
retries, success/failure retention policy, atomic progress checkpoints,
resume, per-worker failure isolation, statistics, and post-collection
validation.

## Episode capture

Capture includes:

- vision, tactile, proprioception, task state, and end-effector state;
- requested and executed action;
- Contact/force fields with source and validity;
- phase/reward/success/failure;
- control, physics, rendering, and sensor timestamps;
- all randomization factors, including friction, stiffness, object,
  trajectory, material, sensor noise, delay, drift, and dropout;
- expert, suite, task, protocol, and collection-job provenance.

## Acceptance and retry

Runtime-invalid episodes are never accepted. Failed-task episodes may be
retained in a declared analysis dataset but cannot enter a success-only
training set. Retries use bounded, logged seeds and never overwrite the
original failure record.

## Progress and interruption

Progress is append-only and resumable. Resume verifies config, source, task,
asset, split, and schema digests before scheduling remaining episode IDs.
Interrupted workers cannot create pseudo-complete episodes.

## Community registration

Researchers may register robots, sensors, tasks, experts, and modalities
through versioned public registries. A plugin must pass schema, lifecycle,
capability, determinism, and no-privileged-leakage tests before its data can be
marked benchmark-compatible.

## G3/G4 boundary

G3 accepts collection infrastructure with reference fixtures and smoke data.
G4 accepts the official 16-task dataset, leakage audit, quality report, and
simulator replay.

# Data Model and State Transitions

**Feature**: `001-benchmark-reconstruction`
**Schema family**: `benchmark-reconstruction/1.0.0`

## 1. Gate

Represents one reviewable capability transition.

| Field | Type | Rules |
|---|---|---|
| `gate_id` | enum `G0`-`G6` | Stable and unique |
| `title` | string | Human-readable outcome |
| `status` | enum | `NOT_STARTED`, `IN_PROGRESS`, `BLOCKED`, `PASS_SMOKE`, `PASS_BENCHMARK` |
| `claim_class` | enum | `mock`, `dry_run`, `runtime_smoke`, `physical_runtime`, `dataset`, `evaluation`, `benchmark`, `release` |
| `predecessors` | list[gate_id] | All must satisfy the transition rule |
| `requirements` | list[FR/SC IDs] | Non-empty for implementation gates |
| `verification_commands` | list[string] | Exact, non-interactive where possible |
| `evidence` | list[artifact reference] | Hash-bound manifests only |
| `blockers` | list[object] | Required when status is `BLOCKED` |
| `reviewed_at` | timestamp/null | Set only by a review action |

### Gate transition rules

```text
NOT_STARTED -> IN_PROGRESS
IN_PROGRESS -> BLOCKED | PASS_SMOKE | PASS_BENCHMARK
BLOCKED -> IN_PROGRESS
PASS_SMOKE -> IN_PROGRESS | PASS_BENCHMARK
PASS_BENCHMARK -> IN_PROGRESS   # semantic input changed; evidence becomes stale
```

`PASS_SMOKE` never satisfies a predecessor that requires physical, dataset, evaluation, benchmark,
or release evidence. `PASS_BENCHMARK` requires a clean commit and fresh immutable evidence.

## 2. EvidenceManifest

Identity and provenance for one command and its artifacts. The normative shape is
[evidence-manifest.schema.json](./contracts/evidence-manifest.schema.json).

Core identity is the tuple:

```text
(repository_commit, dirty_patch_digest, config_digests, asset_digests,
 command, environment_digest, artifact_schema_version)
```

An artifact is `STALE` when a semantic input required by its gate differs from this tuple. Dirty
evidence can support development smoke states but cannot support `PASS_BENCHMARK`.

## 3. TaskCard

| Field group | Required content |
|---|---|
| Identity | task ID, version, name, language instruction, suite |
| Scene | assets, licenses, initial-state distribution, randomization, reset oracle |
| Robot | robot/config version, permitted controller, action semantics, frames |
| Task truth | observable state variables, success duration/threshold, failure, termination |
| Safety | workspace, joint, velocity, collision/penetration, budgets, stop conditions |
| Metrics | units, direction, missing rules, aggregation contribution |
| Evidence | scripted oracle, physical episodes, replay tolerance/report |
| Splits | train/validation/test allocation policy and leakage risks |
| Status | candidate, blocked, accepted, deprecated; acceptance manifest |

Only `accepted` task cards may appear in formal collection/evaluation. Changing success, reset,
randomization, assets, or action semantics requires a task version change and new evidence.

## 4. RuntimeEpisode

| Field | Type | Validation |
|---|---|---|
| `episode_id` | globally unique string | Duplicate write is an error |
| `task_id`, `task_version` | string | Must resolve to accepted TaskCard for formal data |
| `backend`, `robot_version`, `sensor_version` | string | Immutable per episode |
| `seed`, `split` | scalar | Split assignment frozen before evaluation |
| `initial_state` | structured snapshot | Required for physical replay |
| `observations` | time-major arrays | Same length or declared alignment rule |
| `actions` | `[T, 7]` | Finite, clipped/executed action both identifiable |
| `timestamps` | monotonic arrays | Units and clocks declared; skew bounded |
| `tactile_masks` | time-major masks | Capability, validity, drop, delay, saturation represented |
| `task_state` | time-major values | Includes observed button travel/reset/release |
| `safety_events` | list | Empty for a passing formal episode |
| `outcome` | structured | Success, failure, termination reason, metrics |
| `provenance` | manifest reference | Resolves to code/config/assets |

### Runtime state machine

```text
RESETTING -> READY -> APPROACH -> PRESS -> HOLD -> RELEASE -> RETRACT -> COMPLETE
     |         |         |         |       |         |          |
     +---------+---------+---------+-------+---------+----------+-> ABORTED
```

Every transition has an operator step and wall-time deadline. `COMPLETE` requires released/reset
button state and safe robot state; success alone is insufficient.

## 5. DatasetRelease

| Field | Rules |
|---|---|
| `dataset_id`, `version` | Immutable release identity |
| `schema_version` | Has reader and migration policy |
| `episodes` | Unique IDs and content checksums |
| `splits` | Frozen, disjoint, leakage-audited |
| `task_cards` | Exact versions embedded or hash-referenced |
| `validation_report` | Schema, shape, finite, timestamp, masks, checksum results |
| `replay_report` | Per-episode physical replay results and tolerances |
| `card` | Collection method, limitations, license, intended use |
| `claim_class` | `runtime_smoke` or `dataset`; smoke cannot become formal by renaming |

Lifecycle: `DRAFT -> VALIDATED -> REPLAY_ACCEPTED -> FROZEN -> RELEASED`. Any content mutation after
`FROZEN` creates a new version.

## 6. EvaluationRun

Contains frozen policy/checkpoint, dataset, task/sensor suite, seed set, configuration, hashes,
per-episode records, derived aggregates, uncertainty, failures, logs, and optional media.

Lifecycle: `PLANNED -> RUNNING -> EPISODES_COMPLETE -> AGGREGATED -> VERIFIED`. Aggregation reads
immutable episode records; hand-edited summary values are invalid. Missing metrics retain an
explicit reason and never silently become zero.

## 7. BaselineRun

Contains model/modality contract, train/validation/test split hashes, normalization statistics
derived from train only, optimizer/budget/seeds, parameter count, encoders/fusion, compute,
checkpoints, validation-only selection decision, and linked EvaluationRun.

Lifecycle: `CONFIGURED -> TRAINING -> VALIDATED -> CHECKPOINT_SELECTED -> EVALUATED -> REVIEWED`.
Test data cannot influence any state before `CHECKPOINT_SELECTED`.

## 8. TraceabilityRecord

One row links:

```text
requirement_id -> user_story -> gate_id -> task_ids -> test_ids/commands -> artifact paths -> status
```

Each FR, buildable SC, and acceptance scenario requires at least one complete row. A row is complete
only when the artifact exists, its manifest validates, and its claim class satisfies the gate.

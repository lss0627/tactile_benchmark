# TactiLIBERO Phase 2 and Phase 3 Checkpoint Report

## Outcome and claim boundary

Phase 2 T009–T020 is complete. Phase 3's import-safe implementation
foundation, T021–T032, is complete. The formal physical execution and evidence
tasks T033–T039 are not complete because G0 remains `BLOCKED`, G1 remains
`BLOCKED`, and this checkpoint did not start Isaac Sim. T040 and Phase 4 have
not started.

This checkpoint therefore establishes contracts and executable foundations; it
does not claim that PressButton, any of the 16 paper tasks, an official
dataset, replay, training, generalization evaluation, baseline results,
leaderboard, or paper release has passed. Component-only dry or injected-fake
results may report `PASS_SMOKE`, but the enclosing G1 status remains
`BLOCKED`.

## Frozen paper-v1 benchmark constants

The paper-v1 contract remains unchanged:

- four suites with four tasks each, for 16 tasks total;
- protocols `GP-01`, `GP-02`, and `GP-03`;
- at least 50 accepted training demonstrations per task and at least 800
  overall;
- three policy seeds;
- at least 20 evaluation episodes for every task condition and policy seed.

## Phase 2: schemas, validation, and registries

All 26 record schemas use version `1.0.0`, strict validation, canonical sorted
compact UTF-8 JSON, SHA-256 digests, stable semantic-version parsing, and
same-major compatibility:

- task: `TaskFamily`, `TaskInstance`, `DomainVariant`, `SuiteManifest`;
- interoperability: `SensorDomain`, `ExpertAdapter`, `CommunityPlugin`;
- collection/data: `CollectionJob`, `CollectionProgress`,
  `DemonstrationEpisode`, `DatasetManifest`, `SplitManifest`, `ReplayRecord`;
- training: `TrainingConfig`, `TrainingRun`, `CheckpointMetadata`;
- evaluation/release: `ProtocolDefinition`, `LeakageAudit`,
  `MetricDefinition`, `PolicyCapability`, `EvaluationCell`, `EpisodeResult`,
  `GeneralizationAggregate`, `ResultBundle`, `LeaderboardSubmission`, and
  `LeaderboardEntry`.

The seven registry contracts are `robot`, `task`, `sensor`, `expert`,
`observation_modality`, `policy`, and `training_algorithm`. Each advertises a
versioned capability contract. Expert, observation-modality, and
training-algorithm registration is enforced now; robot, task, sensor, and
policy retain their public legacy behavior and are explicitly labeled
foundation-only until T040.

## Phase 3: accepted-environment foundation

The Phase 3 implementation adds:

- an authoritative PressButton mechanism/task-state oracle;
- deterministic reset provenance bound to seed, mechanism thresholds, joint
  identity, requested state, and observed travel;
- finite, joint, workspace, requested translation/rotation, collision,
  penetration, and action-budget guards, including planned-target validation
  before command send;
- exact Contact and raw-Contact normalization, USD authority provenance,
  cadence/freshness validation, invalid-force masks, and fail-closed retention;
- RGB/depth source frame, source timestamp, cadence, and synchronization
  validation that accepts a static but freshly captured scene;
- a monotonic `APPROACH → PRESS → HOLD → RELEASE → RETRACT → COMPLETE`
  episode state machine with immediate abort and no post-abort actuation;
- pre-observation retention of sent commands, including HOLD commands;
- atomic evidence staging plus writer-before-environment/app-close behavior;
- an explicit boundary preventing legacy evidence from being interpreted as
  Phase 3-complete G1 evidence.

These paths are import-safe and were tested with injected fakes. That is not a
substitute for T033–T039 physical execution.

## RED to GREEN record

Behavior changes followed RED→GREEN:

| Selection | RED | GREEN |
| --- | ---: | ---: |
| Initial T021–T025 acceptance | 20 failed | Phase 3 focused selection green |
| Final adversarial audit | 8 failed | 8 passed |
| HOLD pre-observation retention | 1 failed | 1 passed |
| C2a optional execution-identity compatibility | 13 collateral full-suite failures | 29 relevant nodes passed |
| Final Phase 3 focused files | — | 102 passed, 8 intentional future-RED deselected |

The final code review returned `READY` after reset authority, stale Contact
readiness, phase regression, raw Contact cross-validation, command retention,
constructor cleanup, camera freshness, and component/Gate claim boundaries
were hardened.

## Test and static-verification inventory

The frozen repository inventory is:

```text
total/current/portable/external/future = 1151/1026/1025/1/125
current collection-order SHA-256 =
13b9077715a1150f2a87561e5d8702d659a748a094f7bb1f0f444f6006472057
current sorted SHA-256 =
d41cf8b44eae78aa2ec5061e235bff02abf44b729331ab6ec67c2a56ce2c638e
future-RED manifest SHA-256 =
1fa55636bb69fe4c2618844a080543eb2f639894c12a40350db2ce28c468d6b7
```

Schema/contract validation reports 26 valid schemas and seven valid registry
contracts without importing Isaac Sim. The deprecated Isaac API scan covers
446 files with zero warnings and zero errors. Import-safety checks load the
Phase 2 contract modules and Phase 3 runtime/sensor foundations without loading
`isaacsim`, `omni`, or `pxr`. `git diff --check` and Python byte-compilation
pass.

The no-simulator current-GREEN suite contains exactly two pre-existing
blockers:

1. `tests/test_g1_static_pose_runtime_cli.py::test_c2a_runtime_failure_preserves_exact_code_message_writes_before_shutdown`
2. `tests/test_g1_tracking_envelope.py::test_each_class_requires_64_readiness_256_measurement_three_scenes_and_no_window_reset`

The final full current-GREEN run reports `1024 passed, 2 failed, 125
deselected`; the 125 deselections are the unchanged intentional future-RED
partition.

They remain intentionally visible and keep G0 blocked. They were not added to,
removed from, or hidden by the 125-node intentional future-RED manifest.

## Evidence and Gate state

The Phase 2 source-bound G0 package is:

```text
outputs/evidence/G0/tactilibero-generalization-6e50a18bcb8ec040f9d55bd3200cb7530b4a4b00/
```

It is fresh for source commit
`6e50a18bcb8ec040f9d55bd3200cb7530b4a4b00` and correctly reports
`BLOCKED`. The final Phase 3 checkpoint regenerates the same immutable evidence
shape under
`outputs/evidence/G0/tactilibero-generalization-<final-source-commit>/`.
Its `PASS_BENCHMARK` field, if ever attainable, means repository integrity
only; with the two current-GREEN failures it remains `BLOCKED`.

Current Gate state:

```text
G0 = BLOCKED
G1 = BLOCKED
G2 = NOT_STARTED
G3 = NOT_STARTED
G4 = NOT_STARTED
G5 = NOT_STARTED
G6 = NOT_STARTED
```

G2–G6 retain the acceptance meanings: contracts/registries; sensors/collection
foundation; 16 tasks/official dataset/replay; unified training/generalization
evaluation; and baseline/leaderboard/paper release, respectively. Foundation
code in an earlier phase does not advance those Gate statuses.

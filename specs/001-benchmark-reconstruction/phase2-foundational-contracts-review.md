# Phase 2 Foundational Contracts Review

## Scope and claim boundary

This checkpoint implements only T009–T020. It provides the versioned data
contracts, compatibility validators, registry foundations, Gate definitions,
repository verification inventory, and G0 evidence packaging needed before
the 16-task benchmark is implemented.

It does not implement or accept any task, collect an official dataset, train a
policy, run a generalization evaluation, publish a baseline, or advance G1–G6.
Isaac Sim is not run. T021 and Phase 3 remain out of scope.

## Schema and validation design

All Phase 2 record schemas use version `1.0.0`, strict top-level fields, an
import-safe dependency-free validator, semantic invariants, and exportable
JSON Schema draft 2020-12 definitions. The 26-entry catalog contains:

- task: `TaskFamily`, `TaskInstance`, `DomainVariant`, `SuiteManifest`;
- interoperability: `SensorDomain`, `ExpertAdapter`, `CommunityPlugin`;
- collection/data: `CollectionJob`, `CollectionProgress`,
  `DemonstrationEpisode`, `DatasetManifest`, `SplitManifest`, `ReplayRecord`;
- training: `TrainingConfig`, `TrainingRun`, `CheckpointMetadata`;
- evaluation/release: `ProtocolDefinition`, `LeakageAudit`,
  `MetricDefinition`, `PolicyCapability`, `EvaluationCell`, `EpisodeResult`,
  `GeneralizationAggregate`, `ResultBundle`, `LeaderboardSubmission`, and
  `LeaderboardEntry`.

The shared registry layer uses canonical sorted compact UTF-8 JSON, SHA-256
digest verification, stable `MAJOR.MINOR.PATCH` parsing, same-major
compatibility, capability checks, immutable manifest copies, and
registration-before-factory validation.

Central contract definitions cover robot, task, sensor, expert, observation
modality, policy, and training algorithm. Expert, modality, and training
algorithm foundations enforce `register_contract`; legacy robot, task, sensor,
and policy registries retain their existing public registration behavior until
T040. The validation report labels that boundary explicitly rather than
claiming G2 completion.

## RED to GREEN and inventory

The initial T009–T015 contract selection produced 30 expected failures. Shared
digest/version, catalog, validation CLI, and Gate behavior were also introduced
through focused RED nodes before their implementations. A review-driven
fail-closed pass added RED coverage for nested selection evidence, attempted-ID
resume retention, observation masks/timestamps, dataset counts and digests,
metric availability, aggregate ranges/formulas, registry bypass/mutability,
and G0 report integrity.

The focused Phase 2 regression is 63 nodes. Phase 2 adds exactly 40 GREEN nodes
to the repository inventory:

```text
total/current/portable/external/future = 1131/1006/1005/1/125
current collection-order SHA-256 =
81f3775aa7f436e091ca5a5d3ed0552cdb8304146ac08ff8bbbf5c85755aec10
current sorted SHA-256 =
adc2478567818fd12ebd674df556ad96c963c739ead43e1539b88ba35d9f7c60
```

The intentional future-RED inventory remains byte-identical at 125 nodes and
SHA-256
`1fa55636bb69fe4c2618844a080543eb2f639894c12a40350db2ce28c468d6b7`.

## G0 evidence

Formal output uses the non-overwriting namespace:

```text
outputs/evidence/G0/tactilibero-generalization-<source-commit>/
```

The output contains the command log, test inventory and results, schema and
registry validation, deprecated-import and import-safety checks, diff and
clean-checkout reports, repository state, Python version, manifest, checksums,
and freshness review. The packager reconciles authoritative report details
before deriving `PASS_BENCHMARK` or `BLOCKED`, stages atomically, and binds the
exact clean source commit.

`PASS_BENCHMARK`, when attainable, means repository integrity only. A failed
current-GREEN or clean-checkout check produces `BLOCKED`; it is never converted
into a task, dataset, training, evaluation, baseline, leaderboard, paper,
release, or simulator claim. The exact commit namespace and reviewed final
status are recorded in the immutable output manifest generated after the final
tracked Phase 2 commit.

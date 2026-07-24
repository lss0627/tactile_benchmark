# Feature Specification: TactiLIBERO Generalization Benchmark

**Feature Branch**: `001-benchmark-reconstruction`
**Created**: 2026-07-24
**Status**: In progress
**Working Title**: TactiLIBERO: A Generalization Benchmark for Contact-Rich Manipulation

## 1. Research Goal

TactiLIBERO evaluates whether contact-rich manipulation policies generalize beyond the objects, contact conditions, materials, and sensor domains observed during training.

The benchmark is a complete research loop:

```text
Task Suite
+ Data Generation
+ Standard Offline Dataset
+ Unified Training Pipeline
+ Generalization Evaluation Protocol
+ Baseline Results and Toolkit
```

The primary contribution is the generalization protocol and comparable training/evaluation contract—not the number of environments.

The central scientific question is:

> Under identical data, splits, budgets, and evaluation rules, what kinds of generalization do tactile and visuo-tactile manipulation policies achieve, and where do they fail?

## 2. Contribution Hierarchy

1. **Generalization protocols** — leakage-safe seen/unseen evaluation.
2. **Unified data collection and training** — offline and online research under one contract.
3. **Task suites** — structured contact-rich coverage.
4. **Contact-aware metrics** — success, efficiency, force quality, slip, and recovery.
5. **Baseline zoo** — matched algorithms and modality ablations.
6. **Evaluation toolkit and leaderboard** — one-command, verifiable reporting.
7. **Task scale** — useful coverage, but not the primary novelty.

## 3. Paper-v1 Scope

### Required

- Four task suites with four accepted tasks each: 16 canonical task instances.
- Three official core generalization protocols.
- An official offline dataset and an online environment/data-generation path.
- Expert collection from scripted/oracle, controller, teleoperation, trained-policy rollout, and user-provided expert adapters.
- Automatic resumable batch collection with parallel environments, retries, filtering, statistics, and validation.
- A unified training interface for Behavior Cloning, ACT, Diffusion Policy, a Transformer policy, and a UniVTAC-compatible baseline.
- Vision-only, tactile-only where meaningful, and visual-tactile fusion modality configurations.
- One-command evaluation with generalization and contact-aware metrics.
- A static, verifiable leaderboard and extensible policy/task/sensor/plugin registries.

### Extension-ready but not paper-v1 blocking

- One hundred-task expansion through additional task-family variants.
- Trajectory, task-transfer, scene, and continual-learning protocols.
- OpenVLA and π0 adapters.
- A hosted untrusted-checkpoint execution service.
- Real-robot transfer.

### Out of scope

- A task-only “UniVTAC++” contribution.
- Claiming generalization from one aggregate success rate.
- Treating 16 tasks as 16 unrelated hand-authored code paths.
- Hidden test data entering training, model selection, or checkpoint selection.
- Real-robot safety certification.
- Mandatory proof of every possible articulated collision-free path.
- Treating invalid force, wrench, Contact, or tactile fields as physical zero.
- Claiming “first benchmark” before a related-work audit supports the wording.

## 4. Task Suites

The first release contains exactly 16 accepted task instances:

| Suite | Required tasks | Generalization variables |
|---|---|---|
| Precision | Peg insertion, USB-like insertion, key insertion/turn, pin/socket alignment | geometry, clearance, initial offset, orientation |
| Articulation | Button press/release, switch actuation, drawer motion, cap/knob twist | damping, stiffness, direction, required travel |
| Surface Interaction | Sliding, wiping, scraping, surface following | friction, material, speed, contact duration |
| Deformable Contact | Soft pressing, sponge compression, fabric pull/place, cable/soft-part seating | compliance, deformation, load, contact distribution |

Every task contains:

- scene, object, robot, and tactile sensor configuration;
- reset and randomization;
- task-state success and failure;
- reward or phase labels;
- budgets;
- train/validation/test variant rules;
- scripted/oracle feasibility;
- asset and license provenance.

## 5. Core Generalization Protocols

### GP-01 Object and Geometry Generalization

Train on declared object identities, dimensions, and geometries; validate on new parameter combinations; test on held-out identities or geometry families.

Forbidden test leakage includes shared mesh instances, transformed duplicates, and undeclared asset-family overlap.

### GP-02 Contact, Material, and Physics Generalization

Train on declared friction, stiffness, compliance, clearance, contact-pattern, and force-profile ranges; test on held-out combinations within published runtime safety limits.

Valid evaluation distinguishes generalization from unsafe out-of-distribution physics.

### GP-03 Sensor and Observation Generalization

Train in one tactile sensor/calibration/noise domain and evaluate on held-out sensor domains, calibration, latency, noise, drift, dropped frames, occlusion, or scene appearance.

Zero-shot, calibration-only, and task-data adaptation results are separate.

### Extension protocols

The contracts remain extensible to:

- trajectory/path generalization;
- task and skill transfer;
- scene/layout generalization;
- robustness and recovery stress tests;
- continual and lifelong learning.

These may be released after the three core protocols pass without changing paper-v1 claims.

## 6. User Scenarios and Acceptance

### User Story 1 — Run a trustworthy reference environment

As a benchmark maintainer, I can reproduce the repository and run an accepted PressButton environment before expanding the suite.

**Acceptance scenarios**

1. Repository integrity passes from a clean checkout.
2. PressButton completes 100 resets, a rendered 500-step rollout, and 10 consecutive task-state press/release/safe-retract episodes.
3. Runtime safety, Contact truth, media, and evidence pass.

### User Story 2 — Define and use the 16-task suite

As a task researcher, I can select a suite/task, choose a split variant, reset the environment, train online or collect data, and evaluate task-state success.

**Acceptance scenarios**

1. Each suite contains exactly four accepted tasks.
2. Every task has complete configuration, randomization, success/failure, reward/phase, split, and feasibility records.
3. Variant generation is deterministic.
4. No task uses geometric or action-count success fallback.

### User Story 3 — Collect official or custom data

As a dataset author, I can collect expert trajectories from multiple sources, resume interrupted collection, validate episodes, and create a versioned dataset.

**Acceptance scenarios**

1. Scripted, controller, teleoperation, trained-policy, and custom expert adapters share one collection contract.
2. Batch collection supports parallel environments, retries, success filtering, resume, progress, and integrity checks.
3. Episodes retain visual/tactile/proprioceptive state, actions, task phases, success, timestamps, and all randomization/split parameters.
4. Community users can register robots, sensors, tasks, experts, and observation modalities through validated plugins.

### User Story 4 — Train policies through one interface

As a policy researcher, I can train multiple algorithms and modality ablations using identical data loading, normalization, horizons, seeds, budgets, validation selection, logging, and checkpoint rules.

**Acceptance scenarios**

1. BC, ACT, Diffusion Policy, Transformer, and UniVTAC-compatible configurations use the unified trainer.
2. Vision-only, tactile-only, and visual-tactile variants use the same split and action contract.
3. Checkpoint selection uses validation data only.
4. Training outputs are reproducible and bound to data/config/source hashes.

### User Story 5 — Evaluate generalization with one command

As a benchmark user, I can evaluate a checkpoint on GP-01, GP-02, or GP-03 and receive complete per-episode and aggregate results.

**Acceptance scenarios**

1. The protocol loader resolves immutable train/validation/test manifests and passes a leakage audit.
2. Evaluation reports seen/unseen success, generalization gap, runtime validity, completion time, contact metrics, slip, recovery, modality-drop degradation, and valid force metrics.
3. JSON, CSV, radar data, HTML, manifest, checksums, and a result bundle are generated.
4. Failed or invalid episodes are never silently replaced.

### User Story 6 — Train and evaluate online

As an online learning researcher, I can interact with the same registered environments and splits used by the offline benchmark.

**Acceptance scenarios**

1. Online rollout uses the same observation/action/task/sensor contracts.
2. Environment steps expose reward or phase labels and termination semantics.
3. Online-collected data can be validated and replayed by the official dataset tools.
4. Offline and online results identify their data regime and cannot be silently aggregated.

### User Story 7 — Compare and publish baseline results

As a result reviewer, I can compare matched baselines and regenerate leaderboard entries from submitted episode records.

**Acceptance scenarios**

1. Every policy publishes modalities, model/compute scale, action conversion, horizon, training budget, and supported protocols.
2. Aggregates regenerate exactly from episode records.
3. Duplicate, stale, incomplete, tampered, or incompatible result bundles are rejected.
4. A static leaderboard and radar/HTML report are generated from validated bundles.

## 7. Functional Requirements

### Foundation and task environments

- **FR-001**: The active repository, contracts, configs, tests, and documentation MUST reproduce from a clean tracked checkout.
- **FR-002**: The benchmark MUST retain the existing Gate/status/claim and immutable-evidence rules.
- **FR-003**: PressButton MUST pass active G1 acceptance before formal suite data collection.
- **FR-004**: Every task MUST define scene, objects, robot, tactile sensors, reset/randomization, task-state success/failure, reward or phase labels, budgets, and split rules.
- **FR-005**: Task success MUST NOT use geometric, timer, or action-count fallback.
- **FR-006**: Runtime execution MUST retain finite, joint, workspace, motion, collision, penetration, budget, abort, and safe-retract guards.
- **FR-007**: Contact, scalar force, vector force, wrench, raw impulse, and tactile fields MUST remain distinct and explicitly masked.
- **FR-008**: Historical failed evidence MUST remain immutable.

### Task suites and variants

- **FR-009**: Paper-v1 MUST contain exactly 16 accepted task instances.
- **FR-010**: Each of four official suites MUST contain exactly four accepted tasks.
- **FR-011**: Every task MUST bind assets/licenses, object/material/physics attributes, language, reset distribution, reward/phase, task-state success, budgets, and protocol eligibility.
- **FR-012**: Every task MUST have scripted/oracle feasibility and reset-stability evidence.
- **FR-013**: Task and variant identifiers and semantic versions MUST remain stable across data, training, evaluation, and leaderboard artifacts.
- **FR-014**: Variant generation MUST be deterministic from tracked config and seed.
- **FR-015**: Suite coverage reports MUST expose skill, object, contact, material, trajectory, sensor, and difficulty distributions.

### Generalization protocols and splits

- **FR-016**: GP-01, GP-02, and GP-03 MUST be separately versioned protocols.
- **FR-017**: Every core protocol MUST publish immutable train, validation, test-seen, and test-unseen manifests.
- **FR-018**: Split manifests MUST bind every identity and randomization field relevant to the protocol.
- **FR-019**: A leakage auditor MUST reject forbidden object, mesh-family, parameter, generator-seed, sensor-calibration, scene, or privileged-metadata overlap.
- **FR-020**: Seen and unseen results MUST be reported separately under matched episode populations and budgets.
- **FR-021**: Generalization Gap MUST equal protocol-defined seen success minus unseen success.
- **FR-022**: Zero-shot, calibration-only, task-data adaptation, offline, and online regimes MUST be distinct.
- **FR-023**: Test manifests MUST NOT influence training, tuning, or checkpoint selection.

### Data collection

- **FR-024**: Collection MUST support scripted/oracle, classical controller, teleoperation, trained-policy rollout, and custom expert adapters.
- **FR-025**: Every collected step MUST retain visual observations, tactile observations, joint state, end-effector pose, action, task phase/reward, success state, timestamps, and randomization/split parameters.
- **FR-026**: Randomization records MUST include applicable object, material, friction, stiffness/compliance, trajectory, scene, sensor, noise, latency, drift, and seed identities.
- **FR-027**: The batch collector MUST support task/suite selection, requested episode count, policy selection, parallel environments, retries, success filtering, resume, progress persistence, and statistics.
- **FR-028**: Collection interruption MUST retain validated completed episodes and resumable progress without duplicating episode IDs.
- **FR-029**: Dataset validation MUST reject duplicates, non-finite values, schema errors, invalid masks, missing timestamps, split leakage, and stale hashes.
- **FR-030**: The official offline release MUST contain at least 50 accepted training demonstrations per task and a declared validation set.
- **FR-031**: Held-out test variants MUST expose no training demonstrations.
- **FR-032**: Simulator replay MUST report outcome agreement, state/timing/Contact divergence, and first divergence.
- **FR-033**: Privileged replay/task state MUST remain separate from policy observations.
- **FR-034**: Community plugins for robots, sensors, tasks, experts, and observation modalities MUST pass versioned registration and contract validation.

### Unified training

- **FR-035**: One training entry point MUST support BC, ACT, Diffusion Policy, Transformer, and UniVTAC-compatible algorithms.
- **FR-036**: The trainer MUST use shared dataset loading, split resolution, normalization, observation/action horizon, seed, checkpoint, logging, and validation-selection contracts.
- **FR-037**: Vision-only, tactile-only where meaningful, and visual-tactile fusion MUST be selectable without changing the task or split.
- **FR-038**: Every training run MUST bind dataset, split, task, sensor, algorithm, source, and config digests.
- **FR-039**: Checkpoint selection MUST use validation data only.
- **FR-040**: Baseline fairness MUST record model parameters, training steps, optimizer/schedule, data count, compute, and selection rule.
- **FR-041**: Online training MUST use the same registered environment and observation/action contracts as offline training.
- **FR-042**: Online rollouts MUST be exportable through the official dataset writer and validator.

### Metrics and evaluation

- **FR-043**: Evaluation MUST report success, completion time, action/trajectory smoothness, trajectory efficiency, runtime validity, and safe retract.
- **FR-044**: Evaluation MUST report Contact Efficiency, contact count/duration/stability, slip, recovery, and modality-drop degradation when sources are valid.
- **FR-045**: Maximum/cumulative force, Force Smoothness, and force-limit metrics MUST be computed only from valid declared force measurements.
- **FR-046**: GP-01 through GP-03 MUST report matched seen/unseen results and absolute Generalization Gap.
- **FR-047**: Every evaluation MUST retain complete per-episode records and failure taxonomies.
- **FR-048**: One evaluation entry point MUST generate JSON, CSV, radar data, HTML, logs, manifest, checksums, and a result bundle.
- **FR-049**: Aggregates MUST regenerate deterministically from per-episode records.
- **FR-050**: Evaluation counts, seeds, budgets, and uncertainty methods MUST be frozen before formal runs.

### Baselines, toolkit, and release

- **FR-051**: Paper-v1 MUST provide scripted/oracle plus unified BC, ACT, Diffusion Policy, Transformer, and UniVTAC-compatible baseline configurations.
- **FR-052**: Policy manifests MUST declare modalities, context/action horizon, model/compute scale, training budget, adaptation mode, and unsupported combinations.
- **FR-053**: Matched comparisons MUST use identical data, splits, reset seeds, action contracts, budgets, and checkpoint-selection rules.
- **FR-054**: Result bundles MUST include per-episode results, policy/protocol/runtime metadata, hashes, and checksums.
- **FR-055**: The leaderboard validator MUST reject incomplete, duplicate, stale, tampered, or incompatible submissions.
- **FR-056**: The paper release MUST generate a static leaderboard and protocol radar/HTML report.
- **FR-057**: OpenVLA, π0, additional tasks, and hosted evaluation MUST remain compatible extension points without blocking paper-v1.
- **FR-058**: The release MUST package task/data/training/evaluation contracts, official data, baseline configs, results, licenses, and limitations.
- **FR-059**: Development driver `550.144.03` MUST remain `UNVALIDATED`; final release claims require reference/validated-driver revalidation or an explicit limitation.
- **FR-060**: The paper MUST report protocol-specific results rather than only aggregate success.
- **FR-061**: “First generalization benchmark” wording MUST be used only if the final related-work audit supports it.
- **FR-062**: Every active requirement and acceptance scenario MUST map to dependency-ordered tasks, tests, commands, and evidence.

## 8. Success Criteria

- **SC-001**: Clean-checkout integrity and the PressButton reference task pass required Gates.
- **SC-002**: Exactly 16 accepted tasks exist, four per suite, with zero missing task-card or license records.
- **SC-003**: GP-01, GP-02, and GP-03 manifests pass leakage audits with zero forbidden overlap.
- **SC-004**: The official dataset contains at least 800 accepted training demonstrations, at least 50 per task, plus a declared validation set and no test-variant training demonstrations.
- **SC-005**: Batch collection resumes after interruption without duplicate episode IDs and validates all retained episodes.
- **SC-006**: At least two expert-source types besides scripted/oracle successfully collect schema-valid data.
- **SC-007**: BC, ACT, Diffusion Policy, Transformer, and UniVTAC-compatible configurations train through the same entry point and produce validated checkpoints.
- **SC-008**: Vision-only and visual-tactile variants use identical data/splits/budgets and differ only by declared modality/config changes.
- **SC-009**: One evaluation command produces every required artifact and exact aggregate regeneration.
- **SC-010**: Each core protocol reports three policy seeds and at least 20 test episodes per task variant/condition per seed.
- **SC-011**: Contact/force/slip/recovery metrics are never emitted as valid when required source measurements are invalid.
- **SC-012**: Offline and online results identify their data regime and are never silently aggregated.
- **SC-013**: A validated static leaderboard and radar/HTML report regenerate from result bundles.
- **SC-014**: No released result omits runtime-invalid episodes or silently replaces failed seeds.
- **SC-015**: Final tables and figures regenerate from the release package on the declared environment.
- **SC-016**: Paper claims are traceable to completed Gates and protocol-specific evidence.

## 9. Key Entities

- `TaskFamily`
- `TaskInstance`
- `SuiteManifest`
- `DomainVariant`
- `SensorDomain`
- `ProtocolDefinition`
- `SplitManifest`
- `LeakageAudit`
- `ExpertAdapter`
- `CollectionJob`
- `DemonstrationEpisode`
- `ReplayRecord`
- `TrainingRun`
- `PolicyCapability`
- `EvaluationCell`
- `EpisodeResult`
- `GeneralizationAggregate`
- `ResultBundle`
- `LeaderboardEntry`
- `CommunityPlugin`

## 10. Edge Cases

- One asset appears under different filenames or transforms.
- Parameter ranges overlap seen/unseen boundaries.
- A collector restarts after writing an episode but before updating progress.
- Success filtering removes hard but valid demonstrations and biases the dataset.
- A custom expert writes a different action convention.
- Teleoperation timestamps or sensor frames are incomplete.
- Tactile data is unavailable but the schema contains placeholder arrays.
- A training algorithm changes normalization or horizon outside the shared config.
- A test checkpoint was selected using test performance.
- Online data is mixed with offline data without labeling.
- An evaluation aborts before task completion.
- Aggregates are correct but episode records were altered.
- A community plugin uses an incompatible contract version.

All cases must fail explicitly or follow a published rule; silent fallback is forbidden.

## 11. Gate Order

```text
P0/G-1A/G-1B migration
→ G0 repository integrity
→ G1 PressButton reference runtime
→ G2 task/sensor/data/training/evaluation contracts and registries
→ G3 tactile sensor interoperability and collection foundation
→ G4 16 tasks, official data, online collection, and replay
→ G5 unified training, three generalization protocols, metrics, and evaluation toolkit
→ G6 baseline results, static leaderboard, paper, and release
```

Offline schema/design work may proceed in parallel, but formal data collection or benchmark claims cannot bypass predecessor Gates.

## 12. Assumptions

- Paper-v1 uses 16 high-quality tasks rather than 100 shallow variants.
- The official dataset starts with at least 50 accepted training demonstrations per task and expands based on quality/data-scaling studies.
- Default formal evaluation uses three policy seeds and at least 20 test episodes per task condition per seed.
- At least two tactile sensor domains are available for GP-03; exact names depend on adapter, asset, and license validation.
- Scripted/oracle, BC, ACT, Diffusion Policy, Transformer, and UniVTAC-compatible configurations are paper-v1 scope.
- OpenVLA, π0, continual learning, and hosted leaderboard execution are planned extensions.

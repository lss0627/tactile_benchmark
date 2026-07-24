# Acceptance Gates

## Global Rules

1. Status remains `NOT_STARTED`, `IN_PROGRESS`, `BLOCKED`, `PASS_SMOKE`, or `PASS_BENCHMARK`.
2. Every pass requires fresh evidence tied to the producing commit and complete checksums.
3. Historical failed evidence remains immutable.
4. Later Gates cannot repair earlier Gates.
5. Runtime safety and sensor truth remain fail closed.
6. Development driver `550.144.03` remains `UNVALIDATED`.
7. Offline and online results are distinct tracks.
8. Optional expansion work cannot elevate paper-v1 Gates.

## G0 — Repository Integrity

### PASS_BENCHMARK

- Clean tracked checkout.
- Python/runtime inputs and asset provenance.
- Current/portable/external/future test inventories.
- Evidence schemas and freshness.
- No forbidden first-party deprecated Isaac imports.
- Generalization rebaseline documents and contracts tracked.

Current state: prior G0 passed; a fresh rebaseline-bound G0 is required.

## G1 — PressButton Reference Runtime

### Required

- Isaac Sim 6.0.1/Python 3.12 runtime identity.
- Public `make_env → reset → step → close`.
- Movable button and task-state press/release.
- Hard runtime guards and failure retention.
- Truthful Contact/raw Contact and validity masks.
- RGB/depth timing and media.
- 100 complete reset/lifecycle cycles.
- One rendered 500-step bounded rollout.
- 10 consecutive press/release/safe-retract episodes.
- Zero discarded formal failures and zero post-abort actuation.

Full-sweep/GJK/cooked-shape diagnostics remain optional.

Current state: `BLOCKED` pending new benchmark-oriented runtime evidence.

## G2 — Contracts and Registries

### PASS_BENCHMARK

Versioned contracts and tests exist for:

- public environment/action/observation/lifecycle;
- task family/instance/variant/suite;
- sensor domains and masks;
- robot/task/sensor/expert/modality plugins;
- collection job/episode/progress;
- dataset/split/replay;
- training config/run/checkpoint;
- protocol/leakage;
- policy capability;
- evaluation/result/submission/leaderboard.

Required registries:

- robot;
- task;
- sensor;
- expert;
- observation modality;
- policy/training algorithm.

Community plugins fail explicitly on incompatible versions or missing capabilities.

## G3 — Sensor and Collection Foundation

### Sensor acceptance

- At least two tactile sensor domains satisfy the same observation/timing/mask contract.
- Zero-shot, calibration-only, and task-data adaptation metadata are distinct.
- Noise, latency, drift, drop, and occlusion configs are deterministic.
- Invalid vector force/wrench remains unavailable.

### Collection acceptance

- Scripted/oracle plus at least two additional expert-source types produce valid data.
- Batch collector supports parallel environments, resume, bounded retry, configurable retention, progress journal, statistics, and validation.
- Interruption/resume produces no duplicate episode IDs.
- Custom expert/task/sensor/plugin registration passes contract tests.
- A tiny offline dataset can be replayed.
- A tiny online rollout can be exported through the same writer/validator.

## G4 — Four Suites, 16 Tasks, Official Dataset, and Replay

### Task acceptance

- Exactly four suites.
- Exactly four accepted tasks per suite.
- Exactly 16 accepted task instances total.
- Every task card contains complete environment, sensor, reset/randomization, success/failure, reward/phase, budget, split, asset/license, and feasibility fields.
- Every task passes reset stability and scripted/oracle feasibility.
- No geometric/action-count success fallback.

### Protocol/split acceptance

- GP-01, GP-02, and GP-03 train/validation/test-seen/test-unseen manifests exist.
- Leakage audit reports zero forbidden overlap.
- All randomization/sensor/domain parameters required by splits are recorded in data.

### Dataset acceptance

- At least 50 accepted training demonstrations per task: at least 800 total.
- A declared validation set exists.
- Test variants expose no training demonstrations.
- Dataset validator reports zero duplicates, non-finite records, schema errors, invalid masks, timestamp failures, and split leakage.
- Collection-source and rejection statistics are published.
- Simulator replay reports declared outcome agreement and first-divergence records.
- Dataset card, licenses, and provenance are complete.

## G5 — Unified Training and Generalization Evaluation

### Training acceptance

- BC, ACT, Diffusion Policy, Transformer, and UniVTAC-compatible algorithms run through the same entry point.
- Shared dataset loading, normalization, horizons, seeds, checkpointing, logging, budgets, and validation selection are enforced.
- Vision-only, tactile-only where meaningful, and visual-tactile configs use the same task/split/action contracts.
- At least one complete validated training run exists for every required algorithm.
- Checkpoint selection uses validation data only.

### Evaluation acceptance

- GP-01, GP-02, and GP-03 execute from immutable audited manifests.
- Formal results use three policy seeds and at least 20 test episodes per task condition per seed.
- Seen/unseen results and absolute Generalization Gap are reported.
- Success, time, smoothness, efficiency, Contact, slip, recovery, modality-drop, and valid force metrics follow versioned formulas.
- Runtime-invalid/failed episodes are retained and not replaced.
- One command produces JSON, CSV, radar data, HTML, manifest, checksums, and result bundle.
- Aggregates regenerate exactly from per-episode records.
- Offline and online tracks remain separate.

## G6 — Baselines, Leaderboard, Paper, and Release

### Baseline acceptance

- Scripted/oracle reference plus BC, ACT, Diffusion Policy, Transformer, and UniVTAC-compatible baseline results.
- Matched data, splits, reset seeds, action contract, budgets, and checkpoint-selection rules.
- Model/data/compute differences disclosed.
- Three policy seeds complete without selective omission.

### Leaderboard acceptance

- Result-bundle validator rejects incomplete, duplicate, stale, tampered, or incompatible bundles.
- Static leaderboard and radar/HTML reports regenerate from accepted bundles.
- Episode-to-aggregate provenance is complete.
- Offline and online tracks are labeled.

### Release acceptance

- Code, locks, task cards, sensors, plugins, official dataset, training configs, checkpoints/results as permitted, evaluator, leaderboard, licenses, and limitations are packaged.
- Final tables/figures regenerate.
- Reference/validated-driver rerun is complete or release remains explicitly non-reference and cannot pass G6.
- Related-work review supports final novelty wording.
- Paper reports protocol-specific results, not only aggregate success.

## Extension Acceptance

The following are extension targets and do not block paper-v1:

- 100-task expansion;
- trajectory/task/scene/continual protocols;
- OpenVLA and π0;
- hosted checkpoint execution;
- real-robot transfer.

They must use the same versioned contracts and receive separate evidence before any extension claim.

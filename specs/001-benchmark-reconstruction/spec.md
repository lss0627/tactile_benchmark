# Feature Specification: Benchmark Reconstruction Program

**Feature Branch**: `001-benchmark-reconstruction`

**Created**: 2026-07-10

**Status**: Implementation Ready (documentation only; G0-G6 remain unimplemented)

**Input**: User description: "Use Spec Kit to inspect the whole project and the previously reviewed documentation, then generate the specification, design, tasks, acceptance, and implementation-handoff artifacts for a complete reconstruction."

## Scope & Claim Boundary *(mandatory)*

- **In Scope**: Define the dependency-ordered reconstruction of repository integrity, real FR3 safety, a physical PressButton task, the unified runtime contract, tactile sensing, dataset/replay, evaluation, baselines, and release readiness.
- **Out of Scope**: Executing the generated implementation tasks in this documentation run, a full LIBERO migration, multi-robot support, real-hardware deployment, a leaderboard service, and paper performance claims.
- **Highest Allowed Claim**: `IMPLEMENTATION_READY_SPEC` after all Spec Kit artifacts pass consistency analysis. Existing code may retain only its already evidenced mock or runtime-smoke claims.
- **Blocked Follow-on Work**: Formal dataset collection, benchmark evaluation, baseline comparisons, task-suite expansion, and paper claims remain blocked until their preceding implementation gates pass.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Reproduce the Audited Repository (Priority: P1)

A maintainer can obtain a fresh copy of the project and recover every source file, configuration, test, and instruction required to reproduce the declared mock and diagnostic gates without depending on one developer's untracked files or absolute paths.

**Why this priority**: No runtime, dataset, or benchmark result is reviewable while required source and configuration are ignored or untracked.

**Independent Test**: A clean-room checkout can install the documented dependencies, discover the same public modules and scripts, run the no-simulator test suite, and explain how external simulator assets are supplied.

**Acceptance Scenarios**:

1. **AS-US1-1**: **Given** a fresh checkout, **When** the maintainer follows the quickstart, **Then** all required source packages and configuration files are present and no required module is hidden by ignore rules.
2. **AS-US1-2**: **Given** a machine with a different home directory, **When** runtime assets are configured, **Then** no mandatory command requires `/mnt/data/home/lss/...` or an ignored output file.
3. **AS-US1-3**: **Given** a completion claim, **When** the maintainer inspects its evidence manifest, **Then** the producing code revision, configuration digest, command, timestamp, and claim class are available.

---

### User Story 2 - Validate a Safe Physical PressButton Loop (Priority: P1)

A robotics engineer can run a bounded real-FR3 Isaac Sim PressButton approach, press, release, and retract sequence in which button success comes from movable task state and every safety rule is enforced during motion.

**Why this priority**: The current geometric TCP projection and unsafe retract cannot support a physical task, dataset, or evaluation gate.

**Independent Test**: The engineer runs a single-task physical runtime gate that proves actual button travel, safe press-and-retract behavior, zero workspace or joint-limit violations, and truthful force availability.

**Acceptance Scenarios**:

1. **AS-US2-1**: **Given** a configured FR3 and movable button mechanism, **When** the press sequence runs, **Then** success is derived from observed button state rather than TCP position or commanded depth.
2. **AS-US2-2**: **Given** a retract command, **When** observed motion diverges, exceeds a budget, violates the workspace, or increases unsafe penetration, **Then** the run aborts immediately and reports a failed gate.
3. **AS-US2-3**: **Given** unavailable physical force data, **When** the task reports contact and success, **Then** force and wrench masks remain false and no geometric value is encoded as tactile force.
4. **AS-US2-4**: **Given** a changed controller or safety configuration, **When** an older runtime artifact is inspected, **Then** it is marked stale and cannot satisfy the current gate.

---

### User Story 3 - Use One Unified Benchmark Contract (Priority: P2)

A benchmark developer can create mock, placeholder, and real-FR3 environments through the same public API and receive versioned action, observation, tactile, task, and metric contracts with explicit capability differences.

**Why this priority**: Standalone smoke scripts cannot be trained, evaluated, or extended as a benchmark backend.

**Independent Test**: Contract tests instantiate each supported backend, verify the stable 7D action interface and observation schema, and reject configurations whose joint or frame names do not match runtime introspection.

**Acceptance Scenarios**:

1. **AS-US3-1**: **Given** a real-FR3 backend request, **When** the public environment factory creates PressButton, **Then** reset, step, close, action clipping, observations, info, and termination follow the benchmark contract.
2. **AS-US3-2**: **Given** a robot configuration, **When** it is validated against introspection, **Then** incorrect joint, gripper, camera, EE, or tactile-frame names block the gate.
3. **AS-US3-3**: **Given** a 7D action, **When** the backend does not support an action component, **Then** it rejects or versions that limitation rather than silently ignoring rotation or gripper fields.
4. **AS-US3-4**: **Given** any tactile mode, **When** data are absent, **Then** shapes remain loadable while validity and modality masks accurately represent absence.

---

### User Story 4 - Produce Auditable Tasks, Data, Replay, and Evaluation (Priority: P3)

A dataset and evaluation maintainer can accept tasks through complete task cards, collect versioned episodes, validate synchronization and integrity, replay simulator behavior, and generate statistically compliant evaluation outputs.

**Why this priority**: Schema-only replay and deterministic step-count tasks cannot establish dataset quality or policy performance.

**Independent Test**: One accepted PressButton task produces a mini-scale dataset that passes schema, integrity, synchronization, physical replay, split, and evaluation-output gates before any suite expansion.

**Acceptance Scenarios**:

1. **AS-US4-1**: **Given** a duplicate episode identifier, **When** a writer attempts to store it, **Then** the operation fails without replacing the existing episode.
2. **AS-US4-2**: **Given** a recorded episode, **When** validation runs, **Then** array lengths, finite values, timestamps, sensor skew, masks, metadata, checksums, and task labels are checked.
3. **AS-US4-3**: **Given** a replay request, **When** it runs, **Then** it restores simulator/task state and compares physical success and metrics rather than only checking saved array shapes.
4. **AS-US4-4**: **Given** evaluation results, **When** they are reported, **Then** per-episode, per-task, per-suite, aggregate, configuration, hash, uncertainty, and failure-taxonomy artifacts are present.
5. **AS-US4-5**: **Given** a task without a complete card or accepted physical oracle/replay, **When** suite expansion is requested, **Then** expansion remains blocked.

---

### User Story 5 - Train Fair Baselines and Release Reviewable Artifacts (Priority: P4)

A research maintainer can train and compare declared baselines only on frozen validated splits, select checkpoints without test leakage, reproduce reported tables, and publish a licensed, installable artifact package.

**Why this priority**: Baseline results are meaningful only after tasks, datasets, and evaluation protocols pass.

**Independent Test**: A release reviewer can train one supported baseline on the frozen training split, reproduce evaluation from its checkpoint, trace every result to configuration and data hashes, and complete the release checklist.

**Acceptance Scenarios**:

1. **AS-US5-1**: **Given** a baseline marked trainable, **When** training is requested, **Then** it either performs real optimization for its declared modalities or is explicitly labeled a non-result skeleton.
2. **AS-US5-2**: **Given** training and evaluation splits, **When** a checkpoint is selected, **Then** only training and validation data influence selection.
3. **AS-US5-3**: **Given** a tactile-versus-vision comparison, **When** results are produced, **Then** data, action space, training budget, visual/language encoders, seeds, parameters, and compute are reported fairly.
4. **AS-US5-4**: **Given** a public release candidate, **When** the release gate runs, **Then** licenses, citation, environment lock, CI, checksums, cards, known issues, and reproduction commands are complete.

### Edge Cases

- Required source is present locally but excluded by ignore rules or absent from version control.
- A runtime artifact was generated before the safety code or configuration it purports to validate.
- Simulator modules are unavailable and only a dry-run can execute.
- A contact signal exists but no valid force vector or wrench is available.
- The robot introspection report contains additional joints or different names from the planned configuration.
- A controller reaches commanded depth while the physical button state does not change.
- Motion stays finite but violates workspace, joint, cumulative drift, direction, or operator budget constraints.
- Dataset arrays have valid individual shapes but inconsistent timestep lengths.
- Replay succeeds in a mock counter-based environment but fails to reproduce simulator state.
- A metric is unavailable for a modality or failed episode.
- A task, model, or dataset attempts to advance while its predecessor gate is `BLOCKED`.
- Existing historical documents disagree with the canonical Spec Kit state.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: The project MUST track every required source, configuration, test, schema, and canonical document needed by a declared gate.
- **FR-002**: Ignore rules MUST distinguish generated datasets from the `isaac_tactile_libero/datasets` source package and MUST NOT hide required runtime configuration.
- **FR-003**: The project MUST provide a fresh-checkout setup and no-simulator verification path with pinned or locked environment requirements.
- **FR-004**: External simulator, robot, and tactile assets MUST have configurable paths, versions, provenance, license information, and availability diagnostics.
- **FR-005**: Every status and artifact MUST declare one of `NOT_STARTED`, `IN_PROGRESS`, `BLOCKED`, `PASS_SMOKE`, or `PASS_BENCHMARK` and MUST declare its claim class.
- **FR-006**: Force and wrench fields MUST only become valid from documented physical or simulator force sources; displacement, TCP pose, proximity, and success labels MUST NOT populate them.
- **FR-007**: The PressButton runtime MUST use a movable mechanism with observable travel, limits, reset state, and release state.
- **FR-008**: PressButton success MUST be derived from observed button state for a required duration and MUST NOT be derived solely from end-effector pose or commanded motion.
- **FR-009**: Real runtime motion MUST enforce workspace, joint-position, joint-velocity, finite-value, collision/penetration, direction, per-step, cumulative-drift, and stop-condition limits.
- **FR-010**: Runtime motion MUST enforce operator step and wall-time budgets as hard limits.
- **FR-011**: Runtime evidence MUST record code revision, dirty-state status, configuration digests, asset identifiers, command, timestamp, and artifact schema version; stale evidence MUST fail readiness checks.
- **FR-012**: The public environment factory MUST support the accepted real-FR3 PressButton backend through the standard reset, step, close, observation, action, info, and termination interfaces.
- **FR-013**: Robot configuration MUST be validated against runtime joint names, joint limits, base/EE/gripper/tactile/camera frames, and default pose before control is enabled.
- **FR-014**: All supported backends MUST implement the declared 7D action semantics, or reject/version unsupported rotation and gripper behavior without silent omission.
- **FR-015**: Tactile modes MUST share one observation contract and accurately represent supported, missing, delayed, dropped, saturated, or invalid modalities.
- **FR-016**: At least one physical PressButton task card MUST pass all task-acceptance rules before additional benchmark tasks or formal datasets are allowed.
- **FR-017**: Formal task success, reward, and metrics MUST depend on task state and actions rather than deterministic elapsed-step counters.
- **FR-018**: Dataset writing MUST reject duplicate episode identifiers and store complete version, task, robot, action, observation, timestamp, calibration, frame, split, checksum, and provenance metadata.
- **FR-019**: Dataset validation MUST check required keys, matching lengths, shapes, finite values, monotonic timestamps, sensor skew, masks, saturation, frame drops, checksums, split leakage, and task-specific fields.
- **FR-020**: Replay acceptance MUST restore simulator/task state, execute recorded actions through the accepted controller, and compare success, state deviation, and metric consistency.
- **FR-021**: Evaluation MUST emit frozen configuration, per-episode, per-task, per-suite, aggregate, failure, log, and optional video artifacts with dataset, checkpoint, task, and sensor hashes.
- **FR-022**: Evaluation MUST implement the documented unweighted aggregation, confidence intervals, seed-level uncertainty, robustness splits, and missing-metric rules.
- **FR-023**: Main baseline training MUST remain blocked until the formal dataset gate passes and MUST implement real optimization, validation-only model selection, normalization from train data, checkpointing, and declared modality filtering.
- **FR-024**: Baseline comparison MUST record common splits, action space, training budget, seeds, parameter counts, encoders, fusion, compute, checkpoint hashes, and privileged-input status.
- **FR-025**: Public release MUST include license and citation files, environment lock, CI, installation and reproduction instructions, dataset/model cards, checksums, provenance, known issues, and a release archive.
- **FR-026**: Canonical project status and acceptance documents MUST be generated from or synchronized with Spec Kit artifacts and MUST not contradict current gate evidence.
- **FR-027**: Every requirement and acceptance scenario MUST map to dependency-ordered tasks, tests, exact verification commands, and expected evidence artifacts.
- **FR-028**: Gate ordering MUST block formal collection, evaluation, baseline comparison, suite expansion, and paper claims until all predecessor gates pass.

### Key Entities

- **Gate**: A named transition with prerequisites, allowed claim, status, verification commands, and evidence artifacts.
- **Requirement**: A stable FR or SC identifier linked to user stories, tasks, tests, and gates.
- **Evidence Artifact**: A report, log, dataset, screenshot, manifest, or hash bundle tied to the code and configuration that produced it.
- **Task Card**: The complete, versioned definition of a benchmark task, including reset, success, failure, termination, metrics, splits, assets, and leakage risks.
- **Runtime Episode**: One reset-to-termination interaction with synchronized observations, actions, task state, safety events, and outcome.
- **Dataset Release**: A versioned collection of episodes with frozen splits, validation, replay evidence, cards, checksums, and provenance.
- **Evaluation Run**: A frozen policy/task/sensor/split configuration with per-episode results, aggregates, uncertainty, failures, and hashes.
- **Baseline Run**: A training and evaluation record with modality contract, data split, normalization, budget, checkpoint, and compute metadata.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: A fresh checkout contains 100% of required source/configuration files, installs through one documented path, and passes the complete no-simulator test suite with zero missing-module failures.
- **SC-002**: No mandatory configuration contains a developer-specific absolute path, and every external asset dependency has a documented configurable resolution path and provenance record.
- **SC-003**: The physical PressButton gate completes at least 10 consecutive press-release-retract episodes with 100% observed button reset/release, zero safety violations, and zero stale-artifact acceptance.
- **SC-004**: Every enforced runtime safety rule has at least one passing boundary test and one failing/abort test, and operator budgets are never exceeded in a passing run.
- **SC-005**: The accepted real backend passes 100 resets, a 500-step bounded rollout, complete 7D contract tests, and frame/joint introspection validation without NaN or persistent penetration.
- **SC-006**: One PressButton mini-scale dataset has at least 10 valid physical episodes, zero missing/shape/NaN/timestamp/checksum errors, and at least 90% simulator replay success before formal collection begins.
- **SC-007**: Evaluation produces every artifact required by FR-021 and reproduces aggregate values from per-episode records with 100% consistency.
- **SC-008**: Every FR, buildable SC, and acceptance scenario has at least one task and one named evidence path; cross-artifact analysis reports 100% task coverage and zero CRITICAL inconsistencies.
- **SC-009**: The five original core tasks do not advance to accepted status until each has a complete card and physical/scripted-oracle evidence; no 20-30 task expansion starts earlier.
- **SC-010**: Main baseline and paper-result gates remain blocked until dataset and evaluation gates pass, with no skeleton or runtime-smoke output labeled as a benchmark result.
- **SC-011**: A release reviewer can install, validate a sample dataset, replay one physical episode, run one evaluation, and regenerate one mini result table using the published instructions and artifacts.

## Assumptions

- The existing mock, pusher, and EE-placeholder paths remain useful regression fixtures but cannot satisfy physical or benchmark gates.
- Isaac Sim and the licensed FR3 asset remain external dependencies rather than files redistributed by this repository.
- The stable public action and observation schemas remain version `0.1.0` until an explicitly planned incompatible change is approved.
- The reconstruction starts with one accepted PressButton task; task-suite scaling is deliberately deferred.
- Existing runtime-smoke HDF5 files remain diagnostic evidence and will not be promoted into the formal dataset.
- This Spec Kit feature produces implementation-ready documents only; application-code changes require a later explicit `/speckit-implement` run.

## Evidence & Traceability *(mandatory)*

- **Required Evidence Classes**: Requirement-quality checklist, unit/schema tests, clean-room install checks, runtime safety tests, physical task runs, dataset validation/replay, evaluation reproduction, and release audit.
- **Required Artifacts**: Versioned JSON/JSONL/CSV/YAML reports, logs, screenshots or videos where relevant, HDF5 plus checksums, task/dataset/model cards, and an evidence manifest with code/config digests.
- **Requirement Mapping Rule**: Every FR/SC and acceptance scenario MUST map to tasks and evidence.
- **Freshness Rule**: Runtime artifacts MUST identify the code/config version that produced them and MUST be regenerated after meaning-affecting changes.

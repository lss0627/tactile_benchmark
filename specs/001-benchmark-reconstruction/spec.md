# Feature Specification: Benchmark Reconstruction Program

**Feature Branch**: `001-benchmark-reconstruction`

**Created**: 2026-07-10

**Status**: Implementation In Progress (Isaac Sim 6.0.1 migration checkpoints and G0 complete;
G1-G6 remain pending)

**Input**: User description: "Use Spec Kit to inspect the whole project and the previously reviewed documentation, then generate the specification, design, tasks, acceptance, and implementation-handoff artifacts for a complete reconstruction."

## Scope & Claim Boundary *(mandatory)*

- **In Scope**: Establish Isaac Sim 6.0.1 and Python 3.12 as the development baseline without
  changing the installed driver; preserve Isaac Sim 5.1 as an archived reference; and define the
  dependency-ordered reconstruction of repository integrity, real FR3 safety, a physical
  PressButton task, the unified runtime contract, tactile sensing, dataset/replay, evaluation,
  baselines, and release readiness.
- **Out of Scope**: Changing the NVIDIA driver, migrating Isaac Lab, downloading the five-part
  Complete Assets Pack, validating native GPU-physics Contact, a full LIBERO migration,
  multi-robot support, real-hardware deployment, a leaderboard service, and paper performance
  claims.
- **Highest Allowed Claim**: G0 may report `PASS_BENCHMARK` for repository integrity and the
  P0/G-1A/G-1B compatibility checkpoints may report `PASS_SMOKE` with claim class
  `runtime_smoke`. These results do not establish G1-G6, physical, dataset, evaluation, baseline,
  release, or paper claims.
- **Blocked Follow-on Work**: Formal physical Contact evidence, dataset collection, benchmark
  evaluation, baseline comparisons, task-suite expansion, release claims, and paper claims remain
  blocked until their predecessor gates pass. Release-level simulator evidence additionally
  requires revalidation on an NVIDIA reference/validated driver.

## User Scenarios & Testing *(mandatory)*

### User Story 0 - Establish the Isaac Sim 6.0.1 Development Baseline (Priority: P0)

A maintainer can reproduce the layered migration from the archived Isaac Sim 5.1/Python 3.11
reference to an independent Isaac Sim 6.0.1/Python 3.12 development environment while keeping
driver `550.144.03` unchanged and preserving truthful runtime-support metadata.

**Why this priority**: Every later simulator-facing task must target one declared API and runtime
baseline. Performing this migration before G1 avoids implementing new benchmark components on
removed APIs and then migrating them again.

**Independent Test**: The maintainer uses the frozen candidate environment to reproduce P0,
G-1A, G0, and G-1B evidence; confirms the public environment chain, Contact and Camera contracts,
100 reset lifecycles, and a 500-step rendered rollout; and verifies that the formal Python 3.12
lock is exactly the promoted candidate lock.

**Acceptance Scenarios**:

1. **AS-US0-1**: **Given** the existing 5.1 reference installation and driver `550.144.03`,
   **When** P0 creates the independent 6.0.1 environment and runs compatibility and 100-step
   startup checks, **Then** the driver and 5.1 installation remain unchanged and the report marks
   the observed driver `UNVALIDATED` rather than unsupported or validated.
2. **AS-US0-2**: **Given** the licensed 5.1 FR3 and scene assets, **When** G-1A loads them under
   6.0.1, **Then** every reference, payload, mesh, texture, articulation joint, limit, default pose,
   and required frame resolves, and a 500-step physics-plus-rendering check completes without NaN,
   crash, or persistent penetration.
3. **AS-US0-3**: **Given** a Contact Sensor created when the timeline starts, **When** each of 100
   stop/reset/play cycles runs, **Then** it becomes ready within the configured window, detects the
   scripted press within the onset tolerance, clears contact within the release timeout and
   debounce window, and never exposes an invalid three-dimensional force or wrench mask.
4. **AS-US0-4**: **Given** the repository environment factory, **When** G-1B executes `make_env`,
   reset, a bounded 7D action, observation/info, Contact, Camera, release/reset, and close, **Then**
   public contract snapshots remain stable and no fake force, stale sensor handle, invalid reading,
   NaN, or persistent penetration is accepted.
5. **AS-US0-5**: **Given** G0 and G-1B passing evidence, **When** the candidate environment is
   promoted, **Then** Python 3.12 and Isaac Sim 6.0.1 become the development baseline, the 5.1
   inputs move to archived/reference status, and release-level gates remain blocked pending
   reference-driver revalidation.

---

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

- Isaac Sim 6.0.1 starts on driver `550.144.03`, which is usable for migration development but is
  not in the declared reference-driver set.
- Contact is queried immediately after `play()` before the dynamically created sensor becomes
  live, or release state oscillates for one or more physics steps.
- CPU-physics Contact passes while native GPU-physics Contact crashes, hangs, or produces unstable
  readings; the runtime must fail fast instead of silently changing evidence class.
- A sensor reading is valid and reports scalar force magnitude, but no validated force vector or
  wrench exists for the public observation contract.
- RGB or depth buffers are allocated but remain constant, stale, non-finite, outside the clipping
  range, or unsynchronized with the physics step.
- A 5.1 asset root opens but contains unresolved nested references, payloads, meshes, or textures
  under 6.0.1.
- The Python 3.12 G-1B environment differs from the candidate lock that was reviewed at G0.
- A first-party source file reintroduces removed `omni.isaac.*`, dynamic-control, or deprecated
  core imports after cutover.
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
- **FR-029**: The development baseline MUST be Isaac Sim `6.0.1`, Python `3.12`, and the promoted
  Python 3.12 dependency lock; Isaac Sim 5.1/Python 3.11 MUST remain reproducible only as an
  archived/reference baseline after cutover.
- **FR-030**: P0, G-1A, and G-1B MUST remain compatibility checkpoints rather than new formal
  Gates, and their reports MUST use the existing `PASS_SMOKE` or `BLOCKED` status and
  `runtime_smoke` claim class with a separate compatibility scope, runtime-support record,
  compatibility result, and blocker codes.
- **FR-031**: Runtime-support metadata MUST record simulator and Python versions, observed driver,
  reference driver, and driver validation. Driver `550.144.03` MUST remain unchanged and MUST be
  labeled `UNVALIDATED`; release-level physical, dataset, replay, evaluation, and baseline evidence
  MUST be rerun on a currently reference/validated driver.
- **FR-032**: The reproducible Python 3.12 candidate lock MUST exist before G-1B and pin the Python
  patch version, package tooling, simulator package, PyTorch/CUDA distribution, and all direct and
  transitive dependencies. Promotion MUST preserve its content in the formal lock and archive the
  prior 5.1 inputs without erasing them.
- **FR-033**: First-party Python MUST contain no `omni.isaac.*` or dynamic-control imports. Imports
  from deprecated `isaacsim.core.api`, `isaacsim.core.prims`, or `isaacsim.core.utils` MAY only be
  warnings during G-1A and MUST be errors at cutover; third-party environments, logs, historical
  documents, and copied examples are outside the scan.
- **FR-034**: Contact acceptance MUST distinguish sensor readiness, contact state, scalar force
  magnitude, raw position/normal/impulse, force vector, and wrench validity. Scalar magnitude or
  raw impulse MUST NOT populate public force-vector or wrench fields, and no impulse-to-force or
  wrench derivation is accepted in this migration.
- **FR-035**: Contact lifecycle acceptance MUST allow at most 5 physics steps for readiness, 2
  physics steps for contact onset, and 5 physics steps for release; released state MUST remain
  stable for 3 steps, and no-contact scalar magnitude MUST remain at or below `1.0e-4`. All valid
  force magnitudes and raw impulses MUST be finite.
- **FR-039**: Full-robot design-time collision qualification MAY normalize only the source-bound
  representation difference between an OpenUSD analytic Cylinder authored on Z and the approved
  PhysX source analytic Cylinder on X. The raw poses MUST remain retained, the exact mapping MUST
  be version- and digest-bound, and the unchanged strict same-frame placement comparator MUST run
  after normalization. Representation equivalence MUST NOT assert backend shape identity,
  property-query placement authority, cooked-shape authority, or narrowphase authority, and
  runtime Contact/collision MUST remain an independent fail-closed truth source.
- **FR-036**: RGB/depth acceptance MUST validate contract shapes, RGB `uint8`, depth `float32`,
  finite and non-constant updating frames, configured valid-depth ratio and clipping behavior,
  real rendering ticks, timestamps, and sensor skew no greater than one camera tick.
- **FR-037**: The 5.1/6.0.1 A/B comparison MUST hash the common assets, trajectory, and physics and
  rendering configuration. Allowed zero-action drift MUST equal
  `min(max(2 * drift_5.1, 0.05 mm), 1.0 mm)`; first-contact timing may differ by at most two
  physics steps; and penetration MUST remain below both the absolute safety limit and the 5.1
  result plus 1 mm when a 5.1 result exists.
- **FR-038**: The accepted development runtime MUST use CPU physics for Contact while retaining GPU
  rendering. A request for native GPU-physics Contact MUST stop with an explicit
  `GPU_CONTACT_NATIVE_INSTABILITY` blocker until that path passes the same lifecycle and truthfulness
  checks on a reference-driver environment.

### Key Entities

- **Gate**: A named transition with prerequisites, allowed claim, status, verification commands, and evidence artifacts.
- **Requirement**: A stable FR or SC identifier linked to user stories, tasks, tests, and gates.
- **Evidence Artifact**: A report, log, dataset, screenshot, manifest, or hash bundle tied to the code and configuration that produced it.
- **Task Card**: The complete, versioned definition of a benchmark task, including reset, success, failure, termination, metrics, splits, assets, and leakage risks.
- **Runtime Episode**: One reset-to-termination interaction with synchronized observations, actions, task state, safety events, and outcome.
- **Dataset Release**: A versioned collection of episodes with frozen splits, validation, replay evidence, cards, checksums, and provenance.
- **Evaluation Run**: A frozen policy/task/sensor/split configuration with per-episode results, aggregates, uncertainty, failures, and hashes.
- **Baseline Run**: A training and evaluation record with modality contract, data split, normalization, budget, checkpoint, and compute metadata.
- **Compatibility Report**: A non-Gate evidence artifact for P0, G-1A, or G-1B containing scope,
  existing status and claim class, runtime-support metadata, compatibility result, blocker codes,
  artifact hashes, and producing revision.
- **Runtime Support Record**: Simulator, Python, observed-driver, reference-driver, and validation
  metadata that separates a passing development run from a reference-driver release claim.
- **Sensor Truth Record**: Per-reading readiness, contact, scalar magnitude, raw-contact, vector,
  wrench, mask, physics-step, rendering-tick, and timestamp validity without cross-populating
  unsupported modalities.

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
- **SC-012**: P0 reproduces a 6.0.1/Python 3.12 startup, Compatibility Checker result, minimal stage,
  and 100 simulation steps while preserving driver `550.144.03` and the complete 5.1 reference
  inventory.
- **SC-013**: G-1A resolves 100% of required FR3/scene asset dependencies, validates all declared
  articulation names, limits, poses, and frames, and completes 500 physics-plus-rendering steps
  with zero NaN, crash, or persistent penetration.
- **SC-014**: Contact passes 100 stop/reset/play cycles with zero invalid-after-ready or stale
  handles, 100% scripted contact detection inside the onset tolerance, 100% stable release inside
  the release window, and zero false public force-vector or wrench masks.
- **SC-015**: G-1B completes 100 repository lifecycle resets and a bounded 500-step rollout through
  the public environment chain with stable contract snapshots, finite observations, updating RGB
  and depth, zero persistent penetration, and a passing fixed-trajectory A/B report.
- **SC-016**: The cutover import scan covers every first-party Python file and reports zero removed,
  dynamic-control, or deprecated core imports; every original Python 3.12 regression node ID is
  present and passing or has a one-to-one migration-manifest replacement.
- **SC-017**: Every release-level physical, dataset, replay, evaluation, and baseline artifact
  produced only on the unvalidated development driver remains mechanically ineligible for a
  release claim until equivalent evidence passes on a reference/validated driver.

## Assumptions

- The existing mock, pusher, and EE-placeholder paths remain useful regression fixtures but cannot satisfy physical or benchmark gates.
- Isaac Sim 6.0.1 and the licensed FR3/tactile assets remain external dependencies rather than
  files redistributed by this repository; 5.1 assets are treated as versioned inputs requiring
  resolution checks, not relabeled as native 6.0.1 assets.
- Driver `550.144.03` is retained because it is required by the installed 4090 48 GB configuration;
  it is suitable for development evidence only and is not declared reference-validated.
- The NVIDIA EULA is accepted for the already authorized installation workflow. This migration
  does not change the driver, migrate Isaac Lab, or download the Complete Assets Pack.
- Contact uses CPU physics and RTX rendering may use the GPU. Native GPU-physics Contact remains a
  declared blocker rather than an automatic fallback or successful capability.
- The archived Isaac Sim 5.1/Python 3.11 environment and historical evidence remain immutable
  references; new implementation work is required to target only 6.0.1/Python 3.12 after cutover.
- The stable public action and observation schemas remain version `0.1.0` until an explicitly planned incompatible change is approved.
- The reconstruction starts with one accepted PressButton task; task-suite scaling is deliberately deferred.
- Existing runtime-smoke HDF5 files remain diagnostic evidence and will not be promoted into the formal dataset.
- G0 and the Isaac Sim 6.0.1 migration implementation are complete in the current development
  branch; G1-G6 application work still requires their dependency-ordered implementation tasks.

## Evidence & Traceability *(mandatory)*

- **Required Evidence Classes**: Requirement-quality checklist, compatibility/runtime-smoke,
  unit/schema tests, clean-room install checks, runtime safety tests, physical task runs, dataset
  validation/replay, evaluation reproduction, reference-driver revalidation, and release audit.
- **Required Artifacts**: Versioned compatibility reports, runtime-support records,
  JSON/JSONL/CSV/YAML reports, dependency locks, import scans, lifecycle/A/B/Camera/Contact reports,
  logs, screenshots or videos where relevant, HDF5 plus checksums, task/dataset/model cards, and an
  evidence manifest with code/config/asset digests.
- **Requirement Mapping Rule**: Every FR/SC and acceptance scenario MUST map to tasks and evidence.
- **Freshness Rule**: Runtime artifacts MUST identify the code/config version that produced them and MUST be regenerated after meaning-affecting changes.

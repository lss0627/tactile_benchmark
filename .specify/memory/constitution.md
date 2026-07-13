<!--
Sync Impact Report
- Version change: unratified template -> 1.0.0
- Added principles:
  - I. Evidence Before Claims
  - II. Reproducible Repository First
  - III. Stable Contracts and Versioned Change
  - IV. Safety-Gated Runtime Control
  - V. Traceable Test-First Delivery
- Added sections:
  - Benchmark Boundaries and Release Constraints
  - Development Workflow and Quality Gates
- Removed sections: none; template placeholders were replaced.
- Template synchronization:
  - ✅ .specify/templates/plan-template.md
  - ✅ .specify/templates/spec-template.md
  - ✅ .specify/templates/tasks-template.md
  - ✅ .specify/templates/checklist-template.md (already compatible)
  - ⚠ README.md remains historical runtime guidance and is covered by the generated feature tasks.
- Deferred items: none.
-->

# Isaac-Tactile-LIBERO Constitution

## Core Principles

### I. Evidence Before Claims

Every status, completion claim, benchmark result, and release decision MUST be backed by a
reproducible command and a versioned artifact. Mock, dry-run, runtime-smoke, geometric proxy,
synthetic force, real physics, and benchmark result states MUST remain explicitly distinct in
code, metadata, documentation, and reports. A passing schema test MUST NOT be presented as a
passing physical or benchmark acceptance gate. Force or wrench MUST never be inferred from
button displacement, TCP position, proximity, or other geometric proxies.

### II. Reproducible Repository First

All source, configuration, tests, schemas, and documentation required for a declared gate MUST
be tracked by the repository and available from a fresh clone. Required runtime paths MUST be
configurable and MUST NOT depend on one developer's absolute filesystem layout. Generated data,
large outputs, and proprietary simulator assets MAY remain external, but their acquisition,
version, checksum or provenance, and expected placement MUST be documented. A gate that relies
on ignored or untracked required files is automatically BLOCKED.

### III. Stable Contracts and Versioned Change

The public environment API, 7D action schema, observation schema, tactile masks, dataset schema,
metric definitions, task success conditions, and evaluation splits are contracts. Incompatible
changes MUST receive the semantic version bump required by `docs/benchmark_spec.md`, a migration
note, and contract tests. Runtime backends MAY vary internally, but they MUST expose the same
declared contract or identify themselves as non-benchmark diagnostics. Silent fallback, silent
episode replacement, and ambiguous rotation, frame, unit, or success semantics are forbidden.

### IV. Safety-Gated Runtime Control

Any non-dry-run robot or simulator motion MUST enforce workspace bounds, joint limits, finite
state checks, per-step and cumulative motion limits, operator step/time budgets, stop conditions,
and explicit failure status. Configured safety rules MUST be exercised by tests or runtime
evidence; merely recording a rule in YAML is insufficient. Physical task success MUST come from
task state or a documented simulator signal, not commanded TCP motion. A failed retract, unsafe
pose, stale artifact, or safety abort blocks evaluation and dataset collection until a new passing
artifact is produced by the current code and configuration.

### V. Traceable Test-First Delivery

Every functional requirement and acceptance scenario MUST map to one or more dependency-ordered
tasks, tests, exact verification commands, and expected artifacts. Tests MUST be written or
identified before implementation work and MUST distinguish unit/schema checks from simulator and
benchmark acceptance. Each task has one owner-visible outcome, exact file paths, a Definition of
Done, and evidence requirements. Checklist status MUST reflect observed evidence; unchecked,
partial, blocked, and complete states MUST not be conflated.

## Benchmark Boundaries and Release Constraints

- The project advances through explicit gates: repository integrity, physical single-task loop,
  unified real backend, tactile sensing, accepted tasks, dataset/evaluation, and baselines/release.
- A later gate MUST NOT begin until all blocking requirements of the previous gate pass, except
  for clearly labeled offline research that cannot create benchmark claims or datasets.
- The pusher, EE placeholder, and mock paths remain regression tools; they do not satisfy real FR3
  or tactile acceptance.
- Runtime-smoke datasets MUST retain `benchmark_result=false` and
  `not_for_paper_claims=true`. Formal datasets require frozen splits, validation, simulator replay,
  checksums, provenance, and a dataset card.
- Public release requires a license, citation metadata, environment lock, CI, install/quickstart
  instructions, known issues, and artifact provenance.
- Expansion beyond one accepted physical task is prohibited until the single-task physical loop,
  safety, replay, and evaluation contracts pass.

## Development Workflow and Quality Gates

1. Update or create the Spec Kit specification and requirements-quality checklist.
2. Complete research and design artifacts; resolve all clarification markers.
3. Pass the Constitution Check in `plan.md` before task generation.
4. Generate traceable tasks with tests before implementation and explicit artifact outputs.
5. Execute one gate at a time. Stop on failed non-parallel tasks or safety evidence.
6. Run the exact verification commands and record fresh outputs from the current code/config.
7. Run cross-artifact analysis before implementation and again before release.
8. Update project status and acceptance checklists in the same change as the evidence they cite.

Required review states are `NOT_STARTED`, `IN_PROGRESS`, `BLOCKED`, `PASS_SMOKE`, and
`PASS_BENCHMARK`. Only `PASS_BENCHMARK` may support benchmark or paper-result claims.

## Governance

This constitution supersedes conflicting project plans, task lists, status notes, and informal
milestone labels. Amendments require a documented rationale, a semantic version change, a sync
impact report, and review of dependent Spec Kit templates and active feature artifacts.

Every specification, plan, tasks file, implementation review, dataset gate, evaluation run, and
release review MUST include an explicit constitution compliance check. Violations of a MUST rule
are CRITICAL and block implementation or release until corrected. Complexity exceptions require
written justification in the active `plan.md`; safety, truthful-claim, and reproducibility rules
cannot be waived by a complexity exception.

**Version**: 1.0.0 | **Ratified**: 2026-07-10 | **Last Amended**: 2026-07-10

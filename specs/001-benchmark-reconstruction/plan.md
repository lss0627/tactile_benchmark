# Implementation Plan: Isaac Tactile LIBERO-Style Benchmark

**Branch**: `001-benchmark-reconstruction`
**Specification**: `spec.md`
**Runtime**: Isaac Sim 6.0.1, Python 3.12
**Rebaseline date**: 2026-07-24

## Summary

The project is rebaselined from a formal full-robot motion-proof effort to a reproducible, physics-backed tactile manipulation benchmark intended for publication.

The engineering sequence is:

```text
accepted single-task runtime
→ stable public environment API
→ truthful tactile capability
→ compact task suite and dataset
→ replay and evaluation
→ baselines and paper release
```

Practical runtime safety remains mandatory. Exhaustive continuous-sweep/GJK/cooked-shape proofs remain available as optional diagnostics but are no longer acceptance prerequisites.

## Technical Context

| Area | Decision |
|---|---|
| Simulator | Isaac Sim 6.0.1 |
| Python | `>=3.12,<3.13` |
| Physics | CPU physics, MBP broadphase, GPU dynamics disabled |
| Rendering | RTX camera path; GPU metadata recorded |
| Development driver | `550.144.03`, `UNVALIDATED` |
| Robot | Franka Research 3 |
| Reference task | Physics-backed PressButton |
| Public action | 7D bounded Cartesian/gripper command |
| Sensors | RGB, depth, Contact/raw Contact, optional tactile |
| Dataset | Versioned episode schema with hashes and masks |
| Evaluation | Task/seed aggregates and explicit failure taxonomy |
| Paper-suite target | Eight accepted contact-rich tasks |

## Constitution Check

| Principle | Plan response |
|---|---|
| Evidence before claims | Every Gate requires fresh, checksummed evidence tied to the producing commit. |
| Reproducibility | G0 and release locks remain mandatory. |
| Stable contracts | Public action/observation/task/dataset interfaces are versioned and tested. |
| Runtime safety | Hard guards, Contact truth, safe abort/retract, and budgets remain required. |
| Traceable TDD | Every active requirement maps to tasks and acceptance checks. |

No constitution principle requires formal proof of all possible articulated motion. Treating those proofs as optional diagnostics is therefore compliant.

## Architecture

```text
User / baseline policy
        |
        v
make_env(config)
        |
        v
Unified benchmark environment
  ├── reset lifecycle
  ├── 7D action validation
  ├── observation assembly
  ├── task-state success
  └── evidence hooks
        |
        v
Isaac Sim 6 adapter
  ├── SimulationApp/timeline/stage
  ├── FR3 controller
  ├── PressButton mechanism
  ├── Contact/raw Contact
  ├── RGB/depth
  └── optional tactile adapter
        |
        +--> dataset/replay
        +--> evaluation
        +--> optional formal diagnostics
```

The simulator adapter owns lifecycle and simulator-specific imports. Public benchmark code consumes normalized records and never infers unavailable measurements.

## Gate Plan

### Migration checkpoints — complete

P0, G-1A, and G-1B established Isaac Sim 6.0.1/Python 3.12 compatibility and the active runtime baseline. Their evidence remains historical and immutable.

### G0 — Repository integrity

Required:

- clean tracked checkout;
- pinned Python inputs;
- test inventory and intentional future-RED inventory;
- dependency, asset, config, and source hashes;
- evidence schemas and freshness review;
- no deprecated first-party Isaac imports.

Current state: `PASS_BENCHMARK` for repository integrity only.

### G1 — PressButton benchmark runtime

Implement one trustworthy paper-quality environment path:

```text
make_env
→ reset
→ approach
→ press
→ release
→ safe retract
→ close
```

Acceptance:

- 100 complete reset cycles;
- one rendered 500-step bounded rollout;
- 10 consecutive successful episodes;
- task-state success only;
- truthful Contact/raw Contact and false unavailable force/wrench masks;
- zero NaN/Inf, zero sustained penetration beyond the limit, zero post-abort actuation;
- media, logs, manifests, and checksums.

The following are optional diagnostics and do not block G1:

- full-robot continuous-sweep proof;
- exhaustive GJK route certification;
- private PhysX cooked-shape authority;
- backend narrow-phase equivalence.

### G2 — Unified environment contract

Stabilize and test:

- factory/config selection;
- 7D action meaning and limits;
- observation/info schemas;
- reset/step/close behavior;
- seed determinism;
- mock/runtime boundary;
- error and termination semantics.

### G3 — Tactile capability

Validate:

- capability negotiation;
- timestamp/frame/synchronization fields;
- native versus derived versus unavailable measurements;
- tactile image/field shape and dtype;
- no fake vector force or wrench;
- Contact/tactile lifecycle across reset.

### G4 — Task suite, dataset, and replay

Deliver:

- eight accepted task cards;
- versioned assets and initial-state distributions;
- a dataset collection pipeline;
- schema and duplicate validation;
- simulator replay;
- dataset card, license, and provenance.

Recommended paper-v0 collection target: at least 50 accepted demonstrations per task, subject to G4 quality review rather than a raw-count-only pass.

### G5 — Evaluation

Deliver:

- fixed train/evaluation splits;
- task success and failure taxonomy;
- runtime/safety validity metrics;
- tactile/contact validity metrics;
- task/seed aggregates;
- confidence intervals or another declared uncertainty method;
- table/figure generation from machine-readable results.

Recommended paper-v0 protocol: three training seeds and 50 evaluation episodes per task per seed, unless a documented power/variance study justifies another count.

### G6 — Baselines and release

Deliver:

- scripted/oracle reference;
- one visual baseline;
- one visual-tactile baseline;
- matched training/evaluation conditions;
- reproducibility package;
- paper tables/figures/limitations;
- reference/validated-driver rerun for final release claims.

## G1 Execution Design

### Reset/lifecycle

- Stop, rebuild or reset, play, and wait for a bounded readiness window.
- Validate articulation, button state, Contact handles, cameras, and seed.
- Count invalid-after-ready, stale handles, and cleanup failures.
- Run exactly 100 complete cycles for acceptance.

### Bounded rollout

- Use the approved task-ready reset and public 7D action path.
- Render RGB/depth on declared ticks.
- Enforce current hard limits and total budgets.
- Retain the first failing sample before abort.
- Run 500 steps without using optional formal proof as a pass criterion.

### Episodes

- Run one pilot episode to verify evidence plumbing.
- If the pilot has a software/infrastructure failure, fix it with RED→GREEN and rerun the pilot.
- If the pilot has a genuine task/control failure, adjust only through an explicit, documented task/control decision.
- After a clean pilot, run 10 consecutive formal episodes from fresh resets.
- Do not discard failed episodes or cherry-pick a passing suffix.

### Evidence

Required G1 artifacts:

```text
command.log
episodes.jsonl
samples.jsonl
reset_cycles.jsonl
camera_timing.jsonl
media/
report.json
manifest.json
checksums.sha256
review.md
```

The report records simulator/Python/driver/GPU/physics metadata, task/config/asset hashes, thresholds, outcome counts, Contact truth, masks, budgets, and blockers.

## Data and Evaluation Design

### Episode record

Each step binds:

- episode/task/seed/step identifiers;
- action and executed-action records;
- RGB/depth/tactile references;
- proprioception and task state;
- Contact/raw Contact provenance;
- measurement validity masks;
- timestamps and physics/render ticks;
- success/termination/failure codes;
- source/config/asset/dataset digests.

### Replay

Replay executes recorded actions through the simulator. It reports:

- task outcome match;
- button/task-state trajectory difference;
- action/observation timing skew;
- reset-state difference;
- Contact-event alignment;
- first divergence.

### Metrics

Primary:

- task success rate;
- macro-average success across tasks.

Required secondary:

- invalid runtime rate;
- safe-retract rate;
- Contact/tactile valid rate;
- episode length and wall time;
- replay outcome agreement;
- failure taxonomy counts.

## Documentation and Claim Policy

Active documents describe the benchmark acceptance path. Historical G1 formal-safety documents remain in the repository as investigations and must carry no authority over current Gate dependencies.

The paper and README must state:

- simulation-only scope;
- driver-validation status;
- sensor validity limitations;
- task and dataset scale;
- no real-robot safety claim;
- optional nature of formal diagnostics.

## Risks and Mitigations

| Risk | Mitigation |
|---|---|
| G1 becomes an endless research project | Freeze benchmark-oriented acceptance; move formal proofs to optional diagnostics. |
| Geometric fallback inflates success | Require task-state success for benchmark evidence. |
| Contact data is overinterpreted | Preserve raw provenance and masks; never synthesize vector force/wrench. |
| Small task suite weakens paper | Target eight diverse contact-rich tasks after PressButton acceptance. |
| Dataset count hides poor quality | Gate on schema, duplicates, replay, task balance, and provenance. |
| Unvalidated driver weakens release | Allow development; require G6 reference-driver rerun or limit the release claim. |
| API drift during suite expansion | Freeze G2 contracts before G4 data collection. |

## Deliverables

```text
isaac_tactile_libero/
  envs/
  robots/
  tasks/
  sensors/
  datasets/
  metrics/
configs/
requirements/
scripts/
tests/
docs/
specs/001-benchmark-reconstruction/
outputs/evidence/
```

The authoritative execution list is `tasks.md`; the authoritative acceptance interpretation is `acceptance.md`.

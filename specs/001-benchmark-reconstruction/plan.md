# Implementation Plan: TactiLIBERO Generalization Benchmark

**Branch**: `001-benchmark-reconstruction`
**Specification**: `spec.md`
**Runtime**: Isaac Sim 6.0.1, Python 3.12
**Paper-v1 target**: 16 tasks, offline/online data, unified training, 3 core protocols, baseline results, toolkit, and static leaderboard

## Summary

TactiLIBERO is implemented as a full benchmark lifecycle:

```text
trusted runtime
→ task/sensor/plugin contracts
→ four suites and 16 tasks
→ expert and online data collection
→ official offline dataset and replay
→ unified training
→ leakage-safe generalization evaluation
→ matched baselines
→ verifiable reports and leaderboard
```

The first paper release deliberately favors a complete 16-task training/data/evaluation loop over a shallow 100-task catalog.

## Technical Context

| Area | Decision |
|---|---|
| Simulator | Isaac Sim 6.0.1 |
| Python | `>=3.12,<3.13` |
| Robot | FR3 |
| Physics | CPU physics, MBP, GPU dynamics disabled |
| Rendering | RTX |
| Development driver | `550.144.03`, `UNVALIDATED` |
| Task scale | 4 tasks × 4 suites = 16 |
| Core protocols | GP-01 object/geometry, GP-02 contact/material/physics, GP-03 sensor/observation |
| Official data minimum | 50 accepted training demonstrations per task plus validation data |
| Collection modes | scripted, controller, teleoperation, trained policy, custom expert |
| Training algorithms | BC, ACT, Diffusion Policy, Transformer, UniVTAC-compatible |
| Modalities | vision-only, tactile-only where meaningful, visual-tactile, proprioception |
| Policy seeds | 3 |
| Formal test episodes | At least 20 per task condition per seed |
| Outputs | JSON, CSV, radar data, HTML, checksums, result bundle |

## Constitution Check

| Principle | Compliance |
|---|---|
| Evidence Before Claims | Task, data, training, protocol, episode, aggregate, and leaderboard artifacts are versioned and checksummed. |
| Reproducible Repository First | Generators, manifests, training configs, and evaluators are tracked; external assets/checkpoints require provenance. |
| Stable Contracts and Versioned Change | Environment, task, sensor, dataset, training, metric, and submission contracts are versioned. |
| Safety-Gated Runtime Control | G1 passes before formal suite collection; all later runtime retains safety and invalid-episode handling. |
| Traceable Test-First Delivery | Every feature maps to RED tests, implementation tasks, commands, and artifacts. |

No formal task collection starts before G1–G3. Offline schema, adapter, and unit-test work may proceed without creating benchmark claims.

## System Architecture

```text
Robot / Sensor / Task / Expert / Modality Registries
                         |
                         v
              Environment + TaskInstance
                 /                  \
                v                    v
       Online interaction       CollectionJob
                |                    |
                |                    v
                |           DemonstrationEpisode
                |                    |
                └──────────┬─────────┘
                           v
                 Dataset + Replay + Splits
                           |
                           v
                  Unified TrainingRun
                           |
                           v
                 PolicyCapability/Checkpoint
                           |
                           v
              Protocol + LeakageAudit + Evaluator
                           |
                           v
                EpisodeResults + Aggregates
                           |
                           v
                 HTML/Radar/Leaderboard
```

## Task Layer

### Four suites

1. **Precision**
   - Peg insertion
   - USB-like connector insertion
   - Key insertion/turn
   - Pin/socket alignment

2. **Articulation**
   - Button press/release
   - Switch actuation
   - Drawer open/close
   - Cap/knob twist

3. **Surface Interaction**
   - Sliding
   - Wiping
   - Scraping
   - Surface following

4. **Deformable Contact**
   - Soft pressing
   - Sponge compression
   - Fabric pull/place
   - Cable/soft-part seating

### Task contract

Each task binds:

- scene/object/robot/sensor definitions;
- reset distribution and randomization;
- task-state success and failure;
- dense reward or phase labels;
- action/time/safety budgets;
- train/validation/test variant rules;
- scripted/oracle feasibility;
- assets and licenses.

Task families share implementation. Canonical cards are individually reviewed and accepted.

## Data-Collection Layer

### Expert adapters

One interface supports:

- scripted oracle;
- traditional controller;
- teleoperation;
- trained policy rollout;
- human demonstration;
- community custom expert.

The adapter returns public actions and provenance; it cannot bypass the environment contract.

### Batch collector

Canonical command:

```bash
python scripts/collect_data.py \
  --suite precision \
  --task peg_insert \
  --num-episodes 1000 \
  --policy scripted \
  --num-envs 8 \
  --resume \
  --output datasets/runs/<run>
```

Required behavior:

- deterministic job/schedule identity;
- multi-environment collection;
- bounded retries;
- configurable success/failure retention;
- crash-safe progress journal;
- unique episode IDs;
- statistics and failure taxonomy;
- validation before promotion;
- no silent replacement of failed episodes.

### Episode content

- RGB/depth and tactile observations;
- joint state and end-effector pose;
- requested/executed action;
- Contact and valid force fields;
- reward/task phase;
- task state and success/failure;
- timestamps;
- every randomization and split parameter;
- runtime/source/config/asset/sensor identities.

### Community extensions

Robot, tactile sensor, task, expert, and modality plugins provide:

- manifest and semantic version;
- declared contract versions;
- factory entry point;
- capability schema;
- license/provenance;
- unit/contract tests.

Plugins cannot enter official data/evaluation until validation passes.

## Offline and Online Benchmark Modes

### Offline

Researchers use the official dataset and fixed splits. This is the primary path for reproducible baseline comparison.

### Online

Researchers interact with the same registered environments for reinforcement learning, active touch, adaptation, or data-efficiency studies.

Online results record:

- environment steps and compute;
- initial data/checkpoint;
- exploration privileges;
- generated data;
- policy updates;
- final evaluation protocol.

Offline and online results remain separate leaderboard tracks.

## Unified Training Layer

Canonical command:

```bash
python scripts/train.py \
  --algo diffusion_policy \
  --suite precision \
  --tasks peg_insert usb_insert key_turn pin_socket \
  --modalities vision tactile proprio \
  --dataset configs/datasets/tactilibero_v1.yaml \
  --seed 1701 \
  --output outputs/training/<run>
```

Shared services:

- dataset and split resolution;
- modality selection and masks;
- normalization;
- observation/action horizon;
- batching and sampling;
- seed control;
- optimizer/schedule configuration;
- checkpoint/log writing;
- validation-only model selection;
- training budget and compute accounting;
- resume.

Adapters implement only algorithm-specific model/loss/update behavior.

Required paper-v1 algorithms:

- Behavior Cloning;
- ACT;
- Diffusion Policy;
- Transformer policy;
- UniVTAC-compatible policy.

Every algorithm must support declared modality capabilities and fail explicitly when unsupported.

## Core Generalization Protocols

### GP-01 Object and Geometry

Hold out object identities or geometry families. Audit mesh/content/family overlap.

### GP-02 Contact, Material, and Physics

Hold out combinations of friction, stiffness, compliance, clearance, contact pattern, and materials inside safety bounds.

### GP-03 Sensor and Observation

Hold out sensor family/domain, calibration, noise, latency, drift, dropped-frame, occlusion, or scene-observation conditions.

Every protocol definition contains:

- scientific hypothesis;
- train/validation/test-seen/test-unseen queries;
- forbidden overlap identities;
- adaptation regime;
- episodes/seeds/budgets;
- required metrics;
- manifest and audit hashes.

## Leakage Prevention

The auditor checks normalized identities:

- asset content and geometry family;
- object/category/instance;
- material and physics parameter cell;
- trajectory generator and seed;
- sensor family/calibration/preprocessing;
- scene/camera/light/layout;
- task family and variant;
- randomization seed;
- privileged replay metadata;
- language template if relevant.

Evaluation cannot start until the selected manifest passes.

## Metrics

### Always available

- Task Success
- Completion Time
- Action/Trajectory Smoothness
- Trajectory Efficiency
- Runtime Valid Rate
- Safe Retract Rate
- Contact Count and Duration when Contact is valid

### Source-dependent

- Maximum and cumulative contact force
- Force Smoothness
- Force-limit rate
- Slip count/duration
- Contact Stability
- Recovery Success/Time

Source-dependent metrics are unavailable—not zero—when required measurements are invalid.

### Generalization

```text
Generalization Gap = Seen Success − Unseen Success
```

Also report relative gap where defined, worst-group performance, perturbation degradation, and modality-drop degradation.

## Evaluation Toolkit

Canonical command:

```bash
python scripts/evaluate.py \
  --checkpoint outputs/training/<run>/best.ckpt \
  --benchmark configs/benchmark/tactilibero_v1.yaml \
  --protocol GP-01 \
  --seeds 1701 1702 1703 \
  --output outputs/evaluation/<run>
```

Outputs:

```text
command.log
schedule.json
episodes.jsonl
failures.jsonl
summary.json
summary.csv
radar.json
report.html
manifest.json
checksums.sha256
submission/
```

Aggregates consume validated episode records only.

## Baseline and Fairness Plan

Paper-v1 learned baselines:

- BC;
- ACT;
- Diffusion Policy;
- Transformer;
- UniVTAC-compatible.

Required reference:

- scripted/oracle feasibility and upper-bound diagnostic.

Modality comparisons:

- vision + proprioception;
- tactile + proprioception where meaningful;
- vision + tactile + proprioception.

Matched comparisons freeze:

- dataset/split;
- samples and sampling;
- action contract;
- horizons;
- optimizer/schedule where applicable;
- training steps/updates;
- validation selection;
- evaluation cells/seeds/budgets.

Parameter/compute differences are reported.

## Leaderboard and Extension Plan

Paper-v1:

- offline/online track label;
- result-bundle validation;
- aggregate regeneration;
- duplicate/stale/tamper rejection;
- static HTML/CSV leaderboard;
- radar views by protocol/metric/modality.

Future:

- OpenVLA and π0 adapters;
- additional protocols and tasks;
- hosted checkpoint execution in an isolated service.

## Gate Plan

### G0 — Repository integrity

Refresh after this rebaseline.

### G1 — PressButton reference runtime

- 100 resets;
- rendered 500-step rollout;
- 10 consecutive task-state episodes;
- runtime/Contact/camera/media evidence.

### G2 — Contracts and registries

Freeze environment, task, sensor, plugin, expert, collection, dataset, training, protocol, metric, policy, result, and submission contracts.

### G3 — Sensor and collection foundation

Pass sensor interoperability, expert adapters, collector durability, plugin validation, and tiny end-to-end offline/online data smoke.

### G4 — 16 tasks and official data

Pass four suites, task feasibility, variants/splits, official dataset minimums, validation, and replay.

### G5 — Unified training and generalization evaluation

Pass five training algorithms, three core protocols, metrics, one-command evaluation, and aggregate/report regeneration.

### G6 — Baseline results and release

Pass matched baseline matrix, offline/online track reporting, static leaderboard, paper artifacts, related-work claim audit, and reference-driver revalidation.

## Phased Delivery

### MVP-A

G1 PressButton and G2 contracts.

### MVP-B

Four Precision tasks, scripted/BC/ACT, batch collection, official mini dataset, and GP-01.

### Paper Beta

All 16 tasks, official dataset, five algorithms, GP-01 through GP-03, reports, and static leaderboard.

### Paper Release

Three-seed formal results, ablations, release package, reference-driver rerun, and claim review.

## Risks

| Risk | Mitigation |
|---|---|
| Deformable tasks consume too much engineering | Require four accepted tasks but allow asset/solver feasibility review before formal collection. |
| Success-only filtering biases demonstrations | Retain rejection statistics and optional failed trajectories; publish collection policy. |
| Parallel collection corrupts IDs/progress | Transactional episode promotion and crash-safe journal. |
| Baselines silently use different preprocessing | Shared dataloader/normalizer/horizon and capability manifests. |
| Generalization splits leak identities | Content/family/parameter/calibration-aware leakage audit. |
| Cross-sensor comparison mixes adaptation modes | Separate zero-shot, calibration-only, and adaptation results. |
| Force metrics are unavailable | Mask and omit metrics; never substitute geometry or impulse. |
| Online results are incomparable | Separate online track with environment-step/data/compute budgets. |
| G1 remains incomplete | Keep it blocking; later work may only prepare offline contracts/tests. |

## Deliverables

```text
configs/benchmark/
configs/suites/
configs/tasks/cards/
configs/protocols/
configs/sensors/
configs/experts/
configs/datasets/
configs/training/
configs/policies/
isaac_tactile_libero/registry/
isaac_tactile_libero/collection/
isaac_tactile_libero/training/
isaac_tactile_libero/protocols/
isaac_tactile_libero/evaluation/
isaac_tactile_libero/leaderboard/
scripts/collect_data.py
scripts/train.py
scripts/evaluate.py
scripts/build_leaderboard.py
datasets/
outputs/training/
outputs/evaluation/
release/
```

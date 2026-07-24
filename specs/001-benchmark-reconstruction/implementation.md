# Implementation Guide

## Required order

```text
G0 refresh
→ G1 PressButton
→ G2 contracts/registries
→ G3 sensors/collection foundation
→ G4 16 tasks + official data + replay
→ G5 unified training + generalization evaluation
→ G6 baseline results + leaderboard + release
```

## Development method

1. Freeze behavior with RED tests.
2. Implement the smallest contract-complete path.
3. Run no-simulator verification.
4. Run bounded simulator smoke where required.
5. Produce fresh evidence before changing status.
6. Preserve failed attempts and all invalid/rejected counts.

## G1

Use the benchmark-oriented PressButton runner:

- task-state-only success;
- 100 resets;
- rendered 500-step rollout;
- 10 consecutive formal episodes;
- hard guards, Contact truth, media, and checksums.

Optional full-sweep/geometry diagnostics cannot block or pass G1.

## Registries

Implement typed/versioned registration for:

- robot;
- task family and task instance;
- tactile sensor;
- expert;
- observation modality;
- training algorithm;
- policy adapter.

Registration validates capability and contract versions before factory use.

## Task implementation

Each task family exposes:

- stage/assets/robot/sensors;
- reset and randomization;
- task state;
- success/failure;
- reward or phase labels;
- budgets;
- variant generation;
- scripted/oracle expert.

Canonical cards are generated, reviewed, and promoted; generation alone is not acceptance.

## Collection implementation

`scripts/collect_data.py` orchestrates:

- schedule generation;
- parallel environments;
- expert adapters;
- unique episode IDs;
- retries;
- retention/filter policy;
- crash-safe journal;
- episode validation;
- atomic promotion;
- statistics.

The collector must retain randomization and split parameters at every episode.

## Dataset implementation

One schema supports visual, tactile, proprioceptive, action, task-state, reward/phase, timestamp, randomization, mask, and provenance data.

Validation rejects:

- duplicate IDs/content;
- incomplete arrays;
- non-finite values;
- invalid masks;
- missing timestamps;
- action/observation mismatch;
- split leakage;
- stale hashes.

Replay executes through the simulator and records first divergence.

## Training implementation

`scripts/train.py` owns shared:

- dataset/split loading;
- modalities and masks;
- normalization;
- horizons;
- seeds;
- training budget;
- checkpoint/log lifecycle;
- validation selection;
- resume and provenance.

Algorithm adapters own model, loss, optimizer-specific logic, and inference only.

Paper-v1 adapters:

- BC;
- ACT;
- Diffusion Policy;
- Transformer;
- UniVTAC-compatible.

## Evaluation implementation

`scripts/evaluate.py`:

1. validates checkpoint/policy capability;
2. loads protocol and split manifest;
3. verifies leakage audit;
4. builds the complete fixed schedule;
5. runs every episode without replacement;
6. validates episode results;
7. aggregates seen/unseen metrics;
8. builds JSON/CSV/radar/HTML;
9. writes a submission bundle and checksums.

## Metric implementation

Every metric declares source fields, formula/version, units, validity predicate, aggregation, and unavailable behavior.

Force, slip, and recovery metrics never consume invalid proxy values.

## Offline/online separation

Training and results record `data_regime=OFFLINE|ONLINE`.

Online runs additionally record environment steps, generated episodes, initial checkpoint/data, update budget, and exploration privileges.

## Leaderboard

Leaderboard ingestion validates result bundles and recomputes aggregates. It never trusts submitted summary values alone.

Static HTML/CSV is required; hosted execution is optional.

## Change control

Require a written versioned decision before changing:

- task success/failure;
- randomization or split definitions;
- sensor/calibration domains;
- dataset fields;
- normalization/horizons;
- training/evaluation budgets;
- metric formulas;
- modality privileges;
- result/leaderboard schemas.

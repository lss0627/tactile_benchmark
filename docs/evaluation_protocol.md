# Evaluation Protocol

## Default design

```text
tasks: 8
training seeds: 3
evaluation episodes per task per seed: 50
```

Another count requires a documented variance/power and compute review before evaluation begins.

## Episode classification

Each episode is classified separately as:

- runtime valid/invalid;
- task success/failure;
- safe retract success/failure;
- Contact/tactile valid/invalid;
- terminated/truncated.

Runtime-invalid episodes are never silently converted into task success.

## Primary metrics

- per-task success rate;
- macro-average task success;
- seed mean and declared uncertainty interval.

## Secondary metrics

- runtime-valid rate;
- safe-retract rate;
- Contact valid rate;
- tactile valid rate;
- post-contact failure rate;
- episode steps and wall time;
- replay outcome agreement;
- failure-code distribution.

Force/wrench quality metrics are reported only when their masks are valid and their measurement source is declared.

## Fairness

All baselines use:

- identical task/data splits;
- identical evaluation seeds and reset distributions;
- identical action space and budgets;
- identical policy observation rules except the declared modality ablation;
- matched model/training budget or an explicit parameter/compute normalization.

## Aggregation

- retain all per-episode records;
- aggregate by task, seed, and suite;
- macro average weights tasks equally;
- report missing/invalid counts;
- do not drop failed seeds or episodes;
- generate tables and figures from machine-readable results.

## Robustness

Robustness splits may include pose, occlusion, friction, clearance, or geometry variation only after each split is versioned and checked for train leakage.

## Evidence

G5 evidence includes:

- protocol/config hashes;
- per-episode results;
- aggregates;
- uncertainty method;
- failure taxonomy;
- table/figure generation logs;
- checksums and review.

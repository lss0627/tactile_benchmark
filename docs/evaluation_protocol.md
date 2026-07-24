# Evaluation Protocol

## Command

```bash
python scripts/evaluate.py \
  --checkpoint outputs/checkpoints/act-precision-seed0.ckpt \
  --protocol object_geometry \
  --split test_unseen \
  --output outputs/evaluation/act-precision-seed0-gp01
```

The evaluator resolves the checkpoint capability declaration, suite/task
cards, split manifest, sensor configuration, seed schedule, and budgets before
constructing environments.

## Paper-v1 minimum

```text
policy seeds: 3
episodes per task condition per seed: >= 20
protocols: GP-01, GP-02, GP-03
```

The exact cell grid is versioned before baseline runs. Missing or invalid cells
remain visible and cannot be replaced after observing results.

## Episode classification

Each episode separately records runtime validity, task outcome, termination,
safe retract, Contact/tactile validity, protocol cell, and failure taxonomy.
Runtime-invalid episodes are not silently counted as success or dropped.

## Primary metrics

- seen and unseen success rate;
- macro success by task, suite, and protocol;
- generalization gap: `seen_success - unseen_success`;
- three-seed mean and declared confidence interval.

## Tactile-specific secondary metrics

- completion time;
- maximum and cumulative contact force when valid;
- action smoothness;
- contact and slip counts;
- recovery success rate;
- post-contact failure rate;
- performance degradation under tactile missingness;
- runtime-valid and safe-retract rates.

Force/wrench metrics are absent—not zero—when the source or mask is invalid.

## Fairness

All baselines share task/split manifests, official training data, evaluation
reset seeds, action and timing budgets, policy observation rules except the
declared modality ablation, checkpoint selection, and either matched compute
or an explicit normalization.

Online methods additionally report environment interactions, accepted
episodes, wall time, and update count.

## Outputs

Every run emits:

- per-episode JSONL;
- aggregate JSON;
- tabular CSV;
- failure and leakage audit;
- radar-plot data and images;
- HTML report;
- checksums and result-bundle manifest.

The paper and static leaderboard consume only validated result bundles.

# Baseline Protocol

## Required configurations

Reference:

1. scripted/oracle feasibility policy.

Learned:

1. Behavior Cloning;
2. ACT;
3. Diffusion Policy;
4. Transformer policy;
5. UniVTAC-compatible policy.

Each compatible learned algorithm must expose vision-only, tactile-only, and
vision–tactile fusion modality configurations where its architecture permits.
Unsupported combinations are explicit capability failures, not silently
modified models.

## Shared interface

All learned baselines use the same dataset loader, split manifests,
normalization artifacts, observation/action horizons, action schema,
checkpoint/logging contract, seeds, validation selection rule, and evaluator.

Target command:

```bash
python scripts/train.py \
  --algo diffusion_policy \
  --suite precision \
  --protocol object_geometry \
  --modalities vision tactile proprio \
  --seed 0
```

## Matching rules

Modality ablations match:

- accepted episodes and split;
- task cards, variants, and randomization;
- training/evaluation seed schedule;
- optimizer, schedule, and update budget;
- checkpoint selection;
- action space and rollout budget;
- model capacity or declared compute-normalized comparison.

No model receives privileged replay/task state unless that input is an
explicitly reported baseline condition.

## Offline and online reporting

Offline baselines report dataset version, samples/updates, and compute.
Online baselines additionally report environment interactions, collected and
accepted episodes, retries, and wall time. Offline and online scores are not
pooled without a declared regime.

## Required report

For every run:

- source/config/data/checkpoint digests;
- capability and modalities;
- parameters, FLOPs or declared compute proxy;
- seed and exact command;
- selected checkpoint rule;
- seen/unseen per-task and aggregate metrics;
- runtime-invalid and failure counts;
- generalization gaps.

The scripted/oracle result is a feasibility reference, not a learned baseline.

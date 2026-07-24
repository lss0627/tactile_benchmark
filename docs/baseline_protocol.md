# Baseline Protocol

## Required baselines

1. Scripted/oracle reference.
2. Visual policy.
3. Matched visual-tactile policy.

Optional:

- Contact-only;
- tactile dropout;
- data-scale;
- temporal-context;
- architecture ablations.

## Matching rules

Visual and visual-tactile comparisons must match:

- task cards and assets;
- demonstrations and splits;
- action contract;
- evaluation resets/seeds;
- optimizer/schedule;
- training steps;
- selection rule;
- evaluation episodes;
- model capacity or declared compute normalization.

The tactile model cannot receive privileged replay/task state not available to the visual model.

## Scripted/oracle role

The scripted policy establishes task feasibility and reference behavior. It is not a learned baseline and is reported separately.

## Reporting

For each baseline:

- code/config/source digest;
- observations used;
- parameters and compute;
- training seeds;
- selected checkpoint rule;
- per-task and aggregate results;
- runtime-invalid count;
- failure taxonomy.

## No-force rule

Do not label a baseline force-aware unless valid force fields are actually provided. A false-masked vector/wrench field is unavailable, not zero-force input.

# Reproducibility and Release

## Release bundle

- source commit/tag;
- Python/Isaac environment locks;
- simulator/driver/GPU/physics metadata;
- task cards and configs;
- asset/license manifest;
- dataset and dataset card;
- replay report;
- evaluation protocol/results;
- baseline configs/checkpoints as permitted;
- table/figure generation;
- Gate evidence/checksums;
- limitations.

## Reproduction levels

1. No-simulator: schemas, contracts, dataset/evaluation tooling.
2. Runtime: accepted task and sensor smokes.
3. Dataset: validation and replay.
4. Evaluation: regenerate per-episode aggregates/tables.
5. Training: reproduce baseline checkpoints/results.

## Driver rule

Development results on `550.144.03` remain explicitly `UNVALIDATED`. G6 requires rerunning final physical, dataset, replay, and evaluation evidence on a current NVIDIA reference/validated driver.

If that rerun cannot be performed, the development artifact may be shared but G6 remains blocked and the paper/release cannot claim reference-driver validation.

## Immutable evidence

- Evidence directories are never overwritten.
- Failed attempts remain failed.
- Every formal result is bound to source/config/task/asset/data hashes.
- Checksums and freshness are reviewed before claims.

## Paper artifact

The paper artifact must regenerate reported tables and figures from machine-readable episode results and identify any external assets or credentials needed for simulator execution.

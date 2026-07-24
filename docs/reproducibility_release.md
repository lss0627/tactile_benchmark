# Reproducibility and Release

## Release bundle

- tagged source and Python/Isaac locks;
- simulator/driver/GPU/physics metadata;
- four suite manifests and 16 task cards;
- sensor, expert, policy, and protocol registries;
- asset/license manifest;
- collection commands and official dataset;
- dataset card, split manifests, leakage and replay reports;
- training configs/logs/checkpoints as permitted;
- evaluation records and result bundles;
- JSON/CSV/radar/HTML generation;
- static leaderboard;
- Gate evidence, checksums, and limitations.

## Reproduction levels

1. **No simulator**: contracts, dataset loading, training/evaluation fixtures,
   and report generation.
2. **Runtime**: accepted tasks, sensors, and online interaction.
3. **Collection**: reproduce reference collection jobs and validation.
4. **Dataset**: validate official data and simulator replay.
5. **Training**: reproduce selected offline/online baseline runs.
6. **Evaluation**: regenerate protocol aggregates, figures, and leaderboard.

## One-command workflows

Documented commands must cover:

```text
collect_data.py
validate_dataset.py
replay_dataset.py
train.py
evaluate.py
build_leaderboard.py
```

Each output is immutable and bound to source/config/task/split/data/checkpoint
hashes.

## Driver rule

Development results on `550.144.03` remain `UNVALIDATED`. G6 requires final
physical, dataset/replay, and evaluation evidence on a current NVIDIA
reference/validated driver. If unavailable, the development artifact may be
shared but the corresponding release claim remains blocked.

## Paper artifact

The paper artifact regenerates tables, plots, HTML reports, and the static
leaderboard solely from validated machine-readable result bundles. Offline
data remains usable without Isaac Sim; online reproduction states all external
assets, credentials, and hardware requirements.

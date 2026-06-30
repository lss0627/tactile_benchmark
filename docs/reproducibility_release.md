# reproducibility_release.md — 复现、开源、License 与 Artifact Review 计划

## 1. Release philosophy

Benchmark 论文的价值取决于别人能不能：

```text
install
run tasks
load data
replay demos
train baselines
evaluate policies
reproduce tables
inspect failures
extend benchmark
```

因此 release 不是附属工作，而是论文贡献的一部分。

## 2. Repository layout

```text
isaac-tactile-libero/
  README.md
  LICENSE
  CITATION.cff
  CHANGELOG.md
  pyproject.toml
  environment.yml
  Dockerfile

  isaac_tactile_libero/
  configs/
  scripts/
  docs/
  tests/
  examples/

  task.md
  checklist.md
```

## 3. Installation

README 必须包含：

```bash
git clone <repo>
cd isaac-tactile-libero
conda env create -f environment.yml
conda activate isaac-tactile-libero
pip install -e .
python scripts/smoke_test.py --quick
```

必须写清：

```text
OS
Python version
CUDA version
NVIDIA driver
Isaac Sim version
Isaac Lab version
GPU memory recommendation
```

## 4. Docker / container

建议提供：

```text
Dockerfile
docker-compose.yaml optional
```

Dockerfile 目标：

- 固定系统依赖；
- 固定 Python 依赖；
- 不把大型数据和 proprietary assets 打进镜像；
- 支持 smoke test。

## 5. Environment lock

至少提供一种：

```text
environment.yml
requirements.txt with pinned versions
uv.lock
poetry.lock
```

并写明：

```text
tested commit
tested date
tested GPU
tested Isaac version
```

## 6. Continuous Integration

CI 不一定跑完整 Isaac Sim，但至少跑：

```text
lint
unit tests
schema tests
registry tests
dataset reader/writer tests
metric tests
small no-sim tests
```

如果可以跑 headless sim，则加：

```text
1 task × 1 tactile mode × 5 steps smoke test
```

## 7. Tests

建议测试结构：

```text
tests/
  test_registry.py
  test_schema.py
  test_action.py
  test_tactile_schema.py
  test_dataset_writer_reader.py
  test_metrics.py
  test_task_cards.py
  test_eval_result_schema.py
```

## 8. Reproduction scripts

必须提供：

```text
scripts/list_tasks.py
scripts/smoke_test.py
scripts/collect_demos.py
scripts/validate_dataset.py
scripts/replay_demos.py
scripts/train_bc.py
scripts/evaluate.py
scripts/report_results.py
scripts/export_dataset.py
```

## 9. Reproducing paper results

README 或 docs 必须包含：

```bash
# 1. Download dataset
python scripts/download_dataset.py --version 0.1.0

# 2. Validate dataset
python scripts/validate_dataset.py --dataset datasets/isaac_tactile_libero_v0

# 3. Train baseline
python scripts/train_bc.py --policy vision_bc --config configs/policies/vision_bc.yaml

# 4. Evaluate baseline
python scripts/evaluate.py --policy vision_bc --split test_seen

# 5. Generate tables
python scripts/report_results.py --results results/ --paper-tables
```

## 10. Data release

需要：

- dataset card；
- split manifests；
- checksums；
- sample episodes；
- sample videos；
- validation report；
- replay report；
- download script；
- license。

## 11. Model release

每个 baseline checkpoint 需要 model card：

```markdown
# Model Card: VisionForceBC

## Input modalities
## Training data
## Architecture
## Hyperparameters
## Evaluation results
## Intended use
## Limitations
## Checkpoint hash
```

## 12. Asset provenance

必须维护：

```text
docs/asset_licenses.csv
```

列：

```text
asset_name
source
license
modified
used_in_tasks
redistribution_allowed
notes
```

不要使用不能重新发布的资产作为 benchmark 必需资产。

## 13. License

建议：

- code：Apache-2.0 或 MIT；
- dataset：CC BY 4.0 或 CC BY-NC 4.0，按需求选择；
- assets：遵循原始许可；
- pretrained models：明确许可；
- third-party dependencies：单独列出。

## 14. Citation

`CITATION.cff` 包含：

```yaml
title:
authors:
version:
doi:
date-released:
url:
```

论文中也应给 BibTeX。

## 15. Project page

Project page 至少包含：

- one-sentence summary；
- teaser video；
- task suite gallery；
- dataset statistics；
- benchmark leaderboard；
- install quickstart；
- download links；
- paper；
- citation；
- contact；
- license。

## 16. Leaderboard rules

如果做 leaderboard，必须写清：

```text
allowed training data
allowed pretrained models
whether external robot data allowed
whether privileged state allowed
submission format
number of submissions
hidden test usage
ranking metrics
```

建议排行榜分：

```text
closed-loop standard
external data allowed
oracle/privileged
```

## 17. Artifact review package

准备一个 `artifact_review/`：

```text
artifact_review/
  README.md
  quickstart.sh
  expected_outputs/
  small_dataset_sample/
  precomputed_results/
  docker_instructions.md
```

目标是让审稿人 30–60 分钟内能跑：

```text
install
load small dataset
replay one demo
run one evaluation
regenerate one mini table
```

## 18. Known issues

必须维护：

```text
KNOWN_ISSUES.md
```

包括：

- simulator version quirks；
- GPU memory requirements；
- deterministic limitations；
- task-specific instability；
- dataset conversion limitations。

## 19. Release checklist

- [ ] release tag。
- [ ] changelog。
- [ ] code license。
- [ ] dataset license。
- [ ] asset license table。
- [ ] dataset card。
- [ ] model cards。
- [ ] Dockerfile。
- [ ] environment lock。
- [ ] smoke tests。
- [ ] validation report。
- [ ] replay report。
- [ ] paper result scripts。
- [ ] project page。
- [ ] citation。
- [ ] archive/DOI。

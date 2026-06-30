# risk_register.md — 风险清单、缓解策略与 Go/No-Go 标准

## 1. 风险评级

```text
Severity:
  H = high, 会影响投稿主线
  M = medium, 会影响实验完整性
  L = low, 可作为 limitation

Likelihood:
  H = likely
  M = possible
  L = unlikely
```

## 2. 风险总表

| ID | 风险 | Severity | Likelihood | 症状 | 缓解策略 | Go/No-Go |
|---|---|---:|---:|---|---|---|
| R1 | 任务太多但定义不严 | H | H | success condition 不一致，结果不可比 | 先做 task card，未完成 task card 不进 v0 | 没有 30 个合格 task 就降到 20 个 |
| R2 | 触觉模拟被质疑不真实 | H | M | 审稿人认为 VT 是假信号 | 明确是 modular proxy，写 noise/latency/saturation，做 force-only 强结果 | 不 claim real tactile equivalence |
| R3 | tactile 提升来自 privileged leakage | H | M | tactile 或 state 暗含目标 pose/success | dataloader 分离 policy obs 和 replay metadata；oracle 单独报告 | leakage 未排除不能投稿 |
| R4 | baseline 不公平 | H | H | tactile 模型参数更多或训练更多 | 固定训练预算，报告参数量，加 VisionBC-wide | 不满足公平规则则不能用作主结论 |
| R5 | dataset 不能 replay | H | M | replay success 低，demo 不稳定 | 先修控制和记录；设 replay threshold | replay 不达标则停止采更多数据 |
| R6 | 接触物理不稳定 | H | M | NaN、穿模、爆力 | 限制 task 参数，调 physics，加入 smoke tests | 不稳定 task 不进 v0 |
| R7 | 评测方差太大 | M | H | seed 间差异大，结论不显著 | 增加 episodes/seeds，报告 CI，做 per-task analysis | 主 claim 只基于稳定 suite |
| R8 | Vision-only 已经很强 | M | M | no-noise 下差异小 | 强调 robustness/contact quality；增加 occlusion/pose noise/low clearance | 不强行 claim no-noise 大幅提升 |
| R9 | 数据规模不足 | M | M | BC 训练不收敛 | 数据 scaling；先发布 30×50，强版扩到 100/task | 不为了规模牺牲可 replay |
| R10 | 工程范围膨胀 | H | H | 多机器人/VLA/真实机器人拖慢 | v0 明确排除，多机器人/VLA 放 P2/P3 | v0 未完成前不扩 scope |
| R11 | Lightwheel 集成耗时 | M | M | 被 backend API 卡住 | Lightwheel 作为 optional backend；核心独立可运行 | 如果集成卡住，不阻塞主 benchmark |
| R12 | 资产 license 不清 | H | M | 不能公开数据/任务 | 建 asset license table，优先用可再发布资产 | license 不清的资产不进 release |
| R13 | 任务语言模板泄漏 | M | M | policy 记住模板而非理解任务 | train/test templates 分离；paraphrase split | 主结果用 held-out templates |
| R14 | Contact metrics 定义主观 | M | M | jamming/contact stability 难复现 | 给公式和代码；用 synthetic tests | 没有公式的 metric 不进主表 |
| R15 | 论文贡献显得像工程堆叠 | H | M | 审稿人看不到科学问题 | 围绕“触觉在接触不确定性下的价值”组织实验 | 主图主表必须服务 claim |

## 3. High-risk mitigation details

### R1: 任务太多但定义不严

缓解顺序：

1. 先写 task card；
2. 再实现任务；
3. 再写 metrics；
4. 最后采 demos。

不要反过来。

### R2: 触觉模拟真实性

写法：

```text
We do not claim to perfectly simulate a specific commercial tactile sensor. Instead, we provide a modular force-and-visuo-tactile interface with configurable noise, latency, saturation, and calibration metadata, enabling controlled modality ablations.
```

### R3: Privileged leakage

必须检查：

- observation 不包含 object pose；
- tactile 不包含 success；
- metrics 不进 policy input；
- replay metadata 不进 dataloader；
- oracle baseline 单独分组。

### R4: Baseline fairness

必须报告：

```text
training steps
batch size
optimizer
learning rate
parameter count
input modalities
model selection rule
```

并提供 parameter-matched control 或在 limitation 中说明。

### R5: Dataset replay

如果 replay 失败：

1. 检查 action frequency；
2. 检查 simulator determinism；
3. 检查 reset state；
4. 检查 object pose record；
5. 检查 controller；
6. 检查 task success threshold。

不要通过人工删失败 episode 来“美化”数据，必须报告过滤规则。

## 4. Go/No-Go gates

### Gate 1: 最小 tactile loop

继续扩到 30 tasks 前，必须满足：

- 5 tasks；
- 4 tactile modes；
- smoke test；
- metrics；
- evaluate.py；
- 每个 task 有 task card。

如果不满足，禁止采集大规模 demos。

### Gate 2: Dataset v0

训练 baseline 前，必须满足：

- 1500 demos 或合理降级版；
- validation report；
- replay report；
- fixed splits；
- dataset card；
- no schema errors。

如果不满足，不训练主 baseline。

### Gate 3: Main results

写论文前，必须满足：

- main baseline results；
- robustness results；
- contact quality results；
- confidence intervals；
- failure examples；
- result scripts。

如果触觉没有显著提升，不应强行 claim。可以转为“benchmark reveals when tactile matters”并分析条件。

### Gate 4: Release

公开前，必须满足：

- license 清楚；
- asset provenance；
- install instructions；
- smoke test；
- data checksum；
- known issues；
- citation。

## 5. Scope control

### v0 必须做

```text
FR3-Tactile
30 tasks, or 20 well-defined tasks if quality requires
1500 demos, or fewer but fully validated demos
4 tactile modes
contact metrics
baseline comparison
robustness splits
release scripts
```

### v0 不做

```text
multi-robot support
full real-robot benchmark
full LIBERO 130-task migration
large VLA finetuning as main result
many tactile sensor hardware replicas
```

### v1 再做

```text
LeRobot export
RLDS export
ACT / Diffusion Policy
unseen geometry expansion
real-robot sanity check
leaderboard server
```

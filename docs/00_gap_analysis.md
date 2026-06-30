# 00. 投稿级缺口诊断：原计划还需要补什么

## 1. 总体判断

原计划已经具备 benchmark 论文的骨架：定位明确，贡献点完整，任务套件、触觉插件、数据格式、评测指标和 baseline 都已经覆盖。

但如果目标是投稿到 robotics / embodied AI / datasets & benchmarks 类 venue，仅有“做什么”还不够。还需要补充“如何证明它是可靠 benchmark”的协议化内容：

- 明确 benchmark contract；
- 明确 task card；
- 明确数据质量验证；
- 明确评测统计与公平性；
- 明确 baseline 训练预算；
- 明确传感器 realism 与 limitations；
- 明确 release、license、CI 和 artifact review 标准。

## 2. 原计划已有优势

### 2.1 贡献边界清楚

原计划强调不是简单迁移 LIBERO，也不是 fork Lightwheel 加任务，而是围绕 tactile sensing、contact-rich task design、dataset format 和 evaluation protocol 建 benchmark。这一点非常重要，建议保留为论文 abstract 和 introduction 的主线。

### 2.2 技术模块基本完整

已经覆盖：

- FR3-Tactile embodiment；
- tactile sensor plugin；
- observation / action schema；
- contact-rich task suites；
- HDF5 dataset；
- contact-aware metrics；
- Vision-only / Force / VT / Force+VT baselines；
- Lightwheel optional backend。

### 2.3 最小可投稿版本边界合理

30 tasks、1500 trajectories、force/wrench、visuo-tactile、contact metrics、robustness split 和 baseline 对比，作为第一版 benchmark paper 是合理的。不要一开始承诺 130 LIBERO tasks、多机器人或真实机器人大规模数据。

## 3. 关键缺口总览

| 缺口 | 当前状态 | 为什么重要 | 应补文件 |
|---|---|---|---|
| Related work positioning | 有 Lightwheel/LIBERO 关系，但缺少系统对照表 | 审稿人会问“和 LIBERO、CALVIN、ManiSkill、RoboCasa、tactile benchmark 相比新在哪里” | `paper_plan.md` |
| Benchmark contract | 有模块结构，但缺少严格版本和接口约束 | 防止后续任务、数据、baseline 不兼容 | `benchmark_spec.md` |
| Task card | 有任务名和指标，但缺少每个任务的 reset、success、failure、randomization | benchmark 最常被质疑的是任务定义不清 | `task_cards.md` |
| Dataset validation | 有 episode 字段，但缺少同步、单位、坐标系、hash、质量门槛 | 数据能否训练和 replay 取决于这些细节 | `dataset_protocol.md` |
| Evaluation statistics | 有指标和 robustness split，但缺少 seed、episode 数、置信区间、聚合规则 | 没有统计协议，结果表不可信 | `evaluation_protocol.md` |
| Baseline fairness | 有 baseline 名称，但缺少训练预算、模型选择、超参、融合方式 | 审稿人会怀疑 tactile 提升来自更大模型或更多训练 | `baseline_protocol.md` |
| Tactile realism | 有 sensor modes，但缺少噪声、延迟、标定、饱和、传感器物理假设 | 触觉 benchmark 最容易被问模拟是否真实 | `benchmark_spec.md`, `dataset_protocol.md` |
| Release / reproducibility | 有 README、scripts，但缺少 Docker、CI、artifact、license、asset provenance | benchmark 论文通常重视 artifact 可用性 | `reproducibility_release.md` |
| Risk control | 有开发阶段，但缺少 go/no-go 标准 | 避免陷入过多工程和不可投稿状态 | `risk_register.md` |
| GitHub 任务化 | 有 phase，但缺少可执行 issue 列表 | 团队协作和每日推进需要更细任务 | `task.md` |
| Submission checklist | 有最终目标，但缺少投稿验收表 | 防止临近投稿才发现缺失 ablation / license / doc | `checklist.md` |

## 4. 必须补充的论文 claim

原计划已经说“触觉有帮助”。投稿时应改成更可验证的三条 claim：

### Claim A: Contact-rich manipulation needs contact feedback under uncertainty

证据要求：

- 在 no noise 下 vision-only 可能也能做；
- 在 pose noise、occlusion、low-clearance、friction randomization 下 tactile policy 下降更慢；
- 报告 effect size，而不只是 success rate。

### Claim B: A modular tactile interface enables controlled modality ablations

证据要求：

- 同一任务、同一 policy family、同一训练预算下切换 `none / force / VT / force+VT`；
- 视觉分支、动作空间、训练轮数保持一致；
- tactile 模态不是通过额外 state 泄漏答案。

### Claim C: Contact-aware metrics reveal failure modes hidden by success rate

证据要求：

- 成功率相同但 max force / jamming / contact loss 不同；
- 给出至少一个 case study；
- 说明安全/柔顺/装配质量不能只看 binary success。

## 5. 最应该新增的实验

### 5.1 Task scaling experiment

问题：任务数从 5 到 30 是否真的构成 benchmark？

建议：

```text
5-task core suite
15-task medium suite
30-task full suite
```

报告：

- 每组任务平均 success；
- suite 内方差；
- task difficulty 分布；
- training data scaling curve。

### 5.2 Sensor ablation experiment

必须控制：

- 相同 demonstrations；
- 相同 train/val/test split；
- 相同视觉 backbone；
- 相同 action head；
- 相同训练步数；
- 只改变 tactile input。

### 5.3 Robustness stress test

至少包括：

```text
pose noise: 0 / 2 / 5 / 10 mm
yaw noise: 0 / 2 / 5 / 10 degrees
visual occlusion: none / mild / heavy
clearance: easy / medium / tight
friction: nominal / randomized
```

### 5.4 Contact-quality experiment

在成功 episode 中继续比较：

- max force；
- force violation rate；
- mean insertion force；
- jamming count；
- contact stability；
- completion time。

这能说明 tactile 不只是提高成功率，也提高接触质量。

### 5.5 Dataset quality audit

随机抽取每个 suite 的 demos：

- replay success rate；
- timestamp monotonicity；
- frame drop rate；
- force saturation rate；
- invalid tactile frame rate；
- action/observation shape consistency；
- seed reproducibility。

## 6. 不建议继续扩大但不补协议的部分

不建议优先做：

- 更多机器人；
- 真实机器人大规模部署；
- 复杂 VLA finetuning；
- 完整迁移 LIBERO 130 tasks；
- 过多 sensor 类型；
- 过多 fancy baseline。

这些会稀释 benchmark 论文主线。第一版最重要的是：任务定义标准、数据可靠、评测公平、触觉收益可信。

## 7. 最小补充路线

第一阶段只需完成以下补充即可显著提高投稿质量：

1. `task_cards.md`：每个任务至少有 task card。
2. `evaluation_protocol.md`：固定 seeds、episodes、统计、aggregation。
3. `baseline_protocol.md`：固定公平 baseline 训练协议。
4. `dataset_protocol.md`：数据同步、质量验证、split 和 dataset card。
5. `paper_plan.md`：claim、figure、table、experiment matrix。
6. `checklist.md`：投稿前逐项验收。

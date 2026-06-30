# paper_plan.md — 论文叙事、实验与图表计划

## 1. 论文核心叙事

### 1.1 推荐标题

```text
Isaac-Tactile-LIBERO:
A Modular Force-and-Visuo-Tactile Benchmark for Language-Conditioned Contact-Rich Robot Manipulation
```

### 1.2 一句话 thesis

现有语言条件 manipulation benchmark 主要评估视觉、语言和多任务泛化；而大量真实操作失败来自接触不确定性、插入卡滞、过力、滑移和接触丢失。Isaac-Tactile-LIBERO 提供一个可复现、可替换传感器、可做 modality ablation 的 tactile/contact-rich benchmark。

### 1.3 最稳妥的投稿 claim

不要 claim：

```text
We solve tactile simulation.
We provide realistic GelSight-level simulation.
We outperform all methods.
We migrate LIBERO to Isaac Sim.
```

应该 claim：

```text
We provide a modular benchmark that isolates the role of force and visuo-tactile feedback in language-conditioned contact-rich manipulation.
```

## 2. Contributions

建议最终写成 5 点：

1. **Benchmark platform**：基于 Isaac / Lightwheel 生态的 contact-rich language-conditioned manipulation benchmark。
2. **Modular tactile interface**：统一 force/wrench、visuo-tactile、force+VT 和 no tactile modes，可做受控 ablation。
3. **Task suites**：30+ contact-rich tasks，覆盖 contact control、assembly、articulated objects、long-horizon manipulation。
4. **Dataset and schema**：可 replay、可验证、可训练的 multimodal demonstration dataset。
5. **Evaluation and baselines**：contact-aware metrics、robustness splits，以及 vision-only / force / VT / force+VT baselines。

## 3. 文章结构

### Abstract

应包含：

- 问题：language-conditioned manipulation 缺少触觉/接触评测；
- 方法：Isaac-Tactile-LIBERO benchmark；
- 内容：tasks、sensors、dataset、metrics、baselines；
- 结果：触觉在 pose noise / occlusion / low-clearance 下更稳；
- release：code、data、configs、evaluation scripts。

### 1. Introduction

建议段落顺序：

1. 语言条件 robot learning 发展很快，但多数 benchmark 仍以视觉可观测任务为主。
2. 接触丰富任务需要检测接触、调节力度、从碰撞/卡滞恢复。
3. 现有 benchmark 不足：触觉 modality 不统一、contact metrics 少、缺少受控 modality ablation。
4. 本文提出 Isaac-Tactile-LIBERO。
5. 总结贡献。

### 2. Related Work

推荐小节：

- Language-conditioned manipulation benchmarks；
- Robot learning simulation environments；
- Tactile sensing for manipulation；
- Multimodal robot learning datasets；
- Contact-aware evaluation and safe manipulation。

需要一张 comparison table。

### 3. Benchmark Overview

内容：

- FR3-Tactile；
- environment / task registry；
- sensor plugin；
- observation/action schema；
- evaluation flow。

主图：系统框架图。

### 4. Task Suites

内容：

- Base-LIBERO-Compatible；
- Tactile-Contact；
- Tactile-Assembly；
- Tactile-Articulated；
- Tactile-Long。

每个 suite 给：

- task examples；
- why tactile matters；
- metrics；
- difficulty variants。

### 5. Tactile Sensor Interface

内容：

- sensor modes；
- schema；
- calibration metadata；
- noise/latency/saturation；
- modality masking；
- limitations。

### 6. Dataset

内容：

- collection method；
- 1500 demos / 30 tasks；
- episode schema；
- data validation；
- replay；
- train/val/test/OOD splits；
- export formats。

### 7. Evaluation Protocol

内容：

- metrics；
- robustness splits；
- seeds；
- aggregation；
- confidence interval；
- hidden test/leaderboard plan。

### 8. Baselines

内容：

- policy classes；
- fairness rules；
- training budget；
- fusion；
- oracle upper bound。

### 9. Experiments

推荐实验顺序：

1. Main comparison；
2. Robustness；
3. Contact quality；
4. Modality ablation；
5. Dataset scaling；
6. Qualitative failure modes。

### 10. Limitations

必须诚实写：

- tactile rendering 不是完整真实传感器；
- simulated contacts depend on physics settings；
- FR3-only；
- dataset size smaller than internet-scale robot datasets；
- no large-scale real robot validation in v0；
- benchmark may not capture all deformable contact physics。

## 4. 主图计划

### Figure 1: Benchmark overview

显示：

```text
language instruction
RGB cameras
FR3-Tactile
tactile plugin
task suites
dataset
evaluation metrics
baselines
```

目的：一眼说明论文贡献。

### Figure 2: Sensor plugin and observation schema

显示：

```text
none
force_wrench
visuotactile
force_plus_visuotactile
```

同一 policy/evaluation pipeline 下可替换。

### Figure 3: Task suite gallery

每个 suite 2–3 张任务图：

- SoftPress；
- PegInsert；
- PlugSocket；
- OpenDrawer；
- PickAlignInsert。

### Figure 4: Robustness curves

x-axis:

```text
pose noise / occlusion / clearance
```

y-axis:

```text
success rate
```

曲线：

```text
Vision
Vision+Force
Vision+VT
Vision+Force+VT
Oracle
```

### Figure 5: Contact metric case study

展示两个 policy 都成功，但：

- Vision-only max force 高；
- tactile policy force smoother；
- jamming count 更少。

## 5. 主表计划

### Table 1: Related benchmark comparison

列：

```text
Benchmark
Language-conditioned
Contact-rich
Force tactile
Visuo-tactile
Dataset
Replay
Contact metrics
Robustness splits
Open-source
```

### Table 2: Task suite summary

列：

```text
Suite
#Tasks
Main contact challenge
Example tasks
Metrics
Robustness variants
```

### Table 3: Dataset statistics

列：

```text
Suite
#Tasks
#Demos
Avg length
Hours
Modalities
Replay success
Invalid episode rate
```

### Table 4: Main baseline results

行：

```text
StateBC
VisionBC
VisionStateBC
VisionForceBC
VisionVTBC
VisionForceVTBC
OracleStateBC
```

列：

```text
Contact
Assembly
Articulated
Long
Overall
```

### Table 5: Robustness results

列：

```text
No noise
2mm
5mm
10mm
Occlusion
Low clearance
Friction randomization
```

### Table 6: Contact quality metrics

列：

```text
Max force
Force violation
Jamming
Contact loss
Insertion depth
Completion time
```

## 6. 实验矩阵

### 6.1 Main comparison

```text
Tasks: 30
Demos: 50/task
Eval episodes: 50/task/split
Seeds: 3
Policies: Vision, Vision+Force, Vision+VT, Vision+Force+VT, Oracle
Splits: test_seen
```

### 6.2 Robustness

```text
Splits:
- pose_noise_2mm
- pose_noise_5mm
- pose_noise_10mm
- occlusion_mild
- occlusion_heavy
- low_clearance
- friction_randomization
```

### 6.3 Ablations

```text
A1: tactile mode ablation
A2: force history length
A3: VT resolution
A4: early vs late fusion
A5: contact metrics with/without force limit
```

### 6.4 Dataset scaling

```text
10 demos/task
25 demos/task
50 demos/task
100 demos/task, if available
```

### 6.5 Task scaling

```text
5-task core
15-task medium
30-task full
```

## 7. Reviewer concerns and preemptive answers

### Concern 1: “Is tactile information just privileged state?”

Answer:

- tactile force/wrench comes from contact readings, not object pose；
- no hidden object pose in tactile schema；
- oracle state is reported separately；
- mask and modality ablation prove controlled comparison。

### Concern 2: “Does tactile help because the model has more parameters?”

Answer:

- parameter counts reported；
- same training budget；
- optional parameter-matched baseline；
- fusion branch ablation。

### Concern 3: “Are tasks too handcrafted?”

Answer:

- task card defines randomization and distributions；
- robustness splits test generalization；
- task suite covers different contact mechanisms；
- hidden seeds used for final evaluation。

### Concern 4: “Can the dataset be replayed?”

Answer:

- replay script；
- replay success report；
- dataset validation report；
- deterministic seed and config snapshot。

### Concern 5: “Is visuo-tactile simulation realistic?”

Answer:

- explicitly treat as modular simulation proxy；
- report noise/latency/saturation settings；
- include limitations；
- optional real-robot sanity check only if feasible。

## 8. Appendix 结构

- Appendix A：Full task cards；
- Appendix B：Observation/action schema；
- Appendix C：Dataset schema；
- Appendix D：Metric definitions；
- Appendix E：Baseline architectures and hyperparameters；
- Appendix F：Additional results；
- Appendix G：Licenses and assets；
- Appendix H：Artifact reproduction instructions。

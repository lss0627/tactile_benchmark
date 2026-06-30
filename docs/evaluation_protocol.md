# evaluation_protocol.md — 评测协议、统计规则与排行榜规范

## 1. Evaluation principles

评测协议必须满足：

```text
same tasks
same splits
same seeds
same action space
same metrics
same aggregation
no test-set model selection
```

任何 baseline 或新方法都必须遵守同一协议。

## 2. CLI

```bash
python scripts/evaluate.py \
  --task-suite tactile_assembly \
  --policy vision_force_bc \
  --checkpoint checkpoints/vision_force_bc.pt \
  --tactile force_wrench \
  --split test_seen \
  --num-episodes-per-task 50 \
  --seeds 0 1 2 \
  --output results/vision_force_bc_test_seen
```

## 3. Evaluation splits

### 3.1 test_seen

同分布 held-out seeds。

用途：

```text
衡量基本任务掌握程度。
```

### 3.2 test_pose_noise

目标物或孔位加入 pose noise。

建议：

```text
translation: 2mm / 5mm / 10mm
yaw: 2deg / 5deg / 10deg
```

用途：

```text
测试视觉定位误差下触觉反馈是否提高鲁棒性。
```

### 3.3 test_occlusion

视觉输入遮挡。

建议：

```text
mild: partial object occlusion
heavy: target area partially occluded during final contact
```

用途：

```text
测试视觉不可见接触阶段的触觉价值。
```

### 3.4 test_low_clearance

装配公差变小。

建议：

```text
easy: clearance >= 4mm
medium: clearance 2mm
tight: clearance <= 1mm
```

用途：

```text
测试插入任务是否需要触觉引导。
```

### 3.5 test_friction_randomization

摩擦系数变化。

用途：

```text
测试接触动力学变化下的泛化。
```

### 3.6 test_unseen_geometry

未见过的 peg/socket/handle geometry。

用途：

```text
测试任务结构泛化。
```

## 4. Episodes and seeds

v0 推荐：

```text
num_eval_seeds = 3
num_episodes_per_task_per_seed = 20
total_per_task = 60
```

资源紧张时最低：

```text
num_eval_seeds = 3
num_episodes_per_task_per_seed = 10
total_per_task = 30
```

论文主结果建议：

```text
50 episodes/task/split or more
```

必须保存：

```text
seed
task config hash
sensor config hash
policy checkpoint hash
evaluation config hash
```

## 5. Metric definitions

### 5.1 Success rate

```text
success_rate = successful_episodes / total_episodes
```

需要 per-task、per-suite、overall 都报告。

### 5.2 Completion time

```text
completion_time = first_success_step / control_frequency_hz
```

失败 episode 可设为 max time，或只在 successful episodes 中报告。必须说明。

### 5.3 Max contact force

```text
max_contact_force = max_t ||force_left_t||_2, ||force_right_t||_2
```

### 5.4 Force violation rate

```text
force_violation_rate = #steps(force_norm > task_force_limit) / #contact_steps
```

如果无 contact steps，则单独标注。

### 5.5 Contact duration

```text
contact_duration = #steps(contact_flag_left or contact_flag_right) / control_frequency_hz
```

### 5.6 Contact loss count

定义为在需要持续接触的任务阶段中：

```text
contact true -> false -> true
```

的次数。

### 5.7 Contact stability

可定义为：

```text
1 / (epsilon + std(contact_force_during_required_contact))
```

或 force jerk 的负向指标。必须固定公式。

### 5.8 Insertion depth

```text
insertion_depth = projection of inserted object along target insertion axis
```

必须用 target frame 定义。

### 5.9 Jamming count

可定义为满足以下条件的连续片段数量：

```text
high axial or lateral force
AND little/no progress in insertion depth
for >= K steps
```

### 5.10 Recovery rate

```text
recovery_rate = #episodes that succeed after a detected collision/jam / #episodes with collision/jam
```

## 6. Aggregation

### 6.1 Per-task

先计算每个 task 的 mean 和 CI。

### 6.2 Per-suite

对 suite 内 task 做 unweighted average：

```text
suite_score = mean(task_score_i)
```

不要让 episode 数多的任务支配 suite score。

### 6.3 Overall

对 suite 做 unweighted average，或报告两个版本：

```text
overall_task_avg
overall_suite_avg
```

论文主表必须说明采用哪个。

## 7. Confidence intervals

推荐 bootstrap：

```text
bootstrap over episodes within each task
then aggregate task scores
1000 resamples
95% confidence interval
```

同时报告 seed-level standard error：

```text
mean ± standard error over seeds
```

## 8. Statistical comparison

不强制做显著性检验，但建议：

- tactile vs vision-only 在每个 suite 上报告 paired bootstrap；
- robustness 曲线报告 AUC；
- contact quality 报告 effect size。

## 9. Robustness score

可定义 robustness area：

```text
robustness_auc = average success over noise levels
```

例如 pose noise：

```text
PoseRobust = mean(SR_0mm, SR_2mm, SR_5mm, SR_10mm)
```

也可报告 degradation：

```text
degradation_10mm = SR_0mm - SR_10mm
```

核心 claim 应使用：

```text
tactile policy has lower degradation under contact uncertainty
```

而不是只看单点 success。

## 10. Contact-aware ranking

不要只用 overall success 排名。建议排行榜分三类：

```text
Success Score
Contact Quality Score
Robustness Score
```

### 10.1 Success Score

```text
mean success across tasks
```

### 10.2 Contact Quality Score

可归一化：

```text
lower max force better
lower violation better
lower jamming better
lower contact loss better
```

### 10.3 Robustness Score

```text
mean success across robustness splits
```

## 11. Hidden test plan

v0 可以先不做在线 server，但文档应预留：

```text
public train/val/test_seen
public robustness config
hidden seeds for leaderboard
hidden object poses for final challenge
```

规则：

- method 不能用 hidden seed 训练；
- 提交 checkpoint 或 docker；
- 统一评测脚本；
- 输出 signed result JSON。

## 12. Evaluation output

必须保存：

```text
results/{run_id}/
  config.yaml
  per_episode.jsonl
  per_task.csv
  per_suite.csv
  aggregate.json
  metrics_summary.md
  videos_optional/
  logs/
```

## 13. Failure taxonomy

每个失败 episode 标注或自动归类：

```text
perception_failure
no_contact
excessive_force
slip
jamming
misalignment
object_drop
timeout
wrong_subgoal
workspace_violation
sim_instability
```

这会显著增强论文 qualitative analysis。

## 14. Evaluation checklist

- [ ] 使用 frozen dataset split。
- [ ] 使用 frozen task configs。
- [ ] 使用 frozen sensor configs。
- [ ] 所有 policy 在同一 action space 下评测。
- [ ] 不用 test set 做 early stopping。
- [ ] 每个 result 有 config snapshot。
- [ ] 每个 result 有 checkpoint hash。
- [ ] 每个 result 有 per-episode logs。
- [ ] 主表可由脚本重现。
- [ ] 失败案例可追踪到 episode id。

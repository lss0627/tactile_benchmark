# baseline_protocol.md — Baseline 训练、公平性与 ablation 协议

## 1. Baseline philosophy

Benchmark paper 的 baseline 目标不是追 SOTA，而是证明：

```text
在公平控制下，触觉模态对 contact-rich language-conditioned manipulation 有可测收益。
```

所以 baseline 必须：

- 简单；
- 可复现；
- 训练预算一致；
- 输入模态可控；
- 不使用 privileged state；
- 有 oracle upper bound 区分。

## 2. Required baselines

### 2.1 RandomPolicy

用途：

- sanity check；
- 测试环境不 crash；
- 估计任务随机成功率。

### 2.2 ReplayPolicy

用途：

- 验证 dataset 和 replay；
- 如果 replay 都不能成功，说明 task/data 有问题。

### 2.3 StateBC

输入：

```text
joint_pos
joint_vel
ee_pose
gripper_state
language embedding optional
```

用途：

- low-dimensional baseline；
- 不看视觉；
- 不应作为主 claim，但提供参考。

### 2.4 VisionBC

输入：

```text
front RGB
wrist RGB
language
robot state optional depending variant
```

用途：

- primary vision-only baseline。

### 2.5 VisionStateBC

输入：

```text
front RGB
wrist RGB
robot state
language
```

用途：

- 更公平的 vision+proprioception baseline。

### 2.6 VisionForceBC

输入：

```text
front RGB
wrist RGB
robot state
force/wrench
language
```

用途：

- 测试 low-dimensional tactile feedback。

### 2.7 VisionVisuoTactileBC

输入：

```text
front RGB
wrist RGB
robot state
tactile RGB/depth/force field
language
```

用途：

- 测试 visuo-tactile feedback。

### 2.8 VisionForceVisuoTactileBC

输入：

```text
front RGB
wrist RGB
robot state
force/wrench
visuo-tactile
language
```

用途：

- full multimodal tactile baseline。

### 2.9 OracleStateBC

输入：

```text
robot state
object poses
target poses
task stage
```

用途：

- upper bound；
- 必须标注 privileged；
- 不参与普通 leaderboard 排名。

## 3. Model architecture rules

### 3.1 Vision encoder

所有 vision baselines 使用同一 encoder：

```text
ResNet18 or small ViT
```

必须报告：

```text
pretrained or from scratch
input resolution
number of parameters
```

### 3.2 Language encoder

v0 可用简单方案：

```text
frozen sentence embedding
or learned task embedding
or CLIP text encoder
```

必须固定，不要让 language 模型差异影响 tactile 对比。

### 3.3 Force encoder

建议：

```text
MLP over force/wrench history
history length: 1 / 4 / 8 optional ablation
```

默认：

```text
history_length = 4
```

### 3.4 Visuo-tactile encoder

建议：

```text
same architecture family as wrist image encoder
smaller input resolution allowed but must report
```

### 3.5 Fusion

v0 默认 late fusion：

```text
feature = concat(vision_feature, state_feature, language_feature, tactile_feature)
action = MLP(feature)
```

P2 可做：

```text
early fusion
cross-attention
temporal transformer
```

## 4. Training budget

所有主 baseline 必须一致：

```yaml
epochs: same
batch_size: same
optimizer: AdamW
learning_rate: same or tuned only on val
weight_decay: same
num_gradient_steps: same
data_augmentation: same for RGB branches
random_seed: [0, 1, 2]
```

如果模型参数量差异很大，必须报告：

```text
parameter_count
training_time
GPU type
```

可选 parameter-matched control：

```text
VisionBC-wide
```

让 VisionBC 参数量接近 tactile model，防止“触觉模型只是更大”。

## 5. Data usage rules

- 所有 baseline 使用相同 train demos。
- 所有 baseline 使用相同 val split 做 early stopping。
- 所有 baseline 不可使用 test split 选择 checkpoint。
- 所有 baseline 不可读取 replay-only metadata。
- 所有 baseline 使用同一 action normalization。
- 所有 baseline 使用同一 observation normalization。

## 6. Normalization

必须记录：

```text
action mean/std
state mean/std
force mean/std
image normalization
tactile image normalization
```

Normalization 只能从 train split 计算。

## 7. Checkpoint selection

默认：

```text
best validation loss
```

或者：

```text
best validation success on val environment
```

必须固定一种，不得看 test。

## 8. Training command examples

### VisionBC

```bash
python scripts/train_bc.py \
  --policy vision_bc \
  --dataset datasets/isaac_tactile_libero_v0 \
  --split train \
  --val-split val \
  --config configs/policies/vision_bc.yaml \
  --seed 0 \
  --output checkpoints/vision_bc_seed0
```

### VisionForceBC

```bash
python scripts/train_bc.py \
  --policy vision_force_bc \
  --dataset datasets/isaac_tactile_libero_v0 \
  --split train \
  --val-split val \
  --config configs/policies/vision_force_bc.yaml \
  --seed 0 \
  --output checkpoints/vision_force_bc_seed0
```

## 9. Evaluation command examples

```bash
python scripts/evaluate.py \
  --policy vision_force_bc \
  --checkpoint checkpoints/vision_force_bc_seed0/best.pt \
  --tactile force_wrench \
  --split test_pose_noise_5mm \
  --num-episodes-per-task 50 \
  --output results/vision_force_bc_pose_noise_5mm_seed0
```

## 10. Ablations

### 10.1 Tactile modality

```text
Vision
Vision + Force
Vision + VT
Vision + Force + VT
```

### 10.2 Force history

```text
history length = 1, 4, 8
```

### 10.3 VT resolution

```text
32x32
64x64
128x128
```

### 10.4 Fusion

```text
late fusion
early fusion
cross-attention optional
```

### 10.5 Noise robustness

Train nominal, test with:

```text
force noise
force bias
tactile image dropout
latency
```

## 11. Reporting format

每个 baseline 报告：

```text
policy name
input modalities
parameter count
training demos
training steps
val selection rule
checkpoint hash
training time
GPU
success rate
contact metrics
robustness metrics
```

## 12. Fairness checklist

- [ ] 相同 train split。
- [ ] 相同 val split。
- [ ] 相同 test split。
- [ ] 相同 action space。
- [ ] 相同 training steps。
- [ ] 相同 batch size。
- [ ] 相同 optimizer。
- [ ] 相同 vision encoder。
- [ ] 相同 language encoder。
- [ ] 参数量报告。
- [ ] 不读取 privileged metadata。
- [ ] 不用 test 做 model selection。
- [ ] 输出 per-task 而非只给 overall。
- [ ] 所有 baseline 有 3 seeds。
- [ ] 所有 baseline 保存 config。

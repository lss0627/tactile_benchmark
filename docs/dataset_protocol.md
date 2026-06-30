# dataset_protocol.md — 数据格式、同步、质量验证与发布协议

## 1. Dataset goals

数据集必须满足：

```text
可训练
可 replay
可验证
可转换
可审计
可复现
```

不要只发布视频或 simulator logs。每条 trajectory 必须包含足够信息来训练 policy、复现实验、重算指标、定位失败。

## 2. Dataset version

```yaml
dataset_name: Isaac-Tactile-LIBERO-v0
dataset_version: 0.1.0
benchmark_version: 0.1.0
schema_version: 0.1.0
robot: fr3_tactile
simulator: isaac_sim_or_isaac_lab_version
```

## 3. Directory layout

```text
datasets/
  isaac_tactile_libero_v0/
    metadata.json
    dataset_card.md
    schema.json
    splits/
      train.json
      val.json
      test_seen.json
      test_pose_noise.json
      test_occlusion.json
      test_low_clearance.json
      test_unseen_geometry.json
    hdf5/
      tactile_contact/
      tactile_assembly/
      tactile_articulated/
      tactile_long/
      libero_compatible/
    validation/
      validation_report.json
      replay_report.json
      quality_audit.csv
    videos/
      samples/
```

## 4. HDF5 episode schema

每个 episode group：

```text
/episodes/{episode_id}/
  attrs:
    episode_id
    task_name
    suite_name
    instruction
    seed
    split
    benchmark_version
    schema_version
    success
    num_steps
    control_frequency_hz
    simulator_version
    robot_name
    tactile_mode
    sensor_config_hash
    task_config_hash

  observations/
    rgb/front
    rgb/wrist
    state/joint_pos
    state/joint_vel
    state/ee_pose
    state/gripper_state
    tactile/contact_flag_left
    tactile/contact_flag_right
    tactile/force_left
    tactile/force_right
    tactile/wrench_left
    tactile/wrench_right
    tactile/vt_rgb_left
    tactile/vt_rgb_right
    tactile/vt_depth_left
    tactile/vt_depth_right
    tactile/force_field_left
    tactile/force_field_right
    tactile/mask

  actions/
    action

  rewards/
    reward

  terminals/
    terminated
    truncated
    success

  metrics/
    max_contact_force
    mean_contact_force
    force_violation_rate
    contact_duration
    contact_loss_count
    contact_stability
    insertion_depth
    jamming_count
    recovery_rate

  timestamps/
    sim_time
    wall_time_optional

  metadata/
    object_poses_for_replay_only
    task_params
    camera_intrinsics
    camera_extrinsics
    tactile_extrinsics
    calibration
```

## 5. Important rule: privileged replay metadata

`object_poses_for_replay_only` 可以保存，但不能进入 policy observation。

必须在 dataloader 中分离：

```text
policy_observation:
  allowed input

replay_metadata:
  only for replay, validation, metric recomputation
```

## 6. Units and coordinate frames

### 6.1 Units

```text
position: meters
rotation: radians
force: Newton
torque: Newton-meter
time: seconds
image: uint8 RGB unless otherwise specified
depth: meters
```

### 6.2 Frames

必须在 dataset metadata 中写清：

```text
world
robot_base
end_effector
gripper
left_tactile
right_tactile
front_camera
wrist_camera
object
target
```

每个 pose 必须明确所属坐标系。

## 7. Synchronization

### 7.1 Required timestamps

每个 timestep 保存：

```text
sim_time[t]
control_step[t]
front_rgb_timestamp[t]
wrist_rgb_timestamp[t]
tactile_timestamp[t]
state_timestamp[t]
action_timestamp[t]
```

### 7.2 Validation rules

必须检查：

```text
timestamp monotonic
no missing control step
RGB/state/tactile/action aligned within tolerance
no duplicated episode_id
num_steps matches all arrays
```

推荐容忍：

```text
max_sensor_time_skew_ms <= 1 control step
```

## 8. Compression

建议：

```text
RGB video: compressed video or HDF5 compression, depending tooling
state/action/force: float32, gzip/lzf
tactile image: uint8 or float16 depending representation
force field/depth: float16 or float32 with documented scale
```

必须保存：

```text
compression type
encoding
decode script
checksum
```

## 9. Dataset splits

### 9.1 Split names

```text
train
val
test_seen
test_pose_noise
test_occlusion
test_low_clearance
test_friction_randomization
test_unseen_geometry
```

### 9.2 Split rules

- `train`：用于训练。
- `val`：用于 model selection 和 early stopping。
- `test_seen`：同分布 held-out seeds。
- robustness tests：只用于最终评估。
- OOD split 中的 pose/object/geometry/language templates 不应出现在 train。
- test instructions 不得只是 train instruction 的重复。

### 9.3 Split manifest

每个 split JSON：

```json
{
  "split_name": "test_seen",
  "benchmark_version": "0.1.0",
  "episodes": [
    {
      "episode_id": "...",
      "task_name": "PegInsert",
      "suite_name": "tactile_assembly",
      "seed": 123
    }
  ]
}
```

## 10. Dataset validation script

命令：

```bash
python scripts/validate_dataset.py \
  --dataset datasets/isaac_tactile_libero_v0 \
  --output datasets/isaac_tactile_libero_v0/validation/validation_report.json
```

必须输出：

```json
{
  "num_episodes": 1500,
  "num_tasks": 30,
  "missing_key_rate": 0.0,
  "timestamp_error_rate": 0.0,
  "shape_error_rate": 0.0,
  "nan_rate": 0.0,
  "invalid_tactile_frame_rate": 0.0,
  "force_saturation_rate": 0.0,
  "frame_drop_rate": 0.0,
  "replay_success_rate": 0.0,
  "schema_version": "0.1.0"
}
```

## 11. Replay protocol

命令：

```bash
python scripts/replay_demos.py \
  --dataset datasets/isaac_tactile_libero_v0 \
  --split train \
  --num_episodes 100 \
  --save_video
```

报告：

```text
replay_success_rate
metric_consistency
state_deviation
action_replay_error
failed_episode_ids
```

最低要求：

```text
train replay success >= 95%
val/test replay success >= 90%
```

如果 replay 达不到，应先修任务/控制/记录，而不是继续采更多数据。

## 12. Dataset card required fields

```markdown
# Isaac-Tactile-LIBERO Dataset Card

## Dataset Summary
## Benchmark Version
## Task Suites
## Modalities
## Data Collection Procedure
## Robot and Simulator
## Sensors
## Dataset Splits
## File Format
## Schema
## Quality Validation
## Replay Protocol
## Intended Use
## Out-of-Scope Use
## Known Limitations
## Licenses
## Citation
## Contact
```

## 13. LeRobot export

映射建议：

```text
observation.images.front
observation.images.wrist
observation.state
observation.tactile.force_left
observation.tactile.force_right
observation.tactile.wrench_left
observation.tactile.wrench_right
observation.tactile.vt_rgb_left
observation.tactile.vt_rgb_right
action
episode_index
frame_index
timestamp
task
instruction
```

要求：

- 保留 episode metadata；
- 保留 task/instruction；
- 保留 tactile masks；
- 导出后用 LeRobot dataloader 做最小读取测试。

## 14. RLDS export

映射建议：

```text
episode:
  episode_metadata
  steps:
    observation
    action
    reward
    discount
    is_first
    is_last
    is_terminal
```

要求：

- 不丢失 language instruction；
- 不丢失 tactile modality；
- 不丢失 success 和 metrics；
- conversion back test 可选。

## 15. Data quality gates

进入论文主实验前必须满足：

```text
missing_key_rate == 0
nan_rate == 0
timestamp_error_rate == 0
shape_error_rate == 0
replay_success_rate >= threshold
invalid_tactile_frame_rate documented
force_saturation_rate documented
all splits frozen
dataset card complete
```

## 16. Dataset leakage checks

必须检查：

- test object poses 不在 train；
- test seeds 不在 train；
- test language templates 不完全重复 train；
- policy dataloader 不读取 privileged metadata；
- success label 不作为 observation；
- insertion depth 不作为 policy input；
- task id 是否会让 policy 走 shortcut，需要在 baseline 说明。

## 17. Dataset release levels

### Level 0: internal

- raw HDF5；
- validation script；
- small sample。

### Level 1: paper submission

- frozen v0 dataset；
- dataset card；
- split manifests；
- replay report；
- validation report。

### Level 2: public release

- download script；
- checksums；
- sample videos；
- LeRobot/RLDS export optional；
- DOI/archive；
- license table。

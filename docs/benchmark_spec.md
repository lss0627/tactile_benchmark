# benchmark_spec.md — Benchmark 契约、版本和接口规范

## 1. Benchmark identity

```text
Name: Isaac-Tactile-LIBERO
Short name: ITL
Version: 0.1.0
Primary robot: FR3-Tactile
Primary simulator: Isaac Sim / Isaac Lab
Task family: language-conditioned contact-rich manipulation
```

## 2. Versioning policy

使用 semantic versioning：

```text
MAJOR.MINOR.PATCH
```

### MAJOR change

以下变化需要 bump MAJOR：

- observation schema 不兼容；
- action schema 不兼容；
- metric definition 不兼容；
- evaluation split 重新定义；
- task success condition 改变；
- dataset episode layout 不兼容。

### MINOR change

以下变化需要 bump MINOR：

- 新增 task；
- 新增 tactile mode；
- 新增 baseline；
- 新增 export format；
- 新增 evaluation split，但不改变已有 split。

### PATCH change

以下变化需要 bump PATCH：

- bug fix；
- documentation update；
- speed improvement；
- non-breaking config fix。

## 3. Registry contract

### 3.1 Task registry

```python
TASK_REGISTRY.register(
    name="PegInsert",
    suite="tactile_assembly",
    cls=PegInsertTask,
    version="0.1.0",
)
```

必须支持：

```python
TASK_REGISTRY.list()
TASK_REGISTRY.get(name)
TASK_REGISTRY.make(name, cfg)
```

### 3.2 Tactile registry

```python
TACTILE_SENSOR_REGISTRY = {
    "none": NoTactileSensor,
    "force_wrench": ForceWrenchSensor,
    "visuotactile": VisuoTactileSensor,
    "force_plus_visuotactile": ForcePlusVisuoTactileSensor,
}
```

### 3.3 Policy registry

Policy 必须实现：

```python
class BasePolicy:
    def reset(self, env_ids=None): ...
    def act(self, obs): ...
    def load(self, checkpoint): ...
```

## 4. Environment API

```python
env = make_env(
    task: str,
    robot: str = "fr3_tactile",
    tactile: str = "none",
    split: str = "train",
    seed: int = 0,
    num_envs: int = 1,
    cfg: dict | None = None,
)
```

返回对象必须支持：

```python
obs = env.reset()
obs, reward, terminated, truncated, info = env.step(action)
env.close()
```

`info` 必须包含：

```python
info = {
    "task_name": str,
    "suite_name": str,
    "instruction": str,
    "success": bool,
    "metrics": dict,
    "seed": int,
    "split": str,
}
```

## 5. Observation schema

### 5.1 Top-level schema

```python
obs = {
    "language": str,
    "rgb": {
        "front": np.ndarray,   # H x W x 3, uint8
        "wrist": np.ndarray,   # H x W x 3, uint8
    },
    "state": {
        "joint_pos": np.ndarray,
        "joint_vel": np.ndarray,
        "ee_pose": np.ndarray,
        "gripper_state": np.ndarray,
    },
    "tactile": dict,
    "time": {
        "step": int,
        "timestamp": float,
    },
}
```

### 5.2 Tactile schema

```python
obs["tactile"] = {
    "valid": bool,

    "contact_flag_left": bool,
    "contact_flag_right": bool,

    "force_left": np.ndarray,       # shape (3,), Newton
    "force_right": np.ndarray,      # shape (3,), Newton

    "wrench_left": np.ndarray,      # shape (6,), N and Nm
    "wrench_right": np.ndarray,

    "vt_rgb_left": np.ndarray | None,
    "vt_rgb_right": np.ndarray | None,

    "vt_depth_left": np.ndarray | None,
    "vt_depth_right": np.ndarray | None,

    "force_field_left": np.ndarray | None,
    "force_field_right": np.ndarray | None,

    "mask": {
        "has_force": bool,
        "has_wrench": bool,
        "has_vt_rgb": bool,
        "has_vt_depth": bool,
        "has_force_field": bool,
    },
}
```

## 6. Action schema

统一 7D delta action：

```python
action = np.ndarray(shape=(7,), dtype=np.float32)
```

解释：

```text
action[0] dx, meters
action[1] dy, meters
action[2] dz, meters
action[3] droll or axis-angle x, radians
action[4] dpitch or axis-angle y, radians
action[5] dyaw or axis-angle z, radians
action[6] gripper command, normalized [-1, 1]
```

必须文档化：

- control frequency；
- action clipping；
- command smoothing；
- gripper threshold；
- coordinate frame。

## 7. Coordinate frames

必须定义：

```text
world
robot_base
end_effector
gripper
left_tactile
right_tactile
front_camera
wrist_camera
object frames
target frames
```

每个 episode 的 metadata 必须存：

```text
frame convention
camera intrinsics
camera extrinsics
tactile extrinsics
unit convention
```

## 8. Sensor realism knobs

每个 tactile config 必须包含：

```yaml
sampling_rate_hz: 30
latency_ms: 0
noise:
  force_std: 0.0
  torque_std: 0.0
  image_noise_std: 0.0
bias:
  force: [0, 0, 0]
saturation:
  force_norm_max: 100.0
contact_threshold_n: 0.5
dropout:
  tactile_frame_drop_prob: 0.0
```

强制要求：

- default config 不加随机噪声，便于 reproducibility；
- robustness config 可开启 noise/latency/dropout；
- dataset metadata 必须保存 sensor config。

## 9. Task config contract

每个 task config 至少包含：

```yaml
task_name:
suite_name:
version:
assets:
language_templates:
reset_distribution:
success_condition:
failure_condition:
termination:
metrics:
robustness_variants:
```

## 10. Evaluation config contract

每个 evaluation config 至少包含：

```yaml
benchmark_version:
task_list:
policy:
checkpoint:
tactile_mode:
split:
num_episodes_per_task:
seeds:
max_steps:
metrics:
save_video:
save_rollout:
```

## 11. Result schema

每次 evaluation 输出：

```text
results/
  config.yaml
  per_episode.jsonl
  per_task.csv
  per_suite.csv
  aggregate.json
  videos/
  logs/
```

`per_episode.jsonl` 每行包含：

```json
{
  "episode_id": "...",
  "task_name": "...",
  "suite_name": "...",
  "split": "...",
  "seed": 0,
  "policy": "...",
  "tactile_mode": "...",
  "success": true,
  "metrics": {},
  "num_steps": 123,
  "wall_time_sec": 12.3
}
```

## 12. Compatibility boundaries

### Required

- HDF5 native dataset；
- Python API；
- CLI evaluation；
- fixed train/val/test split；
- reproducible seed。

### Recommended

- LeRobot export；
- RLDS export；
- Hugging Face dataset card；
- leaderboard format。

### Out of v0 scope

- multi-robot benchmark；
- real-robot large-scale benchmark；
- full LIBERO 130-task migration；
- tactile sensor hardware validation at scale。

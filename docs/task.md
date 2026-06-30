# task.md — Isaac-Tactile-LIBERO 可执行任务清单

本文档可直接转成 GitHub Issues / Project Board。优先级：

- **P0**：没有它不能称为 benchmark。
- **P1**：投稿最小版本必须有。
- **P2**：强投稿版本或 rebuttal 加分项。
- **P3**：可选扩展，不阻塞第一篇论文。

## 0. 项目管理规则

### 0.1 Definition of Done

每个 issue 完成必须满足：

- 有代码或文档 PR；
- 有最小测试；
- 有运行命令；
- 有输出 artifact；
- 有 reviewer 可复现说明；
- 更新 `checklist.md` 对应项。

### 0.2 统一命名

```text
suite/task: tactile_assembly/PegInsert
sensor: none | force_wrench | visuotactile | force_plus_visuotactile
split: train | val | test_seen | test_pose_noise | test_occlusion | test_low_clearance | test_unseen_geometry
policy: random | replay | state_bc | vision_bc | vision_force_bc | vision_vt_bc | vision_force_vt_bc | oracle_state_bc
```

---

# Phase 0 — Repo 与 benchmark contract

## P0-001 新建独立仓库

**Deliverable**

```text
isaac-tactile-libero/
  README.md
  pyproject.toml
  LICENSE
  CITATION.cff
  task.md
  checklist.md
  docs/
```

**Done**

- `pip install -e .` 可运行；
- `python scripts/list_tasks.py` 可运行；
- README 明确项目不是 LIBERO 迁移，也不是 Lightwheel fork。

## P0-002 固定 benchmark version

**Deliverable**

```text
isaac_tactile_libero/version.py
docs/benchmark_spec.md
```

**Done**

- `BENCHMARK_VERSION = "0.1.0"`；
- 版本规则写清：任务、数据 schema、评测协议变更如何 bump version。

## P0-003 实现 registry skeleton

**Deliverable**

```text
isaac_tactile_libero/registry/
  task_registry.py
  robot_registry.py
  tactile_registry.py
  policy_registry.py
```

**Done**

- 可注册 task / robot / tactile / policy；
- 重名注册报错；
- registry 支持 list 和 instantiate；
- 单元测试覆盖 4 个 registry。

## P0-004 实现 make_env contract

**Deliverable**

```text
isaac_tactile_libero/envs/make.py
```

**API**

```python
env = make_env(
    task="PegInsert",
    robot="fr3_tactile",
    tactile="force_wrench",
    split="test_seen",
    seed=0,
    num_envs=1,
)
```

**Done**

- 返回统一 `reset()` / `step(action)`；
- observation keys 与 schema 一致；
- 不同 tactile mode 不改变 action schema。

---

# Phase 1 — FR3-Tactile embodiment

## P0-005 定义 FR3-Tactile robot config

**Deliverable**

```text
isaac_tactile_libero/robots/fr3_tactile/
  robot_cfg.py
  action_cfg.py
  frames.py
  usd_paths.py
```

**Done**

- joint names；
- link names；
- EE frame；
- gripper frame；
- left/right tactile mount frame；
- front/wrist camera frame；
- default joint pose；
- collision settings；
- articulation properties。

## P0-006 固定 action schema

**Deliverable**

```text
isaac_tactile_libero/schemas/action.py
```

**Schema**

```text
action[0:3] = delta position, meters
action[3:6] = delta rotation, axis-angle or Euler, radians
action[6]   = gripper command, normalized [-1, 1]
```

**Done**

- 所有 task、dataset、policy 使用同一 7D action；
- 写清动作频率；
- 写清 clipping 和 smoothing。

## P1-007 Robot smoke test

**Deliverable**

```text
scripts/smoke_test_robot.py
```

**Done**

- reset 100 次无 crash；
- 随机 action 500 steps 无 NaN；
- gripper open/close 正常；
- EE pose 变化方向正确。

---

# Phase 2 — Tactile sensor plugin

## P0-008 BaseTactileSensor interface

**Deliverable**

```text
isaac_tactile_libero/sensors/base.py
```

**API**

```python
class BaseTactileSensor:
    name: str
    def build(self, robot, scene, cfg): ...
    def reset(self, env_ids): ...
    def read(self): ...
    def observation_spec(self): ...
    def metric_spec(self): ...
```

**Done**

- 所有 sensor plugin 继承此接口；
- `read()` 返回统一 tactile schema；
- sensor 不直接改 task reward / success。

## P0-009 NoTactileSensor

**Deliverable**

```text
isaac_tactile_libero/sensors/none.py
```

**Done**

- `obs["tactile"]["valid"] = False`；
- mask 全 False；
- shape 仍然可被 policy dataloader 处理。

## P0-010 ForceWrenchSensor

**Deliverable**

```text
isaac_tactile_libero/sensors/force_wrench.py
```

**Done**

- 输出 left/right force；
- 输出 left/right 6D wrench；
- 输出 contact flag；
- 支持 threshold；
- 支持 normalization；
- 支持 noise / bias / saturation config。

## P0-011 VisuoTactileSensor

**Deliverable**

```text
isaac_tactile_libero/sensors/visuotactile.py
```

**Done**

- 输出 left/right tactile RGB 或 deformation map；
- 输出 valid mask；
- 写清 tactile rendering 是近似模拟还是可替换接口；
- 支持分辨率配置；
- 支持 frame drop 和 latency 模拟开关。

## P1-012 ForcePlusVisuoTactileSensor

**Deliverable**

```text
isaac_tactile_libero/sensors/force_plus_visuotactile.py
```

**Done**

- 组合 force_wrench + visuotactile；
- 同步时间戳；
- 读数与单独 sensor mode 一致。

## P1-013 Sensor calibration metadata

**Deliverable**

```text
configs/tactile/calibration_default.yaml
```

**Done**

- 坐标系；
- 单位；
- scale；
- bias；
- saturation；
- sampling rate；
- latency；
- noise model。

---

# Phase 3 — Task card 与最小任务

## P0-014 Task card template

**Deliverable**

```text
docs/task_cards.md
```

**Done**

每个任务必须填写：

- task id；
- suite；
- natural language templates；
- reset distribution；
- object assets；
- success condition；
- failure condition；
- termination；
- metrics；
- robustness variants；
- leakage risks；
- smoke tests。

## P0-015 实现 PressButton

**Suite**

```text
tactile_contact
```

**Done**

- `--tactile none/force_wrench/visuotactile/force_plus_visuotactile` 均可运行；
- success 与 force threshold 不冲突；
- 支持 force limit variant。

## P0-016 实现 SoftPress

**Done**

- 目标不是按到底，而是达到指定力区间；
- 评估 force tracking error；
- 视觉-only 不应通过隐藏状态作弊。

## P0-017 实现 PushSlider

**Done**

- 成功条件为 slider 位移达到目标区间；
- 统计 contact loss count；
- 支持不同摩擦和阻尼。

## P0-018 实现 PegInsert

**Done**

- 成功条件为 insertion depth；
- 统计 jamming；
- 支持 pose noise；
- 支持 low-clearance variant。

## P0-019 实现 PlugSocketInsert

**Done**

- 成功条件为 plug 插入 socket；
- 统计 alignment error 和 max insertion force；
- 支持 occlusion split。

## P1-020 最小任务 smoke test

**Deliverable**

```text
scripts/smoke_test.py
```

**Done**

- 5 tasks × 4 tactile modes × 3 seeds；
- 每个组合 reset/step 不 crash；
- observation shape 一致；
- metrics 输出 JSON。

---

# Phase 4 — 扩展到 benchmark v0 task suite

## P1-021 扩展 Tactile-Contact 8 tasks

**Target**

```text
PressButton
PressButtonWithForceLimit
SoftPress
FragileTouch
PushSlider
ToggleSwitch
WipeSurface
ContactMaintain
```

**Done**

- 每个任务有 task card；
- 每个任务有 deterministic unit test；
- 每个任务有 random policy rollout。

## P1-022 扩展 Tactile-Assembly 12 tasks

**Target**

```text
PegInsert
RoundPegInsert
SquarePegInsert
ShapeInsert
PlugSocketInsert
ChargerInsert
LowClearanceInsertion
MisalignedInsertionRecovery
GearAlign
KeySlotInsert
CableConnectorInsert
CapTwistInsert
```

**Done**

- 至少 4 个任务需要触觉显著受益；
- 至少 4 个任务有 low-clearance 或 pose-noise split；
- 每个任务定义 jamming。

## P1-023 扩展 Tactile-Articulated 6 tasks

**Target**

```text
OpenDrawer
OpenDrawerWithForceLimit
PullHandle
OpenCabinetDoor
TurnKnob
PressPanelButton
```

**Done**

- opening distance metric；
- handle contact success metric；
- contact loss count metric。

## P1-024 扩展 Tactile-Long 4 tasks

**Target**

```text
PickAlignInsert
OpenRetrieveInsert
PressThenPlace
PickPlugInsertSocket
```

**Done**

- subgoal success；
- failure stage；
- completion time；
- contact recovery。

## P1-025 Base-LIBERO-Compatible 5 tasks

**Done**

- 只做兼容对照，不作为主贡献；
- 使用同一 observation/action/eval schema；
- README 明确与 tactile suites 的关系。

---

# Phase 5 — Dataset v0

## P0-026 HDF5 writer / reader

**Deliverable**

```text
isaac_tactile_libero/datasets/writer.py
isaac_tactile_libero/datasets/reader.py
```

**Done**

- episode-level metadata；
- timestep-level observations/actions/rewards；
- compression；
- checksum；
- version；
- schema validation。

## P0-027 Dataset validation

**Deliverable**

```text
scripts/validate_dataset.py
```

**Done**

输出：

```text
num_episodes
num_invalid_episodes
replay_success_rate
timestamp_monotonicity
frame_drop_rate
force_saturation_rate
invalid_tactile_frame_rate
missing_key_rate
```

## P1-028 Replay demos

**Deliverable**

```text
scripts/replay_demos.py
```

**Done**

- 随机抽样 replay；
- replay 成功率报告；
- replay 视频导出；
- replay 与原始 success label 一致性统计。

## P1-029 Collect 1500 demos

**Target**

```text
30 tasks × 50 demos/task
```

**Done**

- 每个 task train/val/test split；
- 每个 demo 可 replay；
- dataset card；
- validation report。

## P2-030 LeRobot export

**Deliverable**

```text
scripts/export_lerobot.py
```

**Done**

- 映射 observation/action；
- 保存多相机视频；
- 保存 tactile 张量；
- 保存 task/instruction metadata；
- 可被 LeRobot dataloader 读取。

## P2-031 RLDS export

**Deliverable**

```text
scripts/export_rlds.py
```

**Done**

- episode/step schema；
- observation/action/reward/discount/is_terminal；
- metadata；
- conversion validation。

---

# Phase 6 — Evaluation protocol

## P0-032 Metrics implementation

**Deliverable**

```text
isaac_tactile_libero/metrics/
  success.py
  contact.py
  assembly.py
  robustness.py
```

**Done**

- success rate；
- completion time；
- max/mean force；
- force violation rate；
- contact duration；
- contact loss；
- insertion depth；
- jamming count；
- recovery rate。

## P0-033 Evaluation runner

**Deliverable**

```text
scripts/evaluate.py
```

**Done**

- 固定 seeds；
- 固定 episodes per task；
- 输出 per-task 和 aggregate JSON/CSV；
- 保存 config snapshot；
- 支持 resume；
- 支持 deterministic replay。

## P1-034 Robustness configs

**Deliverable**

```text
configs/eval/
  default.yaml
  pose_noise.yaml
  occlusion.yaml
  low_clearance.yaml
  friction_randomization.yaml
  unseen_geometry.yaml
```

**Done**

- 每个 split 有明确分布；
- train/test 不泄漏；
- 每个 task card 标注支持哪些 split。

## P1-035 Statistical report

**Deliverable**

```text
scripts/report_results.py
```

**Done**

- mean；
- standard error；
- bootstrap confidence interval；
- per-suite aggregate；
- normalized score；
- significance markers 可选。

---

# Phase 7 — Baselines

## P0-036 RandomPolicy / ReplayPolicy

**Done**

- random policy sanity check；
- replay policy upper sanity check；
- replay 在 dataset demos 上成功率应接近 100%，否则 dataset/task 有问题。

## P0-037 StateBC / VisionBC

**Done**

- 作为基础 imitation learning baseline；
- 训练/验证 split 固定；
- 超参保存；
- checkpoint 保存。

## P1-038 VisionForceBC

**Done**

- 与 VisionBC 共享视觉 backbone；
- 只增加 force branch；
- 参数量和训练预算报告。

## P1-039 VisionVisuoTactileBC

**Done**

- tactile image branch；
- 支持 late fusion；
- 对齐时间戳；
- 对比 VisionForceBC。

## P1-040 VisionForceVisuoTactileBC

**Done**

- 全模态；
- 与其他 baseline 同训练预算；
- 记录融合方式。

## P1-041 OracleStateBC

**Done**

- 明确 oracle 输入；
- 标为 upper bound；
- 不与普通 policy 混淆。

## P2-042 ACT / Diffusion Policy

**Done**

- 只作为 strong baseline；
- 不阻塞第一版投稿；
- 与 BC 使用相同 split 和 metrics。

---

# Phase 8 — Paper package

## P1-043 Main paper outline

**Deliverable**

```text
docs/paper_plan.md
```

**Done**

- abstract；
- contributions；
- related work matrix；
- method；
- benchmark；
- dataset；
- experiments；
- limitations。

## P1-044 Figures

**Required figures**

1. benchmark overview；
2. tactile plugin architecture；
3. task suite examples；
4. dataset schema；
5. robustness curves；
6. contact-aware metric case study。

## P1-045 Tables

**Required tables**

1. related benchmark comparison；
2. task suite summary；
3. dataset statistics；
4. main baseline results；
5. robustness results；
6. ablation results；
7. metric definitions。

## P1-046 Project page

**Done**

- demo videos；
- task browser；
- dataset download；
- code install；
- benchmark leaderboard；
- citation。

## P1-047 Artifact release

**Done**

- code；
- dataset；
- pretrained baselines；
- configs；
- logs；
- validation reports；
- Dockerfile；
- model cards；
- dataset card。

---

# Phase 9 — Go / No-Go gates

## Gate A: Minimum tactile loop

继续扩任务前必须满足：

- 5 tasks；
- 4 tactile modes；
- `evaluate.py`；
- metrics JSON；
- no crash over 3 seeds。

## Gate B: Dataset v0

进入 baseline 训练前必须满足：

- 1500 demos；
- validation pass；
- replay success rate 达标；
- train/val/test split 固定；
- dataset card 完成。

## Gate C: Submission

投稿前必须满足：

- 30 tasks；
- main baselines；
- robustness evaluation；
- contact-aware metrics；
- paper figures/tables；
- release checklist 80% 以上完成。

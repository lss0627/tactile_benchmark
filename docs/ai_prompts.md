# AI Implementation Prompts for Isaac-Tactile-LIBERO

这份文件用于把 `task.md`、`checklist.md`、`benchmark_spec.md`、`dataset_protocol.md`、`evaluation_protocol.md`、`baseline_protocol.md` 等计划文档转成可执行的 AI 编程任务。

## 0. 使用原则

不要一次让 AI “完成整个 benchmark”。推荐每次只让 AI 完成一个可测试的 slice：接口、一个传感器、一个任务、一个 writer、一个 evaluator、一个 baseline。每次 prompt 都要包含：当前仓库状态、要读取的计划文档、允许修改的文件、交付物、验收标准、测试命令、不能做什么。

推荐工作方式：

1. 先让 AI 读文档并输出实施方案，不改代码。
2. 确认方案后，让 AI 只做一个小任务。
3. 每次要求 AI 添加 smoke test 或 unit test。
4. 每次要求 AI 更新 checklist 中对应条目。
5. 每次让 AI 给出改动摘要、运行命令、失败/未完成项。

---

## 1. Master Prompt：让 AI 进入项目上下文

把下面这段作为每个新 AI 会话的开场 prompt。

```text
你是我的机器人学习 benchmark 工程助手。请基于当前仓库和 docs/ 下的计划文档，帮助我实现 Isaac-Tactile-LIBERO。

项目定位：Isaac-Tactile-LIBERO 是一个面向 language-conditioned contact-rich manipulation 的 Isaac Sim / Isaac Lab 触觉 benchmark。核心贡献不是迁移完整 LIBERO，也不是 fork Lightwheel 加几个任务，而是：FR3-Tactile embodiment、可替换 force/visuo-tactile sensor plugin、contact-rich task suites、统一 observation/action/dataset schema、可 replay dataset、contact-aware metrics 和公平 baseline。

请优先阅读这些文件：
- docs/task.md
- docs/checklist.md
- docs/benchmark_spec.md
- docs/task_cards.md
- docs/dataset_protocol.md
- docs/evaluation_protocol.md
- docs/baseline_protocol.md
- docs/reproducibility_release.md
- docs/risk_register.md

工作规则：
1. 不要一次性重构整个仓库。
2. 不要把主项目写成 Lightwheel fork；Lightwheel 只作为 optional backend / compatibility target。
3. 所有接口必须和 benchmark_spec.md 对齐。
4. 所有任务必须有 task card。
5. 所有数据写入、读取、replay、evaluation 必须可测试。
6. 每完成一个任务，请同步更新 checklist.md 或给出应该勾选的条目。
7. 任何不确定的 API 或依赖，请先说明假设，再给出最小可运行实现。
8. 输出必须包含：改动摘要、修改文件列表、测试命令、剩余风险。

先不要修改代码。请先阅读项目结构和上述文档，然后给出一个分阶段实施计划，并指出第一步应该做的最小可验证任务。
```

---

## 2. Repo Skeleton Prompt：创建最小仓库骨架

```text
请按照 docs/benchmark_spec.md 和 docs/task.md 实现 Isaac-Tactile-LIBERO 的最小仓库骨架。

目标：只完成接口和目录结构，不实现复杂仿真逻辑。

请创建或补齐：
- isaac_tactile_libero/registry/
- isaac_tactile_libero/robots/fr3_tactile/
- isaac_tactile_libero/sensors/
- isaac_tactile_libero/tasks/
- isaac_tactile_libero/schemas/
- isaac_tactile_libero/metrics/
- isaac_tactile_libero/datasets/
- isaac_tactile_libero/policies/
- isaac_tactile_libero/envs/
- configs/
- scripts/
- tests/

必须实现：
- task_registry.py
- robot_registry.py
- tactile_registry.py
- policy_registry.py
- BaseTactileSensor
- BasePolicy
- make_env(task, robot, tactile, seed) 的占位入口
- observation schema 和 action schema 的 dataclass 或 typed dict
- scripts/list_tasks.py
- scripts/smoke_test.py

验收标准：
- `python scripts/list_tasks.py` 可以运行。
- `python scripts/smoke_test.py --task PressButton --tactile none` 可以运行，即使内部是 mock env。
- 添加最少 3 个单元测试，覆盖 registry、schema、sensor mode。
- 不要引入大型训练框架。
- 不要实现 30 个任务；只注册 PressButton / SoftPress / PushSlider / PegInsert / PlugSocketInsert 的占位类。

完成后给出修改文件列表、测试命令和 checklist.md 中应该勾选的条目。

Lightwheel 使用规则：
1. Lightwheel / LW-BenchHub 只作为参考实现、optional backend 和 compatibility target。
2. 可以参考其 task registry、robot registry、env server、policy API、evaluation runner 的接口设计。
3. 不要直接复制 Lightwheel 的代码、资产、配置或数据，除非明确确认 license 允许。
4. 如果需要使用 Lightwheel 的 task assets、USD、object meshes、textures 或 scene files，必须先生成 docs/asset_licenses.csv 条目，记录 source、license、modified、used_in_tasks、redistribution_allowed。
5. 所有 Lightwheel 相关代码必须放在 adapter/wrapper 层，例如 envs/lightwheel_wrapper.py，不要让主 benchmark 依赖 Lightwheel 才能运行。
6. 主贡献必须保持为 tactile sensing interface、contact-rich tasks、dataset schema、contact-aware metrics 和 baseline protocol。
```

---

## 3. Tactile Sensor Prompt：实现触觉插件接口

```text
请基于 docs/benchmark_spec.md 实现 tactile sensor plugin 层。

目标：同一个任务可以通过参数切换：none、force_wrench、visuotactile、force_plus_visuotactile。

请实现：
- sensors/base.py: BaseTactileSensor
- sensors/none.py: NoTactileSensor
- sensors/force_wrench.py: ForceWrenchSensor
- sensors/visuotactile.py: VisuoTactileSensor
- sensors/force_plus_visuotactile.py: ForcePlusVisuoTactileSensor
- sensors/normalization.py
- sensors/history.py，可先做简单 ring buffer
- tactile_registry.py 中注册四种 mode

接口必须支持：
- build(robot, scene, cfg)
- reset(env_ids)
- read()
- observation_spec()
- metric_spec()

输出 schema 必须包含统一 obs["tactile"] 字段，并用 mask 表示哪些模态有效。不要因为某个传感器没有 vt_rgb 或 force_field 就改变 schema。

验收标准：
- 四种 tactile mode 都可以被 registry 创建。
- 每种 mode 的 observation_spec 和 read() 返回结构一致。
- contact_flag 可以从 force threshold 派生。
- 添加 tests/test_tactile_sensors.py。
- 通过：`pytest tests/test_tactile_sensors.py`。

请避免绑定真实 Isaac Sim API；如果必须调用仿真接口，请做 adapter 或 mock，保证 CI 可以在无 Isaac 环境下跑基础测试。
```

---

## 4. Minimal Task Suite Prompt：实现 5 个最小任务

```text
请按照 docs/task_cards.md、docs/benchmark_spec.md 和 docs/evaluation_protocol.md，实现 v0 的 5 个最小任务：
- PressButton
- SoftPress
- PushSlider
- PegInsert
- PlugSocketInsert

目标：每个任务都有标准 task card、reset/randomization/success/metrics 接口，并能在四种 tactile mode 下运行 smoke test。

请为每个任务补齐：
- task class
- default config
- language instruction 模板
- success predicate
- contact-aware metric hooks
- task card markdown 或 YAML
- minimal scripted oracle / replay placeholder，如果真实控制还没完成可先 stub，但要暴露接口

验收标准：
- `python scripts/list_tasks.py` 能列出 5 个任务。
- `python scripts/smoke_test.py --task <task> --tactile <mode>` 对 5x4 组合都能跑通。
- 每个任务至少有一条 task card。
- 成功条件和接触指标不能只写 TODO；可以是 mock 实现，但接口和字段必须完整。
- 添加 tests/test_minimal_tasks.py。

不要扩到 30 个任务；先把 5 个最小任务做稳定。
```

---

## 5. Dataset Prompt：实现 HDF5 数据协议

```text
请按照 docs/dataset_protocol.md 实现 dataset v0。

目标：trajectory 可以写入、读取、验证、replay。优先保证 schema 稳定和同步字段完整，不追求真实大数据量。

请实现：
- datasets/writer.py
- datasets/reader.py
- datasets/validate.py
- datasets/replay.py
- scripts/collect_demos.py
- scripts/replay_demos.py
- scripts/export_dataset.py
- 一个 dataset metadata JSON 模板
- 一个 dataset card 模板

每条 episode 至少包含：
- episode_id, task_name, suite_name, instruction, seed, timestamps
- front_rgb, wrist_rgb
- joint_pos, joint_vel, ee_pose, gripper_state
- force_left/right, wrench_left/right, contact_flag_left/right
- vt_rgb_left/right, vt_depth_left/right, force_field_left/right
- action, reward, success, contact_metrics, metadata

验收标准：
- 可以生成一个 tiny HDF5 demo dataset，例如 2 tasks x 2 episodes。
- `python scripts/validate_dataset.py --path <dataset>` 能通过。
- `python scripts/replay_demos.py --path <dataset>` 能读取并逐步 replay 或 mock replay。
- 添加 tests/test_dataset_io.py。
- 明确记录 frame rate、timestamp、坐标系、单位、缺失模态 mask。

不要添加 LeRobot/RLDS 导出到主线；如果要写，只放 optional stub。
```

---

## 6. Evaluation Prompt：实现接触感知评测

```text
请按照 docs/evaluation_protocol.md 实现 evaluation v0。

目标：统一评测不同 tactile mode 和 baseline，输出 success + contact-aware metrics + robustness split。

请实现：
- metrics/success.py
- metrics/contact.py
- metrics/assembly.py
- metrics/robustness.py
- scripts/evaluate.py
- configs/eval/default.yaml
- configs/eval/pose_noise.yaml
- configs/eval/occlusion.yaml
- configs/eval/low_clearance.yaml

必须支持这些指标：
- success rate
- completion time
- trajectory length
- max contact force
- mean contact force
- force violation rate
- contact duration
- contact loss count
- contact stability
- insertion depth
- jamming count
- recovery after collision

评测设置至少包括：
- no noise
- 2mm pose noise
- 5mm pose noise
- 10mm pose noise
- occlusion
- low clearance

验收标准：
- `python scripts/evaluate.py --task PegInsert --policy random --tactile force_wrench --episodes 3` 可运行。
- 输出 JSON/CSV 两种结果文件。
- 指标定义有 docstring，单位明确。
- 添加 tests/test_metrics.py 和 tests/test_evaluate_script.py。
- 不要只输出 success rate。
```

---

## 7. Baseline Prompt：实现公平 baseline 框架

```text
请按照 docs/baseline_protocol.md 实现 baseline v0。

目标：提供公平比较 vision-only、state、force、visuo-tactile 和 force+visuo-tactile 的 BC baseline 框架。先实现可训练的小模型或 mock trainer，接口稳定优先。

请实现：
- policies/base.py
- policies/random.py
- policies/replay.py
- policies/bc.py
- scripts/train_bc.py
- configs/policies/*.yaml

必须支持：
- RandomPolicy
- ReplayPolicy
- StateBC
- VisionBC
- VisionStateBC
- VisionForceBC
- VisionVisuoTactileBC
- VisionForceVisuoTactileBC
- OracleStateBC

公平性要求：
- 相同 train/val/test split
- 相同 action schema
- 相同训练步数或 epoch
- 相同随机种子列表
- 明确每个 baseline 可见的 observation keys
- 不允许 tactile baseline 额外看到 oracle state，除非是 OracleStateBC

验收标准：
- 可以用 tiny dataset 跑通 `python scripts/train_bc.py --policy VisionForceBC --dataset <path> --epochs 1`。
- 训练输出 checkpoint 和 config snapshot。
- evaluation 可以加载 checkpoint。
- 添加 tests/test_policies.py。
- 输出 baseline matrix，说明每个 policy 使用哪些模态。
```

---

## 8. Paper Experiment Prompt：生成论文实验表格和图

```text
请按照 docs/paper_plan.md、docs/evaluation_protocol.md 和 docs/baseline_protocol.md，整理 benchmark paper 的实验计划与结果表格模板。

目标：不要编造结果，只生成可填数的 LaTeX/Markdown 表格、plot 脚本和实验运行清单。

请生成：
- main result table template: task suite x policy x tactile mode
- robustness table template: no noise / 2mm / 5mm / 10mm / occlusion / low clearance
- contact metric table template: max force / violation rate / contact loss / jamming
- ablation table template: none / force / VT / force+VT
- scripts/plot_results.py，用已有 CSV 生成论文图
- experiments/run_matrix.yaml

验收标准：
- 不编造数值。
- 所有表格列名和 evaluation 输出字段一致。
- plot 脚本可以读取 scripts/evaluate.py 输出的 CSV。
- README 中说明如何复现实验矩阵。
```

---

## 9. Release Prompt：开源和 artifact review 包装

```text
请按照 docs/reproducibility_release.md 和 docs/checklist.md，补齐开源发布和 artifact review 所需文件。

请实现或更新：
- README.md
- LICENSE
- CITATION.cff
- pyproject.toml
- Dockerfile 或 environment.yml
- docs/install.md
- docs/quickstart.md
- docs/dataset_card.md
- docs/model_card.md
- docs/task_index.md
- .github/workflows/ci.yml

验收标准：
- README 有 5 分钟 quickstart。
- CI 在无 Isaac Sim 环境下至少能跑 registry/schema/dataset/metrics 的 mock tests。
- 明确哪些功能需要 Isaac Sim / Isaac Lab。
- 明确 license 和第三方资产限制。
- checklist.md 中 release 相关条目可追踪。
```

---

## 10. Review Prompt：让 AI 审稿人式检查项目

```text
请作为机器人学习 benchmark 论文的严苛审稿人，审视当前 Isaac-Tactile-LIBERO 仓库和 docs/ 计划文件。

请重点检查：
1. benchmark 定位是否清楚，是否和 LIBERO / Lightwheel 区分明确；
2. task suite 是否足以证明 tactile/contact-rich manipulation 的必要性；
3. observation/action/dataset schema 是否稳定且可复现；
4. evaluation 是否不仅仅是 success rate；
5. baseline 是否公平，是否存在某个模态偷看额外信息；
6. dataset 是否可 replay、可验证、可训练；
7. release 是否能通过 artifact review；
8. 当前最可能导致论文被拒的 10 个问题。

输出格式：
- Critical issues
- Major issues
- Minor issues
- Missing experiments
- Recommended next 5 tasks
- Go/no-go judgment for submission

不要泛泛而谈；请引用具体文件、类、脚本或配置。
```

---

## 11. 每次提交前的自检 Prompt

```text
请检查这次改动是否满足计划文档要求。

上下文：我刚完成一个 implementation slice。请阅读 git diff 和 docs/checklist.md。

请回答：
1. 这次改动对应 task.md 中哪些任务？
2. checklist.md 中哪些条目可以勾选？哪些不能？
3. 是否破坏 benchmark_spec.md 的接口契约？
4. 是否有测试覆盖？测试命令是什么？
5. 是否引入了不必要的大型依赖？
6. 是否有 mock 实现被误写成真实完成？
7. 是否需要更新 paper_plan.md 或 risk_register.md？

请给出必须修复项和可延后项。
```

---

## 12. 最推荐的第一条实际执行 Prompt

如果仓库还是空的，直接从这条开始：

```text
你是我的 Isaac-Tactile-LIBERO benchmark 工程助手。请先阅读 docs/task.md、docs/checklist.md、docs/benchmark_spec.md、docs/evaluation_protocol.md 和 docs/dataset_protocol.md。

当前目标：实现 Phase 1 的最小可运行 skeleton，不做真实 Isaac Sim 物理仿真。

请完成：
1. 创建 Python package 目录结构；
2. 实现 task/robot/tactile/policy 四个 registry；
3. 实现 BaseTactileSensor 和四种 tactile mode 的 mock 版本；
4. 实现统一 observation/action schema；
5. 注册 5 个最小任务的占位类：PressButton、SoftPress、PushSlider、PegInsert、PlugSocketInsert；
6. 实现 scripts/list_tasks.py；
7. 实现 scripts/smoke_test.py，让 5 个任务 x 4 个 tactile modes 都能跑一个 mock episode；
8. 添加 pytest 测试。

验收命令：
- python scripts/list_tasks.py
- python scripts/smoke_test.py --task PegInsert --tactile force_wrench --episodes 1
- pytest tests/

约束：
- 不要接入真实 Lightwheel。
- 不要改成 Lightwheel fork。
- 不要实现训练大模型。
- 不要扩展到 30 个任务。
- 所有 TODO 必须标注为 mock/stub，不要伪装成已完成仿真。

完成后请输出：改动摘要、文件列表、运行结果、未完成风险、下一步建议。
```

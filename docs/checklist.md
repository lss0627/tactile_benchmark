# checklist.md — Isaac-Tactile-LIBERO 投稿与开源检查表

## 0. 使用说明

每次 milestone review 前检查一次。投稿前必须完成 P0/P1 项。P2 是强投稿或 rebuttal 加分项。

标记方式：

```text
[ ] 未开始
[-] 进行中
[x] 完成
[!] 阻塞
```

---

# A. 论文定位与贡献

- [ ] 论文标题明确，不暗示只是 LIBERO migration。
- [ ] Abstract 清楚说明 benchmark 关注 language-conditioned contact-rich manipulation。
- [ ] Introduction 说明为什么 vision-only 在接触丰富任务下不足。
- [ ] Contributions 不超过 5 条，且每条都能用实验或 artifact 支撑。
- [ ] 明确 Lightwheel 是 optional backend / integration target。
- [ ] 明确 Base-LIBERO-Compatible suite 只是兼容对照，不是主贡献。
- [ ] 明确 Tactile-Assembly 和 Tactile-Contact 是核心 suite。
- [ ] 不宣称真实触觉 sensor 完全等价，避免 overclaim。
- [ ] Limitations 明确模拟触觉、物理近似、真实机器人迁移边界。

# B. Related Work 与差异化

- [ ] 有 related benchmark comparison table。
- [ ] 与 LIBERO 对比：语言条件、多任务/终身学习 vs 接触/触觉 benchmark。
- [ ] 与 CALVIN 对比：长程语言任务 vs tactile/contact-rich metrics。
- [ ] 与 ManiSkill / Meta-World / RoboSuite 对比：通用 manipulation vs tactile/contact protocol。
- [ ] 与 RoboCasa 对比：家庭任务场景 vs 触觉和接触质量。
- [ ] 与 tactile-specific works 对比：是否有语言条件、dataset、baseline、evaluation。
- [ ] 表格列包含：language, contact-rich, tactile force, visuo-tactile, dataset, replay, robustness, contact metrics, open-source。
- [ ] 贡献边界没有贬低现有工作。

# C. Benchmark contract

- [ ] 有 `BENCHMARK_VERSION`。
- [ ] 有 task registry。
- [ ] 有 robot registry。
- [ ] 有 tactile registry。
- [ ] 有 policy registry。
- [ ] `make_env(task, robot, tactile, split, seed)` 接口稳定。
- [ ] observation schema 固定。
- [ ] action schema 固定。
- [ ] metric schema 固定。
- [ ] config snapshot 会随每次训练和评测保存。
- [ ] 版本升级规则写清楚：任务、schema、metric、dataset 变更如何处理。

# D. FR3-Tactile embodiment

- [ ] joint names 固定。
- [ ] link names 固定。
- [ ] end-effector frame 固定。
- [ ] gripper frame 固定。
- [ ] left/right tactile mount frames 固定。
- [ ] front camera frame 固定。
- [ ] wrist camera frame 固定。
- [ ] state space 文档化。
- [ ] action space 文档化。
- [ ] action 单位、频率、clipping 文档化。
- [ ] robot reset smoke test 通过。
- [ ] random action rollout 通过。
- [ ] 无 NaN / exploding articulation / persistent penetration。

# E. Tactile sensor plugin

- [ ] `none` mode 可运行。
- [ ] `force_wrench` mode 可运行。
- [ ] `visuotactile` mode 可运行。
- [ ] `force_plus_visuotactile` mode 可运行。
- [ ] 每个 mode 不改变 action schema。
- [ ] 每个 mode 输出统一 tactile schema。
- [ ] mask 机制可处理缺失 modality。
- [ ] force 单位清楚。
- [ ] wrench 单位清楚。
- [ ] tactile image 分辨率清楚。
- [ ] coordinate frame 清楚。
- [ ] sensor noise model 清楚。
- [ ] latency model 清楚。
- [ ] saturation / clipping 清楚。
- [ ] contact flag threshold 清楚。
- [ ] sensor calibration metadata 存入 dataset。
- [ ] 不通过 tactile schema 泄漏 task success 或 object pose。

# F. Task suites

## F1. 总体任务要求

- [ ] 每个 task 有 task card。
- [ ] 每个 task 有 task id。
- [ ] 每个 task 有 suite id。
- [ ] 每个 task 有 language templates。
- [ ] 每个 task 有 reset distribution。
- [ ] 每个 task 有 assets 列表。
- [ ] 每个 task 有 success condition。
- [ ] 每个 task 有 failure condition。
- [ ] 每个 task 有 termination condition。
- [ ] 每个 task 有 metrics。
- [ ] 每个 task 有 robustness variants。
- [ ] 每个 task 有 smoke test。
- [ ] 每个 task 支持全部 tactile modes。
- [ ] 每个 task 明确是否 contact-rich。
- [ ] 每个 task 明确是否 tactile-necessary。
- [ ] 每个 task 有 train/test leakage 风险说明。

## F2. 最小 5 tasks

- [ ] PressButton。
- [ ] SoftPress。
- [ ] PushSlider。
- [ ] PegInsert。
- [ ] PlugSocketInsert。
- [ ] 5 tasks × 4 tactile modes × 3 seeds smoke test 通过。

## F3. 30-task v0

- [ ] Base-LIBERO-Compatible ≥ 5。
- [ ] Tactile-Contact ≥ 8。
- [ ] Tactile-Assembly ≥ 12。
- [ ] Tactile-Articulated ≥ 3，推荐 ≥ 6。
- [ ] Tactile-Long ≥ 2，推荐 ≥ 4。
- [ ] 每个 suite 至少有一个主图示例。
- [ ] 任务难度分布报告完成。
- [ ] 任务失败模式分类完成。

# G. Dataset

- [ ] HDF5 writer。
- [ ] HDF5 reader。
- [ ] metadata JSON。
- [ ] dataset version。
- [ ] schema version。
- [ ] episode id 唯一。
- [ ] task name / suite name / instruction / seed 完整。
- [ ] timestamps 单调递增。
- [ ] front RGB 完整。
- [ ] wrist RGB 完整。
- [ ] robot state 完整。
- [ ] action 完整。
- [ ] force/wrench 完整或 mask 正确。
- [ ] tactile image/depth/force field 完整或 mask 正确。
- [ ] reward/success 完整。
- [ ] contact metrics 完整。
- [ ] coordinate frames 文档化。
- [ ] units 文档化。
- [ ] sampling rate 文档化。
- [ ] compression 文档化。
- [ ] checksum/hash 完成。
- [ ] dataset validation script。
- [ ] replay script。
- [ ] replay success rate 达标。
- [ ] dataset card。
- [ ] train/val/test split 固定。
- [ ] OOD split 固定。
- [ ] no train/test object pose leakage。
- [ ] no language template leakage。
- [ ] 数据质量 audit 报告完成。
- [ ] P2：LeRobot export。
- [ ] P2：RLDS export。

# H. Evaluation protocol

- [ ] 每个 task 的 evaluation episodes 数固定。
- [ ] seeds 固定。
- [ ] split 固定。
- [ ] success rate 定义清楚。
- [ ] completion time 定义清楚。
- [ ] trajectory length 定义清楚。
- [ ] max force 定义清楚。
- [ ] force violation rate 定义清楚。
- [ ] contact duration 定义清楚。
- [ ] contact loss count 定义清楚。
- [ ] contact stability 定义清楚。
- [ ] insertion depth 定义清楚。
- [ ] jamming count 定义清楚。
- [ ] recovery rate 定义清楚。
- [ ] pose noise split 完成。
- [ ] occlusion split 完成。
- [ ] low-clearance split 完成。
- [ ] friction randomization split 完成。
- [ ] unseen geometry split 完成或标为 P2。
- [ ] per-task results。
- [ ] per-suite aggregate。
- [ ] overall aggregate。
- [ ] confidence interval。
- [ ] standard error。
- [ ] bootstrap 或 seed-level variance。
- [ ] evaluation config 随结果保存。
- [ ] 结果可由 `scripts/report_results.py` 复现。

# I. Baselines

- [ ] RandomPolicy。
- [ ] ReplayPolicy。
- [ ] StateBC。
- [ ] VisionBC。
- [ ] VisionStateBC。
- [ ] VisionForceBC。
- [ ] VisionVisuoTactileBC。
- [ ] VisionForceVisuoTactileBC。
- [ ] OracleStateBC。
- [ ] 每个 baseline 有训练命令。
- [ ] 每个 baseline 有 evaluation 命令。
- [ ] 每个 baseline 有 config。
- [ ] 每个 baseline 有 checkpoint。
- [ ] 每个 baseline 使用相同 train/val/test split。
- [ ] 每个 baseline 使用相同 action space。
- [ ] Vision-only 和 tactile baseline 训练预算一致。
- [ ] 视觉 backbone 一致。
- [ ] fusion 方式清楚。
- [ ] model selection 使用 val，不使用 test。
- [ ] oracle 输入清楚。
- [ ] 参数量报告。
- [ ] 训练时间/compute 报告。
- [ ] P2：ACT。
- [ ] P2：Diffusion Policy。
- [ ] P2：VLA wrapper。

# J. 结果表与主实验

- [ ] Main Table：no noise baseline comparison。
- [ ] Robustness Table：pose noise / occlusion / low clearance。
- [ ] Contact Quality Table：成功 episode 中的接触质量。
- [ ] Ablation Table：none / force / VT / force+VT。
- [ ] Dataset Statistics Table。
- [ ] Task Suite Summary Table。
- [ ] Related Benchmark Table。
- [ ] Robustness curve figure。
- [ ] Failure case figure。
- [ ] Contact force profile figure。
- [ ] 至少一个 qualitative video。
- [ ] 所有图表可由脚本生成。

# K. Reproducibility

- [ ] Dockerfile。
- [ ] Conda / uv / pip lock file。
- [ ] Isaac Sim / Isaac Lab version 固定。
- [ ] CUDA / driver 要求写清。
- [ ] `pip install -e .` 测试通过。
- [ ] `scripts/smoke_test.py` 测试通过。
- [ ] CI 跑 unit tests。
- [ ] CI 跑 schema tests。
- [ ] CI 跑 lightweight smoke tests。
- [ ] 随机种子统一管理。
- [ ] 所有 experiment configs 存档。
- [ ] 所有 logs 存档。
- [ ] 所有 checkpoints 存档。
- [ ] Dataset checksum 发布。
- [ ] Reproduction commands 写入 README。

# L. Release 与 artifact

- [ ] LICENSE。
- [ ] CITATION.cff。
- [ ] README install。
- [ ] README quickstart。
- [ ] README benchmark evaluation。
- [ ] README dataset download。
- [ ] Dataset card。
- [ ] Model cards。
- [ ] Project page。
- [ ] Demo videos。
- [ ] API docs。
- [ ] Leaderboard 规则。
- [ ] Asset provenance。
- [ ] Third-party license table。
- [ ] Known issues。
- [ ] Changelog。
- [ ] Release tag。
- [ ] Zenodo / DOI 或同等 archive。
- [ ] Artifact review appendix。

# M. 投稿文稿

- [ ] Abstract。
- [ ] Introduction。
- [ ] Related Work。
- [ ] Benchmark Design。
- [ ] Task Suites。
- [ ] Tactile Sensors。
- [ ] Dataset。
- [ ] Evaluation Protocol。
- [ ] Baselines。
- [ ] Experiments。
- [ ] Limitations。
- [ ] Ethics / Responsible release。
- [ ] Appendix A：task cards。
- [ ] Appendix B：dataset schema。
- [ ] Appendix C：metric definitions。
- [ ] Appendix D：baseline hyperparameters。
- [ ] Appendix E：additional results。
- [ ] Appendix F：artifact checklist。

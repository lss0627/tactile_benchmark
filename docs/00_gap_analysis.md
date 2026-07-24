# 投稿级缺口诊断

## 当前判断

Isaac Sim 6.0.1 迁移和 G0 仓库完整性已经完成，但项目还不是可投稿
的 generalization benchmark。缺口也不再定义成“补更多环境”，而是完成一套
可公平比较的闭环：

```text
Task Suite
+ Data Generation
+ Standard Dataset
+ Training Pipeline
+ Evaluation Protocol
+ Baseline Results
```

只有任务和 success rate 会与 UniVTAC 高度重合；真正的论文贡献应是训练和
测试分布如何构造、泛化如何测量、模型如何在相同数据和预算下比较。

## 论文主问题

> 接触丰富操作策略能否泛化到训练时未见的物体/几何、接触/物理条件和
> 传感器/观测条件？

对应三个 paper-v1 协议：

1. GP-01 Object and Geometry Generalization；
2. GP-02 Contact, Material, and Physics Generalization；
3. GP-03 Sensor and Observation Generalization。

## 最小投稿路线

```text
G1 PressButton reference
→ G2 public API and registries
→ G3 sensors and collection foundation
→ G4 4 suites / 16 tasks + official data + replay
→ G5 unified training + 3 protocols + evaluation toolkit
→ G6 baselines + static leaderboard + paper release
```

## 必须补齐

| 缺口 | Paper-v1 交付 |
|---|---|
| 参考任务未验收 | G1：100 resets、500 rendered steps、10 consecutive episodes |
| 接口和扩展点未冻结 | G2：environment/task/sensor/expert/policy registries |
| 采集平台缺失 | G3：批量采集、续跑、重试、过滤、统计、校验、自定义 adapter |
| 任务与 split 未落地 | G4：4 suites、16 tasks、GP-01/02/03 split manifests |
| 官方数据缺失 | 每任务至少 50 条 accepted train demos，总计至少 800 |
| 训练入口不统一 | BC、ACT、Diffusion、Transformer、UniVTAC-compatible |
| 模态比较不公平 | vision-only、tactile-only、fusion 共用 split/预算/预处理 |
| 泛化评测缺失 | seen/unseen、gap、Contact/force/slip/recovery/safety |
| 复现和展示缺失 | JSON/CSV/radar/HTML、result bundle、static leaderboard |

## 不再作为 paper-v1 前置

- 100 个任务；
- trajectory/task/scene/continual 的全部扩展协议；
- OpenVLA/π0 的正式结果；
- 托管不可信 checkpoint 的在线 leaderboard；
- 全机器人未执行轨迹的形式化几何证明；
- 真实机器人安全或 sim-to-real 结论。

它们都应保持接口兼容，但不能拖延完整的 16-task 论文闭环。

## 论文完整度标准

- 四个任务族，每族四个任务；
- 三个有明确 train/val/test 生成规则的泛化协议；
- 官方 offline dataset，同时支持 online collection/training；
- scripted/oracle 和五类 learned algorithm 配置；
- 三个 policy seeds；
- 每个 task condition、每个 seed 至少 20 次评测；
- 成功、效率、接触质量、恢复、模态缺失和泛化差距；
- 一条命令生成机器可读结果、论文图表和静态榜单。

这比堆叠大量相似场景更接近 LIBERO 的贡献方式，也能清楚区别于
“UniVTAC++”。

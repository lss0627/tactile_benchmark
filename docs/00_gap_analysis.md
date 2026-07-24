# 投稿级缺口诊断

## 当前判断

项目已经完成 Isaac Sim 6.0.1 迁移和大量底层诊断，但还不是可投稿 benchmark。主要缺口不是更多形式化几何证明，而是：

- 一个真正通过的参考任务；
- 冻结的公共 API 和触觉合同；
- 八个经过验收的 contact-rich 任务；
- 可验证、可 replay 的数据集；
- 公平的评估和 baseline；
- 可复现的论文 artifact。

## 最小投稿路线

```text
G1 PressButton
→ G2 API
→ G3 tactile
→ G4 8 tasks + dataset + replay
→ G5 evaluation
→ G6 baselines + release
```

## 核心论文问题

1. 触觉是否提高 contact-rich manipulation 成功率？
2. 哪些任务类型获益最大？
3. 触觉是否降低首次接触之后的失败？
4. 数据集 replay 与评测结果是否可复现？
5. 运行时无效、任务失败和传感器无效能否被清楚区分？

## 必须补齐

| 缺口 | 交付 |
|---|---|
| 参考任务未验收 | G1 100 resets、500 steps、10 episodes |
| 接口可能漂移 | G2 contract snapshots |
| 触觉 truth 未冻结 | G3 capability/mask contract |
| 任务规模未落地 | G4 8 task cards |
| 数据质量未知 | schema、duplicate、replay、dataset card |
| 评估公平性未知 | fixed splits/seeds/budgets/aggregation |
| 论文比较缺失 | scripted、visual、visual-tactile |
| 发布环境非参考驱动 | G6 reference-driver rerun |

## 不再作为投稿前置

- 每个动作的全机器人连续碰撞证明；
- 所有 collider pair 的穷举 GJK；
- PhysX 私有 cooked-shape/narrow-phase 权威证明；
- 30 或 130 个任务；
- 真实机器人安全认证。

这些可以成为后续方法研究或诊断工具，但不应阻塞 benchmark 主线。

## 推荐 paper-v0

- 八个任务；
- 每任务至少 50 条合格 demonstration，质量审查优先于数量；
- 三个训练 seed；
- 每任务每 seed 50 次评估；
- task success、runtime validity、safe retract、contact/tactile validity 和 failure taxonomy；
- 视觉与视觉+触觉的严格匹配比较。

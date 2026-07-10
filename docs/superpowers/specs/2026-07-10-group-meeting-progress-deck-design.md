# 组会项目进展汇报设计

## 目标与受众

面向课题组内部汇报，使用 10–12 分钟讲清 Isaac-Tactile-LIBERO 当前“做到哪里、证据是什么、为什么还不能进入 benchmark、下一步如何收敛”。输出为中文、16:9、可编辑 PPTX。

## 核心结论

项目已经从 mock/stub 演进到单任务 PressButton 的 Isaac Sim runtime-smoke 和真实 FR3 诊断控制链，并完成 30 mm full-press 诊断；但该成功仍来自几何 TCP 位移 proxy，回撤失败，且当前工作区不可从干净 checkout 复现，因此正式数据、评估、baseline 和论文结论全部保持阻塞。

## 页面结构

1. 封面：Isaac-Tactile-LIBERO 项目进展。
2. 一页结论：管线闭环、full press 诊断通过、benchmark 尚未开始。
3. 研究目标与 claim boundary：真正需要的是可复现、可审计、物理可信的触觉 benchmark。
4. 当前系统管线：task/runtime → controller → tactile contract → dataset/replay → evaluation/training。
5. 进展路线：mock → pusher → EE placeholder → FR3 load/control/IK → approach → full press → retract blocked。
6. Milestone A 数据：50-episode pusher 与 10-episode EE placeholder，强调 runtime-smoke 和 no-fake-force。
7. FR3 渐进集成：13 joints / local differential IK / near-contact 距离变化及仿真截图。
8. Full press 诊断：30 mm、0.03015 m proxy、无 NaN/无 safety abort；明确不是物理 task pass。
9. Retract 阻塞：proxy 由 30.2 mm 增至 488.3 mm，回撤目标未到达，数据门禁关闭。
10. 最新审计与 G0–G6 重建路线：规范 implementation-ready，实施尚未开始。
11. 下一步与讨论：先 G0，再 G1；定义可验证退出条件与外部依赖。

## 视觉系统

- 16:9；浅灰白底、深海军蓝正文，青绿色表示通过，琥珀表示诊断/限制，红色表示阻塞。
- 中文字体使用 Noto Sans CJK SC；标题 28–34 pt，正文 15–19 pt。
- 使用仓库截图，进行中心裁切和局部放大；所有数值页附带产物路径脚注。
- 用标签严格区分 `PASS_SMOKE`、`BLOCKED`、`NOT BENCHMARK`，不以装饰性图表替代证据。

## 内容边界

- 50/10 episode 的 100% 成功率只作为脚本化 runtime-smoke 稳定性，不作为策略性能。
- force/wrench 不可用；任何几何位移都不编码成 tactile force。
- Full press 的“成功”是 geometric press-depth proxy，不是 movable button joint state。
- 现有 screenshot/JSON 是诊断证据；正式 benchmark 要求 fresh evidence manifest 和干净 checkout。

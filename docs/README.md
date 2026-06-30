# Isaac-Tactile-LIBERO 补充文档包

本文件夹用于补全 `isaac_tactile_libero_benchmark_plan.md` 中已经明确的项目主线，使其更接近一篇可投稿、可复现、可审稿的 benchmark 论文工作计划。

原计划已经清楚定义了：

- 项目定位：面向 language-conditioned contact-rich manipulation 的 Isaac Sim / Isaac Lab 触觉 benchmark。
- 主要贡献：FR3-Tactile embodiment、force / visuo-tactile plugin、contact-rich task suites、dataset schema、contact-aware metrics、baselines。
- 开发路线：先做接口骨架，再做最小 tactile loop，再扩到 30 tasks、1500 demos、baseline evaluation。

本补充包重点补上审稿和开源发布会追问的内容：

1. benchmark 和现有工作的边界是否足够清楚；
2. 任务是否有严格 task card 和成功/失败定义；
3. 数据是否可验证、可同步、可 replay、可复现；
4. 评测协议是否公平、可统计、可防止 train/test leakage；
5. baseline 是否有统一训练预算和模型选择规则；
6. 触觉模拟是否有 calibration、noise、latency、saturation 等 realism 说明；
7. 代码、数据、模型、网页、license、CI 是否符合 benchmark artifact 要求。

## 文件说明

| 文件 | 用途 |
|---|---|
| `00_gap_analysis.md` | 对原计划进行投稿视角缺口诊断，说明还需要补充什么 |
| `task.md` | 可直接转成 GitHub issues / project board 的分阶段任务清单 |
| `checklist.md` | 投稿前、开源前、artifact review 前的总检查表 |
| `paper_plan.md` | 论文叙事、章节结构、主图主表、实验矩阵和审稿风险 |
| `benchmark_spec.md` | benchmark 的版本、接口、配置、命名、CLI 和兼容性契约 |
| `task_cards.md` | 每个任务必须填写的 task card 模板和示例 |
| `dataset_protocol.md` | 数据格式、同步、坐标系、质量验证、split、dataset card 和导出协议 |
| `evaluation_protocol.md` | 评测 split、seed、episode 数、统计显著性、聚合规则和 hidden test 规则 |
| `baseline_protocol.md` | baseline 训练、公平比较、超参、模型选择和 ablation 规则 |
| `reproducibility_release.md` | Docker/环境锁定、CI、release、license、project page、artifact review |
| `risk_register.md` | 论文和工程风险、严重性、缓解策略与 go/no-go 标准 |

## 建议使用方式

第一周不要先扩任务数量，而是先把这些规范文件放进仓库 `docs/` 或根目录：

```text
isaac-tactile-libero/
  README.md
  task.md
  checklist.md
  docs/
    00_gap_analysis.md
    paper_plan.md
    benchmark_spec.md
    task_cards.md
    dataset_protocol.md
    evaluation_protocol.md
    baseline_protocol.md
    reproducibility_release.md
    risk_register.md
```

然后按 `task.md` 的 P0 / P1 / P2 顺序推进。每完成一个模块，用 `checklist.md` 做验收。

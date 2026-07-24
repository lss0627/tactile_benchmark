# Documentation Guide

## Authoritative documents

- Project scope: `specs/001-benchmark-reconstruction/spec.md`
- Engineering plan: `specs/001-benchmark-reconstruction/plan.md`
- Active task list: `specs/001-benchmark-reconstruction/tasks.md`
- Gate rules: `specs/001-benchmark-reconstruction/acceptance.md`
- Runtime contract: `specs/001-benchmark-reconstruction/contracts/benchmark-runtime.md`
- Data/training contract: `specs/001-benchmark-reconstruction/contracts/data-training.md`
- Generalization/evaluation contract: `specs/001-benchmark-reconstruction/contracts/generalization-evaluation.md`
- Rebaseline decision: `specs/001-benchmark-reconstruction/g1-benchmark-rebaseline.md`
- Product rebaseline: `specs/001-benchmark-reconstruction/tactilibero-generalization-rebaseline.md`

## User-facing documents

- `installation.md`
- `benchmark_spec.md`
- `current_project_state.md`
- `single_task_real_backend_plan.md`
- `press_button_runtime_loop.md`
- `tactile_sensor_contract.md`
- `data_collection_protocol.md`
- `dataset_protocol.md`
- `generalization_protocol.md`
- `evaluation_protocol.md`
- `baseline_protocol.md`
- `paper_plan.md`
- `reproducibility_release.md`
- `ai_prompts.md`

## Historical documents

Files describing old G1 C1/C2a/C2b/C3, full-sweep, GJK, cooked-shape, geometry-authority, or failed-attempt investigations are historical engineering records.

They remain useful for debugging and optional diagnostics, but they do not define the active G1 acceptance path. If a historical document conflicts with the active specification or acceptance file, the active documents win.

## Documentation rule

Every paper-facing statement must identify the Gate and evidence that support it. No document may imply that G0 or an optional diagnostic proves task, dataset, evaluation, or baseline performance.

Paper-v1 is one complete platform: four suites/16 tasks, official offline
data, online collection and training, a unified trainer, three core
generalization protocols, baseline results, and a static leaderboard. A scene
collection without these interfaces is not the claimed benchmark.

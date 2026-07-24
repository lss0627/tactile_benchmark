# Task Cards

## Paper-v0 suite target

Eight contact-rich tasks:

| ID | Working task | Contact skill | Status |
|---|---|---|---|
| ITL-01 | PressButton | axial press/release | G1 reference |
| ITL-02 | PegInsert | alignment/insertion | proposed |
| ITL-03 | SlideToStop | surface sliding | proposed |
| ITL-04 | OpenDrawer | articulated pull | proposed |
| ITL-05 | CloseLid | articulated push/closure | proposed |
| ITL-06 | GraspPlaceContact | grasp/place/contact | proposed |
| ITL-07 | ToolSurfaceTrace | tool/surface interaction | proposed |
| ITL-08 | ContactSequence | multi-stage contact | proposed |

Names and assets remain provisional until G4 task-card approval. The suite count and diversity target are authoritative; unvalidated tasks cannot be presented as accepted.

## Required card fields

```yaml
task_id: ITL-01
task_version: 1.0.0
language_instruction: Press and release the button.
robot: fr3
assets: []
initial_state_distribution: {}
action_contract_version: benchmark.action.7d.v1
observation_contract_version: benchmark.observation.v1
success_predicate: {}
release_predicate: {}
budgets: {}
required_capabilities: []
```

Each card must specify:

- asset path/digest/license;
- reset distribution and seed behavior;
- object/mechanism state;
- language instruction/templates;
- task-state success and failure;
- release/retract requirement;
- step/time/action budgets;
- camera/Contact/tactile requirements;
- randomization and split eligibility;
- known limitations.

## Acceptance

A task is accepted only when:

- assets and licenses resolve;
- reset is stable;
- task-state success is observable;
- scripted feasibility passes;
- no geometric fallback is used;
- sensors and masks satisfy contracts;
- failure evidence is retained;
- task card and evidence are versioned.

## PressButton card boundary

ITL-01 passes only after G1:

- 100 resets;
- 500 rendered steps;
- 10 consecutive press/release/retract episodes;
- truthful Contact and media evidence.

Historical smoke success does not accept the task.

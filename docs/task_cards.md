# Task Cards

## Paper-v1 suite

| ID | Suite | Working task | Primary contact skill |
|---|---|---|---|
| TL-P01 | Precision | PegInsert | alignment/insertion |
| TL-P02 | Precision | USBLikeInsert | keyed connector insertion |
| TL-P03 | Precision | KeyInsert | precision orientation/insertion |
| TL-P04 | Precision | PinSocketInsert | small-clearance insertion |
| TL-A01 | Articulation | PressButton | press/release |
| TL-A02 | Articulation | ToggleSwitch | directional toggle |
| TL-A03 | Articulation | OpenDrawer | constrained pull |
| TL-A04 | Articulation | TwistCap | rotational contact |
| TL-S01 | Surface Interaction | SlideToTarget | frictional sliding |
| TL-S02 | Surface Interaction | WipeSurface | coverage/contact maintenance |
| TL-S03 | Surface Interaction | ScrapeSurface | tool/surface force control |
| TL-S04 | Surface Interaction | SurfaceFollow | contour following |
| TL-D01 | Deformable Contact | SoftPress | compliant pressing |
| TL-D02 | Deformable Contact | SpongeCompress | deformation control |
| TL-D03 | Deformable Contact | FabricManipulate | distributed soft contact |
| TL-D04 | Deformable Contact | CableSeat | deformable insertion/seating |

Names and assets are working identifiers until G4. The count, suite balance,
and generalization roles are authoritative paper-v1 requirements.

## Required card fields

```yaml
task_id: TL-A01
task_version: 1.0.0
suite_id: articulation
language_instruction: Press and release the button.
robot_id: fr3
sensor_profile_id: tactile_default
asset_manifest: []
reset_distribution: {}
randomization_factors: {}
phase_labels: []
reward_definition: {}
success_predicate: {}
failure_predicates: []
budgets: {}
protocol_eligibility:
  - GP-01
  - GP-02
  - GP-03
```

Each card also binds units, frames, observation/action versions, object and
material identities, train/validation/test generation, sensor requirements,
licenses, and limitations.

## Acceptance

A task is accepted only when:

- assets/licenses resolve and hashes match;
- deterministic seeded reset passes;
- task-state success/failure and phase labels are observable;
- scripted feasibility and bounded failure paths pass;
- sensor sources/masks/timestamps are truthful;
- protocol variants and leakage rules validate;
- collection and replay work through public interfaces;
- evidence and task card are fresh and versioned.

PressButton remains the G1 reference task and does not accept the other 15
tasks by implication.

# Baseline Framework Skeleton

This phase adds baseline policy interfaces only. No model is trained, no
checkpoint is learned, and no result from these BC skeletons is a benchmark or
paper performance number.

## Policies

| Policy | Allowed modalities | Notes |
| --- | --- | --- |
| `state_bc` | robot state | Low-dimensional untrained BC skeleton. No RGB, tactile, language, or oracle state. |
| `vision_bc` | language, vision | Vision-only skeleton over front/wrist RGB plus language. No robot state or tactile. |
| `vision_state_bc` | language, vision, robot state | Vision plus proprioceptive robot state. No tactile or oracle state. |
| `vision_force_bc` | language, vision, robot state, force/wrench | Adds force and wrench only. No visuotactile images or oracle state. |
| `vision_vt_bc` | language, vision, robot state, visuotactile | Adds tactile RGB/depth/force-field slots only. No force/wrench or oracle state. |
| `vision_force_vt_bc` | language, vision, robot state, force/wrench, visuotactile | Full non-oracle multimodal tactile skeleton. |
| `oracle_state_bc` | robot state, oracle state | Privileged upper-bound mock skeleton. It is excluded from fair main results. |

All skeletons declare:

- `policy_name`
- `required_observation_keys`
- `allowed_modalities`
- `forbidden_modalities`
- `uses_oracle_state`
- `uses_tactile_force`
- `uses_visuotactile`
- `action_schema_version`
- `is_trainable`
- `is_trained`
- `mock_or_stub`

## Action Behavior

The current BC skeletons return deterministic zero actions validated against the
stable 7D action schema. Each call records metadata with:

- `untrained_mock_policy: true`
- `is_trained: false`
- `mock_or_stub: true`

This is intentional. The skeleton proves policy registration, observation
filtering, batch construction, and evaluation plumbing. It does not implement
behavior cloning optimization.

## Observation Filtering

`isaac_tactile_libero.policies.observation_filter` enforces modality boundaries
before a skeleton policy reads an observation. Non-oracle baselines never receive
`oracle_state`. Vision-only baselines never receive robot state or tactile
fields. Tactile variants receive only the tactile branch they declare.

The public observation schema is not changed. OracleStateBC gets a clearly
marked mock privileged slot inside the filtering layer only, so future real
backends can replace that source without changing the policy/evaluate API.

## Batch Builder

`isaac_tactile_libero.policies.batch_builder` reads HDF5 episodes through
`HDF5DatasetReader`, filters observations according to the selected baseline
spec, checks action shape `(7,)`, and checks tactile masks against required
modalities. It returns a mock batch dictionary for inspection and future
`train_bc.py` protocol work.

It does not train, optimize, export LeRobot/RLDS, or create a learned
checkpoint.

## Fairness Boundary

The fair non-oracle comparison set excludes `oracle_state_bc`. It may be used
only as a privileged upper-bound sanity line. Mock BC evaluation is an interface
sanity check and must not be presented as paper experimental evidence.

Future real training should reuse these policy names, modality declarations,
observation filters, batch-builder contract, and evaluate entry points while
replacing the zero-action stub with actual small baseline models.

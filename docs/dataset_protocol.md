# Standard Dataset Protocol

## Goal

Release a trainable, replayable, leakage-audited official dataset for all 16
paper-v1 tasks while preserving online collection compatibility.

## Minimum scale

```text
accepted task instances: 16
accepted training demonstrations per task: >= 50
accepted training demonstrations total: >= 800
validation data: declared per suite/protocol
test-only variant demonstrations in training: 0
```

Counts are acceptance minima, not permission to lower quality requirements.

## Episode contents

Each episode stores:

- suite/task/version/instruction/seed/split/protocol;
- object, material, trajectory, contact, sensor, and randomization IDs;
- simulator/runtime/source/config/asset hashes;
- requested and executed actions;
- joint state and end-effector pose;
- RGB/depth and tactile observations;
- Contact/raw Contact and source/validity/freshness;
- scalar force, vector force, wrench, and raw impulse as distinct masked fields;
- task phase/reward, success/failure, termination, and timestamps;
- expert/policy provenance and collection-job identity.

Privileged state used for replay or metrics is excluded from policy
observations unless explicitly declared by the algorithm contract.

## Splits

Every protocol provides immutable manifests for:

- `train`;
- `validation`;
- `test_seen`;
- `test_unseen`.

The leakage audit checks object/geometry, material/physics, sensor/observation,
seed, trajectory, and episode hashes. Unseen membership is generated from
declared factors, never assigned after inspecting model performance.

## Validation

The validator rejects or reports:

- missing keys, shape/dtype/unit/frame errors;
- non-finite values and invalid mask encodings;
- timestamp gaps or excessive sensor skew;
- duplicate episodes or trajectories;
- split leakage;
- absent randomization provenance;
- invalid runtime or untruthful success;
- replay divergence beyond the task tolerance.

## Replay

Replay restores the recorded reset/randomization and executes recorded actions
in Isaac Sim. It records outcome agreement, state divergence, timing skew,
Contact alignment, first divergence, and failure codes.

## Offline and online relationship

- **Offline benchmark**: download the official frozen dataset and train under
  fixed splits/budgets.
- **Online benchmark**: collect new data and interact with the same registered
  tasks while retaining the same episode and evaluation contracts.

User-collected data is not silently merged with the official dataset. It
receives a new dataset manifest and is reported separately.

## Dataset card

The release card records intended use, versions, tasks, protocols, modalities,
collection policies, rejection statistics, splits, replay results, licenses,
asset provenance, known biases, limitations, citation, and contact.

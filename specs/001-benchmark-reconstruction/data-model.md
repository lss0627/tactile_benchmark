# Data Model

## 1. TaskCard

Required fields:

```yaml
task_id: string
task_version: semver
language_instruction: string
robot: string
assets:
  - path: string
    sha256: string
    license_id: string
initial_state_distribution: object
action_contract_version: string
observation_contract_version: string
success_predicate: object
release_predicate: object | null
budgets: object
required_capabilities: [string]
```

Validation:

- stable ID/version;
- assets resolve and hashes match;
- success is task-state based;
- budgets are positive and finite;
- capability requirements are explicit.

## 2. RuntimeMetadata

```yaml
simulator: "6.0.1"
python: string
driver: string
driver_validation: VALIDATED | UNVALIDATED
gpu: string
physics_device: cpu | gpu
broadphase_type: string
gpu_dynamics: bool
native_gpu_contact: bool
rendering_backend: string
source_commit: string
repository_dirty: bool
```

## 3. ActionRecord

```yaml
requested_7d: [float, float, float, float, float, float, float]
executed_7d: [float, float, float, float, float, float, float]
translation_frame: string
rotation_representation: string
gripper_semantics: string
clipped: bool
accepted: bool
failure_code: string | null
```

Both vectors must be finite and exactly length seven.

## 4. ObservationRecord

```yaml
episode_id: string
step_index: integer
physics_step: integer
timestamp_s: float
robot_state: object
task_state: object
rgb_ref: string | null
depth_ref: string | null
tactile_ref: string | null
contact: ContactRecord
validity_masks: object
```

Every field declares or inherits shape, dtype, units, frame, and source.

## 5. ContactRecord

```yaml
reading_valid: bool
fresh: bool
in_contact: bool
force_magnitude: float | null
force_magnitude_valid: bool
force_vector: [float, float, float] | null
force_vector_valid: bool
wrench: [float, float, float, float, float, float] | null
wrench_valid: bool
raw_contact_valid: bool
raw_contacts:
  - body0: string
    body1: string
    position: [float, float, float] | null
    normal: [float, float, float] | null
    impulse: [float, float, float] | null
    time_s: float | null
    physics_step: integer | null
raw_impulse_used_as_force: false
```

No invalid field may be populated with a proxy.

## 6. EpisodeRecord

```yaml
episode_id: string
task_id: string
task_version: string
seed: integer
reset_record_sha256: string
runtime_metadata_sha256: string
steps: [ObservationRecord]
pressed: bool
released: bool
safe_retract: bool
success: bool
terminated: bool
truncated: bool
failure_codes: [string]
post_abort_actuation_count: integer
wall_time_s: float
source_digests: object
```

Success requires task-state press, required release, and safe retract.

## 7. ResetCycleRecord

```yaml
cycle_index: integer
seed: integer
ready_within_window: bool
articulation_valid: bool
button_reset: bool
contact_handles_valid: bool
camera_valid: bool
stale_handle_count: integer
cleanup_success: bool
failure_codes: [string]
```

## 8. DatasetManifest

```yaml
dataset_id: string
dataset_version: string
task_cards: [string]
episode_count: integer
accepted_episode_count: integer
rejected_episode_count: integer
duplicate_count: integer
splits: object
schema_version: string
source_digests: object
license_summary: object
```

## 9. ReplayRecord

```yaml
episode_id: string
outcome_match: bool
task_state_error: object
timing_skew: object
contact_alignment: object
first_divergence_step: integer | null
failure_codes: [string]
```

## 10. EvaluationEpisode

```yaml
baseline_id: string
seed: integer
task_id: string
episode_id: string
task_success: bool
runtime_valid: bool
safe_retract: bool
contact_valid_rate: float
tactile_valid_rate: float | null
episode_steps: integer
wall_time_s: float
failure_codes: [string]
```

## 11. GateEvidence

The existing manifest/status schemas remain authoritative. A Gate evidence bundle binds:

- Gate/status/claim class;
- evidence-producing commit;
- runtime metadata;
- source/config/task/asset/dataset hashes;
- artifact checksums;
- acceptance results;
- blockers;
- freshness review.

Optional diagnostic evidence uses `runtime_smoke` and cannot satisfy a required Gate item alone.

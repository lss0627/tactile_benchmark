# Data Model

## TaskFamily

```yaml
family_id: string
version: semver
suite_id: string
skill_tags: [string]
generator_config_sha256: string
```

## TaskInstance

```yaml
task_id: string
task_version: semver
family_id: string
suite_id: string
language_instruction: string
assets: [object]
robot_config: string
sensor_configs: [string]
reset_distribution: object
randomization_schema: object
success_predicate: object
failure_predicates: [object]
reward_or_phase_schema: object
budgets: object
split_eligibility: object
protocol_eligibility: [string]
card_sha256: string
```

State:

```text
DRAFT → GENERATED → REVIEWED → FEASIBLE → ACCEPTED → DEPRECATED
```

## SuiteManifest

```yaml
suite_id: precision | articulation | surface_interaction | deformable_contact
suite_version: semver
task_ids: [string]
required_task_count: 4
coverage_summary: object
manifest_sha256: string
```

## DomainVariant

```yaml
variant_id: string
task_id: string
split_candidate: string
object_geometry: object
contact_material_physics: object
sensor_observation: object
trajectory_scene: object
seed: integer
variant_sha256: string
```

## SensorDomain

```yaml
sensor_domain_id: string
sensor_family: string
model_version: string
geometry_sha256: string
calibration_sha256: string
resolution: [integer, integer]
rate_hz: float
latency_model: object
noise_model: object
drift_model: object
drop_model: object
preprocessing_sha256: string
capabilities: object
```

## ProtocolDefinition

```yaml
protocol_id: GP-01 | GP-02 | GP-03
protocol_version: semver
hypothesis: string
train_query: object
validation_query: object
test_seen_query: object
test_unseen_query: object
forbidden_overlap_fields: [string]
adaptation_regime: OFFLINE | ONLINE | ZERO_SHOT | CALIBRATION_ONLY | TASK_ADAPTATION
metrics: [string]
evaluation_seeds: [integer]
episodes_per_condition_per_seed: integer
definition_sha256: string
```

## SplitManifest

```yaml
protocol_id: string
train_variants: [string]
validation_variants: [string]
test_seen_variants: [string]
test_unseen_variants: [string]
manifest_sha256: string
```

## LeakageAudit

```yaml
protocol_id: string
split_manifest_sha256: string
checks: object
violation_count: integer
violations: [object]
passed: bool
audit_sha256: string
```

## ExpertAdapter

```yaml
expert_id: string
expert_type: SCRIPTED | CONTROLLER | TELEOP | TRAINED_POLICY | HUMAN | CUSTOM
version: string
action_contract_version: string
required_observations: [string]
checkpoint_or_config_sha256: string | null
source_and_license: object
```

## CollectionJob

```yaml
job_id: string
suite_ids: [string]
task_ids: [string]
protocol_split: string
expert_id: string
requested_episodes: integer
num_parallel_envs: integer
retry_policy: object
retention_policy: object
seed_schedule: [integer]
progress_journal: string
completed_episode_ids: [string]
statistics: object
job_sha256: string
```

State:

```text
PLANNED → RUNNING → INTERRUPTED → RESUMED → VALIDATED → PROMOTED
                              ↘ FAILED
```

## DemonstrationEpisode

```yaml
episode_id: string
collection_job_id: string
expert_id: string
task_id: string
variant_id: string
split: TRAIN | VALIDATION
sensor_domain_id: string
seed: integer
randomization_parameters: object
runtime_metadata: object
visual_observations: object
tactile_observations: object
joint_state: object
end_effector_pose: object
actions: object
contact_and_force: object
task_phase_or_reward: object
task_state: object
timestamps: object
success: bool
runtime_valid: bool
failure_codes: [string]
source_digests: object
```

## ReplayRecord

```yaml
episode_id: string
outcome_match: bool
task_state_error: object
timing_error: object
contact_alignment: object
first_divergence_step: integer | null
runtime_valid: bool
failure_codes: [string]
record_sha256: string
```

## DatasetManifest

```yaml
dataset_id: string
dataset_version: semver
task_ids: [string]
train_episode_ids: [string]
validation_episode_ids: [string]
test_training_episode_count: 0
collection_job_digests: [string]
schema_version: string
split_manifest_digests: object
validation_report_sha256: string
replay_report_sha256: string
manifest_sha256: string
```

## CommunityPlugin

```yaml
plugin_id: string
plugin_type: ROBOT | SENSOR | TASK | EXPERT | MODALITY
version: semver
entry_point: string
supported_contract_versions: [string]
capabilities: object
source_and_license: object
test_report_sha256: string
```

## TrainingConfig

```yaml
algorithm: BC | ACT | DIFFUSION | TRANSFORMER | UNIVTAC
suite_ids: [string]
task_ids: [string]
modalities: [VISION, TACTILE, PROPRIO]
dataset_manifest_sha256: string
split_manifest_sha256: string
normalization_sha256: string
observation_horizon: integer
action_horizon: integer
seed: integer
optimizer_and_schedule: object
training_budget: object
validation_selection: object
config_sha256: string
```

## TrainingRun

```yaml
run_id: string
training_config_sha256: string
source_commit: string
runtime_and_compute: object
checkpoints: [object]
best_checkpoint_sha256: string
selection_evidence: object
logs_sha256: string
status: string
```

## PolicyCapability

```yaml
policy_id: string
policy_version: string
algorithm_family: string
modalities: [string]
action_contract_versions: [string]
context_length: integer
action_horizon: integer
supported_protocols: [string]
adaptation_regimes: [string]
model_and_compute: object
manifest_sha256: string
```

## EvaluationCell

```yaml
protocol_id: string
split: TEST_SEEN | TEST_UNSEEN
task_id: string
variant_id: string
sensor_domain_id: string
policy_seed: integer
episode_seed: integer
identity_sha256: string
```

## EpisodeResult

```yaml
evaluation_run_id: string
policy_id: string
cell_identity_sha256: string
data_regime: OFFLINE | ONLINE
task_success: bool
runtime_valid: bool
safe_retract: bool
completion_time_s: float
action_smoothness: float
trajectory_efficiency: float
contact_metrics: object
force_metrics: object
slip_metrics: object
recovery_metrics: object
failure_codes: [string]
record_sha256: string
```

## GeneralizationAggregate

```yaml
protocol_id: string
policy_id: string
data_regime: OFFLINE | ONLINE
seen_metrics: object
unseen_metrics: object
generalization_gap: object
modality_drop_degradation: object
seed_statistics: object
missing_invalid_counts: object
source_episode_digest: string
aggregate_sha256: string
```

## ResultBundle

```yaml
bundle_version: string
benchmark_version: string
policy_capability_sha256: string
protocol_definition_sha256: string
split_manifest_sha256: string
leakage_audit_sha256: string
episode_result_digest: string
aggregate_digest: string
runtime_metadata: object
checksums_sha256: string
```

State:

```text
CREATED → VALIDATED → ACCEPTED → PUBLISHED
                     ↘ REJECTED
```

## LeaderboardEntry

```yaml
entry_id: string
bundle_sha256: string
policy_id: string
protocol_id: string
data_regime: OFFLINE | ONLINE
seen_success: float
unseen_success: float
generalization_gap: float
runtime_valid_rate: float
contact_recovery_score: float | null
publication_metadata: object
```

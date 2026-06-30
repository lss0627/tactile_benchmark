# task_cards.md — Task card 模板与示例

## 1. 为什么必须有 task card

Benchmark 论文最容易被质疑的点不是“任务名字不够多”，而是：

- 成功条件是否严谨；
- 随机化是否可复现；
- train/test split 是否泄漏；
- 任务是否真的需要接触；
- 指标是否能反映失败模式；
- 不同方法是否在同一任务定义下评测。

因此每个任务都必须有 task card。没有 task card 的任务不能进入 v0 benchmark。

---

## 2. 标准 task card 模板

```yaml
task_id:
task_name:
suite_name:
version:

short_description:
why_contact_rich:
why_tactile_matters:

robot:
  name: fr3_tactile
  end_effector:
  gripper:

assets:
  objects:
  articulated_objects:
  fixtures:
  textures:
  license:

language:
  templates:
    - ""
  paraphrase_rules:
  prohibited_templates:
  train_templates:
  test_templates:

reset_distribution:
  object_pose:
  target_pose:
  robot_initial_pose:
  distractors:
  friction:
  mass:
  lighting:
  camera:
  train_range:
  test_seen_range:
  test_ood_range:

success_condition:
  type:
  formula:
  threshold:
  duration_required:
  notes:

failure_condition:
  max_steps:
  excessive_force:
  object_drop:
  workspace_violation:
  irreversible_jam:
  other:

termination:
  success_terminates: true
  failure_terminates: true
  timeout_steps:

observations:
  required_rgb:
  required_state:
  required_tactile:
  forbidden_privileged_state:

actions:
  schema: 7D_delta_ee_pose_gripper
  control_frequency_hz:
  clipping:

metrics:
  primary:
  contact:
  assembly:
  robustness:
  safety:

robustness_variants:
  pose_noise:
  occlusion:
  low_clearance:
  friction_randomization:
  unseen_geometry:

dataset:
  demos_required:
  replay_success_threshold:
  quality_checks:

unit_tests:
  reset_test:
  random_rollout_test:
  scripted_oracle_test:
  metric_test:

leakage_risks:
  - risk:
    mitigation:

known_failure_modes:
  - mode:
    expected_metric_signature:

status:
  owner:
  implementation_status:
  last_validated:
```

---

## 3. 示例：PegInsert

```yaml
task_id: tactile_assembly.peg_insert.v0
task_name: PegInsert
suite_name: tactile_assembly
version: 0.1.0

short_description: Insert a cylindrical peg into a matching hole.
why_contact_rich: The peg must make contact with the rim and recover from small misalignment.
why_tactile_matters: Force and contact patterns help detect rim contact, jamming, and insertion progress under pose noise.

robot:
  name: fr3_tactile
  end_effector: gripper
  gripper: parallel_jaw_with_left_right_tactile

assets:
  objects:
    - round_peg
    - insertion_board
  articulated_objects: []
  fixtures:
    - table
  textures:
    - neutral
  license: to_be_filled

language:
  templates:
    - "insert the peg into the hole"
    - "place the round peg into the socket"
    - "push the peg down into the matching hole"
  paraphrase_rules: keep object and target explicit
  prohibited_templates:
    - templates revealing exact pose or coordinates
  train_templates:
    - first two templates
  test_templates:
    - paraphrases not used in training

reset_distribution:
  object_pose:
    x: [-0.05, 0.05]
    y: [-0.05, 0.05]
    yaw_deg: [-15, 15]
  target_pose:
    fixed_in_train: true
    test_pose_noise_mm: [2, 5, 10]
  robot_initial_pose:
    nominal_with_small_joint_noise: true
  distractors: none
  friction:
    train: nominal
    test: randomized_optional
  mass: nominal
  lighting: nominal
  camera: nominal
  train_range: small object pose variation
  test_seen_range: same as train but held-out seeds
  test_ood_range: pose noise and low-clearance variants

success_condition:
  type: insertion_depth
  formula: peg_bottom_z < target_depth_threshold and lateral_error < threshold
  threshold:
    insertion_depth_mm: 25
    lateral_error_mm: 3
  duration_required: 10 consecutive control steps
  notes: success must not be triggered by object teleportation

failure_condition:
  max_steps: 300
  excessive_force:
    force_norm_n: 50
  object_drop: true
  workspace_violation: true
  irreversible_jam: true
  other: peg knocked off board

termination:
  success_terminates: true
  failure_terminates: true
  timeout_steps: 300

observations:
  required_rgb:
    - front
    - wrist
  required_state:
    - joint_pos
    - ee_pose
    - gripper_state
  required_tactile:
    - force_left
    - force_right
    - wrench_left
    - wrench_right
    - optional_vt
  forbidden_privileged_state:
    - exact hole pose in policy input
    - insertion depth in policy input

actions:
  schema: 7D_delta_ee_pose_gripper
  control_frequency_hz: 20
  clipping: benchmark_default

metrics:
  primary:
    - success_rate
    - insertion_depth
  contact:
    - max_contact_force
    - force_violation_rate
    - contact_duration
  assembly:
    - jamming_count
    - failed_insertion_attempts
    - recovery_after_collision
  robustness:
    - success_under_pose_noise
  safety:
    - excessive_force_rate

robustness_variants:
  pose_noise:
    - 2mm
    - 5mm
    - 10mm
  occlusion: wrist_camera_partial
  low_clearance:
    - nominal
    - tight
  friction_randomization: optional
  unseen_geometry: optional

dataset:
  demos_required: 50
  replay_success_threshold: 0.95
  quality_checks:
    - no timestamp gaps
    - no force saturation unless labeled
    - no invalid tactile frames

unit_tests:
  reset_test: reset 100 seeds
  random_rollout_test: 20 random rollouts no crash
  scripted_oracle_test: scripted insertion succeeds >= 80%
  metric_test: synthetic jam triggers jamming_count

leakage_risks:
  - risk: policy sees target pose through state
    mitigation: exclude target pose from policy observation; keep only in simulator info
  - risk: tactile force threshold exactly reveals success
    mitigation: success requires geometry condition, not just force

known_failure_modes:
  - mode: rim collision
    expected_metric_signature: high lateral force, no insertion depth increase
  - mode: jammed insertion
    expected_metric_signature: high normal force, repeated failed attempts

status:
  owner: TBD
  implementation_status: todo
  last_validated: TBD
```

---

## 4. 示例：SoftPress

```yaml
task_id: tactile_contact.soft_press.v0
task_name: SoftPress
suite_name: tactile_contact
version: 0.1.0

short_description: Press a compliant pad to a target force range without exceeding a safety limit.
why_contact_rich: Success depends on controlled contact force, not just reaching a pose.
why_tactile_matters: Vision cannot reliably infer exact normal force after contact.

language:
  templates:
    - "press the soft pad gently"
    - "apply light pressure to the blue pad"
    - "touch the soft button without pressing too hard"

success_condition:
  type: force_tracking
  formula: target_force_min <= normal_force <= target_force_max for K consecutive steps
  threshold:
    target_force_min_n: 2.0
    target_force_max_n: 5.0
    hard_limit_n: 8.0
  duration_required: 15 consecutive control steps

failure_condition:
  max_steps: 200
  excessive_force:
    force_norm_n: 8.0
  object_drop: false
  workspace_violation: true
  irreversible_jam: false

metrics:
  primary:
    - success_rate
    - force_tracking_error
  contact:
    - max_contact_force
    - force_violation_rate
    - contact_duration
    - contact_stability
  safety:
    - hard_limit_exceedance
```

---

## 5. 示例：OpenDrawerWithForceLimit

```yaml
task_id: tactile_articulated.open_drawer_force_limit.v0
task_name: OpenDrawerWithForceLimit
suite_name: tactile_articulated
version: 0.1.0

short_description: Pull a drawer open while maintaining handle contact and staying below force limit.
why_contact_rich: The robot must establish and maintain contact with the handle during pulling.
why_tactile_matters: Tactile feedback helps detect handle slip and excessive pulling force.

success_condition:
  type: articulated_state
  formula: drawer_joint_position >= open_threshold and max_force <= force_limit
  threshold:
    open_distance_m: 0.18
    force_limit_n: 35

failure_condition:
  max_steps: 350
  excessive_force:
    force_norm_n: 35
  contact_loss:
    max_allowed_count: 3
  workspace_violation: true

metrics:
  primary:
    - success_rate
    - opening_distance
  contact:
    - handle_contact_success
    - contact_loss_count
    - force_profile
  safety:
    - force_violation_rate
```

---

## 6. 示例：PickAlignInsert

```yaml
task_id: tactile_long.pick_align_insert.v0
task_name: PickAlignInsert
suite_name: tactile_long
version: 0.1.0

short_description: Pick up an object, align it with a socket, and insert it.
why_contact_rich: The task combines grasping, alignment, contact recovery, and insertion.
why_tactile_matters: Tactile cues are needed during grasp stability and final insertion.

language:
  templates:
    - "pick up the peg, align it, and insert it into the hole"
    - "grasp the connector and plug it into the socket"

success_condition:
  type: sequential_subgoals
  subgoals:
    - grasp_success
    - alignment_with_target
    - insertion_depth
  threshold:
    insertion_depth_mm: 20
    alignment_error_mm: 4

metrics:
  primary:
    - overall_success
    - subgoal_success
  long_horizon:
    - failure_stage
    - completion_time
  contact:
    - contact_recovery
    - jamming_count
    - max_contact_force
```

---

## 7. Task acceptance rule

一个任务进入 v0 benchmark 前必须满足：

```text
1. task card 完整；
2. 支持 4 个 tactile modes；
3. smoke test 通过；
4. random policy 不 crash；
5. scripted or replay policy 可成功；
6. metrics 输出稳定；
7. dataset demo 可 replay；
8. train/test split 不泄漏；
9. 至少一个 contact-aware metric 有意义；
10. 文档中说明 tactile relevance。
```

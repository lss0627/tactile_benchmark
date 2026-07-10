# FR3 Differential IK Diagnostic

This document records the FR3 Local Differential IK / Jacobian Diagnostic Gate.
The scope is narrow: diagnose whether a local Jacobian-based method can map tiny
7D Cartesian deltas to bounded FR3 arm joint deltas. It is not task control,
not dataset collection, not tactile sensing, and not a benchmark result.

## Motivation

The previous local IK safety gate found that Lula global target IK could solve
tiny Cartesian target poses, but the returned joint targets were nonlocal for
1-5 mm translations. Observed arm joint deltas were about 0.76-0.77 rad, so that
path is not acceptable as the default EE controller for tiny runtime actions.

The diagnostic here avoids global target IK. It uses:

- FK from the Isaac/Lula kinematics solver;
- a finite-difference translation Jacobian at the current joint state;
- damped least squares: `dq = J.T @ inv(J @ J.T + lambda^2 I) @ dx`;
- explicit clipping and validation;
- no joint command until the motion smoke gate.

## Artifacts

- Gate 1 FK/Jacobian probe:
  `outputs/fr3_differential_ik/jacobian_fk_probe.json`
- Gate 2 DLS target report:
  `outputs/fr3_differential_ik/target_report.json`
- Gate 3 FK validation report:
  `outputs/fr3_differential_ik/fk_validation_report.json`
- Gate 4 tiny motion status:
  `outputs/fr3_differential_ik/tiny_diffik_motion_status.json`
- Screenshot:
  `outputs/fr3_differential_ik/fr3_differential_ik_motion_smoke.png`

## Gate 1: FK / Jacobian Probe

Command:

```bash
CUDA_VISIBLE_DEVICES=1 conda run -n isaac python scripts/probe_fr3_jacobian_fk.py \
  --robot-config configs/robots/fr3_real_articulation.yaml \
  --controller-config configs/robots/fr3_ee_controller_contract.yaml \
  --safety-config configs/robots/fr3_ee_controller_safety.yaml \
  --runtime-config outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml \
  --headless \
  --webrtc \
  --output outputs/fr3_differential_ik/jacobian_fk_probe.json
```

Result summary:

- `ok=true`
- `runtime_started=true`
- `fr3_loaded=true`
- `articulation_found=true`
- `current_joint_state_read=true`
- `current_ee_pose_read=true`
- `fk_available=true`
- `jacobian_available=true`
- `jacobian_shape=[3, 7]`
- `num_arm_joints=7`
- `jacobian_source=finite_difference_fk_translation`
- `sends_joint_commands=false`

## Gate 2: Damped Least-Squares Targets

Command:

```bash
CUDA_VISIBLE_DEVICES=1 conda run -n isaac python scripts/check_fr3_differential_ik_targets.py \
  --robot-config configs/robots/fr3_real_articulation.yaml \
  --controller-config configs/robots/fr3_ee_controller_contract.yaml \
  --safety-config configs/robots/fr3_ee_controller_safety.yaml \
  --runtime-config outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml \
  --headless \
  --webrtc \
  --output outputs/fr3_differential_ik/target_report.json
```

Result summary:

- `ok=true`
- `bounded_tiny_action_available=true`
- `jacobian_shape=[3, 7]`
- `num_actions=13`
- `safe_actions` includes zero, +/-X, and +/-Z for 0.25 mm, 0.5 mm, and 1 mm.
- maximum clipped `dq` was about `0.0041413683 rad`
- `nan_detected=false`
- `uses_lula_global_ik=false`
- `uses_joint_space_fallback=false`
- `sends_joint_commands=false`

Selected target examples:

- `plus_x_0p25mm`: `max_abs_dq ~= 0.0004519535 rad`
- `plus_x_0p5mm`: `max_abs_dq ~= 0.0009039071 rad`
- `plus_x_1mm`: `max_abs_dq ~= 0.0018078141 rad`
- `minus_z_1mm`: `max_abs_dq ~= 0.0041413683 rad`

## Gate 3: FK Validation

Command:

```bash
CUDA_VISIBLE_DEVICES=1 conda run -n isaac python scripts/validate_fr3_differential_ik_fk.py \
  --target-report outputs/fr3_differential_ik/target_report.json \
  --robot-config configs/robots/fr3_real_articulation.yaml \
  --controller-config configs/robots/fr3_ee_controller_contract.yaml \
  --safety-config configs/robots/fr3_ee_controller_safety.yaml \
  --runtime-config outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml \
  --headless \
  --webrtc \
  --output outputs/fr3_differential_ik/fk_validation_report.json
```

Result summary:

- `ok=true`
- `fk_available=true`
- `num_actions_checked=13`
- `num_valid_predictions=13`
- `direction_alignment_ok=true`
- `max_prediction_error ~= 9.84e-06 m`
- `recommended_runtime_action=plus_x_0p25mm`
- `recommended_delta_meters=[0.00025, 0.0, 0.0]`
- `sends_joint_commands=false`

## Gate 4: Tiny Differential IK Motion Smoke

Command:

```bash
CUDA_VISIBLE_DEVICES=1 conda run -n isaac python scripts/run_fr3_differential_ik_motion_smoke.py \
  --robot-config configs/robots/fr3_real_articulation.yaml \
  --controller-config configs/robots/fr3_ee_controller_contract.yaml \
  --safety-config configs/robots/fr3_ee_controller_safety.yaml \
  --runtime-config outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml \
  --mode tiny_diffik_ee_delta \
  --headless \
  --webrtc \
  --save-screenshot \
  --max-steps 50 \
  --output outputs/fr3_differential_ik/tiny_diffik_motion_status.json
```

Result summary:

- `ok=true`
- `controller_method_used=differential_ik`
- `commanded_ee_delta=[0.00025, 0.0, 0.0]`
- `dq_computed=true`
- `dq_safety_pass=true`
- `joint_command_sent=true`
- `observed_ee_delta ~= [0.00048219, -0.00001735, -0.00011265]`
- `direction_alignment_ok=true`
- `ee_displacement_norm ~= 0.00049548 m`
- `max_abs_dq ~= 0.0004519535 rad`
- `max_joint_velocity_norm ~= 0.0049749181`
- `safety_abort=false`
- `nan_detected=false`
- `uses_lula_global_ik=false`
- `uses_joint_space_fallback=false`
- `benchmark_result=false`
- `not_for_paper_claims=true`
- `screenshot_saved=true`

## Known Runtime Warnings

Isaac Sim emitted expected headless/GLFW warnings, `CUDA_VISIBLE_DEVICES`
warnings, dynamic-control deprecation warnings, requests dependency warnings,
and FR3 inertia warnings around hand/TCP bodies. The JSON status files are the
source of truth for pass/fail.

## Current Non-Claims

- This is not PressButton control.
- This is not a benchmark result.
- This is not dataset collection.
- This is not tactile or force-aware sensing.
- This does not control gripper closure or tactile mounts.
- This does not change the stable 7D action schema.
- This does not make Lula global target IK safe for tiny actions.

## Follow-On Planning Gate

The follow-on FR3 EE-to-PressButton Planning gate reuses the same local
differential IK safety envelope and keeps `uses_lula_global_ik=false` plus
`uses_joint_space_fallback=false`. It only plans button geometry, co-scene
load-only status, and no-command waypoints; it does not make FR3 press the
button.

## Next Stage Recommendation

The diagnostic supports moving to FR3 PressButton approach-only runtime smoke
with a local differential IK controller path. The next stage should move only
to pre-press / near-contact, preserve the same safety checks, and still avoid
dataset collection, press-depth execution, and benchmark claims.

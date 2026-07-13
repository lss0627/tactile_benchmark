# G1 C2a Attempt-02 Preliminary Evidence Review

**Feature**: `001-benchmark-reconstruction`

**Review date**: 2026-07-13

**Evidence-producing branch**: `codex/g1-press-button-safety`

**Evidence-producing commit**: `0ace57ce716961a8f50ec9b75a7ba65ac544925a`

**Evidence directory**:
[`outputs/evidence/G1/c2a-static-preliminary-0ace57ce7169-attempt-02`](../../outputs/evidence/G1/c2a-static-preliminary-0ace57ce7169-attempt-02/)

**Selected static pose**: `task-ready-z-0p55`

**Selected candidate-record SHA-256**:
`f23323bad3bfb29ee642ed74a13798e615a51b88516067f1c8c07911b4913db9`

**Review decision**: accept the immutable attempt-02 artifact as the completed T149 preliminary
evidence acquisition/review. This closes only T149. It does not complete C2, controlled arrival,
direct reset, reset repeatability, G1, or T070.

## 1. Scope and claim boundary

This review independently checks the retained attempt-02 artifacts against the approved C2a
offline/static contract and the candidate-local rejection repair described in
[`g1-c2a-attempt-01-root-cause-review.md`](g1-c2a-attempt-01-root-cause-review.md). It does not run
Isaac Sim, create new evidence, modify a pose/configuration/threshold, select a command cap, or
authorize attempt-04, C2b, C3, or a PressButton episode.

C2a is a spawn-time, pre-Play, zero-action static qualification. A selected C2a pose/hash is an input
to a later pose-conditioned C1 diagnostic. It is not evidence of a safe non-zero route to that pose
and is not a reset or gate claim.

## 2. Evidence integrity and repository provenance

The review reran `sha256sum -c checksums.sha256` from the evidence directory. Every declared
artifact verified:

- `command.log`;
- `offline_candidates.jsonl`;
- `static_scenes.jsonl`;
- `readiness_samples.jsonl`;
- `report.json`;
- `manifest.json`.

All seven required files, including `checksums.sha256`, are present. Report and manifest both record:

- `repository.commit=0ace57ce716961a8f50ec9b75a7ba65ac544925a`;
- `repository.dirty=false`;
- `schema_version=g1.c2a.static.v1`;
- `evidence_stage=preliminary`.

The recorded command matches the approved headless seed-1701 invocation. The real process returned
exit code `0`. The retained Kit log contains exactly one `Simulation App Starting`, one
`Simulation App Startup Complete`, and one `Simulation App Shutting Down`. The runner writes the
evidence/checksums before its one `factory.close(exit_code=0)` path, and the observed exit code is
consistent with `systemic_failure=false` and a non-null selected pose/hash.

## 3. Truthful offline candidate records

All three fixed candidates are retained in the reviewed order. Solver identity is
`isaacsim_lula_fr3`; the solver frame is `fr3_hand_tcp`, base frame is `fr3_link0`, and EE frame is
`/World/FR3/fr3_hand_tcp`. Solver joint order is `fr3_joint1` through `fr3_joint7`; articulation
order appends `fr3_finger_joint1` and `fr3_finger_joint2`. Stage provenance is `1.0 m/unit` and
Z-up. Asset/config/code/pose-list/orientation/transform/dependency digests are present and consistent
across all three records.

| Order | Candidate | Offline truth | Measured FK and residual | Candidate-local result | Executed scenes/actions |
|---:|---|---|---|---|---:|
| 0 | `task-ready-z-0p55` | `ik_solution_valid=true`, `fk_residual_valid=true` | FK position `[0.5499999917764303, -4.903184008490591e-08, 0.5499999866593824] m`; FK quaternion xyzw `[0.9061530520279772, 0.37522277148009675, 0.18031114233077408, 0.07471552726541228]`; position residual `5.1475436075729155e-08 m`; orientation residual `7.049269254967615e-05 rad` | no offline failure | 3 / 192 |
| 1 | `task-ready-z-0p54` | `ik_solution_valid=false`, `fk_residual_valid=false` | solver/FK/residual outputs are `null` | `G1_C2A_IK_FAILED`: `Lula failed candidate task-ready-z-0p54` | 0 / 0 |
| 2 | `task-ready-z-0p53` | `ik_solution_valid=false`, `fk_residual_valid=false` | solver/FK/residual outputs are `null` | `G1_C2A_IK_FAILED`: `Lula failed candidate task-ready-z-0p53` | 0 / 0 |

The 0p55 measured residuals are finite and strictly below the unchanged `0.0001 m` and
`0.0001 rad` limits. Its finite Lula solution expands by exact joint name into the nine-DOF
articulation order and remains within the recorded joint limits.

For 0p54 and 0p53, `solver_joint_values`, `articulation_joint_values`, FK position/orientation, and
both residuals are all `null`; `scene_count=0`, `readiness_sample_count=0`, and
`actuation_performed=false`. Neither record contains the former `1.0/1.0` sentinel, target-as-FK,
or warm-start-as-solution. The warm start is retained only in the named warm-start fields. Their
exact non-empty codes/messages are candidate-local and do not appear as top-level systemic blockers.

## 4. Fresh static scenes and pre-Play authoring

Only the offline-valid 0p55 candidate creates static scenes. All three scene IDs, fresh tokens,
stage object IDs, articulation object IDs, and target-latch identities are distinct:

| Scene | Fresh token | Stage object | Articulation object | Target latch |
|---|---|---:|---:|---:|
| `task-ready-z-0p55-scene-0` | `c2a-task-ready-z-0p55-0-1701` | `125774094857408` | `125775326683088` | `125775328493936` |
| `task-ready-z-0p55-scene-1` | `c2a-task-ready-z-0p55-1-1701` | `125774094848672` | `125770268877568` | `125783164968048` |
| `task-ready-z-0p55-scene-2` | `c2a-task-ready-z-0p55-2-1701` | `125775221998080` | `125775329010528` | `125775326440688` |

Each scene records `timeline_playing_before_author=false` and
`timeline_play_invoked_by_author=false`. Joint-prim bijection and drive-target equality are true;
arm positions are authored in degrees, finger positions in metres, and all authored velocities are
zero. This proves the reviewed pose was authored before Play rather than introduced as active
runtime motion.

## 5. Readiness action and state review

The evidence contains exactly 192 separate readiness JSONL records: 64 actions in each of three
scenes. The embedded scene samples and `readiness_samples.jsonl` match exactly. All 192 records are
real-runtime records and none is synthetic.

For every scene:

- action indices are exactly `0..63`;
- `requested_vector_m=[0.0,0.0,0.0]` for every action;
- every action advances exactly three physics substeps;
- `target_before` equals `target_after`, and the same target remains immutable across all 64 actions;
- send results and finite flags are true;
- all pre/post q, qd, and TCP values are finite;
- q remains inside the recorded joint limits, qd inside velocity limits, and TCP inside the
  configured workspace;
- maximum nine-DOF native-unit q change from the initial sample is
  `1.7642974853515625e-05`;
- maximum absolute qd is `0.012316840700805187` in native joint units per second;
- maximum TCP displacement from the initial sample is `1.6318521477246407e-05 m`.

The three scenes have the same observed TCP range:

- x: `[0.5499635934829712, 0.5499647259712219] m`;
- y: `[-0.0001342366449534893, -0.0001315707340836525] m`;
- z: `[0.5490697026252747, 0.549085795879364] m`.

## 6. Contact, collision, penetration, button, and truth masks

Across all 192 samples:

- `contact_valid=true`; Contact count and raw Contact count are zero;
- `collision_report_valid=true`; unsafe collision count is zero and monitor error is empty;
- `penetration_provenance_valid=true`; maximum observed penetration is `0.0 m`;
- `button_released=true` and `button_reset=true` throughout;
- button travel is `4.357218858785927e-05 m` throughout;
- `force_vector_valid=false`;
- `wrench_valid=false`;
- `raw_impulse_used_as_force=false`;
- `post_abort_actuation_count=0`.

Runtime metadata records Isaac Sim `6.0.1`, CPU physics, MBP broadphase, GPU dynamics disabled,
driver `550.144.03`, and `driver_validation=UNVALIDATED`. Vulkan rendering used active GPU 0 in
headless mode; that rendering device is not promoted to GPU physics or native GPU Contact.

## 7. Selected pose/hash verification

The selector chose the first/highest candidate with a valid offline record and three distinct
passing scenes: `task-ready-z-0p55`. Independent review canonicalized the complete selected
candidate record using sorted compact JSON and recomputed:

`f23323bad3bfb29ee642ed74a13798e615a51b88516067f1c8c07911b4913db9`

This exactly matches `selected_pose_sha256` in report and manifest. Selection provenance is the
complete immutable offline candidate record, not a post-review pose projection. The shared
`pose_list_sha256` remains `2b3d6b8d38c350bc64cf0e2a6f5fcceb4939cbf840dca2586ed57160d3ae3087`.

## 8. Kit warning classification and follow-up ownership

The retained Kit log is:

`/mnt/data/home/lss/miniconda3/envs/isaac6/lib/python3.12/site-packages/isaacsim/kit/logs/Kit/Isaac-Sim Python/6.0/kit_20260713_135915.log`

It contains 22 warning lines and zero Kit error lines. The warnings are classified as follows:

1. **Environment/headless warnings**: inability to inspect the default display, repeated GLFW
   initialization/plugin startup failures under `--no-window`, CPU powersave profile, and IOMMU
   enabled. These record the execution environment and do not supply physical acceptance evidence.
2. **Runtime-framework warnings**: Hydra source already registered, deprecated `pxr.Semantics`,
   Replicator material configuration fallback, and USD warnings hidden by muted diagnostics. These
   remain runtime warnings; muted USD diagnostics must not be interpreted as proof that no USD
   diagnostic existed.
3. **Physics-provenance warnings requiring T151 review**: `/World/FR3/fr3_hand_tcp` and
   `/World/FR3/fr3_link8` report possibly invalid inertia tensors and negative mass, for which PhysX
   used a small-sphere approximation. These warnings do not negate this zero-action static C2a
   selection, but T151 must review their asset/provenance and possible dynamic impact before C2b or
   any formal physical evidence.
4. **TGS warning requiring T151 review**: the FR3 articulation uses more than four velocity
   iterations in a TGS scene and Isaac reports changed related behavior. T151 must record and review
   this warning before attempt-04/C2b execution.

Pre-Kit stderr also recorded duplicate gRPC health protobuf registration warnings. They did not
become Kit errors, but are retained as startup provenance. No warning category is converted into a
new physics, reset, arrival, C2, or G1 pass claim.

## 9. Preliminary status and T149 closure

Report and manifest correctly retain:

- `status=BLOCKED`;
- blocker `C2A_PRELIMINARY_NOT_GATE_EVIDENCE`;
- `claim_eligible=false`;
- `controlled_arrival=false`;
- `direct_reset_qualified=false`;
- `reset_repeatability_qualified=false`;
- `c2_completed=false`;
- `selected_command_cap_m=null`;
- `gate_status_updated=false`;
- `t070_completed=false`.

`status=BLOCKED` and `C2A_PRELIMINARY_NOT_GATE_EVIDENCE` are the required preliminary/no-claim
boundary, not a contradiction of the selected static pose/hash. Candidate-local offline failures
are separated from the top level: `systemic_failure=false` and the systemic code/message are null.

T149 is therefore closed as **preliminary C2a evidence acquisition plus independent evidence
review only**. It is not a C2 gate completion. Controlled arrival, direct reset, reset
repeatability, C2, G1, T151, and T070 remain incomplete. The command-matrix recommendation is
recorded separately in [`g1-c1-command-matrix-decision.md`](g1-c1-command-matrix-decision.md); T150
remains open until that recommendation receives separate user approval.

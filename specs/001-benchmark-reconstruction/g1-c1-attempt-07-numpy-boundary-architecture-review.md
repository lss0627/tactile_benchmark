# G1 C1 attempt-07 NumPy boundary architecture review

## 1. Review decision

This review is bound to repository commit
`6d8f245dc312a8b305f4f316303e51e9444b30a9` and to the immutable failed
evidence directory:

```text
outputs/evidence/G1/
c1-tracking-pose-conditioned-6d8f245dc312-attempt-07
```

The exact root cause is confirmed from source, not inferred from the exception
text:

```text
the real scene preserves the requested vector
→ the real runtime normalizes the public 7D action to np.ndarray
→ DLS evaluates that multi-element ndarray in Python boolean context
→ NumPy raises before a DifferentialIKResult or qualifying record exists
```

The defective expression is:

```python
commanded_7d_action or fallback
```

at
`isaac_tactile_libero/robots/fr3_differential_ik.py:253`.

The approved architecture is option C:

- missing optional values are represented only by `None`;
- optional/default selection uses explicit `is None`;
- vectors are normalized once at the numerical boundary for exact shape and
  finiteness;
- NumPy arrays remain valid internal numerical values but never participate in
  Python boolean fallback;
- the authoritative requested action is forwarded, not recomputed;
- JSON-safe lists are created only at public/evidence record boundaries;
- no DLS, Jacobian, governor, command-matrix, safety, or physical formula
  changes are permitted.

This is a software integration defect. It is not evidence that the command
matrix, `0.0005 m` hard limit, `0.005 m` clearance, trajectory motif, budget,
physics policy, Contact policy, or force/wrench truth boundary should change.

## 2. Authority and scope

This document is a pure architecture review. It does not authorize or perform:

- a production or test change;
- a config, threshold, matrix, task, or evidence change;
- Isaac Sim startup;
- C1 attempt-08;
- C2b, C3, T070, or any episode;
- a command-cap claim from attempt-07.

The inherited reviews remain authoritative:

- `g1-c1-attempt-04-evidence-lifecycle-review.md` defines evidence-before-close
  and single-shutdown failure handling;
- `g1-c1-attempt-05-requested-vector-root-cause-review.md` defines exact
  requested-vector provenance;
- `g1-c1-attempt-06-trial-identity-stop-tail-root-cause-review.md` defines
  deterministic trial identity, retained failure provenance, and canonical
  stop-tail semantics.

The attempt-07 failure does not weaken or replace any of those contracts.

## 3. Immutable attempt-07 facts

The following facts were re-read from the immutable artifacts and must not be
rewritten by a later repair:

| Fact | Immutable value |
|---|---|
| repository commit | `6d8f245dc312a8b305f4f316303e51e9444b30a9` |
| repository dirty | `false` |
| checksum-file SHA-256 | `2b025fe91e042e248d747f704d8349db88c82cbba6767f4c27600d2d875c36c9` |
| checksum verification | all listed artifacts pass |
| shell / shutdown exit code | `1 / 1` |
| status | `BLOCKED` |
| systemic failure | `true` |
| top blocker | `G1_C1_NO_ELIGIBLE_COMMAND` |
| selected command cap | `null` |
| selected pose | `task-ready-z-0p55` |
| selected pose SHA-256 | `8a15451319f4fb2ad65f7b402daff86df89683ba6e21071a21a442d871d68d02` |
| trials started / complete | `19 / 18` |
| readiness / measurement samples | `1216 / 4609` |
| retained failed candidate | `0.00025 m` |
| retained class / scene | `C1_LOCAL_APPROACH_AXIS_RT_V1 / scene 0` |
| retained action / window | `0 / 0` |
| candidate blocker | `G1_C1_COMPATIBILITY_CONTROLLER_FORBIDDEN` |
| cap-eligible measurement records | `0` |
| qualifying kernel record | `null` |
| post-abort actuation | `0` |
| physics / broadphase / GPU dynamics | `CPU / MBP / disabled` |
| driver validation | `550.144.03 / UNVALIDATED` |
| entry task state | `T151=[x]`, `T152=[x]`, `T070=[ ]` |
| entry gate state | `G1=BLOCKED`, `G2=NOT_STARTED` |
| attempt-08 | absent and unauthorized |

The retained first non-zero requested vector is:

```text
[
  4.111785954114802e-11,
  2.4515926583592856e-10,
  -0.0002499999999998764
]
```

Its recorded norm is the scheduled `0.00025 m`. The exact retained detail is:

```text
CONTROLLER_FAILURE:
The truth value of an array with more than one element is ambiguous.
Use a.any() or a.all()
```

The conservative aggregation is also immutable:

| Term | Value |
|---|---:|
| `N_data` | `1.471637021681381e-07` |
| `N_scene` | `0.0` |
| `N_upper` | `1.471637021681381e-07` |
| `G_data` | `0.0` |
| `G_scene` | `0.0` |
| `G_time` | `0.0` |
| `G_command` | `0.0` |
| `G_upper` | `1.0` |
| `C_raw` | `0.0004998528362978318` |

`C_raw` is not an eligible tested command and cannot be promoted to a cap.

Attempt-07 records:

- no Contact or raw-contact event;
- no unsafe collision or penetration;
- valid collision/penetration provenance;
- no NaN/Inf sample;
- no governor intervention;
- no target send after the controller failure;
- no post-abort actuation;
- `force_vector_valid=false`;
- `wrench_valid=false`;
- `raw_impulse_used_as_force=false`.

The failure evidence was completed before the single
`SimulationApp.close`. It remains failed evidence; it must never be rebuilt,
deleted, or overwritten.

## 4. Exact data-flow root cause

All line numbers below refer to the reviewed entry commit.

### 4.1 Real value and type propagation

| Layer | Source | Exact value/type transition |
|---|---|---|
| authoritative motif | `run_g1_tracking_envelope.py:1055-1065` | The scheduled vector is materialized as a Python `list[float]` and validated as an exact finite three-float tuple. |
| trial runner | `run_g1_tracking_envelope.py:1066-1071` | The original list is passed to `scene.step(requested_vector_m=...)`. |
| real scene normalization | `run_g1_tracking_envelope.py:2394-2397` | `_PoseConditionedIsaacTrackingScene.step()` creates `requested = np.asarray(requested_vector_m, dtype=float)`, a `float64 ndarray` with shape `(3,)`. |
| public action construction | `run_g1_tracking_envelope.py:2423-2425` | On the non-zero measurement path it creates a Python list with shape `(7,)`: the exact requested XYZ followed by four `0.0` values. |
| shared invoke | `run_g1_tracking_envelope.py:2426-2446` and `g1_nonzero_kernel.py:634-653` | `_invoke_g1_qualifying_kernel()` validates a seven-element public `Sequence` and forwards `dict(kernel_input)` to the runtime without scaling or recomputing the action. |
| real runtime normalization | `fr3_differential_ik.py:438-445` | `compute_governed_translation_target()` creates `action = np.asarray(requested_action_7d, dtype=np.float64)`, shape `(7,)`; q and qd become `float64 ndarray`, shape `(9,)`. |
| action delta | `fr3_differential_ik.py:452-457` and `374-395` | The exact `action` ndarray is passed through `compute_action_delta()`. XYZ is forwarded as the Cartesian delta, and the original ndarray is forwarded as `commanded_7d_action`. |
| DLS | `fr3_differential_ik.py:206-253` | `compute_damped_least_squares_delta()` computes the numerical solve, then evaluates `(commanded_7d_action or fallback)` while constructing result provenance. |

For the retained attempt-07 sample, the effective public action immediately
before runtime normalization is:

```text
[
  4.111785954114802e-11,
  2.4515926583592856e-10,
  -0.0002499999999998764,
  0.0,
  0.0,
  0.0,
  0.0
]
```

After `np.asarray(..., dtype=np.float64)`, the values are unchanged and the
runtime type is:

```text
numpy.ndarray, dtype=float64, shape=(7,)
```

### 4.2 Exact failing operation

The relevant DLS code is:

```python
action = tuple(
    float(x)
    for x in (
        commanded_7d_action
        or (*dx.tolist(), 0.0, 0.0, 0.0, 0.0)
    )[:7]
)
```

Python implements `a or b` by testing the truth value of `a`. It therefore
calls boolean conversion on the seven-element ndarray. NumPy deliberately
rejects a single truth value for a multi-element array and raises
`ValueError`.

This is not:

- a failed DLS solve;
- a bad Jacobian;
- a non-finite action;
- a governor rejection;
- an Isaac buffer failure;
- a Contact, collision, or penetration event;
- a limit or budget failure.

DLS has already computed `raw`, `clipped`, and `predicted` by the time line 253
is reached. The exception occurs while constructing action provenance for
`DifferentialIKResult`.

Consequently, none of the following can occur:

```text
DifferentialIKResult return
→ validate_differential_ik_result
→ compute_observed_q_target
→ jacobian_provenance
→ governor evaluation
→ JSON-safe kernel record
→ send
→ accepted-target latch update
```

### 4.3 Why only the non-zero real path triggers

The real scene invokes the shared qualifying kernel only when:

```python
phase == "measurement" and float(np.linalg.norm(requested)) > 0.0
```

at `run_g1_tracking_envelope.py:2423`.

Readiness and zero-command measurement samples use the already seeded
scene-local target latch. They do not call
`compute_governed_translation_target()` and do not reach DLS. This explains
the exact attempt-07 shape:

- all 18 zero-command class/scene trials complete;
- the first non-zero action of the first non-zero trial fails;
- remaining scenes, classes, and higher commands form the canonical stop-tail.

The real scene catches the exception, appends one structured controller safety
event, aborts the scene-local latch, and does not send. The returned sample
still preserves `requested_vector_m`, so the prior requested-vector contract
remains valid.

## 5. Why existing tests did not catch it

### 5.1 Named coverage gaps

| Existing node | What it proves | Why it misses attempt-07 |
|---|---|---|
| `test_c1_nonzero_path_invokes_shared_qualifying_kernel_with_observed_state` | Public kernel inputs, plan/trial identity, real-scene call site, and send/latch ordering. | Its real-scene composition section monkeypatches the complete `_invoke_g1_qualifying_kernel` and `_execute_g1_qualifying_kernel_send` layers. The spy returns a prebuilt Python-list record, so the real runtime, action delta, and DLS never execute. |
| `test_qualifying_kernel_bases_target_on_current_observed_q` | The runtime method exists and the pure observed-q recurrence expands solver dq by exact joint name. | It calls `compute_observed_q_target()` directly. It never calls `FR3DifferentialIKRuntime.compute_governed_translation_target()`. |
| `test_damped_least_squares_delta_solves_identity_translation` | Identity-Jacobian DLS math, clipping, and validation. | It omits `commanded_7d_action`, so DLS uses the tuple fallback. Tuple truthiness is valid and no ndarray is tested. |
| `test_c1_runtime_failure_writes_evidence_before_shutdown` | Requested-vector provenance, structured evidence, null cap, zero post-abort actuation, and evidence-before-single-shutdown. | Its `_real_pose_scene_sample` calls with the default `phase="readiness"` even for the nominally non-zero vector; the qualifying kernel is therefore not invoked. Its trial double returns Python lists. |
| `test_c1_shared_kernel_latch_updates_only_after_successful_send` | Failed sends do not update the accepted target; successful sends do. | It consumes a prebuilt list-valued kernel result and scalar send booleans. It does not execute numerical runtime composition. |

### 5.2 Component coverage versus composition coverage

Current tests cover:

- list-valued public actions;
- isolated ndarray target arithmetic;
- isolated DLS with implicit fallback;
- isolated governor validation;
- isolated send/latch behavior;
- real-scene requested-vector preservation;
- lifecycle and evidence ordering.

They do not cover this exact composition:

```text
real public list
→ real FR3DifferentialIKRuntime
→ real ndarray normalization
→ real compute_action_delta
→ real DLS provenance construction
→ real observed-q target
→ real governor
→ real JSON-safe kernel record
```

The defect is therefore a type-composition gap, not an absence of component
tests.

### 5.3 Fake versus real type

The fake runtime and spies return Python lists. The real runtime converts the
action to an ndarray before calling DLS. Both carry the same numerical values,
but only the ndarray exposes the invalid boolean fallback.

Tests that assert value equality without preserving the real intermediate type
cannot protect this boundary.

## 6. Reachable array-truth architecture audit

The audit read all required modules and the shared target latch, then
enumerated every `or`, `and`, direct `bool(...)`, simple `if value`, and
`if not value` site. Classification is based on the actual reachable type, not
on syntax resemblance.

### 6.1 Audit table

| Source / expression | Actual possible type | C1 reachability | Classification | Required disposition |
|---|---|---|---|---|
| `fr3_differential_ik.py:253` — `commanded_7d_action or fallback` | Declared `Sequence | None`; actually `np.ndarray(shape=(7,))` on real non-zero C1 | reached before every real non-zero send | **runtime-reachable and unsafe** | Replace missing selection with explicit `is None`; normalize/validate exact shape and finite values without boolean conversion. |
| `fr3_differential_ik.py:215,451` — `config or DifferentialIKConfig(...)` | `DifferentialIKConfig | None` | reached | runtime-reachable but proven scalar/object | Not the defect. Under option C, use explicit `is None` for consistent optional semantics. |
| `fr3_differential_ik.py:365` — `abs(float(epsilon)) or 1e-4` | Python `float` | reached during numerical Jacobian | runtime-reachable but proven scalar | Safe from array ambiguity. No numerical change is authorized by this review. |
| `fr3_differential_ik.py:504-511` — combined finite predicate | NumPy scalar booleans reduced by `np.all`/`np.isfinite` | reached after successful DLS | runtime-reachable but proven scalar | Safe and explicit; retain. |
| `g1_nonzero_kernel.py:641-653` — public action read/forward | list/tuple under current public contract; tensor/ndarray is rejected by `Sequence` check | reached | **needs explicit validation** | Preserve the public seven-value schema; validate finite exact values before real method invocation. Do not silently recalculate it. |
| `g1_nonzero_kernel.py:665,673` — `physical_context or {}` | `Mapping | None` | reached by send seam | runtime-reachable but proven mapping | Safe from NumPy ambiguity. Empty mapping and missing context intentionally produce no overlay. |
| `g1_nonzero_kernel.py:677-678` — `target = list(...); if not target` | concrete Python `list` | reached after kernel result | runtime-reachable but proven list | Safe because conversion precedes the branch. An empty target correctly fails closed. |
| `g1_nonzero_kernel.py:699` — `if not send_succeeded` | Python `bool` derived from `sent is not False` | reached | runtime-reachable but proven scalar | Safe; retain exact send semantics. |
| `run_g1_tracking_envelope.py:761,766` and `1653-1684` — contact/collision/force flag truth | real scene supplies Python `bool`; public mapping could otherwise be malformed | reached | runtime-reachable but proven scalar in real scene; public records need explicit schema validation | Do not treat as ndarray defect. Continue fail-closed exact-bool validation at the formal boundary. |
| `run_g1_tracking_envelope.py:1084-1102` — `nonzero` | Python `bool` from tuple scalar comparisons | reached | runtime-reachable but proven scalar | Safe. |
| `run_g1_tracking_envelope.py:1158-1160` — governor aggregation | per-sample scalar boolean | reached after samples | runtime-reachable but proven scalar | Safe under sample schema. |
| `run_g1_tracking_envelope.py:1221,1246` — trial/identity collection truth | concrete Python lists | reached | runtime-reachable but proven list | Safe. |
| `run_g1_tracking_envelope.py:1413` — `run_result or {}` | `Mapping | None` | evidence writing | runtime-reachable but proven mapping | Safe from array ambiguity. |
| `run_g1_tracking_envelope.py:1598` — `route_validation or {...}` | `ContactExclusionRouteResult | None` | pose-conditioned orchestration | runtime-reachable but proven dataclass/object | Safe from array ambiguity. Option C does not require treating it as a vector. |
| `run_g1_tracking_envelope.py:1664,1684` — `if step.get("safety_events")` | real scene emits Python `list` | legacy C1 sample path | runtime-reachable but proven list; needs boundary validation for injected records | Not the attempt-07 path. |
| `run_g1_tracking_envelope.py:2195,2326,2382,2418,2495-2516` | timeline result, Contact validity, abort latch, and safety decisions are scalar booleans | real C1 scene | runtime-reachable but proven scalar | Safe. |
| `run_g1_tracking_envelope.py:2423,2496` — non-zero gate | `float(np.linalg.norm(requested)) > 0.0` | real C1 scene | runtime-reachable but explicitly reduced scalar | Safe; no epsilon or threshold change. |
| `run_g1_tracking_envelope.py:2480,2484` — blocker text fallback | `str | None` | send failure only | runtime-reachable but proven scalar/string | Safe. |
| `run_g1_tracking_envelope.py:2499` — `if not sent` | `FR3DifferentialIKRuntime.send_joint_position_targets()` returns Python `bool` | reached on zero hold or after real send seam | runtime-reachable but proven scalar | Safe. |
| `run_g1_tracking_envelope.py:2535-2539` — finite predicate | NumPy scalar booleans explicitly reduced by `np.all`, then `bool` | reached | runtime-reachable but proven scalar | Safe. |
| `run_g1_tracking_envelope.py:2573-2599` — `kernel_record` conditionals | `dict | None` | reached while creating sample | runtime-reachable but proven mapping | Safe from array ambiguity; success record content still requires composition testing. |
| YAML/config `... or {}` sites | YAML mapping or `None`; malformed non-mapping values are subsequently rejected | preflight/factory | runtime-reachable but proven mapping contract | Not a vector issue. |
| `fr3_ik_controller.py:335` — `if not config` | Isaac loader mapping or `None` | kinematics construction used by C1 build | runtime-reachable but proven mapping | Safe from array ambiguity. |
| `fr3_ik_controller.py:371` — `current_joint_state or ...` | `FR3JointState | None` | global IK path, not qualifying Lula-FD C1 solve | unreachable from qualifying C1; object-safe | No change required for this defect. |
| `fr3_ik_controller.py:530-531,1099-1100` — pose tuple fallback | `tuple[float, float, float] | None` | tiny-IK motion/failure reports | unreachable from qualifying C1; tuple-safe | No change required. |
| `fr3_ik_controller.py:864` — `step_action[0] or step_action[2]` | Python scalar floats | offline substep report | unreachable from qualifying C1; scalar-safe | No change required. |
| `fr3_target_latch.py` target handling | ndarray after explicit `np.asarray`; shape/finite checks use attributes and `np.all` | real send/latch | runtime-reachable and explicitly safe | Retain; arrays are never evaluated for truth. |
| test spies/fake runtimes | Python lists, mappings, and scalar booleans | test-only | test-only | Replace the full invoke monkeypatch with a real-type composition seam in the RED contract. |

### 6.2 Audit conclusion

There is exactly one source-proven, runtime-reachable unsafe array truth
operation on the qualifying C1 path:

```text
fr3_differential_ik.py:253
```

Other syntactically similar expressions are not automatically defects. Their
actual types are scalars, dataclasses, strings, mappings, or lists, or the
sites are unreachable from qualifying C1.

The audit also identifies boundary inputs that deserve exact validation so a
future tensor-like or injected record cannot reach boolean context. That does
not authorize a broad mechanical rewrite of every truthiness expression.

## 7. Architecture options

### 7.1 Option A — change only the failing expression

Example:

```python
source = fallback if commanded_7d_action is None else commanded_7d_action
```

Advantages:

- smallest production diff;
- directly removes the attempt-07 exception;
- no formula or value change for valid inputs.

Limitations:

- does not define one repository-wide optional-vector rule;
- does not itself reject empty, wrong-shape, or non-finite actions;
- does not close the real-type composition coverage gap;
- permits another vector/default truthiness defect at a later seam.

Option A is an acceptable line-level mechanism but is insufficient as the
complete architecture.

### 7.2 Option B — convert every shared-kernel input to immutable tuple/list

Advantages:

- Python collection truthiness is defined;
- public records are immediately JSON-safe.

Limitations:

- pushes serialization types into numerical code;
- encourages repeated list↔array conversions;
- risks reconstructing or drifting the authoritative requested action;
- hides rather than forbids boolean fallback on numerical vectors;
- weakens type fidelity between real runtime and tests;
- makes the fake-list path the architecture instead of testing the real
  ndarray path.

Option B is rejected.

### 7.3 Option C — explicit missing semantics plus a real-type composition seam

Option C retains ndarray internally and defines:

1. `None` is the only missing optional value.
2. Default selection is always explicit `is None`.
3. The authoritative public action is normalized to float64 once at the
   numerical boundary.
4. Exact shape and finiteness are validated before numerical use.
5. Internal ndarrays never enter Python boolean fallback.
6. The exact authoritative action is forwarded through DLS, diagnostics, and
   governor without scaling, inference, or reconstruction.
7. JSON-safe list conversion occurs at the returned kernel/evidence boundary.
8. Import-safe tests compose the real runtime method with deterministic
   FK/Jacobian seams.

Option C is the sole recommendation.

## 8. Recommended boundary contract

### 8.1 Optional values

For optional parameters:

```python
value = fallback if supplied is None else supplied
```

The following are forbidden:

```python
supplied or fallback
if supplied
if not supplied
bool(supplied)
```

when `supplied` may be a vector, array, tensor-like value, or Isaac buffer.

### 8.2 Numerical normalization

The authoritative 7D action must:

- contain exactly seven components;
- be finite;
- retain exact component order and values;
- keep XYZ exactly equal to the authoritative three-component requested
  vector;
- use four exact tail zeros for the current translation-only command;
- fail closed if empty, wrong-shape, non-numeric, NaN, or Inf.

Tuple, list, and ndarray inputs with identical values must produce numerically
identical results. This does not permit an epsilon or `isclose` policy. Exact
provenance equality remains required at record boundaries.

The `None` case may construct the existing exact fallback from `dx` and four
zeros. An empty collection is supplied-but-invalid; it is not missing and must
not use fallback.

### 8.3 Internal and public representations

Internal numerical representation:

```text
float64 ndarray with validated exact shape
```

Public/kernel/evidence representation:

```text
JSON-safe list copied from the validated authoritative array
```

The requested action must not be inferred from:

- observed TCP motion;
- joint delta;
- gain;
- predicted delta;
- governor output;
- later state.

### 8.4 Shared-kernel attestation

There is one separate composition requirement revealed by source review:

```text
run_g1_tracking_envelope.py requires
qualifying_kernel.shared_kernel is True
```

The current real runtime/wrapper does not author that marker. This is not the
cause of attempt-07 because attempt-07 raises before any kernel record exists.

The next RED must expose the requirement. If GREEN authors the marker, only the
real shared invoke boundary may add:

```json
{"shared_kernel": true}
```

and only after the actual
`FR3DifferentialIKRuntime.compute_governed_translation_target()` call succeeds.
A spy, caller, or evidence writer must not fabricate it. This metadata change
must not rewrite any numerical field or candidate eligibility decision.

## 9. Next-stage RED contract

The next stage must be import-safe and must not start Isaac Sim.

Prefer extending existing frozen nodes:

- `test_c1_nonzero_path_invokes_shared_qualifying_kernel_with_observed_state`;
- `test_qualifying_kernel_bases_target_on_current_observed_q`;
- `test_damped_least_squares_delta_solves_identity_translation`;
- `test_c1_runtime_failure_writes_evidence_before_shutdown`;
- `test_c1_shared_kernel_latch_updates_only_after_successful_send`.

Do not rename or remove nodes and do not change parametrization expansion.

### 9.1 Required composition proof

RED must:

1. construct an actual `FR3DifferentialIKRuntime` instance without invoking its
   Isaac-dependent constructor;
2. call the real
   `FR3DifferentialIKRuntime.compute_governed_translation_target()` method;
3. supply real `np.ndarray` action, q, and qd inputs;
4. inject deterministic FK/numerical-Jacobian behavior at the existing
   method seam, without importing or starting Isaac;
5. execute the real chain:

   ```text
   runtime method
   → compute_action_delta
   → compute_damped_least_squares_delta
   → observed-q target
   → governor
   → JSON-safe kernel record
   ```

6. prove a seven-element ndarray does not undergo truth-value evaluation;
7. prove tuple and list inputs with identical values produce numerically
   identical action, dq, target, and governor records;
8. prove only `None` selects the fallback;
9. prove empty, wrong-shape, non-numeric, NaN, and Inf actions fail closed;
10. prove exact seven-component action and exact three-component requested
    vector provenance are preserved;
11. prove the successful record has:

    ```text
    shared_kernel=true
    controller_qualification=lula_fd_translation
    jacobian_provider=lula_fd_translation
    benchmark_cap_eligible=true
    ```

12. prove the real-scene composition section no longer monkeypatches the
    complete invoke layer;
13. prove send and accepted-target latch update occur only after the real
    kernel and governor succeed;
14. prove kernel failure produces no send and no latch update.

The deterministic seam may inject FK position and a finite `3 x 7` Jacobian.
It must not replace the real governed method, action-delta method, DLS helper,
observed-q target helper, governor, or JSON conversion.

### 9.2 Required failure lifecycle proof

The existing lifecycle node must continue to prove:

- invalid input produces a non-empty structured G1 blocker;
- selected cap remains `null`;
- failure evidence is written before the unique shutdown;
- checksum completion precedes shutdown;
- the shutdown exit code is `1`;
- post-abort actuation remains `0`;
- no second close occurs.

### 9.3 Required invariant proof

RED and later GREEN must prove unchanged:

- exact command matrix;
- `0.0005 m` observed hard limit;
- `0.005 m` clearance;
- trajectory motifs and exact decimal schedules;
- readiness and measurement counts;
- all six required class identities and order;
- stop-tail semantics;
- DLS/Jacobian/governor formulas and constants;
- safety limits and budgets;
- CPU physics / MBP / GPU dynamics disabled;
- Contact, collision, and penetration truth;
- `force_vector_valid=false`;
- `wrench_valid=false`;
- `raw_impulse_used_as_force=false`.

### 9.4 Node inventory gate

The preferred design fits the existing frozen nodes because those nodes
already own:

- DLS math;
- observed-q recurrence;
- shared-kernel composition;
- send/latch behavior;
- failure evidence lifecycle.

If an existing node cannot contain the required behavior without changing its
identity or parametrization, implementation must stop and open an explicit
node-inventory migration:

1. document why a new node is unavoidable;
2. update the approved manifest;
3. recompute collection-order and sorted node-ID digests;
4. review the inventory/digest change before projection.

Silently adding a node or claiming the old digests is forbidden.

## 10. GREEN scope

The minimal GREEN may touch only the code needed to implement option C and the
approved RED:

- `isaac_tactile_libero/robots/fr3_differential_ik.py`;
- if proven necessary by RED,
  `isaac_tactile_libero/runtime/g1_nonzero_kernel.py` for public validation and
  real shared-kernel attestation;
- only the existing focused test nodes described above.

The expected production behavior is:

1. use explicit `is None` for `DifferentialIKConfig` and optional commanded
   action selection;
2. normalize a supplied action without changing its values;
3. validate exact shape and finiteness;
4. pass the same authoritative array through action delta and DLS;
5. preserve exact requested XYZ and four zero tail components;
6. produce a JSON-safe real kernel record;
7. permit governor/send/latch only after successful validation;
8. retain structured failure evidence on every failure path.

No caller-specific type guessing is allowed. No implementation may:

- try ndarray and then fall back to a list;
- catch and ignore the NumPy `ValueError`;
- treat an empty array as missing;
- infer action provenance from state;
- scale, clamp, round, or replace the request;
- add an epsilon or `isclose`;
- change a numerical formula.

If the RED exposes a materially different root cause or requires a safety,
threshold, matrix, physics, or truth-boundary change, GREEN is not authorized
by this review and work must stop.

## 11. Forbidden changes

The following remain forbidden:

- command matrix changes;
- a lower candidate;
- `0.0005 m` hard-limit changes;
- `0.005 m` clearance changes;
- trajectory motif or schedule changes;
- gain, DLS, Jacobian, governor, or budget formula changes;
- physics or driver policy changes;
- Contact/collision/penetration policy changes;
- force/wrench/raw-impulse truth changes;
- attempt-07 evidence mutation;
- cap inference from `C_raw`;
- T151/T152 rollback;
- early T070 completion;
- C2b, C3, or episode execution before a separately authorized successful C1.

## 12. Projection, G0, and freshness

After a separately authorized RED→GREEN implementation:

1. preserve attempt-07 as immutable failed evidence;
2. run the focused import-safe tests and all required affected regression;
3. verify the frozen node inventory and approved digests, or complete the
   explicit migration gate;
4. verify hard limit, clearance, matrix, motif, physics, and force-truth
   invariants;
5. create a clean production-fix projection;
6. run P-bound final verification;
7. refresh formal G0 repository-integrity review at the projection HEAD;
8. require fresh checksums and repository provenance;
9. push and synchronize local/tracking/origin/PR head;
10. keep the PR Draft;
11. keep:

    ```text
    T151=[x]
    T152=[x]
    T070=[ ]
    G1=BLOCKED
    G2=NOT_STARTED
    ```

A passing G0 proves repository integrity only. It does not create an eligible
tested C1 cap.

## 13. Attempt-08 authorization boundary

C1 attempt-08 remains prohibited in this review and in the unprojected
RED→GREEN phase.

It may be considered only after:

- RED fails for the intended missing capability;
- GREEN makes the real-type composition contract pass;
- all required regression and inventory checks pass;
- projection and formal G0 are fresh;
- attempt-07 checksums remain unchanged;
- a new attempt-08 output directory is absent;
- worktree and repository provenance are clean and synchronized;
- a separate one-shot runtime authorization is issued.

Any future attempt-08 authorization permits exactly one process. It does not
authorize a retry, attempt-09, C2b, C3, T070, or episodes.

## 14. Final architecture boundary

The exact distinction is:

```text
None proves “no action was supplied”
an ndarray proves “this exact action entered numerical computation”
```

Python truthiness must never be used to blur those meanings.

The smallest line repair is explicit `is None`; the complete approved
architecture is explicit optional semantics plus a real-type composition test.
This removes the software defect without changing the physical experiment,
safety envelope, or benchmark truth boundary.

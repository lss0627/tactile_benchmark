# G1 Option A Diagnostic Receipt-Binding Fix Projection

## 1. Projection boundary

This projection records the second and final software-repair cycle permitted
by the Option A diagnostic runtime authorization. Its implementation parent
is:

```text
3d3455c5f71a2329ee8d7647e06134829ab94383
```

The repair topology is:

```text
4302b496e6d42ccd9958ed871b242bd7b803d10b
→ 347ae7f  attempt-05 root-cause review
→ 399f6f3  behavior RED
→ 3d3455c  minimal GREEN implementation
→ this clean projection
```

Attempt-05 remains immutable at:

```text
outputs/evidence/G1/
c2a-full-robot-diagnostic-4302b496e6d4-attempt-05
```

Its checksum-file SHA-256 remains:

```text
bd5f0d1be26224cde4a5e9c1d34afbe115179c9bd9428b8d516c43a422a7e0c9
```

## 2. RED to GREEN result

The frozen C2a real-runtime node now exercises a deterministic same-transform
case in which the strict gate and retained record evaluate the rotation
component residual through different serialization paths:

```text
strict matrix residual:
0.7301587301587299

retained quaternion-roundtrip residual:
0.7301587301587303
```

Before GREEN, the node fails because
`validate_property_query_geometry_binding()` discards the complete receipt
with:

```text
geometry disagreement receipt differs from strict gate
```

After GREEN, the receipt-binding seam validates that representation difference
with the existing `gamma_n_float32_query_pose_binding` transform-chain rule.
It then raises the unchanged strict blocker:

```text
G1_FULL_ROBOT_OFFSET_UNRESOLVED
property-query local pose differs from USD geometry
```

with the complete `g1.full_robot.geometry_disagreement.v1` record attached.
Stage/lifecycle identity, paths, operation and shape indices, raw query
values, collider type, declared dimensions and bound policy remain exact
binding checks.

No test function or parameterization was added, removed or renamed.

## 3. Unchanged safety and authority boundary

The repair does not change the strict comparison calculation, residual bound,
decision operator or disagreement result. It does not select USD,
property-query or cooked-shape placement as collision authority.

The following remain unchanged:

```text
Cartesian observed hard limit = 0.0005 m
TCP declared-solid clearance = 0.005 m
force_vector_valid = false
wrench_valid = false
raw_impulse_used_as_force = false
physics_device = cpu
broadphase_type = MBP
gpu_dynamics_enabled = false
native_gpu_contact = false
driver = 550.144.03 / UNVALIDATED
```

The pose candidates and command matrix remain unchanged:

```text
0
0.00025
0.00035
0.00040
0.00045 m
```

`REFERENCE_DRIVER_REVALIDATION_REQUIRED` remains active. Attempt-09 remains
the authoritative right-finger/Button Contact and collision failure.

## 4. Verification ledger

The repair-bound verification produced:

| Verification | Result |
|---|---:|
| exact frozen RED node before GREEN | 1 expected assertion failure |
| exact frozen RED node after GREEN | 1 passed |
| Option A / C2a / TCP focused | 118 passed |
| C1 tracking/kernel/math/safety | 231 passed |
| T152 authoritative file | 113 passed |
| exact hard limit | 4 passed |
| TCP Contact analytic | 38 passed |
| clean-checkout and migration | 16 passed |
| full collection | 1091 nodes |
| deprecated Isaac API scan | 413 files, 0 errors, 0 warnings |

The formal projection-bound G0 must independently verify:

```text
current/portable/external/future = 966/965/1/125
future classification = 78/29/10/8
```

and the approved current-GREEN digests:

```text
collection-order:
1c8e6a8e9b09da6b06435ea6c75191c5fb4b3c3fa7e1b97161951e65249d45ad

sorted:
00a6e84c5d2e1f623f2211db8272ca95859e8050417f7c25cbfeef9afd84efc7
```

## 5. Historical evidence integrity

All payload checks pass and the checksum-file SHA-256 values remain:

```text
attempt-09:
d20cb2bc5cb97af9408066cf4857855721faec0ef4df754ec239d3dbda822d7c

preliminary attempt-03:
d4417f995308c272fbec12c578ab54caabf68451a5448224bbc4652123aba5ca

diagnostic attempt-04:
d0ab2638a795f6b798586bfb43a1d4a9079b08f426397d0bb104e43f11f0b169

diagnostic attempt-05:
bd5f0d1be26224cde4a5e9c1d34afbe115179c9bd9428b8d516c43a422a7e0c9
```

No historical evidence was modified, replaced or rebuilt. Attempt-10 remains
absent.

## 6. Runtime boundary

After this projection is committed and pushed, a fresh formal G0 must bind
the clean projection HEAD. Only after G0 passes may one diagnostic attempt-06
run at a new output path.

Attempt-06 is the last evidence-producing SHA permitted by this authorization.
It is diagnostic only: it cannot approve a pose, matrix, command cap, C2a,
C1, G1 or any collision authority. If it still cannot retain a complete
record, the result must stop as classification A3.

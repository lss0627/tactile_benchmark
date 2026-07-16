# G1 PressButton Unscaled Rigid Housing Review

**Status:** `APPROVED_PENDING_RED_GREEN`

**Decision:** `UNSCALED_KINEMATIC_BODY_WITH_SCALED_COLLIDER_CHILD`

**Scope:** Correct the USD body/collider hierarchy and joint-frame authoring
without changing declared geometry, task truth, safety thresholds, physics
policy, command matrix, or mechanism/task versions.

## 1. Observed failure and root cause

The immutable C2a attempt-04 at
`outputs/evidence/G1/c2a-static-current-c2412f118b19-attempt-04` failed before
scene/readiness qualification. Real Isaac Sim 6.0.1 / USD 0.25.5 diagnosis
showed that `/World/PressButton/Housing` was both a scaled `UsdGeom.Cube` and a
kinematic rigid body. Its local translation was `z=-0.025`, its Cube scale was
`z=0.02`, and the joint used `localPos0.z=+0.025`. USD therefore evaluated the
body0 anchor through the rigid body's non-uniform scale:

```text
housing body0 world anchor z = 0.445499999996...
button  body1 world anchor z = 0.47
delta                         = -0.0245000000037 m
```

On Play, the constraint pulled the button toward the housing center and created
approximately 24.8 mm of apparent travel. This is a stage-authoring defect, not
button movement and not a state-oracle, threshold, joint-limit, or timing defect.
`read_stage()` must continue to report the real button world movement and must
not subtract, clamp, delay, or relabel the bad displacement.

## 2. Approved hierarchy

The exact hierarchy is:

```text
/World/PressButton/Housing                    UsdGeom.Xform
  xformOp:translate = geometry.housing.center_local_m
  scale op = absent
  UsdPhysics.RigidBodyAPI = enabled
  physics:kinematicEnabled = true

/World/PressButton/Housing/Geometry           UsdGeom.Cube
  xformOp:translate = [0, 0, 0]
  xformOp:scale = 2 * geometry.housing.half_extents_m
  UsdPhysics.CollisionAPI = applied
  UsdPhysics.RigidBodyAPI = absent

/World/PressButton/Button                     existing UsdGeom.Cylinder
  existing dynamic rigid body, collision, and mass semantics
```

The housing body path remains exactly `/World/PressButton/Housing`. The housing
collider path is fixed as `/World/PressButton/Housing/Geometry`; it is derived as
the direct `Geometry` child of the configured housing path and is not a second
geometry authority. Collision on the child inherits the parent rigid body.

The analytic declared-solid contract is unchanged: the housing oriented box
still has the configured center and half extents, the button capped cylinder is
unchanged, and both continue to feed the contact-exclusion calculation. Root
position/orientation, `geometry_sha256`,
`world_from_mechanism_root_sha256`, task-config digest semantics, the exact
`0.005 m` clearance, and all route geometry remain unchanged.

## 3. Joint frames and pre-Play invariant

The prismatic joint remains at the configured path with the same axis, matching
rotations, lower/upper limits, drive type, target, stiffness, and damping:

```text
body0 = /World/PressButton/Housing
body1 = /World/PressButton/Button
localPos0 = button.center_local_m - housing.center_local_m
localPos1 = [0, 0, 0]
```

Because body0 is now an unscaled Xform, the two anchor expressions are the same
point before Play:

```text
world_from_root * (housing.center_local_m + localPos0)
== world_from_root * (button.center_local_m + localPos1)
```

The implementation must validate this invariant twice:

1. an import-safe helper validates finite contract-derived frames, exact body
   and collider paths, identity and rotated roots, and nonzero centers;
2. `build_stage()` reads the authored USD transforms and fails closed before
   returning if the actual body0/body1 world anchors differ by more than
   `1e-9 m` on any axis.

The second check proves the authored hierarchy, rather than merely replaying the
input arithmetic. A missing prim, scaled housing body, wrong relationship, or
anchor mismatch uses `G1_PRESS_BUTTON_STAGE_BUILD_INCOMPLETE` with a non-empty
message. An inverse-scale anchor, literal `0.025` compensation, or a fallback
frame is forbidden.

## 4. Geometry receipt v2 and migration boundary

`g1.press_button.geometry_authoring_receipt.v1` froze one
`housing_prim_path`, whose historical implementation conflated the body and
collider prim. Recording both paths changes that exact versioned contract, so
the corrected receipt is explicitly:

```text
schema_version = g1.press_button.geometry_authoring_receipt.v2
housing_body_prim_path = /World/PressButton/Housing
housing_collision_prim_path = /World/PressButton/Housing/Geometry
geometry_only = true
complete_stage = false
benchmark_cap_eligible = false
```

The v2 receipt retains the exact parsed `PressButtonGeometryContract` object and
both existing digests. Mechanism version stays `1.1.0`; the receipt bump does not
change the task, geometry, root transform, or physical policy.

Migration is deliberately narrow and has the exact interface:

```python
def migrate_press_button_geometry_authoring_receipt_v1(
    receipt: Mapping[str, Any],
    *,
    config: PressButtonMechanismConfig,
) -> PressButtonGeometryAuthoringReceipt: ...
```

The input key set must equal the frozen v1 fields exactly:

```text
schema_version
mechanism_version
contract
geometry_sha256
world_from_mechanism_root_sha256
root_prim_path
housing_prim_path
button_prim_path
geometry_only
complete_stage
benchmark_cap_eligible
```

The migrator requires schema `g1.press_button.geometry_authoring_receipt.v1`,
mechanism `1.1.0`, the exact non-null `config.geometry_contract` object, both
digests equal to that contract, all three paths equal to config, and booleans
exactly `true/false/false`. Strings must be non-empty absolute prim paths, the
three configured paths must be distinct, and the housing path must not already
end in `/Geometry`. Missing or unexpected fields, wrong types or values,
different contract identity/digests, an unknown schema, a malformed path, or
any claim-bearing boolean fail closed.

Only after those checks does v1 `housing_prim_path` become the v2 body path and
its direct `Geometry` child become the collider path. The returned object is the
exact v2 receipt and retains the same contract/digests/no-claim booleans. It
cannot prove that a historical USD stage used the corrected hierarchy. Existing
receipt tests must exercise the success and rejection cases in place, without
adding, deleting, renaming, or re-parameterizing test functions.

## 5. Authoring and truth boundaries

`author_declared_geometry()` remains import-safe. A recording fake may prove
ordered transfer, paths, values, object identity, digests, and the three
no-claim booleans. It must not import `pxr`, build physics, or claim a complete
stage.

The real lazy USD adapter authors the root, the unscaled housing body, the
scaled collider child, and the button geometry. Only real `build_stage()` may
then apply collision, rigid body, kinematic, mass, prismatic joint, relationship,
anchor, limit, and drive schemas. Its returned scene contract must include:

```text
housing_body_prim_path
housing_collision_prim_path
anchor_alignment_valid = true
```

The scene contract is returned only after every real prim and schema operation
and the authored world-anchor check succeed. A geometry-only receipt cannot
claim `complete_stage`, runtime success, C1 readiness, cap eligibility, or Gate
completion.

Task truth and sensor truth remain unchanged. `read_stage()` observes the button
world transform against the unchanged mechanism root; it does not compensate a
stage defect. Contact scalar magnitude/raw position/normal/impulse remain
distinct from force vector and wrench, invalid vector/wrench masks remain false,
and raw impulse is never promoted to force.

## 6. Test-first delivery and stop rules

Delivery is fixed as:

```text
c2412f118b195a767ba3f79345cad1ae57396d45
-> docs(g1): review unscaled PressButton housing body
-> test(g1): require unscaled PressButton rigid housing
-> fix(g1): separate PressButton rigid body and collider scale
-> real Isaac 6 in-memory USD acceptance
-> complete regression and production-fix projection
-> fresh formal G0
-> exactly one preliminary C2a attempt-05
```

RED extends existing nodes only and must fail through behavior assertions, not
collection, import, fixture, or Isaac environment failure. GREEN changes only
`isaac_tactile_libero/tasks/press_button_mechanism.py` and must not copy geometry
authority or change config values.

Stop before C2a if the real in-memory stage does not have the approved hierarchy,
the authored anchors do not coincide, source needs a constant/inverse-scale
compensation, the node inventory/digests drift without explicit migration, or
any threshold, budget, command matrix, physics policy, driver policy, force
truth, root pose, joint limit, or mechanism/task version changes.

The unvalidated local driver remains `550.144.03`. Even complete local physical
behavior can support at most `PASS_SMOKE` for G1 and must retain
`REFERENCE_DRIVER_REVALIDATION_REQUIRED`; it cannot be represented as a release
`PASS_BENCHMARK`.

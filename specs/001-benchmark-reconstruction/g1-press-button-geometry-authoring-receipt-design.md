# G1 PressButton Geometry Authoring Receipt Design

**Status:** `UNSCALED_RIGID_HOUSING_REVISION_APPROVED_PENDING_RED_GREEN`

**Decision:** `APPROVED_OPTION_A`

**Architecture:** `GEOMETRY_ONLY_RECEIPT_SEAM_PLUS_REAL_COMPLETE_BUILD_STAGE`

This document resolves the stage-authoring gap recorded in
[`g1-press-button-stage-authoring-adapter-blocker.md`](g1-press-button-stage-authoring-adapter-blocker.md).
It approves an import-safe geometry-only seam and retains the real
`PressButtonMechanism.build_stage()` as the sole complete USD/PhysX stage
operation. It does not implement Task 7, complete T152, authorize runtime
execution, or create evidence.

The hierarchy correction approved in
[`g1-press-button-unscaled-rigid-housing-review.md`](g1-press-button-unscaled-rigid-housing-review.md)
supersedes the historical single-prim housing authoring below wherever they
conflict. The geometry-only/complete-stage separation remains unchanged.

## 1. Decision and separation of responsibilities

Declared geometry transfer and complete physical stage construction are two
different operations:

| Operation | Permitted responsibility | Explicitly excluded |
|---|---|---|
| `author_declared_geometry()` | Transfer the already parsed root, housing, and button geometry to a three-method adapter and return a geometry-only receipt | Collision, rigid bodies, mass, joint, drive, runtime success, cap eligibility |
| `build_stage()` | Create real USD geometry through the geometry-only seam, then complete every required USD/PhysX physical and joint property | Fake/custom adapter injection, early success from a geometry receipt |

The geometry-only operation is import-safe and testable with a recording fake.
The complete operation is the real runtime path and performs lazy `pxr`
imports. Production control flow may not depend on fake identity, caller
identity, adapter class name, or test module identity.

## 2. Geometry-only adapter

The approved protocol name states its limited scope:

```python
class PressButtonDeclaredGeometryAuthoringAdapter(Protocol):
    def author_root(
        self,
        *,
        root_path: str,
        position_m: tuple[float, float, float],
        orientation_xyzw: tuple[float, float, float, float],
    ) -> None: ...

    def author_oriented_box(
        self,
        *,
        path: str,
        center_local_m: tuple[float, float, float],
        half_extents_m: tuple[float, float, float],
    ) -> None: ...

    def author_capped_cylinder(
        self,
        *,
        path: str,
        center_local_m: tuple[float, float, float],
        axis_token: str,
        radius_m: float,
        height_m: float,
    ) -> None: ...
```

The protocol has exactly these three responsibilities. It does not acquire
collision APIs, rigid bodies, mass, joints, drives, Contact validity, runtime
qualification, or benchmark eligibility.

## 3. Geometry-only receipt

The seam returns a frozen, pure-Python receipt:

```python
@dataclass(frozen=True)
class PressButtonGeometryAuthoringReceipt:
    schema_version: str
    mechanism_version: str
    contract: PressButtonGeometryContract
    geometry_sha256: str
    world_from_mechanism_root_sha256: str
    root_prim_path: str
    housing_body_prim_path: str
    housing_collision_prim_path: str
    button_prim_path: str
    geometry_only: bool
    complete_stage: bool
    benchmark_cap_eligible: bool
```

The receipt contract is exact:

```text
schema_version = g1.press_button.geometry_authoring_receipt.v2
mechanism_version = 1.1.0
receipt.contract is config.geometry_contract
geometry_sha256 = contract.geometry_sha256
world_from_mechanism_root_sha256 = contract.world_from_mechanism_root_sha256
housing_body_prim_path = config.housing_prim_path
housing_collision_prim_path = config.housing_prim_path + /Geometry
geometry_only = true
complete_stage = false
benchmark_cap_eligible = false
```

Object identity is an in-process invariant used by unit tests. Persistent
provenance uses the two stable SHA-256 values and never `id(contract)`. The
receipt is neither runtime evidence nor proof of a complete stage. It cannot be
promoted to stage success, C1 success, a selected command cap, gate status, or
benchmark eligibility.

Version `v1` is retained as a historical geometry-only contract. Adding the
collider path is incompatible with its exact frozen field set, so `v2` is an
explicit schema bump. Narrow v1-to-v2 migration maps the old absolute
`housing_prim_path` to `housing_body_prim_path` and derives only its direct
`Geometry` child. The migrator accepts a mapping plus the current formal config;
requires exactly `schema_version`, `mechanism_version`, `contract`,
`geometry_sha256`, `world_from_mechanism_root_sha256`, `root_prim_path`,
`housing_prim_path`, `button_prim_path`, `geometry_only`, `complete_stage`, and
`benchmark_cap_eligible`; requires schema v1, mechanism 1.1.0, the identical
config contract, matching contract digests and config paths, and no-claim
booleans `true/false/false`; and rejects every missing, extra, wrong-typed,
mismatched, malformed, or claim-bearing input. It returns the exact v2 receipt
and never retroactively proves a corrected historical stage.

## 4. Geometry-only seam

The approved method is:

```python
def author_declared_geometry(
    self,
    *,
    authoring_adapter: PressButtonDeclaredGeometryAuthoringAdapter,
) -> PressButtonGeometryAuthoringReceipt:
    ...
```

Its ordered behavior is normative:

1. Require exact formal mechanism version `1.1.0`.
2. Require `geometry_contract_available=true` and the existing non-null
   `config.geometry_contract`.
3. Retain that exact contract object; do not parse YAML again and do not build a
   second geometry object from scalar constants.
4. Call `author_root`, then `author_oriented_box` for housing, then
   `author_capped_cylinder` for button.
5. Pass root and solid paths from mechanism config and all geometry values from
   the retained contract.
6. Pass button full height as `2 * contract.button.half_height_m`.
7. Return the receipt defined in section 3 only after all three calls complete.

This method imports none of `pxr`, `omni`, or `isaacsim`. It creates no
collision, rigid body, mass, joint, or drive. Adapter failure propagates and no
receipt is returned. A legacy mechanism call fails closed with
`G1_PRESS_BUTTON_FORMAL_GEOMETRY_REQUIRED` and a non-empty message.

## 5. Recording fake boundary

The recording fake calls only:

```python
receipt = mechanism.author_declared_geometry(
    authoring_adapter=recording_adapter,
)
```

It verifies:

- exact call order `root -> housing -> button`;
- exact parsed paths and values;
- the same in-process contract object;
- matching geometry and root-transform digests;
- absence of Isaac/USD imports;
- `geometry_only=true`, `complete_stage=false`, and
  `benchmark_cap_eligible=false`.

It does not call the complete stage builder. Its result cannot assert collision,
rigid body, joint, drive, runtime qualification, C1 success, or cap eligibility.

## 6. Real complete stage build

The complete interface remains un-injected:

```python
def build_stage(self, stage: Any) -> dict[str, Any]:
    ...
```

Its required sequence is:

1. Check formal mechanism and runtime-stage eligibility before importing USD.
2. Lazily import the required `pxr` modules.
3. Construct `UsdPressButtonDeclaredGeometryAuthoringAdapter(stage)`.
4. Call `author_declared_geometry()` with that real adapter.
5. Verify the authored root, housing, and button prims exist and are valid.
6. Verify the unscaled housing Xform and its scaled `Geometry` Cube child;
   apply collision to the child and rigid-body enabled/kinematic attributes to
   the parent.
7. Apply button collision, rigid body, rigid-body enabled, and configured mass.
8. Define the prismatic joint, Body0/Body1 relationships, local anchors, local
   rotations, lower/upper limits, linear drive type, drive target, stiffness,
   and damping.
9. Compute both authored USD world anchors and require per-axis agreement within
   `1e-9 m` before Play.
10. Return the complete scene contract only after every required operation
    succeeds, with both housing paths and `anchor_alignment_valid=true`.

The geometry receipt is an internal intermediate value. It is not an early
return and cannot satisfy the complete build result.

## 7. Real USD geometry adapter

`UsdPressButtonDeclaredGeometryAuthoringAdapter` receives the real stage at
construction and lazily imports `pxr`. Its authoring rules are:

- root translation is the parsed `position_m`;
- root orientation converts configured xyzw to USD quaternion ordering as
  `Gf.Quatd(w, Gf.Vec3d(x, y, z))` and authors the orient op explicitly with
  `UsdGeom.XformOp.PrecisionDouble`;
- housing body is an unscaled `UsdGeom.Xform` whose local translation is
  `center_local_m`;
- housing collider is a `UsdGeom.Cube` at the body's direct `Geometry` child,
  with zero local translation and full dimensions `2 * half_extents_m`;
- button local translation is `center_local_m`;
- Cylinder axis receives the parsed `axis_token` unchanged;
- Cylinder radius is the parsed `radius_m`;
- Cylinder height is the supplied full height derived from
  `2 * half_height_m`.

The adapter authors declared geometry only. It neither applies physics APIs nor
returns a complete-stage or benchmark claim.

The explicit op precision is part of the root-transform contract. In USD
0.25.5, default `AddOrientOp()` creates a `quatf` attribute, which cannot accept
the approved `Gf.Quatd` value. The only approved pairing is therefore:

```python
root.AddOrientOp(
    precision=UsdGeom.XformOp.PrecisionDouble
).Set(
    Gf.Quatd(w, Gf.Vec3d(x, y, z))
)
```

The resulting `xformOp:orient` attribute must have type `quatd`. Replacing the
root quaternion with `Gf.Quatf`, using float32 orientation, guessing a type from
the caller, trying `Quatd` and falling back to `Quatf`, or catching/ignoring the
USD type mismatch is forbidden. `pxr` remains absent during module import and is
imported only when the real adapter method is called.

### 7.1 Implemented precision-fix receipt

The immutable failed C2a attempt-03 remains bound to old projection
`50f16a9c74e94313f3edbac0d4793667cc5992c4` with blocker
`G1_PRESS_BUTTON_STAGE_BUILD_INCOMPLETE`. The test-first repair is split into:

- RED `41c9526c68d5f22540623d92e9e2e8347ebffcaa`, which extends the existing
  adapter node without changing its ID or parameterization; and
- GREEN `957304c5d0e958a9c98a7fca171ceb65500fc970`, which changes only the
  production root orient-op precision.

The exact Isaac 6/Python 3.12 no-`SimulationApp` in-memory diagnostic printed
`PRESS_BUTTON_IN_MEMORY_STAGE_PASS`. It observed a valid root
`xformOp:orient` attribute with type `quatd`, non-null value, valid root,
housing, button, and joint prims, and unchanged mechanism version `1.1.0`.
The old projection and its G0 remain historical evidence; the projection commit
containing this receipt receives its SHA from Git and does not predeclare it in
tracked text.

## 8. Single geometry authority and joint derivation

The same `PressButtonGeometryContract` supplies all of these values:

- root position and orientation;
- housing center and dimensions;
- button center, axis token, radius, and height;
- geometry and root-transform digests.

Formal builder and adapter source may not retain the physical geometry literals
`0.035`, `0.018`, `0.09`, or `0.025`. The existing joint housing anchor is
derived from the parsed centers:

```python
joint_local_pos0_m = tuple(
    button - housing
    for button, housing in zip(
        contract.button.center_local_m,
        contract.housing.center_local_m,
    )
)
joint_local_pos1_m = (0.0, 0.0, 0.0)
```

For the approved contract this derivation yields the unchanged physical anchor.
It is interpreted in the unscaled housing body frame; scale exists only on the
collider child and therefore cannot distort the joint frame. The authored
body0/body1 world points must coincide before `build_stage()` returns.
The joint axis token is the same parsed button axis token; existing matching
local rotations retain the configured travel direction, whose collinearity was
already validated by the geometry parser. Display colors may remain independent
visual constants because they do not determine geometry, collision shape, or
provenance.

## 9. Failure and truth boundaries

The implementation remains fail closed:

- missing formal geometry or a legacy caller:
  `G1_PRESS_BUTTON_FORMAL_GEOMETRY_REQUIRED`;
- receipt digest or retained-contract mismatch:
  `G1_C1_CONTACT_EXCLUSION_DIGEST_MISMATCH`;
- authored USD prim missing or invalid, or incomplete physics/joint authoring:
  a non-empty exact stage-build blocker defined by the corrected RED contract.

No geometry/schema operation may set `benchmark_cap_eligible=true`.
Static analytic clearance remains
`TCP_POINT_VS_DECLARED_MECHANISM_SOLIDS`, never full-robot collision clearance.
Every real runtime action still requires Contact, collision, penetration, and
post-action safety truth. `force_vector_valid=false`, `wrench_valid=false`, and
`raw_impulse_used_as_force=false` remain mandatory.

## 10. Historical pre-T152 delivery sequence (superseded)

This section records the state when the original receipt seam was approved. It
is not the current attempt ledger. Section 11 supersedes it: C2a attempt-04 has
run exactly once and is immutable; the separately named pose-conditioned C1
attempt-04 remains unexecuted.

The next authorized implementation sequence is:

```text
Task 7A RED correction
-> receipt/seam GREEN
-> real complete build GREEN
-> Task 7B full verification
-> Task 8 route integration
```

Task 6 remains complete. Task 7 and T152 remain incomplete until their separate
GREEN and repository-wide verification pass. Task 8 has not started. T150
remains `[x]`; T151, T152, and T070 remain `[ ]`; attempt-04 remains
`ATTEMPT_04_PROHIBITED`. Fresh C2a remains separately approval-gated after the
final T152 implementation commit E2.

## 11. Post-attempt-04 USD hierarchy correction

The orientation projection and fresh G0 completed at
`c2412f118b195a767ba3f79345cad1ae57396d45`. Its immutable C2a attempt-04
exposed a second software authoring defect: the scaled housing Cube was also the
kinematic rigid body, so its non-uniform scale transformed `localPos0` and made
the joint's world anchors differ by approximately `0.0245 m`. The observed
initial travel was therefore outside `[0, 0.012]` before qualification.

The approved correction is
`UNSCALED_KINEMATIC_BODY_WITH_SCALED_COLLIDER_CHILD`: keep the configured
housing path as an unscaled Xform/kinematic body; author a fixed `Geometry` Cube
child with zero translation, the existing full extents, and CollisionAPI; keep
the button as the dynamic body; and verify actual authored world-anchor
coincidence before Play. Root transform, declared analytic geometry/digests,
joint values, task truth, force/wrench truth, thresholds, limits, budgets,
command matrix, physics policy, mechanism version, and task version do not
change.

Because the v1 receipt froze a single conflated housing path, the two-path
receipt is explicitly `g1.press_button.geometry_authoring_receipt.v2` with a
tested, fail-closed historical v1 migration boundary. A fake receipt remains
`geometry_only=true`, `complete_stage=false`, and
`benchmark_cap_eligible=false`; only real `build_stage()` may return the
complete scene contract.

### 11.1 Implemented v2 and authored-stage closure

The hierarchy delivery is fixed at docs `2294a928798a47b08cd485248e502a011ea42d7d`,
RED `e785369f98aa815c72ae99c67572f4eec76d291b`, and GREEN
`15c653be66fbc7e02cc66e88459d48eeb59f1185`, in the approved parent order. The
v2 receipt now records the configured unscaled housing body path and its fixed
direct `Geometry` child separately. Its v1 migrator validates the entire frozen
historical key set, contract object, digests, paths, and no-claim booleans
before constructing v2; it never upgrades a historical receipt into a
complete-stage claim.

Real `build_stage()` applies and reads back the parent/child physics APIs,
rigid-body enabled and kinematic values, exact transform-op stacks, reset-stack
flags, Cube size, joint relationships, raw local anchors, and computed world
anchors. The accepted in-memory stage has a maximum per-axis anchor delta of
`3.725290076417309e-10 m`, below the unchanged `1e-9 m` authoring-validation
boundary. Root orientation remains `quatd`; analytic geometry, both existing
digests, root pose, joint/drive values, and mechanism version remain unchanged.

The production-fix pre-projection passed the frozen `1091/966/965/1/125`
partition and both approved current-GREEN digests. The tracked projection that
contains this receipt does not predeclare its own SHA. It leaves T152 `[x]`,
T151/T070 `[ ]`, and C2a attempt-05 unexecuted until projection-bound Task 11
and formal G0 are regenerated and reviewed.

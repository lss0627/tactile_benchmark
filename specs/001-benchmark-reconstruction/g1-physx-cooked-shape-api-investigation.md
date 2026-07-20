# G1 PhysX Cooked-Shape API Investigation

## 1. Scope and conclusion

This is a read-only investigation of the installed Isaac Sim 6.0.1 and
`omni.physx` 110.1.13 environment. It does not select USD, property-query, or
cooked PhysX placement as the geometry authority.

The installed public Python property-query API exposes collider path, local
AABB, volume, local position, and local rotation. It does not expose a stable
backend shape handle, cooked shape type, scale, approximation, canonical
primitive axis, backend local/world pose, or narrowphase pose.

NVIDIA's official PhysX source does establish a representation conversion for
the analytic cylinder path:

```text
OpenUSD cylinder axis Z
→ PhysX convex-core cylinder axis X
→ PxQuat(+π/2, [0, -1, 0])
→ -90 degrees about Y
```

That source-level conversion exactly matches the attempt-07 property-query
rotation. The installed binary's private build suffix is not a public source
commit, however, and the public callback still does not expose final
narrowphase placement or a stable backend shape identity. The source evidence
is therefore suitable for a diagnostic representation hypothesis, not a
final runtime placement claim.

## 2. Installed runtime identity

The inspected installation is:

```text
Isaac Sim version: 6.0.1
omni.physx extension version: 110.1.13
Kit version: 110.1.2
extension directory:
  omni.physx-110.1.13+110.1.2.lx64.r.cp312.u7f4
archive target:
  u7f41+stock+25.11+cxx11
extension build:
  110.1.13+release.78978.c38f7d1e.gl
Kit build:
  110.1.2+production.321334.94790c6d.gl
```

The internal suffixes `c38f7d1e` and `94790c6d` are retained as package build
provenance. They are not asserted to be public Git commit identifiers.

## 3. Source hierarchy

The investigation uses this authority order:

1. installed generated Python stubs and installed NVIDIA extension source;
2. official NVIDIA/OpenUSD documentation;
3. official NVIDIA PhysX source;
4. installed extension metadata.

Forum posts, blogs, screenshots, error-string interpretation, Python memory
addresses, and object `repr` values are excluded.

## 4. API capability table

| Field or claim | API/source | Public/private | Version | Lifecycle availability | Units/frame | Stability | Claim suitability |
|---|---|---|---|---|---|---|---|
| query body/collider | `IPhysxPropertyQuery.query_prim` | public | installed 110.1.13 | attached stage; callback lifetime | USD stage/path IDs | stage-lifecycle bound | valid read-only query provenance |
| collider USD path | collider response `path_id` decoded with `PhysicsSchemaTools.intToSdfPath` | public | installed 110.1.13 | callback lifetime | absolute Sdf path | stable inside bound stage | valid USD-to-query binding key |
| query local AABB | `aabb_local_min/max` | public | installed 110.1.13 | callback lifetime | stage units; callback-local | value stable across repeated query must be tested | valid observed query fact |
| query local position | `local_pos` | public | installed 110.1.13 | callback lifetime | stage units; named only “local” by stub | value retained, frame semantics incomplete | diagnostic fact only |
| query local rotation | `local_rot` | public | installed 110.1.13 | callback lifetime | quaternion xyzw; named only “local” by stub | value retained, frame semantics incomplete | diagnostic fact only |
| callback ordinal | adapter enumeration order | derived public observation | current adapter | one query operation | integer ordinal | not documented as shape identity | diagnostic index only |
| operation index | runner-assigned query invocation ordinal | local diagnostic | current adapter | one stage lifecycle | integer ordinal | stable by runner construction | diagnostic index only |
| property count | number of valid collider callbacks | derived public observation | installed 110.1.13 | one query operation | integer count | repeat-query verifiable | diagnostic completeness fact |
| backend shape handle | not in public collider response | unavailable | installed 110.1.13 | unavailable | unavailable | unavailable | no claim |
| backend shape type | not in public collider response | unavailable | installed 110.1.13 | unavailable | unavailable | unavailable | no claim |
| backend scale | not in public collider response | unavailable | installed 110.1.13 | unavailable | unavailable | unavailable | no claim |
| backend approximation | not in public collider response | unavailable | installed 110.1.13 | unavailable | unavailable | unavailable | no claim |
| backend local/world pose | not in public property/simulation/cooking interfaces | unavailable | installed 110.1.13 | unavailable | unavailable | unavailable | no placement claim |
| narrowphase pose | not in public property/simulation/cooking interfaces | unavailable | installed 110.1.13 | unavailable | unavailable | unavailable | no placement claim |
| USD cylinder axis | `UsdGeomCylinder.axis` | public schema | OpenUSD used by Isaac 6 | composed stage | token X/Y/Z; fallback Z | authored/composed-stage stable | valid USD fact |
| PhysX analytic cylinder axis | `PxConvexCore::Cylinder` source | official source-level | public PhysX source snapshot | build-time implementation | local X | source snapshot stable | representation evidence, not installed handle |
| cylinder representation fixup | `fixupConeAndCylinderQuat` and analytic loader branch | official source-level | public PhysX source snapshot | parse/cook path | Z→X is −90° Y | source snapshot stable | representation evidence with version caveat |
| convex approximation path | cylinder convex-mesh cooking branch | official source-level | public PhysX source snapshot | parse/cook path | authored-axis vertices/scale | source snapshot stable | proves analytic fixup is branch-dependent |
| cylinder approximation setting | `/physics/collisionApproximateCylinders` | public setting | installed runtime | application process | boolean | runtime-readable | branch-selection diagnostic |
| live internal shape | `PxShape*` and private database record | private implementation | source snapshot | scene/actor lifetime | pointer/private record | not stable evidence identity | forbidden claim authority |
| mass-information pose | `InternalShape::mMassInfo` | private implementation | source snapshot | parse/live query | implementation-defined local pose | returned through public value callback | source interpretation only |
| cooking interface | public/private cooking getters | mixed | installed 110.1.13 | extension lifetime | mesh/tet cooking/cache | no per-shape placement | not sufficient |
| scene query | public scene-query interface | public | installed 110.1.13 | collision data after simulation initialization | hit/query coordinates | hit-event scoped | not a reusable shape descriptor |

## 5. Installed public property-query surface

The installed generated stub is:

```text
omni/physx/bindings/_physx.pyi
```

`IPhysxPropertyQuery.query_prim` accepts:

```text
stage_id
prim_id
query_mode
timeout_ms
finished_fn
rigid_body_fn
collider_fn
```

`PhysxPropertyQueryColliderResponse` exposes exactly:

```text
result
stage_id
path_id
aabb_local_min
aabb_local_max
volume
local_pos
local_rot
```

The response type has no backend handle/type/scale/approximation/axis or
narrowphase field. Its documentation calls the pose “local” but does not
define a public actor/body/narrowphase equivalence.

Installed `omni.physx.scripts.propertyQueryRigidBody` maps `path_id` back to a
USD prim and composes USD transforms separately for display. It does not use
callback ordinal as stable shape identity.

## 6. Property-query C++ data flow

NVIDIA's official source snapshot is:

```text
repository: NVIDIA-Omniverse/PhysX
commit: b4b286abff6f2b3debd1d1acb120dc428765cf2e
```

Relevant files are:

```text
omni/include/omni/physx/IPhysxPropertyQuery.h
omni/extensions/runtime/source/omni.physx/plugins/PhysXPropertyQuery.cpp
omni/extensions/runtime/source/omni.physx/plugins/BindingsPhysXPython.cpp
```

The callback's local pose is copied from
`PhysXUsdPhysicsInterface::MassInformation`:

- the parse/cook path obtains it while creating or evaluating the shape;
- the live-runtime path enumerates `PxShape*`, maps `shape->userData` through
  a private database, and returns the associated internal mass information;
- the public callback drops the `PxShape*`, private record type, and internal
  pointer.

`queryPrim` may asynchronously cook colliders. It is a property/mass query,
not a documented per-shape cooked-placement introspection API.

## 7. Cylinder axis and representation transform

OpenUSD's public `UsdGeomCylinder` schema defines the fallback axis as `Z`.
The composed PressButton Button collider is an analytic cylinder with axis
`Z`.

NVIDIA's official PhysX source defines the convex-core cylinder along local
`X`:

```text
physx/include/geometry/PxConvexCoreGeometry.h
physx/source/geomutils/src/GuConvexGeometry.cpp
```

The Omniverse USD loader's analytic cylinder branch constructs:

```text
PxConvexCoreGeometry(PxConvexCore::Cylinder(height, radius), margin)
```

and applies:

```text
desc2.localRot = desc.localRot * fixupConeAndCylinderQuat(desc.axis)
```

The helper in `PhysXTools.h` maps a Z-axis source cylinder by:

```text
PxQuat(PxPiDivTwo, PxVec3(0, -1, 0))
```

This is a negative 90-degree Y rotation. The same adjusted pose is retained in
mass information and is therefore consistent with attempt-07:

```text
USD local rotation: identity
query local rotation: -90° about Y
```

The convex-mesh approximation branch is different: it cooks vertices in the
authored axis with axis-dependent scale and does not use the analytic
convex-core representation fixup. Runtime provenance must therefore record
the actual `collisionApproximateCylinders` setting before applying any
representation interpretation.

## 8. One-to-one binding boundary

The public facts can prove this bounded identity:

```text
stage identifier
+ stage lifecycle token
+ rigid-body absolute path
+ collider absolute path decoded from response.path_id
+ query operation index
+ property count
+ callback ordinal
```

This is a stable diagnostic observation identity within one stage lifecycle
when:

- exactly one stage collider has the decoded path;
- exactly one callback has that decoded path;
- repeated queries return the same path set and values;
- stage and lifecycle tokens match.

It is not a backend shape handle. A callback ordinal, tensor slot, Python
`id()`, pointer, `repr`, or internal record address cannot upgrade it.

## 9. Answers to the required questions

1. **Property-query rotation frame:** the public stub says local rotation but
   does not specify actor/body/narrowphase equivalence. Official source shows
   it comes from mass information after loader representation fixup. It is
   retained as `property_query_mass_information_local`, not asserted as final
   narrowphase placement.
2. **PhysX cylinder canonical axis:** official convex-core source defines
   local X for the analytic cylinder.
3. **USD Cylinder default axis:** OpenUSD defines fallback Z.
4. **Meaning of −90° Y:** official loader source defines exactly this Z-to-X
   analytic representation fixup. Installed-binary byte identity to the
   public source snapshot is not proven by the internal build suffix alone.
5. **Shape-index mapping:** operation/property/shape ordinals can bind a
   unique callback to a USD path inside a stage lifecycle. They cannot map to
   a public stable backend shape handle.
6. **Backend type/scale/geometry/approximation exposure:** not exposed by the
   installed public property-query, cooking, simulation, or scene-query
   Python APIs.
7. **Narrowphase equivalence:** the public query pose is not documented as the
   final narrowphase shape pose.
8. **Capability boundary:** the current adapter omitted source/version/axis
   evidence and bounded path binding; those are implementable. Stable handle,
   per-shape backend type/scale/approximation, and narrowphase pose are absent
   from the inspected public API.

## 10. Investigation decision

The implementation may acquire and digest all available facts, prove the
analytic representation hypothesis at source level, and retain explicit
`UNAVAILABLE` fields for missing backend authority. It must not claim a
stable backend handle or narrowphase placement, and it must keep the current
strict geometry agreement result unchanged.


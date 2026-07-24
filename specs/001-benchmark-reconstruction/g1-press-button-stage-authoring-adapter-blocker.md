# G1 PressButton Stage Authoring Adapter Architecture Blocker

**Status:** `TASK_7_GREEN_UNBLOCKED_BY_DESIGN_PENDING_RED_CORRECTION`

**Scope:** Task 7 of
[`g1-contact-exclusion-t152-implementation-plan.md`](g1-contact-exclusion-t152-implementation-plan.md).
Task 6 is complete and remains valid. This blocker does not authorize Isaac Sim,
C2a, attempt-04, episodes, T151, T152, or T070.

## Observed contract gap

The approved `PressButtonStageAuthoringAdapter` has three methods:

- `author_root(...) -> None`
- `author_capped_cylinder(...) -> None`
- `author_oriented_box(...) -> None`

Those methods can expose the configured root and declared analytic solids to an
import-safe recording fake. They cannot represent or return the USD objects
needed for the rest of `PressButtonMechanism.build_stage()`:

- housing and button collision APIs;
- housing and button rigid-body APIs;
- button mass;
- prismatic joint construction;
- body relationship targets;
- local joint anchors and rotations;
- lower and upper limits;
- linear drive type, target, stiffness, and damping.

The protocol also does not receive the complete
`PressButtonGeometryContract`, so a recording adapter cannot directly prove
contract object identity. It can only observe copied scalar/vector fields.

## Why the current protocol cannot satisfy all approved contracts

All of the following are mandatory:

1. A fake adapter path must not import `pxr`, `omni`, or `isaacsim`.
2. Every successful `build_stage()` must preserve the complete physics and joint
   semantics listed above.
3. A custom adapter path must not claim a complete stage after authoring only
   root and solid geometry.
4. Production code must not branch on fake class name, module name, caller
   identity, or other source-sensitive test identity.
5. The approved three-method protocol must not be expanded without separate
   architecture approval.

With the current signatures, a fake `build_stage()` call has only two ordinary
control-flow choices:

- Continue into the existing physics/joint authoring. This requires `pxr` and
  violates the import-safe fake contract.
- Return after the three adapter calls. This omits collision, rigid body, mass,
  joint, relationships, limits, and drive semantics while presenting the call
  as a complete build.

A fake-specific conditional would violate item 4. Adding physics methods,
returning USD prim handles, or introducing another injected seam would change
the approved architecture and violates item 5 without review. Therefore Task 7
GREEN was not implementable at checkpoint `38ff18d` under that combined
contract.

## RED evidence retained

Commit `38ff18d` adds six import-safe RED nodes in
`tests/test_press_button_mechanism.py`:

- `test_stage_authoring_adapter_records_root_housing_button_order_and_exact_values`
- `test_stage_authoring_uses_the_loaded_contract_and_matching_digests`
- `test_stage_authoring_module_and_fake_path_are_import_safe`
- `test_real_usd_authoring_adapter_keeps_pxr_import_lazy_and_uses_full_dimensions`
- `test_formal_stage_builder_contains_no_geometry_authority_literals`
- `test_adapter_injection_cannot_skip_complete_stage_physics_semantics`

The observed checkpoint is 30 passing mechanism nodes and six assertion REDs.
Every RED reports the missing approved stage-authoring capability; there are no
collection, fixture, import, Isaac, error, or skip failures.

## Options evaluated

At the blocker checkpoint, one of these mutually exclusive seams required
approval before Task 7 GREEN:

1. **Separate geometry authoring from complete stage construction
   (recommended).** Add an import-safe method that passes the existing parsed
   `PressButtonGeometryContract` to the three-method recording adapter and
   returns an explicit geometry-only receipt. Keep `build_stage()` as the real,
   complete USD/PhysX operation and never call it with the geometry-only fake.
   This requires revising the former injected-build fake test to call the
   explicitly geometry-only seam.
2. **Expand the adapter to cover the complete stage.** Add approved methods or
   return handles for collision, rigid body, mass, joint, relationships,
   anchors, limits, and drive authoring. Both fake and real adapters then model
   the full successful `build_stage()` contract.

The first option keeps the analytic geometry seam narrow and makes the
geometry-only truth boundary explicit. The second option creates a larger USD
stage abstraction. Neither option changes the mechanism geometry, 0.005 m
clearance, 0.0005 m hard limit, command matrix, physics policy, or runtime
Contact/collision/penetration truth.

## Architecture resolution

**Decision:** `APPROVED_OPTION_A`

**Approval:** `GEOMETRY_ONLY_RECEIPT_SEAM_PLUS_REAL_COMPLETE_BUILD_STAGE`

The approved resolution is specified normatively in
[`g1-press-button-geometry-authoring-receipt-design.md`](g1-press-button-geometry-authoring-receipt-design.md).
The original three-method protocol is retained only for declared root, housing,
and button geometry and is renamed
`PressButtonDeclaredGeometryAuthoringAdapter`. A recording fake calls only the
import-safe `author_declared_geometry()` seam and receives a frozen
geometry-only receipt. The receipt records `geometry_only=true`,
`complete_stage=false`, and `benchmark_cap_eligible=false`.

The real `build_stage()` interface accepts only a real stage. It lazily creates
the real USD declared-geometry adapter, calls the same geometry-only seam, and
then completes collision, rigid body, mass, prismatic joint, relationships,
anchors, rotations, limits, and drive authoring before returning a complete
scene contract. Production control flow may not distinguish a fake by its class,
module, caller, or source identity.

This decision resolves the interface ambiguity that blocked GREEN design. It
does not implement Task 7. The six RED nodes remain valid historical diagnostics
of the former combined seam, but their exact contracts must be corrected to the
approved split before production changes begin. They may not be mechanically
weakened or converted directly to GREEN.

The next checkpoint is therefore Task 7A RED correction, followed only after
review by Task 7B GREEN implementation. Task 6 remains complete and requires no
rollback.

## Preserved state

- Task 6 formal mechanism 1.1 migration remains complete.
- `press_button_physical.yaml::task_version` remains `1.0.2`.
- `press_button.v1.yaml::task_version` remains `1.0.1`.
- The migrated physical config digest differs from attempt-02; attempt-02 has no
  invented task-card or geometry digest.
- T150 remains `[x]`.
- T151, T152, and T070 remain `[ ]`.
- attempt-04 remains `ATTEMPT_04_PROHIBITED`.
- Task 7 GREEN has not started; architecture is approved pending RED correction.

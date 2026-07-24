# G1 Analytic Cylinder Normalization Code Review

## Review scope

This review covers `40a72feea30fde081aab69242f14a05399eeed19..f382da2a18bcf89d887407df34057536c29bbc74` and the source-bound analytic Cylinder architecture, implementation plan, RED contracts, production implementation, writer integration, and schema migration in that range. It is a repository and software-quality review; it is not C2a, C1, G1, backend-shape, or narrowphase evidence.

The reviewed execution path is:

```text
real USD/property-query observations
-> immutable GeometryAgreementRawInputs
-> source/version-bound analytic Cylinder applicability
-> retained Z-to-X representation transform
-> normalized USD representation pose
-> one unchanged strict same-frame placement comparison
-> run-owned comparison accumulator
-> v4 C2a scene/report/manifest serialization
```

## Findings

Review disposition:

```text
Critical = 0
Important = 0
```

Two defects found during the review were fixed before this disposition was recorded:

1. the initial implementation supplied literal Isaac Sim and PhysX extension versions to the canonical evaluator instead of binding the current installed distribution and extension metadata; and
2. the initial representation validator could accept a modified approved transform or normalized pose if the caller recomputed both digests.

The frozen C2a runtime node was extended to reproduce both defects. Commit `743038fb7a14f7a5fa9d57c8bad69c9633255a41` is the review RED, and `f382da2a18bcf89d887407df34057536c29bbc74` is its GREEN. The final implementation reads the installed `isaacsim` distribution version and the live `omni.physx` extension package metadata, places both observations in the canonical evaluation identity, requires the approved transform value rather than only its digest, reconstructs all retained poses under the declared quaternion/matrix conventions, and independently recomputes normalized pose and placement residual fields.

## Safety and authority audit

- Applicability is limited to an exact analytic `Cylinder` collider with USD axis `Z`, the approved PhysX source commit and X-axis primitive type, the installed `6.0.1` / `110.1.13` version pair, one query observation, a valid stage lifecycle binding, the existing same-frame pose bounds, positive known scale, and the existing one-float32-ULP dimension policy.
- Mesh, convex mesh, triangle mesh, cube, sphere, capsule, unknown primitive, unresolved frame, multiple match, version mismatch, unknown scale, dimension mismatch, or additional transform conditions remain fail closed.
- The representation transform changes only the retained representation orientation. Translation, scale, dimensions, raw USD pose, and raw property-query pose are retained and independently checked.
- The post-normalization placement decision calls the existing strict same-frame comparator once. No epsilon, `isclose`, rounding tolerance, bound widening, or alternate pass path was introduced.
- The strict decision and the offset receipt consume the same immutable canonical evaluation. The runtime adapter appends the evaluation before classification, and the writer consumes the run-owned snapshot.
- `binary_source_identity_verified`, `query_to_backend_binding_valid`, and `backend_narrowphase_authority` remain `false`; `claim_scope` remains `DESIGN_TIME_REJECTION_FILTER_ONLY`.
- Runtime Contact, raw Contact, collision, and penetration remain independent fail-closed truth. The design-time result cannot suppress those observations.
- Cartesian observed hard limit `0.0005 m`, TCP declared-solid clearance `0.005 m`, contact/rest offsets, CPU/MBP/GPU-off policy, force/wrench/raw-impulse truth, pose candidates, and command matrix `0/0.00025/0.00035/0.00040/0.00045 m` are unchanged.

## Schema and evidence review

The migration is explicit and monotonic:

```text
g1.full_robot.geometry_comparison_result.v1
-> g1.full_robot.geometry_comparison_result.v2

g1.full_robot.geometry_comparison_accumulator.v1
-> g1.full_robot.geometry_comparison_accumulator.v2

g1.c2a.static.v3
-> g1.c2a.static.v4

g1.c2a.static.v3.creation_failure
-> g1.c2a.static.v4.creation_failure
```

`g1.full_robot.analytic_primitive_representation.v1` is new. Historical v1/v3 evidence remains immutable and receives neither a synthesized representation record nor a v4 claim. The v4 writer emits `analytic_primitive_representation_records.jsonl`, binds record digests into report/manifest/checksums, and preserves write-before-close ordering.

## Verification reviewed

- exact final RED node: `1 passed`;
- complete C2a static-pose runtime CLI: `50 passed`;
- affected Option D/C2a/C1/kernel/math/safety/Contact/migration set: `353 passed`;
- full collection: `1091`;
- current GREEN inventory: `966`;
- portable GREEN inventory: `965`;
- external inventory: `1`;
- intentional future-RED inventory: `125`;
- current collection-order digest: `1c8e6a8e9b09da6b06435ea6c75191c5fb4b3c3fa7e1b97161951e65249d45ad`;
- current sorted digest: `00a6e84c5d2e1f623f2211db8272ca95859e8050417f7c25cbfeef9afd84efc7`;
- deprecated Isaac API scan: 416 files, 0 errors, 0 warnings;
- Python AST/compile/import checks: pass;
- `git diff --check`: pass.

The formal clean-projection G0 and the one authorized preliminary runtime occur after this review and are recorded in the projection and runtime review rather than anticipated here.

## Residual boundaries

The following are intentional blockers, not review findings:

- no backend shape handle, cooked-shape identity, or narrowphase placement authority is claimed;
- no pose candidate or Decimal matrix is approved by this software change;
- no C1 attempt-10, C2b, C3, T070 episode, or G2 runtime is authorized here; and
- driver `550.144.03 / UNVALIDATED` retains `REFERENCE_DRIVER_REVALIDATION_REQUIRED`.

The reviewed implementation is ready for a clean projection, formal repository-integrity G0, and exactly one preliminary full-robot C2a acquisition under the user-approved stop boundary.

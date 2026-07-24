# G1 Analytic Cylinder Normalization Projection

## Projection scope

This commit projects the source-bound analytic Cylinder representation normalization whose reviewed parent is `56b8728b8ee25155411ac819295f5502c075fb2e`. The production implementation ends at `f382da2a18bcf89d887407df34057536c29bbc74`; the intervening commit contains the independent code review with `Critical = 0` and `Important = 0`.

The projection includes:

- source-authorized USD-Z to PhysX-X analytic Cylinder representation normalization;
- strict placement comparison after normalization under the unchanged numerical bounds;
- installed Isaac Sim and `omni.physx` version binding;
- immutable raw and normalized pose records with independently reproducible digests;
- comparison/accumulator v2 and C2a v4 schema migration; and
- write-before-close C2a evidence retention.

## Commit chain

| Role | Commit | Subject |
|---|---|---|
| architecture/spec | `e0c3320` | `docs(g1): design analytic Cylinder normalization` |
| implementation plan | `c3edc0c` | `docs(g1): plan analytic Cylinder normalization` |
| behavior RED | `da145c7` | `test(g1): require source-bound Cylinder normalization` |
| RED fixture correction | `25ea897` | `test(g1): correct placement rotation fixture` |
| representation GREEN | `e8b5cca` | `fix(g1): model analytic Cylinder representation` |
| comparison GREEN | `8744b2c` | `fix(g1): compare Cylinder placement after normalization` |
| writer GREEN | `3c61aaf` | `fix(g1): retain normalized Cylinder C2a evidence` |
| schema migration | `d1df8e3` | `docs(g1): migrate normalized Cylinder evidence schemas` |
| review RED | `743038f` | `test(g1): bind Cylinder normalization to runtime authority` |
| review GREEN | `f382da2` | `fix(g1): bind Cylinder normalization to runtime authority` |
| code review | `56b8728` | `docs(g1): review analytic Cylinder normalization` |

The original behavior RED was an assertion-only failure. The corrected negative fixture represents an additional real placement rotation rather than the approved representation rotation itself. The review RED then proved that a caller could tamper with transform/normalized-pose fields while recomputing digests and that runtime versions were not yet observation-bound. Both contracts are GREEN at the projection parent.

## Schema migration

```text
new: g1.full_robot.analytic_primitive_representation.v1

g1.full_robot.geometry_comparison_result.v1
-> g1.full_robot.geometry_comparison_result.v2

g1.full_robot.geometry_comparison_accumulator.v1
-> g1.full_robot.geometry_comparison_accumulator.v2

g1.c2a.static.v3
-> g1.c2a.static.v4

g1.c2a.static.v3.creation_failure
-> g1.c2a.static.v4.creation_failure
```

Historical evidence is immutable and no-claim. It is not upgraded in place and does not receive synthetic representation or lifecycle fields.

## Verified repository state

- focused C2a static-pose runtime CLI: 50/50;
- affected Option D/C2a/C1/kernel/math/safety/Contact/migration set: 353/353;
- original GREEN: 748/748;
- collected inventory: 1091;
- current/portable/external/future partition: 966/965/1/125;
- intentional future-RED classification remains 78/29/10/8;
- current collection-order digest: `1c8e6a8e9b09da6b06435ea6c75191c5fb4b3c3fa7e1b97161951e65249d45ad`;
- current sorted digest: `00a6e84c5d2e1f623f2211db8272ca95859e8050417f7c25cbfeef9afd84efc7`;
- deprecated Isaac API scan: 416 files, 0 errors, 0 warnings;
- compile/import boundary and `git diff --check`: pass;
- historical attempt-09, Option D preliminary, Option A 04/05/06, canonical 07, and backend-provenance 03 checksums: pass; and
- C1 attempt-10: absent.

The fresh formal G0 generated from this clean projection is the repository-integrity prerequisite for the single preliminary runtime. It does not establish C2a, C1, G1, geometry-authority, or driver eligibility.

## Truth and authority boundary

Representation equivalence is neither backend shape identity nor narrowphase placement authority. The retained fields remain:

```text
binary_source_identity_verified = false
query_to_backend_binding_valid = false
backend_narrowphase_authority = false
claim_scope = DESIGN_TIME_REJECTION_FILTER_ONLY
```

Runtime Contact, raw Contact, collision, and penetration remain independent fail-closed truth. The exact `0.0005 m` Cartesian observed hard limit, `0.005 m` TCP declared-solid clearance, current contact/rest offsets, CPU/MBP/GPU-off policy, false force/wrench/raw-impulse truth, pose candidates, and command matrix `0/0.00025/0.00035/0.00040/0.00045 m` are unchanged.

## Authorized next action and stop boundary

After this projection is pushed and its formal Python 3.12 G0 is fresh, exactly one preliminary full-robot C2a run may write:

```text
outputs/evidence/G1/
c2a-analytic-cylinder-normalized-<projection-short>-attempt-08
```

That run remains preliminary: final pose and matrix approval are false, selected command cap is null, and no C1 attempt-10, C2b, C3, T070 episode, or G2 execution follows it. Any other collider authority blocker is retained without extending Cylinder normalization to another shape.

The projected task state is:

```text
T151/T152 = [x]
T070 = [ ]
G1 = BLOCKED
G2 = NOT_STARTED
driver = 550.144.03 / UNVALIDATED
REFERENCE_DRIVER_REVALIDATION_REQUIRED
```

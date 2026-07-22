# G1 Hierarchical Route-Segment Evidence Migration

Status: implemented for repository-only qualification; no Isaac runtime has been run.

## Monotonic schema transition

The route-performance change is an additive, monotonic evidence migration. Historical
records remain immutable and retain their original claim boundary.

| Historical schema | Current schema | Migration rule |
|---|---|---|
| `g1.pose_conditioned.command_bound_routes.v1` | `g1.pose_conditioned.command_bound_routes.v1` input | The independently validated TCP route remains the input authority. It is not rewritten. |
| `g1.c2a.option_d.route_diagnostics.v2` | `g1.pose_conditioned.route_diagnostics.v3` | A v3 record adds one canonical route request, one route proof, one geometry-equivalence record, one scene lifecycle binding, and cache statistics per class/command. |
| `g1.c2a.static.v5` | `g1.c2a.static.v6` | A v6 scene adds hierarchical proof artifacts and preserves the v5 work ledger, lifecycle, collision snapshot, Contact, force, and no-claim fields. |
| none | `g1.full_robot.route_proof_request.v1` | New immutable 256-action input with two ordered micro-segments per action. |
| none | `g1.full_robot.route_micro_segment.v1` | New float64-byte-bound governed/stopping segment record. |
| none | `g1.full_robot.geometry_equivalence.v1` | New lifecycle-independent reuse key over all geometry, offsets, kinematics, route, pose, and policy inputs. |
| none | `g1.full_robot.route_segment_proof.v1` | New conservative block-certificate and exact-leaf proof. |
| none | `g1.full_robot.route_proof_lifecycle_binding.v1` | New scene-local binding from lifecycle and collision snapshot to the pure proof. |

The existing `g1.full_robot.collision_snapshot.v1`,
`g1.full_robot.swept_clearance.v1`, `g1.full_robot.sweep_work.v1`, and
`g1.contact.provenance.v1` authorities are unchanged. The exact GJK leaf receipt
therefore retains its previous semantics.

## Compatibility and no-claim boundary

- v1-v5 historical C2a evidence is accepted only by its historical validator. It is
  never upgraded in place and never receives synthesized route certificates.
- A v6 success record requires all 30 class/command route proofs (six classes by five
  existing commands), exact 256-action/512-micro-segment coverage, all 17-by-2
  collider pairs, zero unresolved certificates, and zero false-safe results.
- A blocked v6 record may retain a strict completed prefix, but a command marked
  complete cannot omit its proof or lifecycle binding.
- Every v6 proof remains `DESIGN_TIME_REJECTION_FILTER_ONLY`,
  `claim_eligible=false`, `selected_command_cap_m=null`, and
  `actuation_performed=false`. Runtime Contact/collision remains an independent
  fail-closed truth source.

## Pure proof versus scene lifecycle

`pure_route_proof_sha256` covers the proof core and excludes only the scene collision
snapshot digest and the enclosing record digest. Cache keys bind the pure proof to
`geometry_equivalence_sha256`, the exact route-request digest, phase policy, and proof
policy version. The geometry-equivalence payload excludes scene-local lifecycle
tokens, Python diagnostic object identifiers, and the collision snapshot's self
digest; it retains all raw fields that affect geometry, offsets, transforms,
kinematics, motion bounds, route materialization, or phase policy.

Each fresh scene creates a new `g1.full_robot.route_proof_lifecycle_binding.v1`
record containing its scene/trial identity, lifecycle digest, collision snapshot
digest, geometry-equivalence digest, and projected route-proof digest. A lifecycle
binding is never cached or reused. Geometry, offset, transform, shape, joint, route,
pose, inventory, or policy mutation changes the equivalence digest and forces a cold
proof evaluation.

## Artifact migration

A v6 evidence writer emits the already-validated runtime records without rebuilding
geometric decisions:

- `route_segment_proofs.jsonl`;
- `geometry_equivalence_records.jsonl`;
- the existing `sweep_work_progress.jsonl`;
- existing collision, offset, lifecycle, Contact, and comparison artifacts.

The report and manifest record counts and digest lists. The writer does not call the
geometry-equivalence builder or the exact leaf evaluator. All artifacts are checksum
bound before the unique shutdown.

## Test inventory

No test function was added, removed, renamed, or re-parameterized. The new contracts
extend the existing C1 full-robot sweep node and the existing real C2a runtime node.
Consequently the frozen 1091/966/965/1/125 inventory and its approved ordered and
sorted node-ID digests require no migration. Any later node-ID change requires a
separate explicit inventory migration.

## Historical evidence

Attempt-09 remains immutable performance evidence only. This migration does not
backfill it, does not claim Contact/collision absence, and does not produce a C2a,
C1, G1, or benchmark result.

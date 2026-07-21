# G1 Backend Cooked-Shape Provenance Review

## 1. Decision

The approved read-only backend provenance stage is complete. The runtime
produced nineteen checksum-bound records using:

```text
g1.physx.backend_shape_provenance.v1
```

The Button record proves a source-level analytic-cylinder representation
interpretation:

```text
USD cylinder axis Z
→ official PhysX convex-core cylinder axis X
→ negative 90 degrees around Y
→ observed property-query rotation
```

It does not prove a stable backend shape identity or final narrowphase
placement. The installed public API exposes neither a backend handle nor
backend type, scale, approximation, local/world pose, cooked-data identity,
or narrowphase pose. The installed binary also is not byte-bound to the
official public source snapshot. The runtime classification is therefore:

```text
P4 — BACKEND API DOES NOT PROVIDE NECESSARY CLAIM AUTHORITY
```

The record-level `REPRESENTATION_ONLY` interpretation is a retained
diagnostic, not a P1 authority decision. The existing A3 geometry-authority
blocker remains. This review does not choose USD, property-query, or cooked
shape authority; it does not change the strict agreement gate, pose list, or
command matrix.

## 2. Repository and implementation identity

The stage started at:

```text
ffc96255a9c4b364ad52c55aab8ea021c8751aa7
```

The final runtime projection is:

```text
f940947abb4cfdab02b9eee23ecd223bf70d6d93
```

The implementation chain is:

| Role | Commit | Subject |
|---|---|---|
| API investigation and architecture | `16d7e288f9fc6db8869420d310478688ff52bf88` | `docs(g1): investigate backend shape provenance APIs` |
| implementation plan | `00e2d1eebb156e95ded1b17327dd7c63c70aa7af` | `docs(g1): plan backend shape provenance acquisition` |
| stage-units schema clarification | `747eb6b2e32331614676dd7bef2a6c68ae77375b` | `docs(g1): bind backend provenance stage units` |
| primary behavior RED | `902742539ecf13d01e337cbdf8c66ae4e9eb010d` | `test(g1): require backend shape provenance acquisition` |
| import-safe schema GREEN | `8015a0739bb7165e1062200a648df3d78eaabff0` | `fix(g1): model backend shape provenance explicitly` |
| read-only runtime GREEN | `6ed5b7a885fce06e18fce09b0e11ac0943dbb56a` | `fix(g1): acquire backend provenance without actuation` |
| observed-physics review RED | `3289c50a2ecea2df4d4e7b0099130eeadd58aae7` | `test(g1): bind backend provenance to observed physics` |
| observed-physics review GREEN | `82a83d8c3ecdd241e1060a57ffb516f5b16481e1` | `fix(g1): retain observed physics provenance` |
| initial projection | `e6e7fb49d0f3c65f4d922fb8354be63e30612ea8` | `docs(g1): project backend shape provenance` |
| attempt-01 root-cause review | `0f2877992cd292e187e119b287a205bf19d39404` | `docs(g1): review backend provenance factory failure` |
| factory failure RED | `e411aae9805f37b16a212a8679576a22f834db64` | `test(g1): retain backend factory construction failure` |
| factory failure GREEN | `d2ecdff8834083d2cba27a49c61db2bad237f5fc` | `fix(g1): retain backend factory construction failure` |
| factory-retention projection | `edb72e290747e4e1f0895675f2b5de3dbe2c90b6` | `docs(g1): project backend factory failure retention` |
| attempt-02 root-cause review | `dde1e43b52d09c7e3ca5288b76c918a8ef85eaf2` | `docs(g1): review backend provenance import failure` |
| direct-run import RED | `5703bc368adef84ba2c11a7bdc8a3fd955b04586` | `test(g1): require backend runner repository import path` |
| direct-run import GREEN | `df052495ff43d39bb052008e4d6c5cf8d8715e40` | `fix(g1): expose repository package to backend runner` |
| final runtime projection | `f940947abb4cfdab02b9eee23ecd223bf70d6d93` | `docs(g1): project backend runner import fix` |

The two integration repairs are confined to diagnostic retention and import
reachability. They do not alter geometry, placement, offsets, physics policy,
or the strict comparison.

## 3. API and source authority

The investigated runtime is:

```text
Isaac Sim:             6.0.1
Kit:                   110.1.2
omni.physx:            110.1.13
omni.physx build:      110.1.13+release.78978.c38f7d1e.gl
property-query API:    omni.physx.IPhysxPropertyQuery.query_prim
API visibility:        PUBLIC
official source repo:  NVIDIA-Omniverse/PhysX
official source commit:
  b4b286abff6f2b3debd1d1acb120dc428765cf2e
installed/source match: UNPROVEN
```

The installed public collider response exposes:

```text
stage_id
path_id
aabb_local_min
aabb_local_max
volume
local_pos
local_rot
```

It does not expose:

```text
stable backend shape handle
backend shape type
backend scale
backend approximation
backend local/world pose
backend narrowphase pose
cooked-data identifier
```

Official OpenUSD establishes the Cylinder default axis as Z. Official PhysX
source defines the analytic convex-core cylinder on local X and applies a
negative 90-degree Y fixup for the Z-to-X analytic representation. These are
valid source-level semantics. They do not prove that the installed private
build is byte-identical to that source commit, nor do they expose the final
live backend shape placement.

The public `path_id`, decoded absolute collider path, query operation and
property indices, repeated observation, stage identifier, and lifecycle
token provide a stable query-observation identity for this stage lifecycle.
They do not constitute a backend shape handle.

## 4. Implemented schemas and safety boundary

The implemented evidence schemas are:

```text
g1.physx.backend_shape_provenance.v1
g1.physx.backend_shape_provenance_accumulator.v1
g1.backend_shape_provenance.report.v1
g1.backend_shape_provenance.manifest.v1
```

`isaac_tactile_libero/runtime/g1_backend_shape_provenance.py` owns the
import-safe immutable raw input/evaluation types, canonical JSON, validation,
axis interpretation, one-to-one diagnostic binding, and record/accumulator
digests. The real-stage adapter lazily reads USD and property-query values.
The dedicated runner writes its append-only accumulator before the unique
shutdown.

Every unavailable backend fact remains JSON `null` with a structured field
diagnostic. Unavailable is never rewritten as zero, an inferred default, a
Python address, or an object representation. `claim_eligible` is always
false.

The observed post-Play policy remains:

```text
physics_device = cpu
broadphase_type = MBP
gpu_dynamics_enabled = false
native_gpu_contact_enabled = false
driver = 550.144.03
driver_validation = UNVALIDATED
```

The runner performed no pose selection, readiness sampling, candidate sweep,
controller send, or robot action.

## 5. Formal G0

The current-HEAD formal repository-integrity evidence is:

```text
outputs/evidence/G0/
backend-shape-provenance-import-f940947-py312
```

Its repository commit is
`f940947abb4cfdab02b9eee23ecd223bf70d6d93`. The formal review is
`PASS_BENCHMARK` with freshness `13/13`; every listed checksum passes.

The partition is unchanged:

```text
full/current/portable/external/future = 1091/966/965/1/125
future classification = 78/29/10/8
```

The approved current-GREEN digests remain:

```text
collection-order:
1c8e6a8e9b09da6b06435ea6c75191c5fb4b3c3fa7e1b97161951e65249d45ad

sorted:
00a6e84c5d2e1f623f2211db8272ca95859e8050417f7c25cbfeef9afd84efc7
```

The portable repository is synthetic and clean, has
`portable.archive=true`, reads the original worktree zero times, injects no
historical objects, and preserves identical source-tree digests before and
after synthetic Git initialization. G0 is only a repository-integrity
result; it is not a C2a, C1, or G1 pass.

## 6. Diagnostic lifecycle

### 6.1 Attempt-01

The command was executed once from projection `e6e7fb49…`. It exited `1`
before evidence creation because factory construction failed outside the
runner's original write path. The proposed output directory does not exist.
It was not rerun.

### 6.2 Attempt-02

The command was executed once from projection `edb72e290…` and wrote
structured zero-record evidence at:

```text
outputs/evidence/G1/
backend-cooked-shape-provenance-edb72e290747-attempt-02
```

The shell/runner exit was `1`; the exact blocker was
`G1_BACKEND_SHAPE_PROVENANCE_RUNTIME_FAILED` with
`ModuleNotFoundError: No module named 'isaac_tactile_libero'`. Its
checksum-file SHA-256 is:

```text
c00247d1c696594c61256b4372a008942a332248e4143cb8182ff4de6b93798a
```

All five listed payloads pass. The evidence is immutable and was not rerun.

### 6.3 Attempt-03

The final command was executed once:

```text
OMNI_KIT_ACCEPT_EULA=YES \
/mnt/data/home/lss/miniconda3/envs/isaac6/bin/python \
scripts/run_g1_backend_shape_provenance.py \
  --output \
    outputs/evidence/G1/backend-cooked-shape-provenance-f940947abb4c-attempt-03 \
  --config configs/tasks/press_button_physical.yaml \
  --robot-config configs/robots/fr3_press_button_safe.yaml \
  --task-card configs/tasks/cards/press_button.v1.yaml \
  --headless \
  --seed 1701
```

The immutable evidence path is:

```text
outputs/evidence/G1/
backend-cooked-shape-provenance-f940947abb4c-attempt-03
```

The shell and runner-derived shutdown exit are both `0`. The report status is
`DIAGNOSTIC_COMPLETE`, `systemic_failure=false`, and
`backend_record_count=19`. The checksum-file SHA-256 is:

```text
b179e40ebf7706fa2c43dfbf6927829ef569da355d99d805debc9407176e0b9f
```

All five listed payloads pass. The run allocated, bound, closed, and audited
one scene; all lifecycle counts are one. The Kit log is:

```text
/mnt/data/home/lss/miniconda3/envs/isaac6/lib/python3.12/
site-packages/isaacsim/kit/logs/Kit/Isaac-Sim Python/6.0/
kit_20260721_004127.log
```

The evidence finished before the unique shutdown. The runner reports:

```text
readiness samples = 0
controller commands = 0
actuation performed = false
selected pose = null
selected command cap = null
claim eligible = false
post-abort actuation = 0
force_vector_valid = false
wrench_valid = false
raw_impulse_used_as_force = false
```

The headless display warnings, the two pre-existing rigid-body inertia
warnings, and the articulation iteration warning do not change the no-action
diagnostic result.

## 7. Independent integrity audit

All nineteen records validate against the schema. Each `record_sha256` was
independently recomputed after excluding only its own digest field. The
ordered record digest list exactly matches both report and manifest.

The sealed accumulator was independently reconstructed from the JSONL
records. Its SHA-256 is:

```text
0b9ae5f125a7331e7d9134e88c429b668424d634581b44691e167c0073b85e04
```

That value exactly matches report and manifest.

The lifecycle record SHA-256 is:

```text
4e6d4982a1c1832d9edb857bca58d1f8a4b9f17e952dfec9b7216e5352c83d5c
```

The stage lifecycle token is:

```text
920c6334131ba91c51b8dd7d0293aa224f08a4827cc741cd2848038cf05575e2
```

It is identical in the lifecycle record and all provenance records.

## 8. Button cylinder record

The Button record is:

```text
record_id:
0546a8ba3493e9797f3d9c189a23ac45d80d94e15c2ca37f1a0142e9f5e9b98e

record_sha256:
7fe41181d7fba55987ba3c103d0e5b790c4d88a24e9cc6bde49eb597bf897890

rigid body / collider / geometry:
/World/PressButton/Button
```

The USD binding is:

```text
geometry type = Cylinder
axis = Z
approximation = analytic
radius = 0.035 m
height = 0.018 m
scale = [1, 1, 1]
local rotation xyzw = [0, 0, 0, 1]
world translation m =
  [0.5500000119209292, 0, 0.4699542224395554]
world rotation xyzw = [0, 0, 0, 1]
```

The property-query binding is:

```text
operation/property/shape index = 0/0/0
property count = 1
path identifier = 173313
stage identifier = 9223002
query frame = property_query_mass_information_local
query local translation m = [0, 0, 0]
query local rotation xyzw =
  [0, -0.7071067811865475, 0, 0.7071067811865476]
query world translation m =
  [0.5500000119209292, 0, 0.4699542224395554]
query world rotation xyzw =
  [0, -0.7071067811865475, 0, 0.7071067811865476]
local AABB extent m =
  [0.06999999284744263, 0.07000000029802322,
   0.018000001087784767]
volume m3 = 0.00006927211506990716
```

The stable stage-lifecycle-bound query observation identity is:

```text
05a4b5351daae6489e328208cebfed1d97208f93137e6d1fdf7a7653f398f857
```

It comes from
`STAGE_LIFECYCLE_USD_PATH_QUERY_OBSERVATION`. The decoded path has exactly
one stage collider match and one property-query match, so
`usd_to_query_binding_valid=true`. The record deliberately retains:

```text
backend_shape_match_count = null
query_to_backend_binding_valid = false
```

The source-level backend cylinder axis is X. The retained representation
quaternion is:

```text
[0, -0.7071067811865476, 0, 0.7071067811865476]
```

The absolute quaternion dot product against the observed query rotation was
independently recomputed as exactly `1.0`, including the q/−q equivalence
rule. The unique source-level representation transform is therefore
negative 90 degrees around Y.

The record classification is `REPRESENTATION_ONLY`, but its acquisition
status is `PARTIAL`. The following facts remain unavailable:

```text
backend stable shape handle
backend shape type
backend scale
backend approximation
backend local pose
backend world pose
backend narrowphase pose
cooked-data identifier
installed-binary/public-source byte identity
```

Consequently, the record cannot prove that the query pose is the final
narrowphase placement or exclude an additional placement rotation.

## 9. P1/P2/P3/P4 classification

P1 is not satisfied. Although the official source and observed query values
support the analytic Z-to-X representation transform, the required stable
backend binding, installed-binary identity, independently exposed backend
shape type/scale, and proof that no additional actor/narrowphase rotation is
present are absent.

P2 is not satisfied because no stable backend shape handle, cooked geometry
identity, or backend local/world/narrowphase placement is exposed.

P3 is not satisfied because no separately authoritative placement component
is available to decompose from the representation rotation.

P4 is the only evidence-supported classification:

```text
current public API does not expose the backend facts needed to choose
representation, placement, or combined geometry authority
```

The prior attempt-07 A3 classification remains valid. The runtime adds strong
source-level representation evidence, but it does not cross the final
authority boundary.

## 10. Next unique architecture recommendation

The next stage must first obtain a version- and lifecycle-bound backend
shape-introspection contract from one of these explicitly approved sources:

1. an NVIDIA-supported public API that exposes the per-shape stable identity,
   cooked type/scale/approximation, actor binding, and local/world/narrowphase
   pose; or
2. a separately reviewed and explicitly approved internal binding that
   exposes the same facts without using a raw pointer or Python memory
   address and includes installed-binary provenance.

Only after such an API is available may a RED contract compare the observed
backend representation and placement components against the retained USD and
property-query facts. The strict gate must remain unchanged until that
authority decision is separately approved.

If neither supported nor approved internal introspection is available, an
asset-side explicit collision primitive is a distinct architecture decision,
not an inferred fix. It requires its own geometry, migration, safety, and
runtime review. This review does not approve it.

No Option B, axis normalization, USD-only comparison, query-placement
authority, pose change, matrix change, or command-cap selection is approved.
C1 attempt-10 remains prohibited.

The subsequent approved source-bound normalization architecture is defined
in
`g1-analytic-cylinder-representation-normalization-architecture.md`. It uses
the retained `REPRESENTATION_ONLY` source observation solely to normalize the
analytic Cylinder coordinate representation before the unchanged strict
placement comparison. It does not revise this review's P4 conclusion or
claim a backend handle/narrowphase authority.

## 11. Verification and historical immutability

The completed pre-runtime verification remains:

| Check | Result |
|---|---|
| primary backend RED→GREEN | 4 expected assertion failures → 4 passed |
| observed-physics review RED→GREEN | 1 expected assertion failure → 1 passed |
| static-pose runtime focused file | 50 passed |
| affected regression | 478 passed |
| original GREEN | 748 passed |
| current GREEN | 966 passed, 125 deselected |
| detached portable GREEN | 965 passed, 126 deselected |
| external historical node | 1 passed |
| intentional future-RED | 125 expected failures; 78/29/10/8 |
| T152 authoritative file | 113 passed |
| exact hard limit | 4 passed |
| TCP Contact analytic | 38 passed |
| clean-checkout and migration | 16 passed |
| full collection | 1091 collection-order and 1091 unique |
| deprecated Isaac API scan | 415 files, 0 errors, 0 warnings |
| import/compile boundary | passed |
| independent review | Critical 0, Important 0 |

All historical payload checks pass. Their checksum-file SHA-256 values remain:

```text
attempt-09:
d20cb2bc5cb97af9408066cf4857855721faec0ef4df754ec239d3dbda822d7c

Option D preliminary attempt-03:
d4417f995308c272fbec12c578ab54caabf68451a5448224bbc4652123aba5ca

Option A diagnostic attempt-04:
d0ab2638a795f6b798586bfb43a1d4a9079b08f426397d0bb104e43f11f0b169

Option A diagnostic attempt-05:
bd5f0d1be26224cde4a5e9c1d34afbe115179c9bd9428b8d516c43a422a7e0c9

Option A diagnostic attempt-06:
3542a87ee2405a3520f780c205c033fe71ad8288b98f406645bc444b59794634

canonical diagnostic attempt-07:
6e44be8989cf06f7836cceaad926133bdc3b158f23265e0c7c2b0ac6be0f79b6

backend diagnostic attempt-02:
c00247d1c696594c61256b4372a008942a332248e4143cb8182ff4de6b93798a
```

No historical evidence was modified, backfilled, deleted, or rebuilt.

## 12. Final gate state

The immutable attempt-09 Contact fact remains:

```text
/World/FR3/fr3_rightfinger
↔ /World/PressButton/Button
```

The exact `0.0005 m` Cartesian hard limit, exact `0.005 m` TCP clearance,
strict geometry agreement bounds, PhysX offsets, collider geometry, pose
list, command matrix, DLS/Jacobian/governor/motif/budget, Contact fail-closed
truth, and force/wrench boundaries are unchanged.

The final state is:

```text
T151/T152 = [x]
T070 = [ ]
G1 = BLOCKED
G2 = NOT_STARTED
C1 attempt-10 = absent
driver = 550.144.03 / UNVALIDATED
REFERENCE_DRIVER_REVALIDATION_REQUIRED
```

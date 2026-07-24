# G1 Backend Provenance Import Fix Projection

## Projection scope

This commit projects the second and final software-integration correction
authorized for backend provenance acquisition. Its implementation parent is
`df052495ff43d39bb052008e4d6c5cf8d8715e40`.

The preceding writer-retention projection is
`edb72e290747e4e1f0895675f2b5de3dbe2c90b6`. The backend provenance schemas,
source investigation, physics observations, no-authority boundary and
write-ahead behavior remain unchanged.

## Immutable attempt-02

The single runtime on `edb72e290747e4e1f0895675f2b5de3dbe2c90b6`
wrote:

```text
outputs/evidence/G1/
backend-cooked-shape-provenance-edb72e290747-attempt-02
```

Its shell/runner exit was 1, backend record count was zero, and its exact
checksum-file SHA-256 is:

```text
c00247d1c696594c61256b4372a008942a332248e4143cb8182ff4de6b93798a
```

The checksum-bound blocker is:

```text
G1_BACKEND_SHAPE_PROVENANCE_RUNTIME_FAILED
ModuleNotFoundError: No module named 'isaac_tactile_libero'
```

The writer retained no readiness, actuation, pose, cap, force, wrench or raw
impulse claim. Attempt-02 was not rerun.

## Root cause and commits

Direct file execution sets Python's first import path to `scripts/`. The
dedicated runner derived the absolute repository root but did not insert it
into `sys.path`. Isaac Python did not have this worktree package installed,
so the first lazy local-package import failed before factory construction.

| Role | Commit | Subject |
|---|---|---|
| root-cause review | `dde1e43` | `docs(g1): review backend provenance import failure` |
| behavior RED | `5703bc3` | `test(g1): require backend runner repository import path` |
| minimal GREEN | `df05249` | `fix(g1): expose repository package to backend runner` |

The existing frozen import-safety node executes the runner under isolated
`-I -S` path semantics and requires the exact derived repository root at
`sys.path[0]`. It failed by assertion before GREEN and passes after the
runner inserts that root. The probe uses only the standard library; it does
not import or start Isaac.

## Verification

After GREEN:

- the exact import-boundary node passes;
- `tests/test_g1_static_pose_runtime_cli.py` passes 50/50;
- original GREEN passes 748/748;
- current GREEN passes 966/966 with 125 deselected;
- the deprecated Isaac API scan covers 415 files with 0 errors and 0
  warnings;
- compilation and `git diff --check` pass;
- node inventory and approved digests remain unchanged;
- attempt-02 payload checksums and checksum-file SHA remain unchanged; and
- C1 attempt-10 remains absent.

There is no node-ID or inventory migration. The fresh formal G0 bound to this
projection reruns the complete portable/future/external partition; no older
G0 authorizes the next runtime.

Independent review concludes:

```text
Critical = 0
Important = 0
```

## Unchanged policy and truth

The correction changes only the dedicated CLI's local-package import path.
It does not import Isaac at module import time, alter backend or geometry
records, select an authority, modify the strict agreement gate, change
offsets/colliders/pose/matrix, perform readiness or actuation, or authorize
C1 attempt-10.

The exact 0.0005 m hard limit, 0.005 m TCP clearance, CPU/MBP/GPU-off policy,
Contact/collision fail-closed policy, and false force/wrench/raw-impulse truth
remain unchanged.

## Final authorized diagnostic

After this projection is pushed and a new formal Python 3.12 G0 is fresh,
the second software-fix SHA may run exactly one read-only diagnostic at:

```text
outputs/evidence/G1/
backend-cooked-shape-provenance-<projection-short>-attempt-03
```

This is the last runtime allowed by the two-repair boundary. A further
software failure is retained and reviewed without another runtime or patch
loop. Attempt-03 cannot run readiness, actuation, a pose sweep, or C1
attempt-10.

The projected state remains:

```text
T151/T152 = [x]
T070 = [ ]
G1 = BLOCKED
G2 = NOT_STARTED
driver = 550.144.03 / UNVALIDATED
REFERENCE_DRIVER_REVALIDATION_REQUIRED
```

# G1 Backend Provenance Attempt-02 Import-Boundary Review

## Immutable evidence

The only authorized process on projection
`edb72e290747e4e1f0895675f2b5de3dbe2c90b6` wrote:

```text
outputs/evidence/G1/
backend-cooked-shape-provenance-edb72e290747-attempt-02
```

The shell and runner exit code were 1. The checksum-file SHA-256 is:

```text
c00247d1c696594c61256b4372a008942a332248e4143cb8182ff4de6b93798a
```

All six payload checksums pass. The report retained:

```text
status=BLOCKED
systemic_failure=true
blocker_code=G1_BACKEND_SHAPE_PROVENANCE_RUNTIME_FAILED
blocker_message=ModuleNotFoundError: No module named 'isaac_tactile_libero'
backend_record_count=0
lifecycle_record_count=0
readiness_sample_count=0
controller_command_count=0
actuation_performed=false
selected_pose_id=null
selected_command_cap_m=null
post_abort_actuation_count=0
force_vector_valid=false
wrench_valid=false
raw_impulse_used_as_force=false
claim_eligible=false
```

Attempt-02 is immutable and will not be rerun.

## Exact root cause

The command invokes the file path:

```text
/mnt/data/home/lss/miniconda3/envs/isaac6/bin/python
scripts/run_g1_backend_shape_provenance.py
```

For file execution, Python places the `scripts/` directory at `sys.path[0]`;
it does not add the repository root. The dedicated runner computed:

```python
ROOT = Path(__file__).resolve().parents[1]
```

but never inserted that root into `sys.path`. The Isaac 6 Python environment
does not have this worktree package installed. Consequently the lazy import
inside `build_real_backend_provenance_factory()` failed before
`C2ARealSceneFactory` construction:

```python
from isaac_tactile_libero.robots.fr3_static_pose_runtime import (
    C2ARealSceneFactory,
)
```

The repaired writer lifecycle from attempt-01 correctly captured this
previously hidden exception. No `SimulationApp` was created, no stage or
backend query ran, and this result contains no PhysX/geometry fact.

## Minimal correction

The dedicated CLI must insert its already-derived absolute repository root at
the front of `sys.path` before any local-package import:

```python
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
```

This is the same import boundary required for direct repository scripts. It
does not install a package, import Isaac eagerly, select an authority, or
change runtime policy.

## Behavior RED

The existing frozen import-safety node must execute the runner with an
isolated standard-library-only Python path, then assert that evaluating the
runner establishes its exact repository root in `sys.path`. The RED must be a
single assertion failure. It cannot import Isaac, construct
`SimulationApp`, require NumPy/PyYAML, collect a new node, or depend on the
Isaac installation.

## Forbidden changes and next boundary

The GREEN changes only the dedicated runner import path. It does not modify
backend schema, strict geometry comparison, source interpretation, physics,
offsets, collision geometry, pose, matrix, readiness, actuation or evidence
truth.

After GREEN, complete verification, a new projection and fresh formal G0,
the second and final software-fix SHA may produce one new read-only runtime
at attempt-03. C1 attempt-10 remains absent.

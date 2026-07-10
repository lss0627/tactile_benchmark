# External Asset Setup

Licensed Isaac Sim assets stay outside this repository. Configuration stores a
portable resolver key, never a developer-specific absolute path.

The FR3 key is:

```text
Robots/FrankaRobotics/FrankaFR3/fr3.usd
```

Set either of these before running the simulator backend:

```bash
export ISAAC_TACTILE_ASSET_ROOT=/path/to/Assets/Isaac/5.1/Isaac
# or
export ISAACSIM_ASSET_ROOT=/path/to/an/Isaac/assets/root
```

The current machine resolves the retained 5.1 FR3 asset through the default
search path. This means only that the USD was compatibility-tested in Isaac Sim
6.0.1; it is not relabeled or redistributed as a 6.0.1 asset.

Verify resolution without launching Isaac Sim:

```bash
python scripts/audit_repository.py
```

The audit reports the resolver source, resolved path, SHA-256 digest, and any
missing external asset. Override values remain local and must not be committed.

# Lightwheel Backend Probe

This phase is a probe-only adapter layer. It does not connect a real Lightwheel
runtime, does not create an Isaac Sim environment, does not download assets, and
does not run reset or step.

## What The Probe Checks

`scripts/probe_lightwheel_backend.py` reports:

- whether the configured Lightwheel repository path exists;
- whether the configured asset root exists;
- whether runtime import is allowed by config;
- whether the configured Python package is importable when runtime import is
  allowed;
- whether the backend is enabled;
- whether asset provenance metadata passes the license/attribution gate;
- that `runtime_connected=false` and `reset_step_available=false`.

If `enabled=false`, missing local paths do not crash the probe. The report uses
`runtime_status=planned_or_disabled` and returns success as long as the config
and provenance metadata are valid.

## Configuration

Edit `configs/backend/lightwheel_optional.yaml` for local path probing:

- `lightwheel_repo_path`: local Lightwheel or LW-BenchHub checkout path.
- `lightwheel_asset_root`: local asset directory path.
- `lightwheel_python_package`: package name to check only when
  `allow_runtime_import=true`.
- `require_assets`: when true, `lightwheel_asset_root` must exist.
- `allow_runtime_import`: default false. When false, the probe never imports
  Lightwheel.
- `probe_only`: must remain true in this phase.

## Asset Provenance Gate

The probe references `assets/asset_manifest.csv`. If Lightwheel assets or
non-commercial assets are allowed, the manifest must contain source, license,
and attribution for Lightwheel / LW-BenchHub entries. Lightwheel assets must not
be relicensed as the project license.

## Non-Goals

- No Lightwheel code or asset copying.
- No asset download.
- No Isaac Sim scene creation.
- No physical simulation, reset, step, sensor read, or evaluation.
- No benchmark score or paper result.

The next phase may plan a single-task real backend integration, but that should
remain separate from this probe-only gate.

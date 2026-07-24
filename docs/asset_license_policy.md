# Asset License Policy

This policy separates repository code, configs, docs, generated mock data, and
third-party assets.

## Repository Materials

Project code, configuration, tests, and documentation follow the repository
license. This does not automatically apply to third-party assets referenced by
the project.

## Third-Party Assets

Lightwheel / LW-BenchHub assets retain their original license, attribution, and
usage restrictions. Do not re-declare Lightwheel assets as Apache-2.0 or any
other project license. The asset manifest must record original source URL,
license text or license pointer, attribution, modification status, task use,
and redistribution status.

## PressButton Visual Smoke Assets

The planned `PressButton` Isaac Sim visual smoke should prefer Isaac Sim
primitive or built-in assets for the first scene. These can be recorded in the
asset manifest as `isaacsim_builtin` with notes describing the primitive source
and local Isaac Sim dependency.

If a Lightwheel button, table, robot, or scene asset is used instead, the asset
must first be recorded in `assets/asset_manifest.csv` with source, original URL,
license, attribution, modification status, task use, redistribution status, and
notes. Lightwheel assets must keep their upstream terms and must not be
re-licensed as repository code.

## Research and Paper Use

Non-commercial research use must follow upstream Lightwheel terms. Paper
experiments may cite and use third-party assets only when their license permits
the intended use and when attribution is preserved.

## Release Boundary

The paper-v1 official dataset, task suite, project page, and any asset bundle
must include third-party asset statements and provenance metadata. All 16 task
cards bind their asset-manifest entries. Bundles must not include assets whose
redistribution status is unknown or disallowed.

Community task, robot, sensor, and expert plugins retain their own license
metadata and cannot be promoted as benchmark-compatible until registry and
redistribution checks pass.

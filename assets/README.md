# Assets Directory

This directory does not include Lightwheel or LW-BenchHub large assets by
default. It currently stores provenance metadata only.

Future real backend work may use one of two paths:

- a user-provided local `lightwheel_asset_root`;
- a separate download/setup script that preserves upstream license and
  attribution metadata.

Any future asset entry must be recorded in `asset_manifest.csv` with source,
original URL, license, attribution, modification status, task use, redistribution
status, and notes. Do not copy third-party assets into this repository unless
their license explicitly allows it and the manifest is updated first.

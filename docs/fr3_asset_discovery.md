# FR3 Asset Discovery And Config Binding

This stage binds locally installed Isaac Sim Assets to the Isaac-Tactile-LIBERO
FR3 planning config. It is a filesystem/configuration step only.

## What This Stage Does

- Searches local roots for Isaac asset paths.
- Prioritizes the official FR3 relative path:
  `Robots/FrankaRobotics/FrankaFR3/fr3.usd`.
- Writes the selected FR3 USD path into
  `configs/robots/fr3_real_articulation.yaml`.
- Records the Isaac Sim Assets FR3 provenance in `assets/asset_manifest.csv`.
- Updates `probe_fr3_assets.py` readiness so load-only visual smoke can begin
  once the FR3 USD path exists and the manifest gate passes.

## What This Stage Does Not Do

- It does not start Isaac Sim.
- It does not import `isaacsim`, `omni`, or `carb`.
- It does not load USD.
- It does not create a real FR3 articulation.
- It does not attach a controller or IK.
- It does not collect data or train a model.
- It does not redistribute or copy Isaac assets.

## Discovery Command

```bash
python scripts/discover_isaac_assets.py \
  --roots /mnt/data $ISAACSIM_ROOT \
  --patterns FrankaFR3 fr3.usd FrankaRobotics \
  --output outputs/fr3_articulation_probe/isaac_asset_discovery.json
```

The discovery report includes:

- `found_fr3_usd_candidates`;
- `found_gripper_candidates`;
- `found_asset_roots`;
- `recommended_fr3_usd_path`;
- `recommended_asset_root`;
- `warnings`.

## Current Binding

The current recommended FR3 USD is:

`/mnt/data/home/lss/isaacsim_assets/Assets/Isaac/5.1/Isaac/Robots/FrankaRobotics/FrankaFR3/fr3.usd`

The config records:

- `fr3_usd_path` as the local installed Isaac asset path;
- `gripper_usd_path=null`;
- `gripper_embedded_in_fr3_usd=true`;
- `tactile_mount_usd_path=null`;
- `tactile_mounts_planned=true`;
- `asset_source=Isaac Sim Assets`;
- `benchmark_result=false`;
- `not_for_paper_claims=true`.

The gripper is treated as embedded in the FR3 USD for load-only smoke readiness.
Missing tactile mount assets remain a warning and do not block load-only visual
smoke.

## License And Provenance

The manifest entry for the FR3 asset uses source `Isaac Sim Assets`, license
text that points to the NVIDIA/Isaac asset terms, attribution
`NVIDIA Isaac Sim Assets`, and `redistributed=false`.

Do not copy Isaac asset files into this repository and do not relicense them as
the project license.

## Next Gate

After this path binding, `probe_fr3_assets.py` can report
`ready_for_load_only_visual_smoke=true`. The next stage may plan an FR3
load-only visual smoke that starts Isaac Sim and loads the USD, but only after
this planning gate remains green.

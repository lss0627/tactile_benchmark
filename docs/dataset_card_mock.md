# Isaac-Tactile-LIBERO Mock Dataset Card

## Dataset Summary

This is a mock/stub dataset generated from the current lightweight
Isaac-Tactile-LIBERO mock environment. It is only intended to validate HDF5
schema, dataset reader/writer code, validation checks, and replay plumbing.

It is not a real demonstration dataset, not an Isaac Sim rollout, and not a
Lightwheel recording. It must not be used for paper experiment conclusions.

## Schema

Each HDF5 file stores:

- `/metadata/dataset_info`
- `/metadata/schema_version`
- `/metadata/creation_config`
- `/episodes/{episode_id}/observations/...`
- `/episodes/{episode_id}/actions`
- `/episodes/{episode_id}/rewards`
- `/episodes/{episode_id}/success`
- `/episodes/{episode_id}/contact_metrics`
- `/episodes/{episode_id}/metadata`

The observation fields follow the current public observation schema, tactile
schema, and 7D action schema. Missing tactile modalities are represented by
zero-filled arrays plus modality masks.

## Generate

```bash
python scripts/collect_mock_demos.py \
  --config configs/dataset/mock_dataset.yaml \
  --output outputs/mock_dataset/mock_v0.hdf5
```

The default mock config uses 5 tasks, 4 tactile modes, seeds `0 1 2`, and 1
episode per task/mode/seed.

## Validate

```bash
python scripts/validate_dataset.py \
  --dataset outputs/mock_dataset/mock_v0.hdf5 \
  --output outputs/mock_dataset/validation_report.json
```

Validation checks file existence, schema version, episode count, required HDF5
paths, monotonic timestamps, 7D action shape, tactile fields, and contact metric
presence.

## Replay

```bash
python scripts/replay_demos.py \
  --dataset outputs/mock_dataset/mock_v0.hdf5 \
  --max-episodes 3 \
  --headless
```

Current replay validates saved step sequences, action shape, observation schema,
timestamps, and contact metric presence. It does not perform physical replay.

## Future Real Backend

Future Isaac Sim / Lightwheel datasets should reuse this HDF5 schema. The data
source will change from mock/stub arrays to real simulator or sensor outputs,
but path names, action shape, observation layout, tactile masks, timestamps, and
contact metric keys should remain stable unless the benchmark schema version is
intentionally bumped.

# Unified Training Protocol

## Interface

```bash
python scripts/train.py \
  --algo act \
  --suite articulation \
  --protocol contact_material_physics \
  --modalities vision tactile proprio \
  --dataset manifests/tactilibero-v1.json \
  --seed 0
```

The same interface supports BC, ACT, Diffusion Policy, Transformer, and
UniVTAC-compatible configurations.

## Shared preprocessing

Before training, freeze:

- dataset and split manifests;
- task and observation/action contract versions;
- train-only normalization statistics;
- image/tactile resizing and augmentation;
- observation and action horizons;
- padding/mask semantics;
- optimizer, schedule, update budget, and seed list;
- validation metric and checkpoint selection rule.

All modality variants reuse the same eligible episode IDs. Missing modalities
are represented through declared masks, never filled with privileged state.

## Offline training

Offline runs consume only accepted training episodes from the official
dataset. Validation selects the checkpoint. Seen/unseen test results cannot
affect selection.

## Online training

Online methods use the same registered environments and episode schema.
Reports include environment steps, raw/accepted episodes, retries, collection
policy, update count, and wall time. Online interaction may not access
test-only variant identities through reset or privileged metadata.

## Checkpoints and logs

Store:

- algorithm/capability/version;
- source, data, split, config, and normalization digests;
- seed and command;
- runtime/hardware metadata;
- scalar logs and failure codes;
- checkpoint hashes;
- validation selection record.

Resume requires all authoritative digests to match.

## Fairness

Paper-v1 uses three declared seeds. Failed seeds remain in the record.
Differences in parameters, training compute, environment interactions, or
pretraining are disclosed and normalized or analyzed separately.

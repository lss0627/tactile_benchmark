# Data Collection and Training Contract

## Expert adapter

An expert adapter declares:

- type and version;
- required observations;
- public action contract;
- source/checkpoint/config identity;
- license/provenance;
- deterministic seed behavior.

It emits public actions and may not bypass environment safety or task success.

## Collection job

Required inputs:

- suites/tasks/variants/split;
- expert;
- requested episodes;
- parallel environment count;
- seed schedule;
- retry and retention policy;
- output namespace.

Required behavior:

- crash-safe progress;
- unique episode IDs;
- bounded retries;
- explicit successful/failed/invalid counts;
- resume without duplication;
- episode validation before promotion.

## Episode

Required:

- observations/modalities/masks;
- joint and end-effector state;
- requested/executed actions;
- Contact/valid force fields;
- task reward/phase/state/success;
- timestamps;
- complete randomization parameters;
- task/split/sensor/expert/runtime/source identities.

## Training entry point

Shared inputs:

- algorithm;
- dataset/splits;
- suites/tasks;
- modalities;
- normalization;
- observation/action horizons;
- seed;
- optimizer/schedule;
- training budget;
- validation selection;
- output namespace.

Shared outputs:

- command/config;
- training/validation logs;
- checkpoints and hashes;
- selected checkpoint evidence;
- data/source/runtime metadata;
- manifest/checksums.

## Fairness

Matched comparisons freeze data, splits, sampling, action contract, horizons, budgets, validation selection, and evaluation schedule. Differences are declared in policy capability and compute metadata.

## Offline/online regime

Every run declares `OFFLINE` or `ONLINE`.

Online runs additionally record environment steps, generated data, exploration/adaptation privileges, and update budgets.

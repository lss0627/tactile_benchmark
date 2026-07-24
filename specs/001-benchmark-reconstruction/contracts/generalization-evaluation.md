# Generalization Evaluation Contract

## Training command

```bash
python scripts/train.py \
  --algo <bc|act|diffusion_policy|transformer|univtac> \
  --suite <suite-id> \
  --tasks <task-ids> \
  --modalities <vision tactile proprio> \
  --dataset <dataset-config> \
  --seed <seed> \
  --output <new-directory>
```

Training validates dataset/split identity, modalities, normalization, horizons, seed, budget, output nonexistence, and algorithm capability before training.

## Evaluation command

```bash
python scripts/evaluate.py \
  --checkpoint <checkpoint> \
  --benchmark configs/benchmark/tactilibero_v1.yaml \
  --protocol <GP-01|GP-02|GP-03> \
  --seeds <declared-seeds> \
  --output <new-directory>
```

## Evaluation preflight

- source/runtime identity;
- benchmark/task/sensor/policy/protocol versions;
- split manifest and leakage audit;
- supported policy/protocol/modality combination;
- frozen seed/episode/budget schedule;
- checkpoint and training-run provenance;
- output directory nonexistence.

## Required outputs

```text
command.log
schedule.json
episodes.jsonl
failures.jsonl
summary.json
summary.csv
radar.json
report.html
manifest.json
checksums.sha256
submission/
```

## Generalization result

```yaml
seen:
  episode_count: integer
  success_rate: float
unseen:
  episode_count: integer
  success_rate: float
generalization_gap:
  absolute: float  # seen - unseen
  relative: float | null
```

Seen and unseen share policy, policy seed, budgets, and metric versions.

## Metric validity

Every metric declares formula/version, source fields, units, validity predicate, aggregation, and unavailable behavior.

Invalid source data produces an unavailable metric and explicit count, never a fabricated zero.

## Result bundle

The bundle contains per-episode results, aggregates, training/policy capability, protocol/split/leakage identities, runtime metadata, and checksums.

Leaderboard validation recomputes aggregates and rejects mismatches.

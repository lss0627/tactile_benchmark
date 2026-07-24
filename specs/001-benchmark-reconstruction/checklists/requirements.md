# Specification Quality Checklist

## Product and scope

- [x] Scientific question is generalization, not task-count expansion.
- [x] Paper-v1 is fixed at four suites, 16 tasks, and three core protocols.
- [x] Environment, data generation, official dataset, training, evaluation,
  baselines, and leaderboard are all required.
- [x] Offline and online benchmark modes are both required.
- [x] Simulation-only and non-certification scope is explicit.
- [x] Expansion to 100 tasks, extra protocols, large VLAs, hosted evaluation,
  and real robots is non-blocking.

## Contracts and truth

- [x] Isaac Sim 6.0.1/Python 3.12 baseline is explicit.
- [x] Development/reference-driver distinction is explicit.
- [x] G1 empirical acceptance is measurable and optional formal diagnostics do
  not block it.
- [x] Task-state success rejects geometric fallback.
- [x] Contact, scalar force, vector force, wrench, and raw impulse are distinct.
- [x] Task, suite, split, expert, collection, episode, dataset, training,
  evaluation, result, and leaderboard contracts are covered.
- [x] Train/validation/test leakage rules are explicit.
- [x] Public extension registries are specified.

## Measurability and consistency

- [x] Dataset minimum, policy seeds, and evaluation episode minimum are numeric.
- [x] Generalization gap and tactile-specific metrics are defined.
- [x] Every functional requirement is testable.
- [x] Every success criterion is measurable.
- [x] Gate order and claim limits are explicit.
- [x] Historical evidence immutability is explicit.
- [x] No `NEEDS CLARIFICATION`, TODO, TBD, or unresolved placeholder remains.
- [x] Final documentation links, paths, task traceability, and cross-artifact
  analysis pass.

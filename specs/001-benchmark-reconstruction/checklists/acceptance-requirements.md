# Acceptance Requirements Checklist

## G0

- [x] Clean-checkout and evidence infrastructure exists.
- [x] Isaac Sim 6.0.1/Python 3.12 baseline exists.
- [ ] Rebaseline-bound G0 evidence is refreshed.

## G1

- [ ] Public `make_env/reset/step/close` path passes.
- [ ] Button success is task-state based.
- [ ] Runtime hard guards pass.
- [ ] Contact/raw-contact truth and masks pass.
- [ ] 100 reset cycles pass.
- [ ] Rendered 500-step rollout passes.
- [ ] 10 consecutive press/release/retract episodes pass.
- [ ] Media and fresh evidence pass.
- [x] Optional formal diagnostics are excluded from mandatory G1 dependencies in the specification.

## G2

- [ ] Factory/action/observation/info/lifecycle contracts pass.
- [ ] Seed determinism and lazy imports pass.
- [ ] Contract snapshots are fresh.

## G3

- [ ] Tactile capability negotiation and synchronization pass.
- [ ] Native/derived/unavailable/mock sources are distinct.
- [ ] Expert registry and collection job contracts pass.
- [ ] Resume, retry, filtering, statistics, and data validation pass.

## G4

- [ ] Four suites and exactly 16 task cards pass.
- [ ] GP-01/GP-02/GP-03 split manifests and leakage audits pass.
- [ ] At least 50 accepted training demonstrations per task and 800 total pass.
- [ ] Test-only variants contribute zero training demonstrations.
- [ ] Dataset schema, duplicates, hashes, masks, and randomization metadata pass.
- [ ] Simulator replay and dataset card pass.
- [ ] Licenses and asset provenance pass.

## G5

- [ ] Unified BC/ACT/Diffusion/Transformer/UniVTAC-compatible training passes.
- [ ] Vision-only, tactile-only, and fusion modality contracts pass.
- [ ] Offline and online training paths use declared shared preprocessing.
- [ ] GP-01/GP-02/GP-03 evaluation records are complete.
- [ ] Seen/unseen aggregates, generalization gaps, tactile metrics, and
  uncertainty pass.
- [ ] JSON/CSV/radar/HTML artifacts regenerate.

## G6

- [ ] Scripted/oracle and five learned algorithm configurations are reported.
- [ ] Matched vision-only, tactile-only, and fusion comparisons pass.
- [ ] Three seeds and at least 20 episodes per task condition per seed pass.
- [ ] Static leaderboard validates and reproduces from result bundles.
- [ ] Final release package is complete.
- [ ] Reference/validated-driver rerun passes.
- [ ] Paper claims match completed Gates.

# Risk Register

| Risk | Impact | Mitigation | Gate |
|---|---|---|---|
| G1 formal diagnostics consume the project | No benchmark/paper | Use empirical accepted runtime; keep optional diagnostics bounded | G1 |
| Scope grows toward 100 tasks | Shallow, unfinished benchmark | Freeze paper-v1 at four suites/16 tasks | All |
| Work looks like UniVTAC task expansion | Weak novelty | Make controlled generalization protocol the primary contribution | G4–G6 |
| Environments exist without training/collection | Scene collection, not a platform | Require official/online data and unified trainer | G3–G5 |
| Success uses a proxy | Invalid results | Task-state predicates and phase tests | G1/G4 |
| Contact/force fields are conflated | False claims | Source labels, masks, no proxy vectors/wrenches | G1/G3 |
| Reset/sensor lifecycle is unstable | Invalid data/evaluation | deterministic reset, readiness, freshness tests | G1–G3 |
| Randomization metadata is missing | Splits cannot be audited | Episode-level factor provenance | G3/G4 |
| Seen/unseen leakage | Invalid generalization claims | immutable split manifests and leakage audit | G4/G5 |
| Dataset count hides duplicates/failures | Inflated data claims | schema/hash/duplicate/replay/rejection reports | G4 |
| Baselines use different preprocessing | Unfair comparison | shared loader/normalization/horizon/selection | G5/G6 |
| Online methods use more interactions | Misleading comparison | report interactions, accepted data, updates, wall time | G5/G6 |
| Sensor transfer is only simulated | Overstated cross-sensor claim | distinguish calibrated devices from simulated domain shift | G3/G5 |
| Runtime-invalid episodes are dropped | Inflated metrics | retain validity/failure taxonomy in result bundles | G5 |
| Hosted leaderboard executes untrusted code | Security/operations burden | paper-v1 uses validated static result bundles | G6 |
| Driver is non-reference | Weak release reproducibility | retain `UNVALIDATED`; rerun at G6 | G6 |
| External assets/plugins violate licenses | Legal risk | manifests, license checks, registry acceptance | G4/G6 |
| Historical blockers are relabeled | Evidence-integrity loss | immutable evidence and explicit rebaseline decisions | All |

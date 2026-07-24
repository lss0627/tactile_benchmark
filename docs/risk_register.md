# Risk Register

| Risk | Impact | Mitigation | Gate |
|---|---|---|---|
| Formal diagnostics consume the entire project | No benchmark or paper | Keep them optional and bounded; G1 uses empirical runtime acceptance | G1 |
| Task success uses a proxy | Invalid results | Require task-state predicates and tests | G1/G4 |
| Contact/force fields are conflated | False scientific claims | Source labels and validity masks; no proxy vectors/wrenches | G1/G3 |
| Reset is unstable | Invalid datasets/evaluation | 100-cycle G1 acceptance and deterministic reset tests | G1/G2 |
| Camera frames are stale | Invalid visual baseline | Render-tick/update/timing checks | G1/G2 |
| Too many tasks too early | Low-quality suite | Accept PressButton first; target eight tasks | G1/G4 |
| Dataset count hides duplicates or invalid records | Inflated data claims | Hash, schema, duplicate, balance, and replay checks | G4 |
| Runtime-invalid episodes distort metrics | Misleading evaluation | Separate runtime validity and failure taxonomy | G5 |
| Visual/tactile baselines are unmatched | Invalid comparison | Freeze data, model budget, tasks, splits, and evaluation | G6 |
| Driver is non-reference | Weak release reproducibility | Record `UNVALIDATED`; rerun on reference/validated driver at G6 | G6 |
| Historical blockers are relabeled | Evidence integrity loss | Keep evidence immutable and distinguish rebaseline from retroactive pass | All |
| Reuse from UniVTAC/LIBERO violates license/API assumptions | Legal or technical risk | Review licenses, assets, simulator version, and contracts before reuse | G4/G6 |
| Eight-task target is still too large | Schedule slip | Stage task acceptance; release fewer only through explicit paper-scope review | G4 |
| Tactile hardware/plugin is unavailable | Blocks tactile claim | Capability negotiation; benchmark may expose unavailable masks, but tactile paper claim waits | G3/G6 |

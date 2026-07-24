# Research Decisions

## Decision 1 — Generalization protocol is the main contribution

**Decision**: Position TactiLIBERO around leakage-safe seen/unseen evaluation rather than environment count.

**Rationale**: A protocol that separates object, contact/physics, and sensor generalization answers a clearer scientific question than a larger task collection.

**Alternative rejected**: “UniVTAC++” or task-count-first positioning.

## Decision 2 — Complete loop before large scale

**Decision**: Paper-v1 contains 16 tasks across four suites, plus data generation, official data, unified training, evaluation, and baselines.

**Rationale**: A complete 16-task benchmark is more useful and publishable than 100 shallow tasks without training/data support.

**Future extension**: Expand to 100 instances only after the complete loop is stable.

## Decision 3 — Four task suites

**Decision**: Precision, Articulation, Surface Interaction, and Deformable Contact.

**Rationale**: They span contact geometry, articulated mechanisms, sustained surface interaction, and compliant/deformable behavior.

## Decision 4 — Three core protocols

**Decision**: Object/Geometry, Contact/Material/Physics, and Sensor/Observation generalization.

**Rationale**: These directly test why tactile information matters while keeping paper-v1 experimentally tractable.

**Future extension**: Trajectory, task transfer, scene, recovery stress, and continual learning.

## Decision 5 — Offline and online modes

**Decision**: Publish a standard offline dataset and support online environment interaction/collection.

**Rationale**: Offline data enables fair, low-cost reproduction; online access enables closed-loop contact control, reinforcement learning, active touch, and data-efficiency work.

## Decision 6 — Multiple expert sources

**Decision**: One collection contract supports scripted oracle, classical controller, teleoperation, trained policy, human demonstration, and community adapters.

**Rationale**: Dataset provenance and action semantics must remain comparable regardless of the expert source.

## Decision 7 — Batch collector as a first-class product

**Decision**: Require parallel environments, resume, retries, filtering policy, progress journal, statistics, and validation.

**Rationale**: Reliable large-scale collection is necessary for a community benchmark, not an auxiliary script.

## Decision 8 — Randomization metadata is mandatory

**Decision**: Store all protocol-relevant object, material, friction, compliance, trajectory, scene, sensor, noise, latency, drift, and seed values.

**Rationale**: Generalization splits cannot be audited after collection if domain parameters are missing.

## Decision 9 — Unified trainer

**Decision**: Shared data loading, normalization, horizons, seeds, budgets, logging, checkpointing, and validation selection wrap algorithm-specific adapters.

**Rationale**: Otherwise baseline differences can come from preprocessing or training budget rather than policy quality.

## Decision 10 — Paper-v1 algorithms

**Decision**: BC, ACT, Diffusion Policy, Transformer, and UniVTAC-compatible configurations.

**Rationale**: They cover simple imitation, action chunking, diffusion, sequence modeling, and tactile-specific modeling.

**Future extension**: OpenVLA and π0 after adapter/license/compute validation.

## Decision 11 — Dataset minimum

**Decision**: At least 50 accepted training demonstrations per task plus a declared validation set.

**Rationale**: The 800-episode minimum is manageable and supports a first data-scaling study. More data can be released without changing the split contract.

## Decision 12 — Split manifests and leakage audits

**Decision**: Each protocol uses immutable train/validation/test-seen/test-unseen manifests and protocol-specific forbidden-overlap checks.

**Rationale**: “Unseen” requires explicit identity and parameter boundaries.

## Decision 13 — Sensor adaptation modes

**Decision**: Zero-shot, calibration-only, and task-data adaptation results are separate.

**Rationale**: Combining them would overstate cross-sensor generalization.

## Decision 14 — Metric validity

**Decision**: Contact, slip, recovery, and force metrics require declared source fields and validity masks.

**Rationale**: Rich metrics must not fabricate physical measurements.

## Decision 15 — One-command training and evaluation

**Decision**: Canonical `train.py` and `evaluate.py` entry points produce versioned, complete artifacts.

**Rationale**: Low-friction, uniform execution is essential for adoption and fairness.

## Decision 16 — Static leaderboard first

**Decision**: Require offline bundle validation and a static leaderboard; hosted checkpoint execution is optional.

**Rationale**: Hosted untrusted-code execution adds security and operations beyond paper-v1.

## Decision 17 — G1 remains blocking

**Decision**: Pass PressButton before formal multi-task collection.

**Rationale**: This satisfies the constitution and protects later data from an unaccepted runtime.

## Decision 18 — Novelty wording is conditional

**Decision**: Use “a contact-rich generalization benchmark” until the final literature audit justifies “the first.”

**Rationale**: Novelty claims require evidence.

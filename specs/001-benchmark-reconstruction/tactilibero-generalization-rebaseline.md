# TactiLIBERO Generalization Rebaseline

**Date**: 2026-07-24  
**Decision**: Approved  
**Working title**: *TactiLIBERO: A Generalization Benchmark for Contact-Rich Manipulation*

## Why the scope changed

An eight-task UniVTAC-style extension would overlap existing task families and
would mainly contribute more environments. A 100-task first release would
increase engineering cost before the scientific protocol is proven. Neither
scope answers the central question:

> Can a contact-rich manipulation policy generalize beyond the objects,
> contact conditions, and sensing configuration used for training?

Paper-v1 therefore prioritizes evaluation protocol, controlled splits, data
and training fairness, and reusable tooling over raw task count.

## Approved paper-v1 product

```text
4 task suites / 16 tasks
+ 3 core generalization protocols
+ offline official dataset
+ online data collection
+ unified offline and online training
+ evaluation toolkit
+ 5 learned algorithm configurations
+ static leaderboard
```

The task suites are:

1. Precision;
2. Articulation;
3. Surface Interaction;
4. Deformable Contact.

The required protocols are:

1. object and geometry generalization;
2. contact, material, and physics generalization;
3. sensor and observation generalization.

Each task must provide scene/object/robot/sensor definitions, deterministic
reset and declared randomization, success and failure predicates, reward or
phase labels, and leakage-audited train/validation/test variants.

## Data and training are mandatory

The benchmark is incomplete without both:

- an official offline dataset for reproducible supervised learning; and
- online environment access for collection, active interaction, online
  training, and data-efficiency research.

Collection must support scripted experts, traditional controllers,
teleoperation, trained-policy rollout, human demonstrations, and registered
community adapters. Training must use one interface and shared preprocessing
for BC, ACT, Diffusion Policy, Transformer, and UniVTAC-compatible
configurations, including vision-only, tactile-only, and fused modalities.

## Fixed paper-v1 scale

- exactly 16 accepted task instances;
- at least 50 accepted training demonstrations per task;
- at least 800 accepted training demonstrations in total;
- declared validation data;
- zero training demonstrations from test-only variants;
- three policy seeds;
- at least 20 evaluation episodes per task condition per seed.

Changing these constants requires a new versioned decision and corresponding
Spec Kit updates.

## Extension boundary

The following remain supported extension points but do not block paper-v1:

- expansion toward 100 task instances;
- trajectory, task, scene, continual, or lifelong protocols;
- OpenVLA and π0 adapters;
- hosted evaluation of untrusted checkpoints;
- real-robot and sim-to-real studies.

## Gate mapping

- G1 accepts one PressButton reference runtime.
- G2 freezes factories, registries, and public contracts.
- G3 accepts tactile/sensor lifecycle and collection infrastructure.
- G4 accepts all 16 tasks, official data, validation, and replay.
- G5 accepts unified training and the three core generalization protocols.
- G6 accepts baseline results, the static leaderboard, release artifacts, and
  reference-driver revalidation.

No prior G0/G1 evidence is relabeled as evidence for these new product claims.

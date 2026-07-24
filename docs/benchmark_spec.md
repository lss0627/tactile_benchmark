# TactiLIBERO Benchmark Specification

## Objective

TactiLIBERO is a generalization benchmark for contact-rich manipulation. It
combines Isaac Sim tactile environments with LIBERO-style task suites and
controlled train/test protocols. The core contribution is the protocol and
toolchain, not the number of scenes.

## Paper-v1 composition

```text
robot: FR3
simulator: Isaac Sim 6.0.1
task suites: 4
task instances: 16
core generalization protocols: 3
learned algorithm configurations: 5
official training demonstrations: >= 800
policy seeds: 3
evaluation episodes: >= 20 per task condition per seed
```

The six mandatory deliverables are the task suite, data generation, standard
dataset, training pipeline, evaluation protocol, and baseline results.

## Task suites

| Suite | Tasks | Main variation |
|---|---|---|
| Precision | PegInsert, USBLikeInsert, KeyInsert, PinSocketInsert | geometry, clearance, initial offset |
| Articulation | PressButton, ToggleSwitch, OpenDrawer, TwistCap | stiffness, damping, direction |
| Surface Interaction | SlideToTarget, WipeSurface, ScrapeSurface, SurfaceFollow | friction, material, speed |
| Deformable Contact | SoftPress, SpongeCompress, FabricManipulate, CableSeat | compliance, deformation, contact pattern |

Names and asset choices become authoritative only through accepted task cards.

## Core protocols

- **GP-01 Object and Geometry**: unseen instances, dimensions, clearances, and
  initial geometric combinations.
- **GP-02 Contact, Material, and Physics**: unseen friction, stiffness,
  compliance, force/contact patterns, or material combinations.
- **GP-03 Sensor and Observation**: unseen sensor configuration, noise, delay,
  dropout, drift, or modality availability.

Every protocol provides versioned train, validation, seen-test, and
unseen-test manifests plus a leakage audit. Test-only variants contribute zero
training demonstrations.

## Environment contract

Every task supplies:

- scene, objects, robot, and tactile sensor definitions;
- deterministic reset plus declared randomization;
- task-state success and failure predicates;
- reward and/or phase labels;
- action, observation, timing, and safety budgets;
- train/validation/test eligibility.

G1 PressButton first proves the reference runtime with 100 resets, a rendered
500-step rollout, and 10 consecutive task-state episodes.

## Data and online interaction

The official offline dataset enables low-cost, fair reproduction. The same
environment also supports online collection and training.

Expert sources include scripted oracle, traditional controller,
teleoperation, trained-policy rollout, human demonstration, and registered
community adapters. Each episode stores visual/tactile/proprioceptive
observations, actions, task phase, timestamps, outcomes, and all
randomization/split provenance.

## Training

One `train.py` interface covers:

- Behavior Cloning;
- ACT;
- Diffusion Policy;
- Transformer policy;
- UniVTAC-compatible policy.

Vision-only, tactile-only, and vision–tactile fusion variants use shared data
loading, normalization, horizon, seed, checkpoint, logging, and validation
selection contracts.

## Evaluation

One `evaluate.py` command emits per-episode and aggregate records for:

- success and generalization gap;
- completion time and action smoothness;
- maximum/cumulative valid contact force;
- contact and slip counts;
- recovery success;
- tactile-missing degradation;
- runtime validity and safety failures.

Outputs are JSON, CSV, radar plots, HTML reports, and a signed result bundle
used by a static leaderboard.

## Limitations

- Simulation-only.
- Development driver `550.144.03` is `UNVALIDATED`.
- CPU physics/MBP is the accepted Contact path.
- No vector-force or wrench claim without independent validation.
- No real-robot or sim-to-real claim.
- 100-task, continual/lifelong, OpenVLA/π0, and hosted leaderboard extensions
  are outside paper-v1 acceptance.

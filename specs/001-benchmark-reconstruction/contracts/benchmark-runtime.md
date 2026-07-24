# Benchmark Runtime Contract

## Public lifecycle

```python
env = make_env(task=task_id, variant=variant_id, sensors=sensor_ids)
observation, info = env.reset(seed=seed)
observation, reward, terminated, truncated, info = env.step(action_7d)
env.close()
```

Offline collection, online training, and evaluation use this same boundary.

## Action

- exact shape `[7]`;
- finite;
- declared translation/rotation frame and units;
- bounded before execution;
- requested and executed values retained;
- invalid action fails before actuation.

## Observation

Required groups:

- proprioception;
- end-effector pose;
- task state;
- reward or task phase;
- RGB/depth;
- Contact/raw Contact;
- optional tactile;
- timestamps;
- source/validity masks.

Every field has a versioned shape, dtype, units, frame, source, and validity rule.

## Task

Task cards define:

- reset/randomization;
- success/failure;
- reward or phase labels;
- budgets;
- variant/split identity;
- required sensor capabilities.

Success derives from task state.

## Runtime safety

Mandatory:

- finite state/action;
- joint/workspace limits;
- configured motion limits;
- collision/sustained-penetration checks;
- step/time/action budgets;
- abort latch;
- zero post-abort actuation;
- safe retract where required.

## Contact and tactile truth

- Raw Contact/collision is retained when valid.
- Scalar force remains scalar.
- Vector force/wrench remains unavailable unless validated.
- Raw impulse is not force.
- Failed samples are retained before abort when observed.

## Randomization record

Every reset exposes protocol-relevant randomization parameters through `info` and the episode writer, including applicable object, material, physics, trajectory, scene, sensor, noise, latency, drift, and seed values.

## Determinism

Task variant plus reset seed uniquely determines the declared randomized initial condition. Runtime nondeterminism is measured and documented rather than hidden.

## Optional diagnostics

Formal sweep/GJK/cooked-shape diagnostics remain opt-in, bounded, and unable to change task success or Gate status alone.

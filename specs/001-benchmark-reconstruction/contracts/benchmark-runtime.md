# Benchmark Runtime Contract

## Public lifecycle

```python
env = make_env(config)
observation, info = env.reset(seed=seed)
observation, reward, terminated, truncated, info = env.step(action_7d)
env.close()
```

`close()` is safe to call after success, failure, abort, or partial initialization.

## Action

```text
shape: [7]
dtype: float32 or float64 at input; normalized internally
fields:
  0:3 translation delta
  3:6 rotation delta
  6   gripper command
```

Requirements:

- exact length seven;
- finite;
- declared frame and units;
- bounded before execution;
- requested and executed values retained;
- invalid input fails before actuation.

## Observation

Required groups:

- robot proprioception;
- task state;
- RGB;
- depth;
- Contact/raw Contact;
- optional tactile;
- timestamps and validity masks.

Every field has a versioned shape, dtype, units, frame, source, and validity rule.

## PressButton success

```text
success = observed_button_press
       && observed_button_release
       && safe_retract
       && runtime_valid
```

No geometric fallback is allowed for benchmark evidence.

## Runtime safety

Mandatory guards:

- finite values;
- joint/workspace limits;
- configured exact per-step motion limit;
- collision and sustained-penetration checks;
- action/step/wall-time budgets;
- abort latch;
- zero post-abort actuation;
- safe retract.

## Contact truth

- Raw Contact/collision evidence is authoritative when valid.
- Scalar force remains scalar.
- Vector force and wrench remain unavailable unless separately validated.
- Raw impulse is not force.
- A failed sample is retained before abort whenever it was observed.

## Reset contract

A ready reset establishes:

- valid articulation and joint order;
- declared initial task state;
- live Contact/camera/tactile handles within the readiness window;
- deterministic seed provenance;
- no stale handle from the previous lifecycle.

## Evidence contract

The runner records:

- runtime/config/task/asset/source identity;
- reset, rollout, and episode records;
- requested/executed actions;
- observations and masks;
- task-state success;
- safety and failure codes;
- camera timing and media;
- checksums and freshness.

Evidence is written before the unique simulator shutdown.

## Optional diagnostic contract

Formal motion/geometry diagnostics:

- are opt-in;
- use bounded time/work;
- use `runtime_smoke`;
- cannot alter public action or success;
- cannot override runtime Contact/collision;
- cannot pass or block G1 by themselves.

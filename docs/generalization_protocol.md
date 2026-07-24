# Generalization Protocol

## Shared protocol structure

Every protocol fixes:

- factor ontology and allowed values;
- train/validation/seen-test/unseen-test generation;
- minimum cells and episode budgets;
- leakage rules;
- aggregation and uncertainty;
- result-bundle schema.

All splits are generated before training. Test performance never changes split
membership or checkpoint selection.

## GP-01 Object and Geometry

Training and validation use declared object instances and geometry
combinations. Unseen tests hold out object identity, dimensions, clearance, or
initial alignment combinations while preserving task semantics.

Examples:

- train on peg diameters A/B, test on C;
- train on bottle A, test on bottle B;
- train on selected insertion clearances, test on held-out clearances.

## GP-02 Contact, Material, and Physics

Unseen tests hold out friction, stiffness, compliance, damping, material, or
contact-pattern combinations.

Examples:

- train on light/medium press, test on heavy press;
- train on plastic/wood, test on metal/rubber;
- train on selected friction–speed pairs, test on held-out pairs.

## GP-03 Sensor and Observation

Unseen tests change sensor or observation conditions without changing task
success.

Examples:

- sensor model/configuration transfer;
- unseen noise, delay, drift, dropout, or calibration perturbation;
- tactile missingness and camera degradation.

Native cross-device claims require compatible calibrated sensors. Otherwise
the protocol must call the change a simulated sensor-domain shift.

## Measures

For metric \(m\), the primary generalization gap is:

```text
GeneralizationGap(m) = Seen(m) - Unseen(m)
```

For success, lower is better. Reports also include unseen absolute
performance, retained fraction, per-suite gaps, and confidence intervals.

## Extension protocols

Trajectory, task, scene, robustness, and continual/lifelong protocols may be
added with the same manifest contract. They are not required for paper-v1 and
must not be mixed into the three core aggregate scores without a versioned
decision.

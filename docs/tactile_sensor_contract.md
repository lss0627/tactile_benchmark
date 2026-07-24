# Tactile Sensor Contract

## Capability classes

Every tactile/contact field declares one source:

```text
NATIVE_MEASUREMENT
DERIVED_MEASUREMENT
UNAVAILABLE
MOCK
```

Mocks cannot enter benchmark evidence or released datasets.

## Contact

Required when available:

- reading validity and freshness;
- `in_contact`;
- scalar force magnitude;
- raw contact body pairs;
- raw position/normal/impulse;
- physics step and timestamp;
- Contact/raw-contact counts.

## Vector force and wrench

```text
force_vector_valid: false
wrench_valid: false
```

remain the default until independently validated. Scalar force magnitude, raw impulse, geometry, TCP pose, button travel, or success cannot populate these fields.

## Tactile images/fields

Each native tactile modality declares:

- shape and dtype;
- units/normalization;
- sensor frame and extrinsics;
- timestamp;
- valid mask;
- saturation/background rule;
- calibration/config digest.

The sensor profile also declares which protocol dimensions it supports:
device/model identity, resolution, noise, delay, dropout, drift, calibration,
and modality availability. GP-03 test configurations cannot leak into training
normalization or augmentation.

## Synchronization

Record control, physics, camera, Contact, and tactile timestamps. Maximum permitted skew is declared by the observation contract and validated at G3.

## Reset lifecycle

After reset:

- sensor enters ready within the bounded window;
- handles are fresh;
- no previous-scene data is reused;
- first valid sample step is recorded;
- invalid/unavailable states are masked rather than zero-filled.

## Public observation

Fields may include stable names for compatibility, but consumers must use validity/source metadata. An unavailable field is not a physical zero.

## Collection and training

- Dataset records retain native values plus calibration/config provenance.
- Training normalization uses train-split statistics only.
- Sensor dropout and missing modalities use explicit masks.
- Tactile-only, vision-only, and fusion policies receive no hidden modality.
- Community sensors register shape, timing, frame, lifecycle, and calibration
  contracts before producing benchmark-compatible episodes.

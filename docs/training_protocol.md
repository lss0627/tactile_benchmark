# Training Protocol

## Frozen inputs

Before training:

- G4 dataset version and splits;
- task-card versions;
- observation/action contracts;
- preprocessing;
- model family and budget;
- optimizer/schedule;
- seed list;
- checkpoint selection rule.

## Default seeds

Use three declared training seeds. Never replace a failed seed without retaining the failure and documenting the replacement.

## Data handling

- use only accepted training episodes;
- preserve modality masks;
- no test/OOD leakage;
- no privileged replay metadata;
- log duplicate filtering and sample weights;
- keep visual and visual-tactile data selection matched.

## Checkpoint selection

Select checkpoints using the declared validation metric only. Test results cannot influence selection.

## Reproducibility record

Store:

- environment/source/data/config hashes;
- seed;
- command;
- hardware/runtime metadata;
- training logs;
- checkpoint hashes;
- selection result;
- failure codes.

## Compute fairness

Match training steps and optimization settings. If tactile input changes parameters or compute materially, report both and include a matched-compute or matched-capacity comparison.

# Isaac Sim 6.0.1 candidate environment

This is the reproducible G0/G-1B candidate input. It is promoted to the
development lock only after G-1B passes.

- Runtime: Python 3.12.13
- pip: 26.1.2
- setuptools: 78.1.0
- wheel: 0.47.0
- PyTorch: 2.11.0+cu128
- torchvision: 0.26.0
- torchaudio: 2.11.0
- Isaac Sim: `isaacsim[all,extscache]==6.0.1.0`
- Observed driver: 550.144.03
- NVIDIA tested/reference driver: 595.58.03
- Project driver validation: `UNVALIDATED`

The observed driver is intentionally preserved. It is valid for development
compatibility evidence only; release physical/dataset/replay/evaluation gates
must be rerun on a currently validated/reference driver.

The NVIDIA EULA has been accepted for this local environment:

```bash
conda env config vars set -n isaac6 OMNI_KIT_ACCEPT_EULA=YES
```

Install with NVIDIA's package index and the pinned candidate lock. Do not
install or migrate Isaac Lab as part of this candidate.

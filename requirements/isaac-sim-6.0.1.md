# Isaac Sim 6.0.1 development baseline

Promoted after G0 and G-1B passed on 2026-07-10.

- Python 3.12.13
- Isaac Sim `isaacsim[all,extscache]==6.0.1.0`
- PyTorch 2.11.0+cu128
- NVIDIA driver 550.144.03 (`UNVALIDATED`)
- NVIDIA tested/reference driver 595.58.03
- GPU RTX rendering on `cuda:0`
- CPU physics for the experimental Contact Sensor

Install from the promoted lock:

```bash
python -m pip install --extra-index-url https://pypi.nvidia.com \
  -r requirements/lock-py312.txt
export OMNI_KIT_ACCEPT_EULA=YES
```

Native GPU physics Contact is blocked by
`GPU_CONTACT_NATIVE_INSTABILITY` before initialization. This is not a claim
that NVIDIA defines 550.144.03 as an absolute unsupported minimum; it records
that the driver is outside the tested/reference baseline and that the observed
native GPU path was unstable. Release physical/dataset/replay/evaluation gates
must be rerun on a validated/reference driver.

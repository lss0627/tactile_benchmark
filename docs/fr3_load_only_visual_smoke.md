# FR3 Load-Only Visual Smoke

This gate verifies that the locally bound FR3 USD can be loaded into an Isaac
Sim stage for visual inspection. It is not a benchmark run and it is not an FR3
controller integration.

## Scope

- Load the configured FR3 USD at `/World/FR3`.
- Create only a minimal ground plane, light, and camera.
- Optionally save a viewport screenshot.
- Write `status.json` with explicit non-benchmark metadata.

## Non-Scope

- No PressButton task is connected.
- No reset/step/read loop is executed.
- No articulation controller is created.
- No joint command is sent.
- No tactile force, wrench, or contact benchmark claim is made.

## Commands

Dry-run:

```bash
python scripts/run_fr3_load_only_visual_smoke.py \
  --robot-config configs/robots/fr3_real_articulation.yaml \
  --runtime-config outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml \
  --dry-run \
  --output outputs/fr3_load_only_visual_smoke/dry_run_status.json
```

Runtime smoke, only after the dry-run and tests pass:

```bash
CUDA_VISIBLE_DEVICES=1 conda run -n isaac python scripts/run_fr3_load_only_visual_smoke.py \
  --robot-config configs/robots/fr3_real_articulation.yaml \
  --runtime-config outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml \
  --headless \
  --webrtc \
  --save-screenshot \
  --max-runtime-seconds 60 \
  --output outputs/fr3_load_only_visual_smoke/status.json
```

## Expected Status Fields

The report must include `benchmark_result=false`,
`not_for_paper_claims=true`, `controller_connected=false`,
`articulation_control_enabled=false`, and `press_button_connected=false`.
If screenshot capture fails, the script records a warning instead of treating
the visual smoke as a benchmark failure.

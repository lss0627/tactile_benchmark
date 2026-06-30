# Isaac Sim WebRTC Visual Smoke Runtime

This guide is for the single-task `PressButton` visual smoke path. The default
checks and tests stay dry-run safe, but `scripts/run_press_button_visual_smoke.py`
can attempt a minimal Isaac Sim runtime only when local paths are configured and
the user runs it without `--dry-run`.

## Purpose

The first WebRTC smoke should prove that a developer can see a minimal scene,
robot, and button object through the Isaac Sim WebRTC Streaming Client. It is a
visual sanity check, not a benchmark result and not a paper metric.

## Readiness Check

Run readiness first:

```bash
python scripts/check_isaacsim_webrtc_ready.py \
  --config configs/backend/isaacsim_visual_smoke.yaml \
  --output outputs/isaacsim_visual_smoke/readiness.json
```

The default config intentionally leaves local Isaac Sim paths unset. Fill in
`isaacsim_app_path` and `isaacsim_python_path` in
`configs/backend/isaacsim_visual_smoke.yaml` before attempting real runtime.
Use `scene_usd_path` for an existing scene, or keep
`auto_create_minimal_scene: true` to create a primitive visual-smoke scene.

Dry-run status does not import Isaac Sim:

```bash
python scripts/run_press_button_visual_smoke.py \
  --config configs/backend/isaacsim_visual_smoke.yaml \
  --dry-run \
  --output outputs/isaacsim_visual_smoke/dry_run_status.json
```

## Local Client

For local workstation use, connect the WebRTC Streaming Client to `127.0.0.1`.
The exact Isaac Sim app launch command depends on installation style:

- Linux Isaac Sim install: `./isaac-sim.streaming.sh`
- Docker/headless setup: `./runheadless.sh`
- PIP install style: `isaacsim isaacsim.exp.full.streaming --no-window`

The template script `scripts/launch_isaacsim_webrtc_smoke.sh` documents these
paths but does not hard-code a user machine path.

```bash
export ISAACSIM_ROOT=/path/to/isaac-sim
bash scripts/launch_isaacsim_webrtc_smoke.sh
```

Then run the visual smoke from an Isaac Sim-capable Python environment:

```bash
python scripts/run_press_button_visual_smoke.py \
  --config configs/backend/isaacsim_visual_smoke.yaml \
  --headless \
  --webrtc \
  --save-screenshot \
  --max-runtime-seconds 60 \
  --output outputs/isaacsim_visual_smoke/runtime_status.json
```

Open the Isaac Sim WebRTC Streaming Client after the streaming app is running.
For local use, connect to `127.0.0.1`. For remote hosts, connect to the
reachable public/private host IP.

## Remote or Headless Machines

Remote/headless streaming normally needs:

- TCP `49100` open to the client;
- UDP `47998` open to the client;
- Docker host networking or equivalent port exposure;
- a reachable public IP, VPN path, or SSH/network tunnel configured by the
  user.

Docker setups commonly require host networking for Isaac Sim livestream.

## GPU Requirement

Isaac Sim livestream requires an NVIDIA GPU with NVENC support. A100-class GPUs
without NVENC cannot be used for Isaac Sim livestream even if they can run CUDA
compute workloads.

## Current Repository State

- `configs/backend/isaacsim_visual_smoke.yaml` is a planned readiness config.
- `scripts/check_isaacsim_webrtc_ready.py` checks config completeness and local
  path placeholders without launching Isaac Sim.
- `scripts/run_press_button_visual_smoke.py --dry-run` emits status JSON and
  does not import Isaac Sim.
- `scripts/run_press_button_visual_smoke.py` without `--dry-run` only attempts
  a runtime launch after readiness passes. It creates or loads a minimal
  `PressButton` visual scene and can attempt a screenshot.
- `scripts/launch_isaacsim_webrtc_smoke.sh` is a user-run template and prints
  WebRTC connection reminders. It does not open firewalls or change system
  network configuration.
- No reset, step, sensor read, or evaluation runtime is implemented.
- Runtime output is `runtime_status.json` and is marked
  `benchmark_result=false`, `visual_smoke_only=true`.

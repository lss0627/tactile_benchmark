#!/usr/bin/env bash
# PressButton visual smoke preparation launcher template.
#
# This script is intentionally a template. It does not hard-code local Isaac Sim
# paths and is not executed by tests beyond shell syntax validation.
#
# Linux local workstation:
#   export ISAACSIM_ROOT=/path/to/isaac-sim
#   ./isaac-sim.streaming.sh
#
# Docker/headless:
#   ./runheadless.sh
#   Docker WebRTC streaming usually needs host networking.
#
# PIP install style:
#   isaacsim isaacsim.exp.full.streaming --no-window
#
# Remote streaming notes:
# - Local client target is usually 127.0.0.1.
# - Remote/headless hosts need TCP 49100 and UDP 47998 reachable.
# - Public IP, firewall, NAT, and security-group setup are user-managed.
# - Isaac Sim livestream needs an NVIDIA GPU with NVENC support.
# - A100-class GPUs without NVENC cannot be used for Isaac Sim livestream.
# - This is visual smoke preparation, not a benchmark result.

set -euo pipefail

if [[ -z "${ISAACSIM_ROOT:-}" ]]; then
  echo "Set ISAACSIM_ROOT to your local Isaac Sim installation before launching WebRTC smoke." >&2
  exit 2
fi

if [[ ! -d "${ISAACSIM_ROOT}" ]]; then
  echo "ISAACSIM_ROOT does not exist: ${ISAACSIM_ROOT}" >&2
  exit 2
fi

cd "${ISAACSIM_ROOT}"

echo "Isaac Sim WebRTC visual smoke launch template"
echo "Local WebRTC client target is usually: 127.0.0.1"
echo "Remote machines should use their reachable public/private host IP."
echo "Ensure TCP 49100 and UDP 47998 are reachable from the client."
echo "Docker/headless setups usually need host networking."
echo "This script does not open firewalls or change system network settings."

if [[ -x "./isaac-sim.streaming.sh" ]]; then
  exec ./isaac-sim.streaming.sh
fi

if [[ -x "./runheadless.sh" ]]; then
  echo "Falling back to ./runheadless.sh; ensure Docker or container networking is configured for WebRTC." >&2
  exec ./runheadless.sh
fi

echo "No supported Isaac Sim streaming launcher found under ISAACSIM_ROOT." >&2
echo "For PIP installs, run: isaacsim isaacsim.exp.full.streaming --no-window" >&2
exit 2

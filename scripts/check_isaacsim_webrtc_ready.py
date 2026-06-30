#!/usr/bin/env python
"""Check planned Isaac Sim WebRTC visual-smoke readiness without launching runtime."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from isaac_tactile_libero.envs.isaacsim_backend_status import (
    load_isaacsim_visual_smoke_config,
    probe_isaacsim_visual_smoke,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/backend/isaacsim_visual_smoke.yaml")
    parser.add_argument("--output", default="outputs/isaacsim_visual_smoke/readiness.json")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = load_isaacsim_visual_smoke_config(args.config)
    payload = probe_isaacsim_visual_smoke(config).as_dict()
    payload["config_path"] = str(args.config)
    payload["visual_smoke_preparation"] = True
    payload["creates_isaac_sim_environment"] = False
    payload["runs_reset_step"] = False
    payload["downloads_assets"] = False
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if payload["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

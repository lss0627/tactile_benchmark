#!/usr/bin/env python
"""Probe the planned optional Lightwheel backend without connecting runtime."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from isaac_tactile_libero.envs.backend_config import load_backend_config
from isaac_tactile_libero.envs.lightwheel_wrapper import LightwheelEnvAdapter


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/backend/lightwheel_optional.yaml")
    parser.add_argument("--output", default="outputs/lightwheel_probe/status.json")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = load_backend_config(args.config)
    adapter = LightwheelEnvAdapter(cfg=config)
    payload = adapter.probe().as_dict()
    payload["config_path"] = str(args.config)
    payload["probe_only"] = True
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

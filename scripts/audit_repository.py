#!/usr/bin/env python
"""Audit required, generated, and external repository inputs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import yaml

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from isaac_tactile_libero.assets.resolver import resolve_external_asset
from isaac_tactile_libero.repository.audit import audit_repository


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        default="configs/repository/required_files.yaml",
    )
    parser.add_argument("--output")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = yaml.safe_load(Path(args.config).read_text(encoding="utf-8")) or {}
    resolutions = [
        resolve_external_asset(key)
        for key in config.get("external_asset_keys", [])
    ]
    report = audit_repository(
        ROOT,
        required_patterns=config.get("required_patterns", []),
        generated_patterns=config.get("generated_patterns", []),
        external_assets=[
            resolution.path
            for resolution in resolutions
            if resolution.path is not None
        ],
    )
    report["external_asset_resolution"] = [
        {
            "key": resolution.key,
            "path": str(resolution.path) if resolution.path else None,
            "source": resolution.source,
            "ok": resolution.ok,
            "diagnostic": resolution.diagnostic,
        }
        for resolution in resolutions
    ]
    report["clean_checkout_ready"] = bool(
        report["clean_checkout_ready"] and all(item["ok"] for item in report["external_asset_resolution"])
    )
    if args.output:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["clean_checkout_ready"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

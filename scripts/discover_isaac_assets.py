#!/usr/bin/env python
"""Discover local Isaac asset candidates without importing or launching Isaac Sim."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
from typing import Any

OFFICIAL_FR3_SUFFIX = "Robots/FrankaRobotics/FrankaFR3/fr3.usd"
DEFAULT_PATTERNS = ("FrankaFR3", "fr3.usd", "FrankaRobotics")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--roots", nargs="*", default=None, help="Roots to search read-only.")
    parser.add_argument("--patterns", nargs="*", default=list(DEFAULT_PATTERNS), help="Path/name substrings to find.")
    parser.add_argument("--output", help="Optional JSON report output path.")
    return parser.parse_args()


def _normalize_roots(roots: list[str] | None) -> list[Path]:
    candidates = list(roots or [])
    env_root = os.environ.get("ISAACSIM_ROOT")
    if env_root:
        candidates.append(env_root)
    if not roots:
        for default in ("/mnt/data", "/isaacsim_assets", str(Path.home() / "isaacsim_assets")):
            candidates.append(default)

    seen: set[str] = set()
    result: list[Path] = []
    for root in candidates:
        if not root:
            continue
        path = Path(root).expanduser()
        key = str(path)
        if key in seen:
            continue
        seen.add(key)
        if path.exists():
            result.append(path)
    return result


def _safe_walk(root: Path):
    for current, dirs, files in os.walk(root):
        dirs[:] = [name for name in dirs if name not in {".git", "__pycache__", ".cache"}]
        yield Path(current), dirs, files


def _matches_any(path_text: str, patterns: list[str]) -> bool:
    lowered = path_text.lower()
    return any(pattern.lower() in lowered for pattern in patterns if pattern)


def _is_usd(path: Path) -> bool:
    return path.suffix.lower() in {".usd", ".usda", ".usdc"}


def _recommend_fr3(candidates: list[str]) -> str | None:
    official = [path for path in candidates if path.replace("\\", "/").endswith(OFFICIAL_FR3_SUFFIX)]
    if official:
        return sorted(official)[0]
    exact = [path for path in candidates if Path(path).name.lower() == "fr3.usd"]
    return sorted(exact or candidates)[0] if candidates else None


def _asset_root_for(path: str | None, roots: list[Path]) -> str | None:
    if not path:
        return None
    normalized = path.replace("\\", "/")
    if normalized.endswith(OFFICIAL_FR3_SUFFIX):
        return normalized[: -len(OFFICIAL_FR3_SUFFIX)].rstrip("/")
    candidate = Path(path)
    for root in roots:
        try:
            candidate.relative_to(root)
            return str(root)
        except ValueError:
            continue
    return None


def _official_asset_root_for(path: str) -> str | None:
    normalized = path.replace("\\", "/")
    if normalized.endswith(OFFICIAL_FR3_SUFFIX):
        return normalized[: -len(OFFICIAL_FR3_SUFFIX)].rstrip("/")
    return None


def discover_isaac_assets(*, roots: list[str] | None, patterns: list[str] | None) -> dict[str, Any]:
    patterns = list(patterns or DEFAULT_PATTERNS)
    search_roots = _normalize_roots(roots)
    fr3_candidates: set[str] = set()
    gripper_candidates: set[str] = set()
    warnings: list[str] = []

    for root in search_roots:
        try:
            for current, dirs, files in _safe_walk(root):
                current_text = str(current)
                if current.name in {"FrankaFR3", "FrankaRobotics"} and _matches_any(current_text, patterns):
                    pass
                for filename in files:
                    path = current / filename
                    text = str(path)
                    lower = text.lower()
                    if Path(filename).name.lower() == "fr3.usd" or (
                        "frankafr3" in lower and _is_usd(path)
                    ):
                        fr3_candidates.add(text)
                    if _is_usd(path) and any(token in lower for token in ("gripper", "hand", "finger")):
                        gripper_candidates.add(text)
        except OSError as exc:
            warnings.append(f"Could not search {root}: {exc}")

    found_fr3 = sorted(fr3_candidates)
    recommended = _recommend_fr3(found_fr3)
    recommended_root = _asset_root_for(recommended, search_roots)
    found_asset_roots = sorted({root for root in (_official_asset_root_for(path) for path in found_fr3) if root})
    if not found_asset_roots:
        found_asset_roots = sorted(
            {
                root
                for root in (_asset_root_for(path, search_roots) for path in found_fr3)
                if root is not None
            }
        )
    if not found_asset_roots:
        found_asset_roots = [str(path) for path in search_roots]
    return {
        "found_fr3_usd_candidates": found_fr3,
        "found_gripper_candidates": sorted(gripper_candidates),
        "found_asset_roots": found_asset_roots,
        "recommended_fr3_usd_path": recommended,
        "recommended_asset_root": recommended_root,
        "patterns": patterns,
        "warnings": warnings,
        "imports_isaacsim": False,
        "imports_omni": False,
        "imports_carb": False,
        "loads_usd": False,
        "creates_articulation": False,
        "downloads_assets": False,
    }


def main() -> int:
    args = parse_args()
    report = discover_isaac_assets(roots=args.roots, patterns=args.patterns)
    if args.output:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

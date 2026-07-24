#!/usr/bin/env python
"""Validate evidence and review a formal gate without inventing statuses."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys
from typing import Any, Mapping

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from isaac_tactile_libero.evidence.manifest import (  # noqa: E402
    validate_evidence_manifest,
    validate_manifest_freshness,
)


def review_manifest(
    manifest: dict[str, Any],
    *,
    expected_gate: str,
    validate_schema: bool = True,
    validate_freshness: bool = True,
    current_repository_commit: str | None = None,
    semantic_inputs: Mapping[str, str | Path] | None = None,
) -> dict[str, Any]:
    errors: list[str] = []
    if manifest.get("gate_id") != expected_gate:
        errors.append(f"expected gate {expected_gate}, got {manifest.get('gate_id')}")
    if validate_schema:
        errors.extend(validate_evidence_manifest(manifest))
    freshness = {"fresh": True, "checked": 0, "changed": [], "missing": []}
    if validate_freshness:
        freshness = validate_manifest_freshness(
            manifest,
            current_repository_commit=current_repository_commit,
            semantic_inputs=semantic_inputs,
        )
        if not freshness["fresh"]:
            errors.append("evidence is stale")
    if expected_gate == "G0" and manifest.get("status") != "PASS_BENCHMARK":
        errors.append("G0 requires PASS_BENCHMARK")
    return {
        "ok": not errors,
        "gate": expected_gate,
        "status": manifest.get("status") if not errors else "BLOCKED",
        "claim_class": manifest.get("claim_class"),
        "freshness": freshness,
        "errors": errors,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gate", required=True, choices=[f"G{i}" for i in range(7)])
    parser.add_argument("--evidence", required=True)
    parser.add_argument("--output")
    args = parser.parse_args()
    manifest = json.loads(Path(args.evidence).read_text(encoding="utf-8"))
    current_commit = subprocess.run(
        ["git", "rev-parse", "HEAD"], check=True, capture_output=True, text=True
    ).stdout.strip()
    semantic_inputs = {
        str(reference["name"]): str(reference["uri"])
        for collection in ("configuration", "assets")
        for reference in manifest.get(collection, [])
        if isinstance(reference, dict) and reference.get("name") and reference.get("uri")
    }
    review = review_manifest(
        manifest,
        expected_gate=args.gate,
        current_repository_commit=current_commit,
        semantic_inputs=semantic_inputs,
    )
    if args.output:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(review, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(review, indent=2, sort_keys=True))
    return 0 if review["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

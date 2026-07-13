#!/usr/bin/env python
"""Fail when first-party Python uses removed/deprecated Isaac Sim imports."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from isaac_tactile_libero.repository.import_scan import scan_first_party_imports


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output")
    parser.add_argument(
        "--deprecated-as-error",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    args = parser.parse_args()
    report = scan_first_party_imports(ROOT)
    report["deprecated_as_error"] = bool(args.deprecated_as_error)
    report["ok"] = not report["errors"] and (
        not args.deprecated_as_error or not report["warnings"]
    )
    if args.output:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

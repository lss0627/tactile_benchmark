#!/usr/bin/env python
"""Compare the current pytest node IDs with the frozen migration baseline."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys
from typing import Iterable


def compare_nodeids(baseline: Iterable[str], current: Iterable[str]) -> dict:
    old = {line.strip() for line in baseline if "::" in line}
    new = {line.strip() for line in current if "::" in line}
    missing = sorted(old - new)
    added = sorted(new - old)
    return {
        "ok": not missing,
        "baseline_count": len(old),
        "current_count": len(new),
        "retained_count": len(old & new),
        "missing": missing,
        "added": added,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--python", default=sys.executable)
    args = parser.parse_args()
    baseline = Path(args.baseline).read_text(encoding="utf-8").splitlines()
    result = subprocess.run(
        [args.python, "-m", "pytest", "--collect-only", "-q"],
        text=True,
        capture_output=True,
        check=False,
    )
    current = result.stdout.splitlines()
    report = compare_nodeids(baseline, current)
    report["pytest_collect_returncode"] = result.returncode
    report["collect_stderr"] = result.stderr.splitlines()
    report["ok"] = bool(report["ok"] and result.returncode == 0)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

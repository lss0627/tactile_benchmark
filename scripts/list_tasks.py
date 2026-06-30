#!/usr/bin/env python
"""List registered Phase 1 task placeholders."""

from __future__ import annotations

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import isaac_tactile_libero  # noqa: F401
from isaac_tactile_libero.registry.task_registry import TASK_REGISTRY


def main() -> int:
    print(f"Registered tasks ({len(TASK_REGISTRY.list())}):")
    for entry in TASK_REGISTRY.entries():
        suite = entry.metadata.get("suite", "unknown")
        version = entry.metadata.get("version", "unknown")
        contact_rich = entry.metadata.get("contact_rich", False)
        tactile_necessary = entry.metadata.get("tactile_necessary", False)
        print(
            f"- {entry.name} | suite={suite} | version={version} | "
            f"contact_rich={contact_rich} | tactile_necessary={tactile_necessary}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

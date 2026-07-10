#!/usr/bin/env python
"""Thin wrapper for the minimal Isaac contact-force probe scene."""

from __future__ import annotations

import sys

from probe_isaac_contact_force_second import main


if __name__ == "__main__":
    if "--scene" not in sys.argv:
        sys.argv.extend(["--scene", "minimal"])
    raise SystemExit(main())

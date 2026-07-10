#!/usr/bin/env python
"""Summarize whether FR3 PressButton press smoke is ready for one eval run.

This gate deliberately keeps dataset collection disabled. It only checks the
press smoke artifacts and reports whether a future single-episode real FR3
PressButton evaluation gate can be attempted.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--partial-2mm-status", default="outputs/fr3_press_button_press_runtime/partial_press_2mm_status.json")
    parser.add_argument("--partial-10mm-status", default="outputs/fr3_press_button_press_runtime/partial_press_10mm_status.json")
    parser.add_argument("--full-press-status", default="outputs/fr3_press_button_press_runtime/full_press_status.json")
    parser.add_argument("--press-and-retract-status", default="outputs/fr3_press_button_press_runtime/press_and_retract_status.json")
    parser.add_argument("--output", default="outputs/fr3_press_button_press_runtime/dataset_readiness.json")
    return parser.parse_args()


def _read_json(path: str | Path) -> tuple[dict[str, Any], bool]:
    p = Path(path)
    if not p.exists():
        return {}, False
    payload = json.loads(p.read_text(encoding="utf-8"))
    return payload if isinstance(payload, dict) else {}, True


def _write_json(path: str | Path, payload: dict[str, Any]) -> None:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def _clean_status(payload: dict[str, Any], mode: str) -> bool:
    return bool(
        payload.get("ok", False)
        and payload.get("mode") == mode
        and payload.get("press_runtime_smoke", True)
        and payload.get("success_source") == "button_displacement"
        and payload.get("force_source") == "unavailable"
        and payload.get("contact_force_available") is False
        and payload.get("uses_fake_force") is False
        and payload.get("dataset_collection_allowed") is False
        and payload.get("benchmark_result") is False
        and payload.get("not_for_paper_claims") is True
    )


def build_report(args: argparse.Namespace) -> dict[str, Any]:
    partial_2mm, partial_2mm_exists = _read_json(args.partial_2mm_status)
    partial_10mm, partial_10mm_exists = _read_json(args.partial_10mm_status)
    full, full_exists = _read_json(args.full_press_status)
    retract, retract_exists = _read_json(args.press_and_retract_status)
    errors: list[str] = []
    for label, exists in (
        ("partial_2mm_status", partial_2mm_exists),
        ("partial_10mm_status", partial_10mm_exists),
        ("full_press_status", full_exists),
        ("press_and_retract_status", retract_exists),
    ):
        if not exists:
            errors.append(f"missing_{label}")

    partials_passed = _clean_status(partial_2mm, "partial_press_2mm") and _clean_status(
        partial_10mm, "partial_press_10mm"
    )
    full_press_passed = bool(
        _clean_status(full, "full_press")
        and full.get("press_target_executed") is True
        and full.get("button_displacement") is not None
    )
    press_and_retract_passed = bool(
        _clean_status(retract, "press_and_retract")
        and (retract.get("button_pressed_during_press_phase", False) or retract.get("button_pressed", False))
        and retract.get("retract_executed", False)
        and retract.get("final_ee_to_button_distance_increased_after_retract", False)
    )
    press_runtime_smoke_passed = bool(partials_passed and full_press_passed and press_and_retract_passed and not errors)
    ready_for_eval = bool(press_runtime_smoke_passed)

    return {
        "ok": ready_for_eval,
        "ready_for_single_episode_real_fr3_press_button_eval": ready_for_eval,
        "ready_for_dataset_collection": False,
        "press_runtime_smoke_passed": press_runtime_smoke_passed,
        "partial_press_2mm_passed": _clean_status(partial_2mm, "partial_press_2mm"),
        "partial_press_10mm_passed": _clean_status(partial_10mm, "partial_press_10mm"),
        "full_press_passed": full_press_passed,
        "press_and_retract_passed": press_and_retract_passed,
        "success_source": "button_displacement",
        "force_source": "unavailable",
        "contact_force_available": False,
        "uses_fake_force": False,
        "dataset_collection_allowed": False,
        "benchmark_result": False,
        "not_for_paper_claims": True,
        "recommended_next_stage": "single_episode_real_fr3_press_button_eval",
        "status_paths": {
            "partial_press_2mm": str(args.partial_2mm_status),
            "partial_press_10mm": str(args.partial_10mm_status),
            "full_press": str(args.full_press_status),
            "press_and_retract": str(args.press_and_retract_status),
        },
        "errors": errors,
        "warnings": ["dataset collection remains intentionally disabled for this gate"],
    }


def main() -> int:
    args = parse_args()
    report = build_report(args)
    _write_json(args.output, report)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

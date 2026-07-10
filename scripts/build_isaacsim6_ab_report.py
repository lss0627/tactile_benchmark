#!/usr/bin/env python
"""Build the fixed-trajectory Isaac Sim 5.1/6.0.1 migration A/B report."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any


def allowed_zero_drift(
    drift_5_1_m: float,
    *,
    relative_multiplier: float = 2.0,
    numerical_floor_m: float = 0.00005,
    absolute_limit_m: float = 0.001,
) -> float:
    return min(
        max(relative_multiplier * float(drift_5_1_m), float(numerical_floor_m)),
        float(absolute_limit_m),
    )


def allowed_penetration(
    penetration_5_1_m: float,
    *,
    delta_m: float = 0.001,
    absolute_limit_m: float = 0.005,
) -> float:
    return min(float(penetration_5_1_m) + float(delta_m), float(absolute_limit_m))


def _load(path: str | Path) -> dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _sha256(path: str | Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def build_report(
    *,
    five_one_controller: str | Path,
    six_g1a: str | Path,
    six_g1b: str | Path,
    six_penetration: str | Path | None = None,
) -> dict[str, Any]:
    old = _load(five_one_controller)
    g1a = _load(six_g1a)
    g1b = _load(six_g1b)
    penetration = _load(six_penetration) if six_penetration else g1b
    fr3 = g1a["checkpoints"]["fr3_asset_articulation"]
    old_drift = float(old["ee_displacement_norm"])
    new_drift = float(fr3["zero_action"]["tcp_drift_m"])
    drift_limit = allowed_zero_drift(old_drift)
    expected_dofs = list(old["joint_names"])
    actual_dofs = list(fr3["dof_names"])
    micro = fr3["micro_motion"]
    penetration_ok = bool(penetration.get("penetration_ok", False))
    checks = {
        "dof_names_and_order": expected_dofs == actual_dofs,
        "joint_limits": bool(fr3.get("dof_lower") and fr3.get("dof_upper")),
        "zero_action_drift": new_drift <= drift_limit,
        "micro_motion_direction": float(micro["target_delta_rad"]) * float(micro["observed_delta_rad"]) > 0,
        "contact": bool(g1a["checkpoints"]["contact_sensor"]["ok"]),
        "camera": bool(g1a["checkpoints"]["camera"]["ok"] and g1b["camera"]["ok"]),
        "lifecycle": bool(
            g1b["reset_cycles_completed"] == 100
            and g1b["invalid_after_reset"] == 0
            and g1b["stale_sensor_handles"] == 0
        ),
        "bounded_rollout": bool(g1b["rollout_steps_completed"] == 500),
        "penetration": penetration_ok,
    }
    return {
        "ok": all(checks.values()),
        "status": "PASS_SMOKE" if all(checks.values()) else "BLOCKED",
        "claim_class": "runtime_smoke",
        "benchmark_result": False,
        "checks": checks,
        "dof_names": {"isaac_sim_5_1": expected_dofs, "isaac_sim_6_0_1": actual_dofs},
        "joint_limits": {
            "comparison_basis": "same retained FR3 USD asset digest; 6.0.1 tensor limits recorded",
            "lower_6_0_1": fr3["dof_lower"],
            "upper_6_0_1": fr3["dof_upper"],
        },
        "zero_action_drift": {
            "drift_5_1_m": old_drift,
            "drift_6_0_1_m": new_drift,
            "relative_multiplier": 2.0,
            "numerical_floor_m": 0.00005,
            "absolute_safety_limit_m": 0.001,
            "allowed_m": drift_limit,
            "formula": "min(max(2 * drift_5_1, 0.00005), 0.001)",
        },
        "penetration": {
            "isaac_sim_5_1": "N/A_NOT_MEASURED",
            "isaac_sim_6_0_1_max_m": penetration.get("max_penetration_m"),
            "persistent_steps": penetration.get("max_persistent_penetration_steps"),
            "absolute_safety_limit_m": 0.005,
        },
        "contact_force_5_1": "N/A_UNAVAILABLE",
        "contact_6_0_1": g1a["checkpoints"]["contact_sensor"],
        "camera_6_0_1": g1b["camera"],
        "fixed_configuration": {
            "physics_dt": 1.0 / 60.0,
            "rendering_dt": 1.0 / 20.0,
            "physics_device": "cpu",
            "rendering_device": "cuda:0",
            "solver": "TGS",
            "stage_units_m": 1.0,
            "up_axis": "Z",
            "trajectory": "bounded 7D alternating +/-0.25mm x every 50 steps",
        },
        "input_sha256": {
            "five_one_controller": _sha256(five_one_controller),
            "six_g1a": _sha256(six_g1a),
            "six_g1b": _sha256(six_g1b),
            "six_penetration": _sha256(six_penetration) if six_penetration else _sha256(six_g1b),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--five-one-controller", required=True)
    parser.add_argument("--six-g1a", required=True)
    parser.add_argument("--six-g1b", required=True)
    parser.add_argument("--six-penetration")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    report = build_report(
        five_one_controller=args.five_one_controller,
        six_g1a=args.six_g1a,
        six_g1b=args.six_g1b,
        six_penetration=args.six_penetration,
    )
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

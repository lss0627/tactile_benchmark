#!/usr/bin/env python
"""FR3 articulation introspection smoke.

Dry-run emits the planned report schema. Runtime mode loads FR3 USD and
traverses USD prims to collect candidate articulation/joint/link/frame names.
It never sends joint commands or connects PressButton.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
import sys
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from isaac_tactile_libero.envs.isaacsim_backend_status import (  # noqa: E402
    load_isaacsim_visual_smoke_config,
    probe_isaacsim_visual_smoke,
)
from isaac_tactile_libero.robots.fr3_articulation_spec import load_fr3_articulation_config  # noqa: E402
from isaac_tactile_libero.robots.fr3_introspection import (  # noqa: E402
    build_planned_introspection_report,
    build_stage_introspection_report,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-config", default="configs/robots/fr3_real_articulation.yaml")
    parser.add_argument("--runtime-config", default="outputs/isaacsim_visual_smoke/isaac_conda_runtime.yaml")
    parser.add_argument("--output", default="outputs/fr3_articulation_introspection/report.json")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--webrtc", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def _write_json(path: str | Path, payload: dict[str, Any]) -> None:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def _runtime_import_available() -> bool:
    return importlib.util.find_spec("isaacsim") is not None or importlib.util.find_spec("omni") is not None


def _import_simulation_app():
    try:
        from isaacsim import SimulationApp  # type: ignore

        return SimulationApp
    except Exception as first_error:
        try:
            from isaacsim import SimulationApp  # type: ignore

            return SimulationApp
        except Exception as second_error:
            raise RuntimeError(
                "Could not import Isaac Sim SimulationApp. Run from Isaac Sim Python. "
                f"isaacsim error: {first_error}; isaacsim error: {second_error}"
            ) from second_error


def _load_fr3_and_collect(args: argparse.Namespace) -> dict[str, Any]:
    robot = load_fr3_articulation_config(args.robot_config)
    if not robot.assets.fr3_usd_path:
        raise RuntimeError("fr3_usd_path is not configured")
    SimulationApp = _import_simulation_app()
    app_config = {"headless": bool(args.headless)}
    if args.webrtc:
        app_config["enable_livestream"] = True
    simulation_app = SimulationApp(app_config)
    warnings: list[str] = []
    try:
        import omni.usd  # type: ignore
        from pxr import UsdGeom  # type: ignore

        context = omni.usd.get_context()
        context.new_stage()
        stage = context.get_stage()
        if stage is None:
            raise RuntimeError("Isaac Sim did not return a USD stage.")
        world = UsdGeom.Xform.Define(stage, "/World")
        stage.SetDefaultPrim(world.GetPrim())
        fr3 = stage.DefinePrim("/World/FR3", "Xform")
        fr3.GetReferences().AddReference(str(robot.assets.fr3_usd_path))
        for _ in range(10):
            simulation_app.update()

        prim_paths: list[str] = []
        joint_paths: list[str] = []
        visual_paths: list[str] = []
        collision_paths: list[str] = []
        articulation_root_path = None
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            prim_paths.append(path)
            type_name = prim.GetTypeName()
            applied = list(prim.GetAppliedSchemas())
            applied_text = " ".join(applied).lower()
            lower = path.lower()
            if "articulation" in applied_text and articulation_root_path is None:
                articulation_root_path = path
            if "joint" in type_name.lower() or "joint" in lower:
                joint_paths.append(path)
            if type_name in {"Mesh", "Capsule", "Cube", "Cylinder", "Sphere"}:
                visual_paths.append(path)
            if "collision" in lower or "collision" in applied_text:
                collision_paths.append(path)
        if articulation_root_path is None:
            warnings.append("No articulation root schema was identified by dry USD prim traversal")
        report = build_stage_introspection_report(
            robot_config_path=args.robot_config,
            runtime_config_path=args.runtime_config,
            output_path=args.output,
            prim_paths=prim_paths,
            joint_paths=joint_paths,
            articulation_root_path=articulation_root_path,
            visual_prim_paths=visual_paths,
            collision_prim_paths=collision_paths,
            warnings=warnings,
        )
        _write_json(args.output, report)
        return report
    finally:
        simulation_app.close()


def main() -> int:
    args = parse_args()
    if args.dry_run:
        report = build_planned_introspection_report(
            args.robot_config,
            runtime_config_path=args.runtime_config,
            output_path=args.output,
            dry_run=True,
            headless=args.headless,
            webrtc=args.webrtc,
        )
        _write_json(args.output, report)
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0

    runtime_cfg = load_isaacsim_visual_smoke_config(args.runtime_config)
    readiness = probe_isaacsim_visual_smoke(runtime_cfg).as_dict()
    if not readiness.get("ready_for_runtime", False):
        report = build_planned_introspection_report(
            args.robot_config,
            runtime_config_path=args.runtime_config,
            output_path=args.output,
            dry_run=False,
            headless=args.headless,
            webrtc=args.webrtc,
        )
        report["ok"] = False
        report["errors"] = list(readiness.get("blocking_conditions", []))
        _write_json(args.output, report)
        print(json.dumps(report, indent=2, sort_keys=True))
        return 1
    if not _runtime_import_available():
        report = build_planned_introspection_report(
            args.robot_config,
            runtime_config_path=args.runtime_config,
            output_path=args.output,
            dry_run=False,
            headless=args.headless,
            webrtc=args.webrtc,
        )
        report["ok"] = False
        report["errors"] = ["Isaac Sim Python modules are not importable from this Python process."]
        _write_json(args.output, report)
        print(json.dumps(report, indent=2, sort_keys=True))
        return 1
    try:
        report = _load_fr3_and_collect(args)
    except Exception as exc:
        report = build_planned_introspection_report(
            args.robot_config,
            runtime_config_path=args.runtime_config,
            output_path=args.output,
            dry_run=False,
            headless=args.headless,
            webrtc=args.webrtc,
        )
        report["ok"] = False
        report["errors"] = [str(exc)]

    _write_json(args.output, report)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

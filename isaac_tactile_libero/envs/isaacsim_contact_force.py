"""Import-safe Isaac Sim contact-force probe abstraction.

This module declares candidate contact-force methods for the PressButton second
probe without importing Isaac Sim, omni, carb, or pxr at module import time.
Runtime methods import Isaac/Omni modules only from inside method calls that are
executed after a SimulationApp exists.
"""

from __future__ import annotations

import importlib
from importlib.machinery import PathFinder
import math
from dataclasses import dataclass
from typing import Any

CONTACT_FORCE_METHODS = ("contact_sensor", "physx_contact_report", "rigid_contact_view")
CONTACT_FORCE_UNIT = "N"


def _vector_norm(value: Any) -> float | None:
    if value is None:
        return None
    try:
        return abs(float(value))
    except Exception:
        pass
    try:
        components = [float(component) for component in value]
    except Exception:
        return None
    if not components:
        return None
    return math.sqrt(sum(component * component for component in components))


def _vector3(value: Any) -> list[float] | None:
    if value is None:
        return None
    try:
        components = [float(component) for component in value]
    except Exception:
        norm = _vector_norm(value)
        if norm is None:
            return None
        return [float(norm), 0.0, 0.0]
    if len(components) != 3:
        return None
    return components


@dataclass(frozen=True)
class ContactForceReport:
    """JSON-safe contact-force method report."""

    method: str
    contact_probe_method: str
    contact_signal_seen: bool
    contact_force_available: bool
    contact_force_norm: float
    max_contact_force_norm: float
    mean_contact_force_norm: float
    contact_force_unit: str = CONTACT_FORCE_UNIT
    contact_force_source: str = "unavailable"
    contact_force_vector: tuple[float, float, float] | None = None
    contact_api_error: str = ""
    physics_contact_available: bool = False
    contact_force_confirmed: bool = False
    benchmark_result: bool = False
    not_for_paper_claims: bool = True
    warnings: tuple[str, ...] = ()

    @classmethod
    def unavailable(cls, *, method: str, error: str, warnings: tuple[str, ...] = ()) -> "ContactForceReport":
        return cls(
            method=str(method),
            contact_probe_method="unavailable",
            contact_signal_seen=False,
            contact_force_available=False,
            contact_force_norm=0.0,
            max_contact_force_norm=0.0,
            mean_contact_force_norm=0.0,
            contact_force_source="unavailable",
            contact_api_error=str(error),
            physics_contact_available=False,
            contact_force_confirmed=False,
            warnings=tuple(warnings),
        )

    @classmethod
    def available(
        cls,
        *,
        method: str,
        force_vector: Any,
        source: str,
        contact_signal_seen: bool = True,
    ) -> "ContactForceReport":
        vector = _vector3(force_vector)
        if vector is None:
            raise ValueError("Available contact-force reports require a 3D force vector or scalar force.")
        norm = float(_vector_norm(vector) or 0.0)
        return cls(
            method=str(method),
            contact_probe_method=str(method),
            contact_signal_seen=bool(contact_signal_seen),
            contact_force_available=True,
            contact_force_norm=norm,
            max_contact_force_norm=norm,
            mean_contact_force_norm=norm,
            contact_force_source=str(source),
            contact_force_vector=(float(vector[0]), float(vector[1]), float(vector[2])),
            contact_api_error="",
            physics_contact_available=True,
            contact_force_confirmed=norm > 0.0,
        )

    def as_dict(self) -> dict[str, Any]:
        return {
            "method": self.method,
            "contact_probe_method": self.contact_probe_method,
            "contact_signal_seen": self.contact_signal_seen,
            "contact_force_available": self.contact_force_available,
            "contact_force_norm": self.contact_force_norm,
            "max_contact_force_norm": self.max_contact_force_norm,
            "mean_contact_force_norm": self.mean_contact_force_norm,
            "contact_force_unit": self.contact_force_unit,
            "contact_force_source": self.contact_force_source,
            "contact_force_vector": list(self.contact_force_vector) if self.contact_force_vector is not None else None,
            "contact_api_error": self.contact_api_error,
            "physics_contact_available": self.physics_contact_available,
            "contact_force_confirmed": self.contact_force_confirmed,
            "benchmark_result": self.benchmark_result,
            "not_for_paper_claims": self.not_for_paper_claims,
            "warnings": list(self.warnings),
        }


def safe_find_spec(name: str) -> dict[str, Any]:
    """Discover a module without importing or executing any parent package."""

    try:
        search_path = None
        spec = None
        qualified_name = ""
        parts = name.split(".")
        for index, part in enumerate(parts):
            qualified_name = part if not qualified_name else f"{qualified_name}.{part}"
            spec = PathFinder.find_spec(qualified_name, search_path)
            if spec is None:
                break
            if index < len(parts) - 1:
                search_path = spec.submodule_search_locations
                if search_path is None:
                    spec = None
                    break
    except Exception as exc:
        return {"name": name, "available": False, "origin": None, "error": f"{type(exc).__name__}: {exc}"}
    return {
        "name": name,
        "available": spec is not None,
        "origin": getattr(spec, "origin", None) if spec else None,
        "error": None,
    }


def discover_contact_force_api_candidates() -> dict[str, Any]:
    """Return safe import-spec discovery for candidate Isaac contact APIs."""

    candidates = {
        "contact_sensor": [
            "isaacsim.sensors.experimental.physics",
        ],
        "physx_contact_report": [
            "omni.physx",
            "omni.physx.scripts",
            "omni.physx.bindings",
            "omni.physx.bindings._physx",
        ],
        "rigid_contact_view": [
            "isaacsim.core.experimental.prims",
        ],
    }
    return {
        "api_discovery_only": True,
        "simulation_app_created": False,
        "benchmark_result": False,
        "not_for_paper_claims": True,
        "candidates": {
            method: [safe_find_spec(name) for name in module_names]
            for method, module_names in candidates.items()
        },
        "note": "Import-spec discovery only; some Isaac/Omni APIs become importable only after SimulationApp starts.",
    }


class ContactForceBackend:
    """Best-effort runtime contact-force backend with multiple candidate methods."""

    def __init__(self, method: str = "auto"):
        if method != "auto" and method not in CONTACT_FORCE_METHODS:
            available = ", ".join(("auto", *CONTACT_FORCE_METHODS))
            raise ValueError(f"Unsupported contact-force method {method!r}. Available: {available}")
        self.method = method

    def read(
        self,
        *,
        stage: Any | None,
        pusher_prim_path: str,
        target_prim_path: str,
        contact_signal_hint: bool = False,
    ) -> ContactForceReport:
        methods = CONTACT_FORCE_METHODS if self.method == "auto" else (self.method,)
        errors: list[str] = []
        for method in methods:
            report = self._read_method(
                method,
                stage=stage,
                pusher_prim_path=pusher_prim_path,
                target_prim_path=target_prim_path,
                contact_signal_hint=contact_signal_hint,
            )
            if report.contact_force_available:
                return report
            errors.append(f"{method}: {report.contact_api_error}")
        return ContactForceReport.unavailable(
            method=self.method,
            error="; ".join(errors) if errors else "No contact-force method was attempted.",
        )

    def _read_method(
        self,
        method: str,
        *,
        stage: Any | None,
        pusher_prim_path: str,
        target_prim_path: str,
        contact_signal_hint: bool,
    ) -> ContactForceReport:
        if stage is None:
            return ContactForceReport.unavailable(method=method, error="USD stage unavailable.")
        prim_error = self._validate_prim_paths(stage, (pusher_prim_path, target_prim_path))
        if prim_error:
            return ContactForceReport.unavailable(method=method, error=prim_error)
        if method == "physx_contact_report":
            return self._read_physx_contact_report(
                stage=stage,
                pusher_prim_path=pusher_prim_path,
                target_prim_path=target_prim_path,
                contact_signal_hint=contact_signal_hint,
            )
        if method == "contact_sensor":
            return self._read_contact_sensor(contact_signal_hint=contact_signal_hint)
        if method == "rigid_contact_view":
            return self._read_rigid_contact_view(contact_signal_hint=contact_signal_hint)
        return ContactForceReport.unavailable(method=method, error=f"Unknown method: {method}")

    @staticmethod
    def _validate_prim_paths(stage: Any, paths: tuple[str, ...]) -> str:
        missing: list[str] = []
        for path in paths:
            try:
                prim = stage.GetPrimAtPath(path)
                valid = bool(prim and (not hasattr(prim, "IsValid") or prim.IsValid()))
            except Exception:
                valid = False
            if not valid:
                missing.append(path)
        return "USD prim path(s) missing for contact-force probe: " + ", ".join(missing) if missing else ""

    @staticmethod
    def _read_contact_sensor(*, contact_signal_hint: bool) -> ContactForceReport:
        del contact_signal_hint
        try:
            importlib.import_module("isaacsim.sensors.experimental.physics")
        except Exception as exc:
            return ContactForceReport.unavailable(method="contact_sensor", error=f"contact sensor module unavailable: {exc}")
        return ContactForceReport.unavailable(
            method="contact_sensor",
            error="experimental contact sensor module importable, but no configured ContactSensor is wired in this legacy probe.",
        )

    @staticmethod
    def _read_rigid_contact_view(*, contact_signal_hint: bool) -> ContactForceReport:
        del contact_signal_hint
        module_errors: list[str] = []
        for module_name in ("isaacsim.core.experimental.prims",):
            try:
                module = importlib.import_module(module_name)
            except Exception as exc:
                module_errors.append(f"{module_name}: {exc}")
                continue
            if hasattr(module, "RigidContactView"):
                return ContactForceReport.unavailable(
                    method="rigid_contact_view",
                    error="RigidContactView is importable, but this minimal probe has no initialized view binding.",
                )
        return ContactForceReport.unavailable(
            method="rigid_contact_view",
            error="rigid contact view module unavailable: " + "; ".join(module_errors),
        )

    @staticmethod
    def _read_physx_contact_report(
        *,
        stage: Any,
        pusher_prim_path: str,
        target_prim_path: str,
        contact_signal_hint: bool,
    ) -> ContactForceReport:
        del stage, contact_signal_hint
        try:
            physx = importlib.import_module("omni.physx")
        except Exception as exc:
            return ContactForceReport.unavailable(method="physx_contact_report", error=f"omni.physx unavailable: {exc}")
        interface_factory = getattr(physx, "get_physx_simulation_interface", None)
        if not callable(interface_factory):
            return ContactForceReport.unavailable(
                method="physx_contact_report",
                error="omni.physx has no get_physx_simulation_interface entry point.",
            )
        try:
            interface = interface_factory()
        except Exception as exc:
            return ContactForceReport.unavailable(method="physx_contact_report", error=f"PhysX interface failed: {exc}")
        for method_name in ("get_contact_report", "get_contact_reports"):
            method = getattr(interface, method_name, None)
            if not callable(method):
                continue
            try:
                report = method()
            except Exception as exc:
                return ContactForceReport.unavailable(
                    method="physx_contact_report",
                    error=f"PhysX {method_name} failed or has unsupported signature: {exc}",
                )
            vector = ContactForceBackend._extract_pair_force_vector(report, pusher_prim_path, target_prim_path)
            if vector is not None:
                return ContactForceReport.available(
                    method="physx_contact_report",
                    force_vector=vector,
                    source="physx_contact_report",
                    contact_signal_seen=True,
                )
        contact_names = sorted(name for name in dir(interface) if "contact" in name.lower())[:12]
        suffix = f" Contact-like methods: {', '.join(contact_names)}." if contact_names else ""
        return ContactForceReport.unavailable(
            method="physx_contact_report",
            error="No supported PhysX contact report method returned pusher-target force." + suffix,
        )

    @staticmethod
    def _extract_pair_force_vector(report: Any, pusher_prim_path: str, target_prim_path: str) -> list[float] | None:
        if report is None:
            return None
        if isinstance(report, dict):
            records = report.get("contacts") or report.get("contactReports") or report.values()
        else:
            records = report
        try:
            iterator = iter(records)
        except TypeError:
            iterator = iter((records,))
        for record in iterator:
            if not ContactForceBackend._record_matches_pair(record, pusher_prim_path, target_prim_path):
                continue
            vector = ContactForceBackend._record_force_vector(record)
            if vector is not None:
                return vector
        return None

    @staticmethod
    def _record_matches_pair(record: Any, pusher_prim_path: str, target_prim_path: str) -> bool:
        text = str(record)
        if isinstance(record, dict):
            values = record.values()
        else:
            values = [getattr(record, name, None) for name in ("body0", "body1", "actor0", "actor1", "prim0", "prim1")]
        combined = " ".join(str(value) for value in values if value is not None) or text
        return bool(pusher_prim_path in combined and target_prim_path in combined)

    @staticmethod
    def _record_force_vector(record: Any) -> list[float] | None:
        if isinstance(record, dict):
            candidates = [
                record.get(name)
                for name in ("force", "normal_force", "normalForce", "impulse", "normal_impulse", "normalImpulse")
            ]
        else:
            candidates = [
                getattr(record, name, None)
                for name in ("force", "normal_force", "normalForce", "impulse", "normal_impulse", "normalImpulse")
            ]
        for candidate in candidates:
            vector = _vector3(candidate)
            if vector is not None:
                return vector
        return None

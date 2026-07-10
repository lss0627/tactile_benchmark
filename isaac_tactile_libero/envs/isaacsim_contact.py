"""Import-safe PressButton contact/displacement hook helpers.

This module intentionally does not import Isaac Sim, omni, carb, or pxr at module
import time. Runtime objects are passed in by the Isaac Sim environment and read
best-effort. If runtime contact APIs are unavailable, callers receive explicit
fallback status instead of a fake physics-contact claim.
"""

from __future__ import annotations

import importlib
import math
from dataclasses import dataclass
from typing import Any


CONTACT_PROBE_METHOD = "physx_contact_report_probe"
CONTACT_FORCE_UNIT = "N"
DEFAULT_PUSHER_PRIM_PATH = "/World/KinematicPusher_Placeholder"
DEFAULT_BUTTON_PRIM_PATH = "/World/PressButton_RedPrimitive"
DEFAULT_BUTTON_TOP_PRIM_PATH = "/World/PressButton_RedPrimitive"


@dataclass(frozen=True)
class PressButtonContactReading:
    physics_contact_available: bool
    contact_signal_seen: bool
    contact_force_available: bool
    button_displacement_available: bool
    button_displacement: float
    button_press_depth: float
    max_button_press_depth: float
    using_geometric_fallback: bool
    contact_force_norm: float = 0.0
    max_contact_force_norm: float = 0.0
    mean_contact_force_norm: float = 0.0
    contact_force_unit: str = CONTACT_FORCE_UNIT
    contact_force_source: str = "unavailable"
    contact_force_confirmed: bool = False
    contact_probe_method: str = CONTACT_PROBE_METHOD
    contact_api_error: str = ""
    pusher_prim_path: str = DEFAULT_PUSHER_PRIM_PATH
    button_prim_path: str = DEFAULT_BUTTON_PRIM_PATH
    button_top_prim_path: str = DEFAULT_BUTTON_TOP_PRIM_PATH
    warnings: tuple[str, ...] = ()

    def as_dict(self) -> dict[str, Any]:
        return {
            "physics_contact_available": self.physics_contact_available,
            "contact_signal_seen": self.contact_signal_seen,
            "contact_force_available": self.contact_force_available,
            "button_displacement_available": self.button_displacement_available,
            "button_displacement": self.button_displacement,
            "button_press_depth": self.button_press_depth,
            "max_button_press_depth": self.max_button_press_depth,
            "using_geometric_fallback": self.using_geometric_fallback,
            "contact_force_norm": self.contact_force_norm,
            "max_contact_force_norm": self.max_contact_force_norm,
            "mean_contact_force_norm": self.mean_contact_force_norm,
            "contact_force_unit": self.contact_force_unit,
            "contact_force_source": self.contact_force_source,
            "contact_force_confirmed": self.contact_force_confirmed,
            "contact_probe_method": self.contact_probe_method,
            "contact_api_error": self.contact_api_error,
            "pusher_prim_path": self.pusher_prim_path,
            "button_prim_path": self.button_prim_path,
            "button_top_prim_path": self.button_top_prim_path,
            "warnings": list(self.warnings),
        }


@dataclass(frozen=True)
class PressButtonContactForceReading:
    physics_contact_available: bool
    contact_signal_seen: bool
    contact_force_available: bool
    contact_force_norm: float
    max_contact_force_norm: float
    mean_contact_force_norm: float
    contact_force_unit: str
    contact_force_source: str
    contact_force_confirmed: bool
    contact_probe_method: str
    contact_api_error: str
    pusher_prim_path: str
    button_prim_path: str
    button_top_prim_path: str
    warnings: tuple[str, ...] = ()

    def as_dict(self) -> dict[str, Any]:
        return {
            "physics_contact_available": self.physics_contact_available,
            "contact_signal_seen": self.contact_signal_seen,
            "contact_force_available": self.contact_force_available,
            "contact_force_norm": self.contact_force_norm,
            "max_contact_force_norm": self.max_contact_force_norm,
            "mean_contact_force_norm": self.mean_contact_force_norm,
            "contact_force_unit": self.contact_force_unit,
            "contact_force_source": self.contact_force_source,
            "contact_force_confirmed": self.contact_force_confirmed,
            "contact_probe_method": self.contact_probe_method,
            "contact_api_error": self.contact_api_error,
            "pusher_prim_path": self.pusher_prim_path,
            "button_prim_path": self.button_prim_path,
            "button_top_prim_path": self.button_top_prim_path,
            "warnings": list(self.warnings),
        }


class IsaacSimPressButtonContactForceProbe:
    """Best-effort PhysX contact-force probe for the single PressButton smoke.

    The class is deliberately conservative: it only reports force/contact as
    available when a runtime API returns a parseable contact record for the
    configured prim paths. Geometry and button displacement never synthesize
    contact force.
    """

    def __init__(
        self,
        *,
        pusher_prim_path: str = DEFAULT_PUSHER_PRIM_PATH,
        button_prim_path: str = DEFAULT_BUTTON_PRIM_PATH,
        button_top_prim_path: str = DEFAULT_BUTTON_TOP_PRIM_PATH,
    ):
        self.pusher_prim_path = str(pusher_prim_path)
        self.button_prim_path = str(button_prim_path)
        self.button_top_prim_path = str(button_top_prim_path)

    def read(
        self,
        *,
        runtime_enabled: bool,
        stage: Any,
        geometric_contact: bool,
        downward_motion: bool,
        step: int,
    ) -> PressButtonContactForceReading:
        del step
        if not runtime_enabled:
            return self._unavailable("runtime disabled; PhysX contact force probe was not attempted.")
        if stage is None:
            return self._unavailable("USD stage unavailable; cannot query pusher-button contact.")

        prim_error = self._validate_prim_paths(stage)
        if prim_error:
            return self._unavailable(prim_error)

        try:
            physx = importlib.import_module("omni.physx")
        except Exception as exc:
            return self._unavailable(f"omni.physx import unavailable at runtime: {exc}")

        interface_factory = getattr(physx, "get_physx_simulation_interface", None)
        if not callable(interface_factory):
            return self._unavailable("omni.physx has no get_physx_simulation_interface contact query entry point.")

        try:
            interface = interface_factory()
        except Exception as exc:
            return self._unavailable(f"PhysX simulation interface unavailable: {exc}")

        force_norm, error = self._read_contact_force_from_interface(interface)
        if force_norm is None:
            method_names = self._contact_method_names(interface)
            detail = f" Contact-like methods present: {', '.join(method_names[:8])}." if method_names else ""
            return self._unavailable((error or "No parseable pusher-button contact report was returned.") + detail)

        contact_seen = bool(force_norm > 0.0)
        return PressButtonContactForceReading(
            physics_contact_available=True,
            contact_signal_seen=bool(contact_seen or (geometric_contact and downward_motion)),
            contact_force_available=True,
            contact_force_norm=float(force_norm),
            max_contact_force_norm=float(force_norm),
            mean_contact_force_norm=float(force_norm),
            contact_force_unit=CONTACT_FORCE_UNIT,
            contact_force_source="physx_contact_report",
            contact_force_confirmed=contact_seen,
            contact_probe_method=CONTACT_PROBE_METHOD,
            contact_api_error="",
            pusher_prim_path=self.pusher_prim_path,
            button_prim_path=self.button_prim_path,
            button_top_prim_path=self.button_top_prim_path,
        )

    def _unavailable(self, message: str) -> PressButtonContactForceReading:
        return PressButtonContactForceReading(
            physics_contact_available=False,
            contact_signal_seen=False,
            contact_force_available=False,
            contact_force_norm=0.0,
            max_contact_force_norm=0.0,
            mean_contact_force_norm=0.0,
            contact_force_unit=CONTACT_FORCE_UNIT,
            contact_force_source="unavailable",
            contact_force_confirmed=False,
            contact_probe_method=CONTACT_PROBE_METHOD,
            contact_api_error=message,
            pusher_prim_path=self.pusher_prim_path,
            button_prim_path=self.button_prim_path,
            button_top_prim_path=self.button_top_prim_path,
        )

    def _validate_prim_paths(self, stage: Any) -> str:
        missing: list[str] = []
        for path in (self.pusher_prim_path, self.button_prim_path, self.button_top_prim_path):
            try:
                prim = stage.GetPrimAtPath(path)
                is_valid = bool(prim and (not hasattr(prim, "IsValid") or prim.IsValid()))
            except Exception:
                is_valid = False
            if not is_valid:
                missing.append(path)
        if missing:
            return "USD prim path(s) missing for contact probe: " + ", ".join(missing)
        return ""

    def _read_contact_force_from_interface(self, interface: Any) -> tuple[float | None, str]:
        for method_name in ("get_contact_report", "get_contact_reports"):
            method = getattr(interface, method_name, None)
            if not callable(method):
                continue
            try:
                report = method()
            except TypeError as exc:
                return None, f"PhysX {method_name} signature is unsupported by this adapter: {exc}"
            except Exception as exc:
                return None, f"PhysX {method_name} failed: {exc}"
            force_norm = self._extract_pair_force_norm(report)
            if force_norm is not None:
                return force_norm, ""
        return None, "No supported PhysX contact report method was available for pusher-button force."

    def _extract_pair_force_norm(self, report: Any) -> float | None:
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
            if not self._record_matches_pair(record):
                continue
            force = self._record_force(record)
            if force is not None:
                return float(force)
        return None

    def _record_matches_pair(self, record: Any) -> bool:
        text = str(record)
        path_values: list[str] = []
        if isinstance(record, dict):
            values = record.values()
        else:
            values = [getattr(record, name, None) for name in ("body0", "body1", "actor0", "actor1", "prim0", "prim1")]
        for value in values:
            if value is not None:
                path_values.append(str(value))
        combined = " ".join(path_values) or text
        pusher_seen = self.pusher_prim_path in combined
        button_seen = self.button_prim_path in combined or self.button_top_prim_path in combined
        return bool(pusher_seen and button_seen)

    @staticmethod
    def _record_force(record: Any) -> float | None:
        candidates: list[Any] = []
        if isinstance(record, dict):
            candidates.extend(
                record.get(name)
                for name in ("force", "normal_force", "normalForce", "impulse", "normal_impulse", "normalImpulse")
            )
        else:
            candidates.extend(
                getattr(record, name, None)
                for name in ("force", "normal_force", "normalForce", "impulse", "normal_impulse", "normalImpulse")
            )
        for candidate in candidates:
            norm = IsaacSimPressButtonContactForceProbe._vector_norm(candidate)
            if norm is not None:
                return norm
        return None

    @staticmethod
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

    @staticmethod
    def _contact_method_names(interface: Any) -> list[str]:
        try:
            names = dir(interface)
        except Exception:
            return []
        return sorted(name for name in names if "contact" in name.lower())


class IsaacSimPressButtonContactHook:
    """Best-effort runtime contact/displacement reader for the PressButton smoke."""

    def __init__(self, *, button_initial_z: float, press_threshold: float = 0.03):
        self.button_initial_z = float(button_initial_z)
        self.press_threshold = float(press_threshold)

    def read(
        self,
        *,
        runtime_enabled: bool,
        button_translate_op: Any,
        button_pose: Any,
        geometric_contact: bool,
        downward_motion: bool,
        step: int,
        previous_max_depth: float,
    ) -> PressButtonContactReading:
        del step
        warnings: list[str] = []
        contact_signal_seen = False
        button_displacement_available = False
        button_displacement = 0.0

        if runtime_enabled:
            z_value = self._read_translate_z(button_translate_op)
            if z_value is None:
                z_value = self._read_pose_z(button_pose)
                if z_value is not None:
                    warnings.append(
                        "Button USD translate op unavailable; using runtime state pose for displacement hook."
                    )
            if z_value is not None:
                button_displacement_available = True
                button_displacement = max(0.0, self.button_initial_z - float(z_value))

        button_press_depth = button_displacement if button_displacement_available else 0.0
        max_button_press_depth = max(float(previous_max_depth), float(button_press_depth))
        if button_displacement_available and button_press_depth > 1e-6:
            contact_signal_seen = True

        # A real force/contact query can be plugged in here later. Until then,
        # never claim physics-contact availability just because the geometric
        # proxy or displacement signal fired.
        physics_contact_available = False
        contact_force_available = False
        using_geometric_fallback = not button_displacement_available and not physics_contact_available
        if using_geometric_fallback and geometric_contact and downward_motion:
            contact_signal_seen = False

        return PressButtonContactReading(
            physics_contact_available=physics_contact_available,
            contact_signal_seen=contact_signal_seen,
            contact_force_available=contact_force_available,
            button_displacement_available=button_displacement_available,
            button_displacement=float(button_displacement),
            button_press_depth=float(button_press_depth),
            max_button_press_depth=float(max_button_press_depth),
            using_geometric_fallback=bool(using_geometric_fallback),
            contact_api_error="",
            warnings=tuple(warnings),
        )

    @staticmethod
    def _read_translate_z(button_translate_op: Any) -> float | None:
        if button_translate_op is None:
            return None
        try:
            value = button_translate_op.Get()
        except Exception:
            return None
        return IsaacSimPressButtonContactHook._read_pose_z(value)

    @staticmethod
    def _read_pose_z(value: Any) -> float | None:
        if value is None:
            return None
        try:
            return float(value[2])
        except Exception:
            pass
        try:
            return float(value.z)
        except Exception:
            return None

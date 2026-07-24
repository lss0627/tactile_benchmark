"""Isaac Sim 6 contact acceptance and runtime adapter contracts."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from typing import Any, Callable, Iterable, Mapping, Sequence


G1_CONTACT_PROVENANCE_SCHEMA_VERSION = "g1.contact.provenance.v1"
G1_CONTACT_RAW_SOURCE_SCHEMA = (
    "isaacsim.sensors.experimental.physics.get_raw_data.v1"
)
G1_CONTACT_RAW_SOURCE_KEYS = frozenset(
    {"body0", "body1", "position", "normal", "impulse", "time", "dt"}
)
G1_CONTACT_NORMALIZED_RAW_KEYS = frozenset(
    {
        "raw_index",
        "source_schema",
        "body0_id",
        "body1_id",
        "body0_prim_path",
        "body1_prim_path",
        "body0_rigid_body_prim_path",
        "body1_rigid_body_prim_path",
        "body0_contact_report_api",
        "body1_contact_report_api",
        "position_m",
        "normal",
        "impulse_n_s",
        "time_s",
        "dt_s",
    }
)
G1_CONTACT_BLOCKER_CODES = frozenset(
    {
        "CONTACT_RECORD_STRUCTURE_INVALID",
        "CONTACT_READING_INVALID",
        "CONTACT_SENSOR_PRIM_INVALID",
        "CONTACT_SENSOR_RIGID_BODY_INVALID",
        "CONTACT_REPORT_API_INVALID",
        "CONTACT_READ_SEQUENCE_INVALID",
        "CONTACT_SENSOR_TIME_INVALID",
        "CONTACT_PHYSICS_STEP_INVALID",
        "CONTACT_RAW_RECORD_INVALID",
        "CONTACT_RAW_BODY_PATH_INVALID",
        "CONTACT_RAW_BODY_AUTHORITY_INVALID",
        "CONTACT_RAW_ATTRIBUTION_UNAVAILABLE",
    }
)


class ContactProvenanceError(ValueError):
    """A structured failure from the import-safe Contact provenance boundary."""

    def __init__(self, code: str, message: str) -> None:
        if code not in G1_CONTACT_BLOCKER_CODES:
            raise ValueError(f"unsupported Contact provenance code: {code}")
        if not isinstance(message, str) or not message.strip():
            raise ValueError("Contact provenance message must be non-empty")
        self.code = code
        self.message = message
        super().__init__(message)


@dataclass(frozen=True)
class ContactAcceptanceConfig:
    sensor_ready_timeout_steps: int = 5
    contact_onset_tolerance_steps: int = 2
    contact_release_timeout_steps: int = 5
    contact_stable_steps: int = 3
    force_zero_epsilon: float = 1.0e-4


@dataclass(frozen=True, init=False)
class ContactSample:
    is_valid: bool
    in_contact: bool
    force_magnitude: float
    time: float
    read_sequence_index: int
    raw_contacts: tuple[dict[str, Any], ...] = ()

    def __init__(
        self,
        is_valid: bool,
        in_contact: bool,
        force_magnitude: float,
        time: float,
        read_sequence_index: int | None = None,
        raw_contacts: Sequence[Mapping[str, Any]] = (),
        *,
        physics_step: int | None = None,
    ) -> None:
        """Retain the old keyword only as a constructor alias for read order."""

        if read_sequence_index is None:
            if physics_step is None:
                raise TypeError("read_sequence_index is required")
            read_sequence_index = physics_step
        elif physics_step is not None:
            raise TypeError(
                "supply read_sequence_index or the legacy physics_step alias, not both"
            )
        object.__setattr__(self, "is_valid", bool(is_valid))
        object.__setattr__(self, "in_contact", bool(in_contact))
        object.__setattr__(self, "force_magnitude", float(force_magnitude))
        object.__setattr__(self, "time", float(time))
        object.__setattr__(self, "read_sequence_index", int(read_sequence_index))
        object.__setattr__(
            self,
            "raw_contacts",
            tuple(dict(item) for item in raw_contacts),
        )

    @property
    def physics_step(self) -> int:
        """Compatibility view; this value is a caller-owned read index, not physics."""

        return self.read_sequence_index


def _blocker(code: str, message: str) -> dict[str, str]:
    if code not in G1_CONTACT_BLOCKER_CODES:
        raise ValueError(f"unsupported Contact provenance code: {code}")
    if not isinstance(message, str) or not message.strip():
        raise ValueError("Contact provenance blocker message must be non-empty")
    return {"code": code, "message": message}


def _append_blocker(
    blockers: list[dict[str, str]],
    code: str,
    message: str,
) -> None:
    item = _blocker(code, message)
    if item not in blockers:
        blockers.append(item)


def _finite_float(value: Any) -> float:
    if isinstance(value, bool):
        raise ValueError("boolean is not a numeric Contact value")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("Contact value must be finite")
    return result


def _xyz(value: Any) -> list[float]:
    if not isinstance(value, Mapping) or set(value) != {"x", "y", "z"}:
        raise ValueError("Contact vector must contain exact x/y/z keys")
    return [_finite_float(value[axis]) for axis in ("x", "y", "z")]


def _absolute_prim_path(value: Any) -> str:
    if not isinstance(value, str) or not value.startswith("/") or value == "/":
        raise ValueError("Contact prim path must be an absolute non-root USD path")
    return value


def _normalize_raw_contact(
    raw: Any,
    *,
    raw_index: int,
    body_path_resolver: Callable[[int], str] | None,
    rigid_body_path_resolver: Callable[[str], str] | None,
    contact_report_api_resolver: Callable[[str], bool] | None,
    blockers: list[dict[str, str]],
) -> dict[str, Any]:
    safe: dict[str, Any] = {
        "raw_index": int(raw_index),
        "source_schema": G1_CONTACT_RAW_SOURCE_SCHEMA,
        "body0_id": None,
        "body1_id": None,
        "body0_prim_path": None,
        "body1_prim_path": None,
        "body0_rigid_body_prim_path": None,
        "body1_rigid_body_prim_path": None,
        "body0_contact_report_api": False,
        "body1_contact_report_api": False,
        "position_m": None,
        "normal": None,
        "impulse_n_s": None,
        "time_s": None,
        "dt_s": None,
    }
    if not isinstance(raw, Mapping) or set(raw) != G1_CONTACT_RAW_SOURCE_KEYS:
        _append_blocker(
            blockers,
            "CONTACT_RAW_RECORD_INVALID",
            f"raw Contact record {raw_index} does not match the exact source allowlist",
        )
        return safe
    try:
        body0_id = raw["body0"]
        body1_id = raw["body1"]
        if type(body0_id) is not int or type(body1_id) is not int:
            raise ValueError("raw body identifiers must be exact integers")
        position = _xyz(raw["position"])
        normal = _xyz(raw["normal"])
        impulse = _xyz(raw["impulse"])
        time_s = _finite_float(raw["time"])
        dt_s = _finite_float(raw["dt"])
        if dt_s <= 0.0:
            raise ValueError("raw Contact dt must be positive")
        safe.update(
            {
                "body0_id": body0_id,
                "body1_id": body1_id,
                "position_m": position,
                "normal": normal,
                "impulse_n_s": impulse,
                "time_s": time_s,
                "dt_s": dt_s,
            }
        )
    except (TypeError, ValueError, OverflowError) as error:
        _append_blocker(
            blockers,
            "CONTACT_RAW_RECORD_INVALID",
            f"raw Contact record {raw_index} is invalid: {error}",
        )
        return safe
    if body_path_resolver is None:
        _append_blocker(
            blockers,
            "CONTACT_RAW_BODY_PATH_INVALID",
            f"raw Contact record {raw_index} has no body-path resolver",
        )
        return safe
    try:
        body0_path = _absolute_prim_path(body_path_resolver(body0_id))
        body1_path = _absolute_prim_path(body_path_resolver(body1_id))
        safe["body0_prim_path"] = body0_path
        safe["body1_prim_path"] = body1_path
    except Exception as error:
        _append_blocker(
            blockers,
            "CONTACT_RAW_BODY_PATH_INVALID",
            f"raw Contact record {raw_index} body path is invalid: {error}",
        )
        return safe
    if rigid_body_path_resolver is None or contact_report_api_resolver is None:
        _append_blocker(
            blockers,
            "CONTACT_RAW_BODY_AUTHORITY_INVALID",
            f"raw Contact record {raw_index} lacks stage authority resolvers",
        )
        return safe
    try:
        body0_rigid = _absolute_prim_path(rigid_body_path_resolver(body0_path))
        body1_rigid = _absolute_prim_path(rigid_body_path_resolver(body1_path))
        safe.update(
            {
                "body0_rigid_body_prim_path": body0_rigid,
                "body1_rigid_body_prim_path": body1_rigid,
                "body0_contact_report_api": bool(
                    contact_report_api_resolver(body0_rigid)
                ),
                "body1_contact_report_api": bool(
                    contact_report_api_resolver(body1_rigid)
                ),
            }
        )
    except Exception as error:
        _append_blocker(
            blockers,
            "CONTACT_RAW_BODY_AUTHORITY_INVALID",
            f"raw Contact record {raw_index} body authority is invalid: {error}",
        )
    return safe


def normalize_g1_contact_provenance(
    *,
    sample: ContactSample | Any,
    execution: Mapping[str, Any],
    sensor_authority: Mapping[str, Any],
    expected_read_sequence_index: int,
    previous_sensor_time_s: float | None,
    previous_observed_physics_step: int,
    observed_physics_step: int | None,
    body_path_resolver: Callable[[int], str] | None = None,
    rigid_body_path_resolver: Callable[[str], str] | None = None,
    contact_report_api_resolver: Callable[[str], bool] | None = None,
) -> dict[str, Any]:
    """Build one exact JSON-safe Contact record without optimistic defaults."""

    blockers: list[dict[str, str]] = []
    freshness_blockers: list[dict[str, str]] = []
    execution_record = json.loads(
        json.dumps(dict(execution), allow_nan=False)
    )
    required_execution = {
        "consumer",
        "trial_id",
        "candidate_id",
        "class_id",
        "scene_id",
        "scene_index",
        "phase",
        "action_index",
        "window_index",
        "requested_vector_m",
    }
    if set(execution_record) != required_execution:
        _append_blocker(
            blockers,
            "CONTACT_RECORD_STRUCTURE_INVALID",
            "Contact execution record does not contain the exact required keys",
        )
    authority = dict(sensor_authority)
    required_sensor = {
        "sensor_prim_path",
        "sensor_prim_type",
        "sensor_rigid_body_prim_path",
        "sensor_rigid_body_source",
        "sensor_prim_authority_source",
        "rigid_body_authority_source",
        "contact_report_api_prim_paths",
        "contact_report_api_verified",
        "contact_report_api_authority_source",
    }
    if set(authority) != required_sensor:
        _append_blocker(
            blockers,
            "CONTACT_SENSOR_PRIM_INVALID",
            "Contact sensor authority does not contain the exact required keys",
        )
    try:
        _absolute_prim_path(authority.get("sensor_prim_path"))
        if authority.get("sensor_prim_type") != "IsaacContactSensor":
            raise ValueError("sensor prim type is not IsaacContactSensor")
    except ValueError as error:
        _append_blocker(
            blockers,
            "CONTACT_SENSOR_PRIM_INVALID",
            f"Contact sensor prim authority is invalid: {error}",
        )
    try:
        _absolute_prim_path(authority.get("sensor_rigid_body_prim_path"))
        if authority.get("sensor_rigid_body_source") != (
            "nearest_ancestor_with_usdphysics_rigid_body_api"
        ):
            raise ValueError("sensor rigid-body source is invalid")
    except ValueError as error:
        _append_blocker(
            blockers,
            "CONTACT_SENSOR_RIGID_BODY_INVALID",
            f"Contact sensor rigid-body authority is invalid: {error}",
        )
    if authority.get("contact_report_api_verified") is not True:
        _append_blocker(
            blockers,
            "CONTACT_REPORT_API_INVALID",
            "sensor rigid body lacks verified Contact Report API",
        )

    contact_valid = getattr(sample, "is_valid", None)
    in_contact = getattr(sample, "in_contact", None)
    try:
        force_magnitude_n = _finite_float(
            getattr(sample, "force_magnitude", None)
        )
        sensor_time_s = _finite_float(getattr(sample, "time", None))
        if sensor_time_s < 0.0:
            raise ValueError("sensor time is negative")
    except (TypeError, ValueError, OverflowError) as error:
        force_magnitude_n = None
        sensor_time_s = None
        _append_blocker(
            blockers,
            "CONTACT_READING_INVALID",
            f"Contact reading is invalid: {error}",
        )
    read_sequence = getattr(sample, "read_sequence_index", None)
    if type(read_sequence) is not int:
        read_sequence = getattr(sample, "physics_step", None)
    if type(read_sequence) is not int or read_sequence != expected_read_sequence_index:
        item = _blocker(
            "CONTACT_READ_SEQUENCE_INVALID",
            "Contact read sequence does not match the expected evidence index",
        )
        freshness_blockers.append(item)
        _append_blocker(blockers, item["code"], item["message"])
    monotonic = (
        sensor_time_s is not None
        and (
            previous_sensor_time_s is None
            or sensor_time_s > previous_sensor_time_s
        )
    )
    if not monotonic:
        item = _blocker(
            "CONTACT_SENSOR_TIME_INVALID",
            "Contact sensor time did not advance strictly",
        )
        freshness_blockers.append(item)
        _append_blocker(blockers, item["code"], item["message"])
    if (
        type(previous_observed_physics_step) is not int
        or previous_observed_physics_step < 0
        or type(observed_physics_step) is not int
        or observed_physics_step < 0
    ):
        observed_delta = None
        physics_valid = False
        observed_source = "unavailable"
    else:
        observed_delta = observed_physics_step - previous_observed_physics_step
        physics_valid = observed_delta == 3
        observed_source = (
            "isaacsim.core.simulation_manager.get_num_physics_steps"
        )
    if not physics_valid:
        item = _blocker(
            "CONTACT_PHYSICS_STEP_INVALID",
            "observed physics-step delta is not exactly 3",
        )
        freshness_blockers.append(item)
        _append_blocker(blockers, item["code"], item["message"])

    raw_source = getattr(sample, "raw_contacts", ())
    if (
        not isinstance(raw_source, Sequence)
        or isinstance(raw_source, (str, bytes, Mapping))
    ):
        raw_source = ()
        _append_blocker(
            blockers,
            "CONTACT_RAW_RECORD_INVALID",
            "raw Contact payload is not an ordered sequence",
        )
    raw_records = [
        _normalize_raw_contact(
            raw,
            raw_index=index,
            body_path_resolver=body_path_resolver,
            rigid_body_path_resolver=rigid_body_path_resolver,
            contact_report_api_resolver=contact_report_api_resolver,
            blockers=blockers,
        )
        for index, raw in enumerate(raw_source)
    ]
    if contact_valid is not True:
        _append_blocker(
            blockers,
            "CONTACT_READING_INVALID",
            "Contact reading validity is false",
        )
    if in_contact is not True and in_contact is not False:
        _append_blocker(
            blockers,
            "CONTACT_READING_INVALID",
            "Contact in_contact value is not an exact boolean",
        )
    if contact_valid is True and in_contact is True and not raw_records:
        _append_blocker(
            blockers,
            "CONTACT_RAW_ATTRIBUTION_UNAVAILABLE",
            "Contact-positive reading has no auditable raw attribution",
        )
    report_paths = authority.get("contact_report_api_prim_paths", [])
    if not isinstance(report_paths, list):
        report_paths = []
    report_paths = sorted(
        {
            value
            for value in report_paths
            if isinstance(value, str) and value.startswith("/")
        }
        | {
            value
            for raw in raw_records
            for value, verified in (
                (
                    raw.get("body0_rigid_body_prim_path"),
                    raw.get("body0_contact_report_api"),
                ),
                (
                    raw.get("body1_rigid_body_prim_path"),
                    raw.get("body1_contact_report_api"),
                ),
            )
            if verified is True and isinstance(value, str)
        }
    )
    authority["contact_report_api_prim_paths"] = report_paths
    freshness_valid = not freshness_blockers
    provenance_valid = not blockers
    record = {
        "schema_version": G1_CONTACT_PROVENANCE_SCHEMA_VERSION,
        "execution": execution_record,
        "sensor": authority,
        "reading": {
            "contact_valid": contact_valid is True,
            "in_contact": in_contact is True,
            "force_magnitude_n": force_magnitude_n,
            "sensor_time_s": sensor_time_s,
            "read_sequence_index": read_sequence,
            "observed_physics_step": (
                observed_physics_step
                if type(observed_physics_step) is int
                and observed_physics_step >= 0
                else None
            ),
            "observed_physics_step_source": observed_source,
        },
        "freshness": {
            "valid": freshness_valid,
            "expected_read_sequence_index": expected_read_sequence_index,
            "previous_sensor_time_s": previous_sensor_time_s,
            "sensor_time_monotonic": monotonic,
            "previous_observed_physics_step": previous_observed_physics_step,
            "expected_physics_step_delta": 3,
            "observed_physics_step_delta": observed_delta,
            "physics_step_relation_valid": physics_valid,
            "blockers": freshness_blockers,
        },
        "raw_contact_count": len(raw_records),
        "raw_contacts": raw_records,
        "provenance": {"valid": provenance_valid, "blockers": blockers},
        "force_vector_valid": False,
        "wrench_valid": False,
        "raw_impulse_used_as_force": False,
    }
    return json.loads(json.dumps(record, allow_nan=False))


def inspect_g1_contact_stage_authority(
    *,
    stage: Any,
    sensor_prim_path: str,
) -> tuple[
    dict[str, Any],
    Callable[[int], str],
    Callable[[str], str],
    Callable[[str], bool],
]:
    """Capture sensor/rigid-body/API truth from the live USD stage."""

    from pxr import PhysxSchema, PhysicsSchemaTools, Sdf, UsdPhysics  # type: ignore

    sensor_path = _absolute_prim_path(sensor_prim_path)
    sensor_prim = stage.GetPrimAtPath(sensor_path)
    if sensor_prim is None or not sensor_prim.IsValid():
        raise ContactProvenanceError(
            "CONTACT_SENSOR_PRIM_INVALID",
            f"Contact sensor prim is unavailable: {sensor_path}",
        )
    sensor_type = str(sensor_prim.GetTypeName())
    if sensor_type != "IsaacContactSensor":
        raise ContactProvenanceError(
            "CONTACT_SENSOR_PRIM_INVALID",
            f"Contact sensor prim type is invalid: {sensor_type}",
        )

    def rigid_body_path(path: str) -> str:
        current = Sdf.Path(_absolute_prim_path(path))
        while str(current) not in {"", "/"}:
            prim = stage.GetPrimAtPath(current)
            if prim is not None and prim.IsValid() and prim.HasAPI(
                UsdPhysics.RigidBodyAPI
            ):
                return str(current)
            current = current.GetParentPath()
        raise ContactProvenanceError(
            "CONTACT_SENSOR_RIGID_BODY_INVALID",
            f"no rigid-body ancestor exists for Contact path: {path}",
        )

    def has_contact_report_api(path: str) -> bool:
        prim = stage.GetPrimAtPath(_absolute_prim_path(path))
        return bool(
            prim is not None
            and prim.IsValid()
            and prim.HasAPI(PhysxSchema.PhysxContactReportAPI)
        )

    sensor_rigid_body = rigid_body_path(sensor_path)
    report_verified = has_contact_report_api(sensor_rigid_body)
    authority = {
        "sensor_prim_path": sensor_path,
        "sensor_prim_type": sensor_type,
        "sensor_rigid_body_prim_path": sensor_rigid_body,
        "sensor_rigid_body_source": (
            "nearest_ancestor_with_usdphysics_rigid_body_api"
        ),
        "sensor_prim_authority_source": (
            "usd_stage_after_contact_sensor_authoring_before_evidence_read"
        ),
        "rigid_body_authority_source": "usd_stage_before_evidence_read",
        "contact_report_api_prim_paths": (
            [sensor_rigid_body] if report_verified else []
        ),
        "contact_report_api_verified": report_verified,
        "contact_report_api_authority_source": (
            "usd_stage_before_evidence_read"
        ),
    }
    return (
        authority,
        lambda body_id: str(PhysicsSchemaTools.intToSdfPath(int(body_id))),
        rigid_body_path,
        has_contact_report_api,
    )


def _require_keys(value: Any, required: set[str], label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping) or set(value) != required:
        raise ContactProvenanceError(
            "CONTACT_RECORD_STRUCTURE_INVALID",
            f"{label} does not contain the exact required keys",
        )
    return value


def _finite_vector(value: Any, *, size: int) -> bool:
    return (
        isinstance(value, Sequence)
        and not isinstance(value, (str, bytes, Mapping))
        and len(value) == size
        and all(
            isinstance(item, (int, float))
            and not isinstance(item, bool)
            and math.isfinite(float(item))
            for item in value
        )
    )


def _validate_classified_raw_contacts(
    raw_contacts: list[Any],
    *,
    sensor_time_s: float,
    report_paths: set[str],
) -> None:
    for index, raw_value in enumerate(raw_contacts):
        raw = _require_keys(
            raw_value,
            set(G1_CONTACT_NORMALIZED_RAW_KEYS),
            f"normalized raw Contact record {index}",
        )
        try:
            paths = (
                _absolute_prim_path(raw["body0_prim_path"]),
                _absolute_prim_path(raw["body1_prim_path"]),
                _absolute_prim_path(raw["body0_rigid_body_prim_path"]),
                _absolute_prim_path(raw["body1_rigid_body_prim_path"]),
            )
        except ValueError as exc:
            raise ContactProvenanceError(
                "CONTACT_RAW_BODY_PATH_INVALID",
                f"normalized raw Contact record {index} has invalid paths",
            ) from exc
        api_flags = (
            raw["body0_contact_report_api"],
            raw["body1_contact_report_api"],
        )
        valid = (
            raw["raw_index"] == index
            and raw["source_schema"] == G1_CONTACT_RAW_SOURCE_SCHEMA
            and type(raw["body0_id"]) is int
            and type(raw["body1_id"]) is int
            and raw["body0_id"] >= 0
            and raw["body1_id"] >= 0
            and all(
                _finite_vector(raw[field], size=3)
                for field in ("position_m", "normal", "impulse_n_s")
            )
            and isinstance(raw["time_s"], (int, float))
            and not isinstance(raw["time_s"], bool)
            and math.isfinite(float(raw["time_s"]))
            and math.isclose(
                float(raw["time_s"]),
                sensor_time_s,
                rel_tol=0.0,
                abs_tol=1.0e-9,
            )
            and isinstance(raw["dt_s"], (int, float))
            and not isinstance(raw["dt_s"], bool)
            and math.isfinite(float(raw["dt_s"]))
            and float(raw["dt_s"]) > 0.0
            and all(type(flag) is bool for flag in api_flags)
            and any(api_flags)
            and all(
                not verified or path in report_paths
                for path, verified in zip(paths[2:], api_flags)
            )
        )
        if not valid:
            raise ContactProvenanceError(
                "CONTACT_RAW_RECORD_INVALID",
                f"normalized raw Contact record {index} is inconsistent",
            )


def classify_g1_contact_provenance(
    record: Any,
    *,
    mirrors: Mapping[str, Any],
    consumer: str,
    phase: str,
    expected_execution: Mapping[str, Any] | None = None,
) -> str:
    """Return ``contact`` or ``no_contact``; invalid records raise fail-closed."""

    root = _require_keys(
        record,
        {
            "schema_version",
            "execution",
            "sensor",
            "reading",
            "freshness",
            "raw_contact_count",
            "raw_contacts",
            "provenance",
            "force_vector_valid",
            "wrench_valid",
            "raw_impulse_used_as_force",
        },
        "Contact provenance",
    )
    if root["schema_version"] != G1_CONTACT_PROVENANCE_SCHEMA_VERSION:
        raise ContactProvenanceError(
            "CONTACT_RECORD_STRUCTURE_INVALID",
            "Contact provenance schema version is invalid",
        )
    execution = _require_keys(
        root["execution"],
        {
            "consumer",
            "trial_id",
            "candidate_id",
            "class_id",
            "scene_id",
            "scene_index",
            "phase",
            "action_index",
            "window_index",
            "requested_vector_m",
        },
        "Contact execution",
    )
    if execution["consumer"] != consumer or execution["phase"] != phase:
        raise ContactProvenanceError(
            "CONTACT_RECORD_STRUCTURE_INVALID",
            "Contact execution consumer/phase is invalid",
        )
    optional_identifiers = (
        execution["trial_id"],
        execution["candidate_id"],
        execution["class_id"],
    )
    if (
        any(
            not isinstance(execution[field], str)
            or not execution[field]
            for field in ("consumer", "scene_id", "phase")
        )
        or any(
            value is not None
            and (not isinstance(value, str) or not value)
            for value in optional_identifiers
        )
        or any(
            type(execution[field]) is not int
            or execution[field] < 0
            for field in ("scene_index", "action_index")
        )
        or (
            execution["window_index"] is not None
            and (
                type(execution["window_index"]) is not int
                or execution["window_index"] < 0
            )
        )
        or not _finite_vector(
            execution["requested_vector_m"],
            size=3,
        )
    ):
        raise ContactProvenanceError(
            "CONTACT_RECORD_STRUCTURE_INVALID",
            "Contact execution fields are invalid",
        )
    if expected_execution is not None:
        for key, value in expected_execution.items():
            if execution.get(key) != value:
                raise ContactProvenanceError(
                    "CONTACT_RECORD_STRUCTURE_INVALID",
                    f"Contact execution field does not match authority: {key}",
                )
    reading = _require_keys(
        root["reading"],
        {
            "contact_valid",
            "in_contact",
            "force_magnitude_n",
            "sensor_time_s",
            "read_sequence_index",
            "observed_physics_step",
            "observed_physics_step_source",
        },
        "Contact reading",
    )
    reading_numeric = (
        reading["force_magnitude_n"],
        reading["sensor_time_s"],
    )
    if (
        type(reading["contact_valid"]) is not bool
        or reading["contact_valid"] is not True
        or type(reading["in_contact"]) is not bool
        or any(
            not isinstance(value, (int, float))
            or isinstance(value, bool)
            or not math.isfinite(float(value))
            or float(value) < 0.0
            for value in reading_numeric
        )
        or type(reading["read_sequence_index"]) is not int
        or reading["read_sequence_index"] < 0
        or type(reading["observed_physics_step"]) is not int
        or reading["observed_physics_step"] < 0
        or reading["observed_physics_step_source"]
        != "isaacsim.core.simulation_manager.get_num_physics_steps"
    ):
        raise ContactProvenanceError(
            "CONTACT_READING_INVALID",
            "Contact reading fields are invalid",
        )
    raw_contacts = root["raw_contacts"]
    if (
        type(root["raw_contact_count"]) is not int
        or not isinstance(raw_contacts, list)
        or root["raw_contact_count"] != len(raw_contacts)
    ):
        raise ContactProvenanceError(
            "CONTACT_RAW_RECORD_INVALID",
            "Contact raw count does not match retained raw records",
        )
    mirror_pairs = {
        "contact_valid": reading["contact_valid"],
        "contact": reading["in_contact"],
        "raw_contact_count": root["raw_contact_count"],
        "force_vector_valid": root["force_vector_valid"],
        "wrench_valid": root["wrench_valid"],
        "raw_impulse_used_as_force": root["raw_impulse_used_as_force"],
    }
    if any(mirrors.get(key) != value for key, value in mirror_pairs.items()):
        raise ContactProvenanceError(
            "CONTACT_RECORD_STRUCTURE_INVALID",
            "Contact convenience mirrors disagree with nested authority",
        )
    if reading["contact_valid"] is not True:
        raise ContactProvenanceError(
            "CONTACT_READING_INVALID",
            "Contact reading validity is false",
        )
    sensor = _require_keys(
        root["sensor"],
        {
            "sensor_prim_path",
            "sensor_prim_type",
            "sensor_rigid_body_prim_path",
            "sensor_rigid_body_source",
            "sensor_prim_authority_source",
            "rigid_body_authority_source",
            "contact_report_api_prim_paths",
            "contact_report_api_verified",
            "contact_report_api_authority_source",
        },
        "Contact sensor authority",
    )
    try:
        sensor_prim_path = _absolute_prim_path(
            sensor["sensor_prim_path"]
        )
        sensor_rigid_body_path = _absolute_prim_path(
            sensor["sensor_rigid_body_prim_path"]
        )
        report_path_values = sensor["contact_report_api_prim_paths"]
        if (
            not isinstance(report_path_values, list)
            or len(report_path_values) != len(set(report_path_values))
        ):
            raise ValueError("Contact Report API path inventory is invalid")
        report_paths = {
            _absolute_prim_path(value)
            for value in report_path_values
        }
    except (TypeError, ValueError) as exc:
        raise ContactProvenanceError(
            "CONTACT_SENSOR_PRIM_INVALID",
            "Contact sensor authority paths are invalid",
        ) from exc
    if (
        sensor_prim_path == sensor_rigid_body_path
        or sensor["sensor_prim_type"] != "IsaacContactSensor"
        or sensor["sensor_rigid_body_source"]
        != "nearest_ancestor_with_usdphysics_rigid_body_api"
        or sensor["sensor_prim_authority_source"]
        != (
            "usd_stage_after_contact_sensor_authoring_before_evidence_read"
        )
        or sensor["rigid_body_authority_source"]
        != "usd_stage_before_evidence_read"
        or sensor["contact_report_api_verified"] is not True
        or sensor["contact_report_api_authority_source"]
        != "usd_stage_before_evidence_read"
        or sensor_rigid_body_path not in report_paths
    ):
        raise ContactProvenanceError(
            "CONTACT_SENSOR_PRIM_INVALID",
            "Contact sensor authority fields are invalid",
        )
    freshness = _require_keys(
        root["freshness"],
        {
            "valid",
            "expected_read_sequence_index",
            "previous_sensor_time_s",
            "sensor_time_monotonic",
            "previous_observed_physics_step",
            "expected_physics_step_delta",
            "observed_physics_step_delta",
            "physics_step_relation_valid",
            "blockers",
        },
        "Contact freshness",
    )
    provenance = _require_keys(
        root["provenance"],
        {"valid", "blockers"},
        "Contact provenance validity",
    )
    previous_sensor_time = freshness["previous_sensor_time_s"]
    sensor_time = float(reading["sensor_time_s"])
    previous_sensor_time_valid = (
        previous_sensor_time is None
        or (
            isinstance(previous_sensor_time, (int, float))
            and not isinstance(previous_sensor_time, bool)
            and math.isfinite(float(previous_sensor_time))
            and float(previous_sensor_time) >= 0.0
        )
    )
    expected_monotonic = (
        previous_sensor_time_valid
        and (
            previous_sensor_time is None
            or sensor_time > float(previous_sensor_time)
        )
    )
    previous_physics_step = freshness[
        "previous_observed_physics_step"
    ]
    observed_delta = (
        reading["observed_physics_step"] - previous_physics_step
        if type(previous_physics_step) is int
        and previous_physics_step >= 0
        else None
    )
    if (
        type(freshness["valid"]) is not bool
        or type(freshness["sensor_time_monotonic"]) is not bool
        or type(freshness["physics_step_relation_valid"]) is not bool
        or freshness["expected_read_sequence_index"]
        != reading["read_sequence_index"]
        or freshness["sensor_time_monotonic"] is not expected_monotonic
        or freshness["expected_physics_step_delta"] != 3
        or freshness["observed_physics_step_delta"] != observed_delta
        or freshness["physics_step_relation_valid"]
        is not (observed_delta == 3)
        or not isinstance(freshness["blockers"], list)
        or type(provenance["valid"]) is not bool
        or not isinstance(provenance["blockers"], list)
    ):
        raise ContactProvenanceError(
            "CONTACT_RECORD_STRUCTURE_INVALID",
            "Contact freshness/provenance fields are inconsistent",
        )
    if (
        sensor["sensor_prim_type"] != "IsaacContactSensor"
        or sensor["contact_report_api_verified"] is not True
        or freshness["valid"] is not True
        or freshness["sensor_time_monotonic"] is not True
        or freshness["expected_physics_step_delta"] != 3
        or freshness["observed_physics_step_delta"] != 3
        or freshness["physics_step_relation_valid"] is not True
        or freshness["blockers"] != []
        or provenance["valid"] is not True
        or provenance["blockers"] != []
        or root["force_vector_valid"] is not False
        or root["wrench_valid"] is not False
        or root["raw_impulse_used_as_force"] is not False
    ):
        raise ContactProvenanceError(
            "CONTACT_RECORD_STRUCTURE_INVALID",
            "Contact provenance is invalid",
        )
    _validate_classified_raw_contacts(
        raw_contacts,
        sensor_time_s=sensor_time,
        report_paths=report_paths,
    )
    if (
        reading["in_contact"] is not True
        and reading["in_contact"] is not False
    ):
        raise ContactProvenanceError(
            "CONTACT_READING_INVALID",
            "Contact in_contact value is not an exact boolean",
        )
    if reading["in_contact"] is True and root["raw_contact_count"] <= 0:
        raise ContactProvenanceError(
            "CONTACT_RAW_ATTRIBUTION_UNAVAILABLE",
            "Contact-positive reading has no raw attribution",
        )
    if reading["in_contact"] is True or root["raw_contact_count"] > 0:
        return "contact"
    return "no_contact"


def validate_contact_physics_policy(physics_device: str) -> list[str]:
    if str(physics_device).lower() != "cpu":
        return ["GPU_CONTACT_NATIVE_INSTABILITY"]
    return []


def _first_index(
    samples: list[ContactSample],
    start: int,
    stop: int,
    predicate: Callable[[ContactSample], bool],
) -> int | None:
    for index in range(max(0, start), min(len(samples), stop + 1)):
        if predicate(samples[index]):
            return index
    return None


def evaluate_contact_lifecycle(
    samples: Iterable[ContactSample],
    *,
    press_step: int,
    release_step: int,
    config: ContactAcceptanceConfig | None = None,
) -> dict[str, Any]:
    cfg = config or ContactAcceptanceConfig()
    trace = list(samples)
    errors: list[str] = []
    ready_step = _first_index(
        trace,
        0,
        cfg.sensor_ready_timeout_steps,
        lambda sample: sample.is_valid,
    )
    if ready_step is None:
        errors.append("SENSOR_READY_TIMEOUT")
    onset_step = _first_index(
        trace,
        press_step,
        press_step + cfg.contact_onset_tolerance_steps,
        lambda sample: sample.is_valid and sample.in_contact,
    )
    if onset_step is None:
        errors.append("CONTACT_ONSET_TIMEOUT")

    stable_release_step = None
    release_deadline = min(
        len(trace) - 1,
        release_step + cfg.contact_release_timeout_steps,
    )
    for index in range(max(0, release_step), release_deadline + 1):
        window = trace[index : index + cfg.contact_stable_steps]
        if len(window) != cfg.contact_stable_steps:
            continue
        if all(sample.is_valid and not sample.in_contact for sample in window):
            stable_release_step = index
            break
    if stable_release_step is None:
        errors.append("CONTACT_RELEASE_TIMEOUT")

    if any(
        sample.is_valid
        and not sample.in_contact
        and abs(float(sample.force_magnitude)) > cfg.force_zero_epsilon
        for sample in trace
    ):
        errors.append("NO_CONTACT_FORCE_NONZERO")
    if any(
        sample.is_valid and not math.isfinite(float(sample.force_magnitude))
        for sample in trace
    ):
        errors.append("NONFINITE_FORCE")

    return {
        "ok": not errors,
        "errors": errors,
        "ready_step": ready_step,
        "onset_step": onset_step,
        "release_step": stable_release_step,
        "sensor_ready_timeout_steps": cfg.sensor_ready_timeout_steps,
        "contact_onset_tolerance_steps": cfg.contact_onset_tolerance_steps,
        "contact_release_timeout_steps": cfg.contact_release_timeout_steps,
        "contact_stable_steps": cfg.contact_stable_steps,
        "force_zero_epsilon": cfg.force_zero_epsilon,
        "contact_valid": ready_step is not None,
        "force_magnitude_valid": onset_step is not None,
        "force_vector_valid": False,
        "wrench_valid": False,
        "raw_contact_valid": any(sample.raw_contacts for sample in trace),
        "public_force_vector_mask": False,
        "public_wrench_mask": False,
    }


PRESS_BUTTON_CONTACT_SCHEMA_VERSION = "g1.press_button.contact.v1"


def _contact_json_value(value: Any) -> Any:
    """Convert raw adapter values without assigning them a new physical meaning."""

    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("raw Contact contains NaN/Inf")
        return value
    if isinstance(value, Mapping):
        return {
            str(key): _contact_json_value(item)
            for key, item in value.items()
        }
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        return [_contact_json_value(item) for item in value]
    tolist = getattr(value, "tolist", None)
    if callable(tolist):
        return _contact_json_value(tolist())
    item = getattr(value, "item", None)
    if callable(item):
        return _contact_json_value(item())
    xyz = [getattr(value, axis, None) for axis in ("x", "y", "z")]
    if all(component is not None for component in xyz):
        return [_finite_float(component) for component in xyz]
    raise ValueError(f"unsupported raw Contact value: {type(value).__name__}")


def normalize_press_button_contact_record(
    sample: ContactSample,
    *,
    sample_index: int,
    observed_physics_step: int,
) -> dict[str, Any]:
    """Normalize one retained PressButton sample with conservative validity masks."""

    if not isinstance(sample, ContactSample):
        raise TypeError("sample must be a ContactSample")
    if (
        type(sample_index) is not int
        or sample_index < 0
        or type(observed_physics_step) is not int
        or observed_physics_step < 0
        or type(sample.read_sequence_index) is not int
        or sample.read_sequence_index < 0
    ):
        raise ValueError("Contact sample/physics indices must be non-negative integers")

    errors: list[str] = []
    raw_contacts: list[Any] = []
    for raw in sample.raw_contacts:
        try:
            raw_contacts.append(_contact_json_value(raw))
        except (TypeError, ValueError):
            errors.append("CONTACT_RAW_RECORD_INVALID")
            raw_contacts.append({"retained_unparsed_type": type(raw).__name__})

    sensor_time_s: float | None = None
    if math.isfinite(float(sample.time)) and float(sample.time) >= 0.0:
        sensor_time_s = float(sample.time)
    else:
        errors.append("CONTACT_SENSOR_TIME_INVALID")
    if sensor_time_s is not None and any(
        not isinstance(raw, Mapping)
        or not isinstance(raw.get("time"), (int, float))
        or isinstance(raw.get("time"), bool)
        or not math.isfinite(float(raw["time"]))
        or not math.isclose(
            float(raw["time"]),
            sensor_time_s,
            rel_tol=0.0,
            abs_tol=1.0e-9,
        )
        for raw in raw_contacts
    ):
        errors.append("CONTACT_RAW_SENSOR_TIME_INVALID")

    force_magnitude_n: float | None = None
    if (
        sample.is_valid
        and math.isfinite(float(sample.force_magnitude))
        and float(sample.force_magnitude) >= 0.0
    ):
        force_magnitude_n = float(sample.force_magnitude)
    elif sample.is_valid:
        errors.append("CONTACT_READING_INVALID")
    if not sample.is_valid:
        errors.append("CONTACT_READING_INVALID")
    if sample.in_contact and not raw_contacts:
        errors.append("CONTACT_RAW_ATTRIBUTION_UNAVAILABLE")
    if (
        sample.is_valid
        and not sample.in_contact
        and abs(float(sample.force_magnitude)) > 1.0e-4
    ):
        errors.append("NO_CONTACT_FORCE_NONZERO")

    record = {
        "schema_version": PRESS_BUTTON_CONTACT_SCHEMA_VERSION,
        "sample_index": sample_index,
        "read_sequence_index": int(sample.read_sequence_index),
        "observed_physics_step": observed_physics_step,
        "sensor_time_s": sensor_time_s,
        "contact_valid": bool(sample.is_valid),
        "contact": bool(sample.in_contact),
        "force_magnitude_n": force_magnitude_n,
        "raw_contact_count": len(raw_contacts),
        "raw_contacts": raw_contacts,
        "force_vector_valid": False,
        "wrench_valid": False,
        "raw_impulse_used_as_force": False,
        "sample_retained": True,
        "usable": not errors,
        "errors": list(dict.fromkeys(errors)),
    }
    # This also rejects values that the evidence writer could not serialize.
    return json.loads(json.dumps(record, allow_nan=False, sort_keys=True))


def validate_press_button_contact_record(record: Any) -> dict[str, Any]:
    """Fail closed on mask promotion, loss of raw samples, or malformed indices."""

    required = {
        "schema_version",
        "sample_index",
        "read_sequence_index",
        "observed_physics_step",
        "sensor_time_s",
        "contact_valid",
        "contact",
        "force_magnitude_n",
        "raw_contact_count",
        "raw_contacts",
        "force_vector_valid",
        "wrench_valid",
        "raw_impulse_used_as_force",
        "sample_retained",
        "usable",
        "errors",
    }
    if not isinstance(record, Mapping) or set(record) != required:
        raise ContactProvenanceError(
            "CONTACT_RECORD_STRUCTURE_INVALID",
            "PressButton Contact record does not contain the exact required keys",
        )
    raw_contacts = record["raw_contacts"]
    sensor_time_s = record["sensor_time_s"]
    force_magnitude_n = record["force_magnitude_n"]
    errors = record["errors"]
    sensor_time_valid = (
        isinstance(sensor_time_s, (int, float))
        and not isinstance(sensor_time_s, bool)
        and math.isfinite(float(sensor_time_s))
        and float(sensor_time_s) >= 0.0
    )
    force_magnitude_valid = (
        isinstance(force_magnitude_n, (int, float))
        and not isinstance(force_magnitude_n, bool)
        and math.isfinite(float(force_magnitude_n))
        and float(force_magnitude_n) >= 0.0
    )
    raw_records_valid = all(
        isinstance(raw, Mapping)
        and set(raw) == G1_CONTACT_RAW_SOURCE_KEYS
        and (
            (
                type(raw["body0"]) is int
                and raw["body0"] >= 0
            )
            or (
                isinstance(raw["body0"], str)
                and raw["body0"].startswith("/")
            )
        )
        and (
            (
                type(raw["body1"]) is int
                and raw["body1"] >= 0
            )
            or (
                isinstance(raw["body1"], str)
                and raw["body1"].startswith("/")
            )
        )
        and all(
            isinstance(raw[field], (list, tuple))
            and len(raw[field]) == 3
            and all(
                isinstance(value, (int, float))
                and not isinstance(value, bool)
                and math.isfinite(float(value))
                for value in raw[field]
            )
            for field in ("position", "normal", "impulse")
        )
        and isinstance(raw["time"], (int, float))
        and not isinstance(raw["time"], bool)
        and math.isfinite(float(raw["time"]))
        and float(raw["time"]) >= 0.0
        and isinstance(raw["dt"], (int, float))
        and not isinstance(raw["dt"], bool)
        and math.isfinite(float(raw["dt"]))
        and float(raw["dt"]) > 0.0
        for raw in raw_contacts
    )
    raw_times_match_sensor = (
        not raw_contacts
        or (
            sensor_time_valid
            and raw_records_valid
            and all(
                math.isclose(
                    float(raw["time"]),
                    float(sensor_time_s),
                    rel_tol=0.0,
                    abs_tol=1.0e-9,
                )
                for raw in raw_contacts
            )
        )
    )
    valid = (
        record["schema_version"] == PRESS_BUTTON_CONTACT_SCHEMA_VERSION
        and all(
            type(record[field]) is int and record[field] >= 0
            for field in (
                "sample_index",
                "read_sequence_index",
                "observed_physics_step",
            )
        )
        and type(record["contact_valid"]) is bool
        and type(record["contact"]) is bool
        and isinstance(raw_contacts, list)
        and type(record["raw_contact_count"]) is int
        and record["raw_contact_count"] == len(raw_contacts)
        and (
            raw_records_valid
            or (
                record["usable"] is False
                and "CONTACT_RAW_RECORD_INVALID" in errors
            )
        )
        and (
            raw_times_match_sensor
            or (
                record["usable"] is False
                and "CONTACT_RAW_SENSOR_TIME_INVALID" in errors
            )
        )
        and record["force_vector_valid"] is False
        and record["wrench_valid"] is False
        and record["raw_impulse_used_as_force"] is False
        and record["sample_retained"] is True
        and type(record["usable"]) is bool
        and isinstance(errors, list)
        and all(isinstance(item, str) and item for item in errors)
        and record["usable"] is (not errors)
        and (
            sensor_time_valid
            or (
                sensor_time_s is None
                and record["usable"] is False
                and "CONTACT_SENSOR_TIME_INVALID" in errors
            )
        )
        and (
            force_magnitude_valid
            or (
                force_magnitude_n is None
                and record["usable"] is False
                and "CONTACT_READING_INVALID" in errors
            )
        )
        and (
            record["contact_valid"] is True
            or (
                record["usable"] is False
                and "CONTACT_READING_INVALID" in errors
            )
        )
        and (
            record["contact"] is False
            or record["raw_contact_count"] > 0
            or (
                record["usable"] is False
                and "CONTACT_RAW_ATTRIBUTION_UNAVAILABLE" in errors
            )
        )
        and (
            record["contact"] is True
            or not force_magnitude_valid
            or float(force_magnitude_n) <= 1.0e-4
            or (
                record["usable"] is False
                and "NO_CONTACT_FORCE_NONZERO" in errors
            )
        )
        and (
            record["usable"] is False
            or (
                sensor_time_valid
                and force_magnitude_valid
                and record["contact_valid"] is True
                and (
                    record["contact"] is False
                    or record["raw_contact_count"] > 0
                )
                and raw_records_valid
                and raw_times_match_sensor
                and (
                    record["contact"] is True
                    or float(force_magnitude_n) <= 1.0e-4
                )
            )
        )
    )
    if not valid:
        raise ContactProvenanceError(
            "CONTACT_RECORD_STRUCTURE_INVALID",
            "PressButton Contact record is invalid",
        )
    return json.loads(json.dumps(record, allow_nan=False, sort_keys=True))


def validate_press_button_contact_trace(
    records: Iterable[Mapping[str, Any]],
    *,
    required_samples: int,
    expected_physics_stride: int = 3,
    expected_sensor_period_s: float = 1.0 / 20.0,
) -> dict[str, Any]:
    """Validate complete Contact cardinality and cross-sample freshness."""

    if type(required_samples) is not int or required_samples <= 0:
        raise ValueError("required_samples must be a positive integer")
    if (
        type(expected_physics_stride) is not int
        or expected_physics_stride <= 0
    ):
        raise ValueError(
            "expected_physics_stride must be a positive integer"
        )
    if (
        not isinstance(expected_sensor_period_s, (int, float))
        or isinstance(expected_sensor_period_s, bool)
        or not math.isfinite(float(expected_sensor_period_s))
        or float(expected_sensor_period_s) <= 0.0
    ):
        raise ValueError(
            "expected_sensor_period_s must be finite and positive"
        )
    trace = list(records)
    errors: list[str] = []
    normalized: list[dict[str, Any]] = []
    for record in trace:
        try:
            normalized.append(
                validate_press_button_contact_record(record)
            )
        except ContactProvenanceError:
            if "CONTACT_RECORD_INVALID" not in errors:
                errors.append("CONTACT_RECORD_INVALID")
    if len(trace) != required_samples:
        errors.append("CONTACT_SAMPLE_COUNT")
    if len(normalized) == len(trace):
        if any(record["usable"] is not True for record in normalized):
            errors.append("CONTACT_SAMPLE_UNUSABLE")
        sample_indices = [record["sample_index"] for record in normalized]
        read_indices = [
            record["read_sequence_index"] for record in normalized
        ]
        physics_steps = [
            record["observed_physics_step"] for record in normalized
        ]
        sensor_times = [
            (
                float(record["sensor_time_s"])
                if isinstance(
                    record["sensor_time_s"],
                    (int, float),
                )
                and not isinstance(record["sensor_time_s"], bool)
                else math.nan
            )
            for record in normalized
        ]
        if any(
            current != previous + 1
            for previous, current in zip(
                sample_indices,
                sample_indices[1:],
            )
        ) or sample_indices != read_indices:
            errors.append("CONTACT_READ_SEQUENCE_INVALID")
        if any(
            current != previous + expected_physics_stride
            for previous, current in zip(
                physics_steps,
                physics_steps[1:],
            )
        ):
            errors.append("CONTACT_PHYSICS_STEP_INVALID")
        tolerance = max(
            1.0e-9,
            float(expected_sensor_period_s) * 1.0e-6,
        )
        if any(
            not math.isclose(
                current - previous,
                float(expected_sensor_period_s),
                rel_tol=0.0,
                abs_tol=tolerance,
            )
            for previous, current in zip(
                sensor_times,
                sensor_times[1:],
            )
        ):
            errors.append("CONTACT_SENSOR_TIME_INVALID")
    return {
        "ok": not errors,
        "errors": list(dict.fromkeys(errors)),
        "sample_count": len(trace),
        "required_samples": required_samples,
        "expected_physics_stride": expected_physics_stride,
        "expected_sensor_period_s": float(expected_sensor_period_s),
    }


class IsaacSim6ContactSensor:
    """Thin runtime wrapper that exposes scalar force and raw contacts only."""

    def __init__(self, prim_path: str, *, sensor_factory: Callable[[str], Any] | None = None) -> None:
        self.prim_path = str(prim_path)
        self._sensor_factory = sensor_factory
        self._sensor = None

    def initialize(self) -> None:
        if self._sensor_factory is None:
            from isaacsim.sensors.experimental.physics import ContactSensor  # type: ignore

            self._sensor_factory = ContactSensor
        self._sensor = self._sensor_factory(self.prim_path)

    def reset(self) -> None:
        if self._sensor is not None and hasattr(self._sensor, "reset"):
            self._sensor.reset()
        self._sensor = None

    def read(self, read_sequence_index: int) -> ContactSample:
        if self._sensor is None:
            raise RuntimeError("Contact sensor is not initialized")
        reading = self._sensor.get_sensor_reading()
        raw = self._sensor.get_raw_data() if hasattr(self._sensor, "get_raw_data") else ()
        contacts = tuple(dict(item) for item in raw)
        return ContactSample(
            is_valid=bool(reading.is_valid),
            in_contact=bool(getattr(reading, "in_contact", False)),
            force_magnitude=float(getattr(reading, "value", 0.0)),
            time=float(getattr(reading, "time", 0.0)),
            read_sequence_index=int(read_sequence_index),
            raw_contacts=contacts,
        )


__all__ = [
    "ContactAcceptanceConfig",
    "ContactProvenanceError",
    "ContactSample",
    "G1_CONTACT_BLOCKER_CODES",
    "G1_CONTACT_PROVENANCE_SCHEMA_VERSION",
    "G1_CONTACT_RAW_SOURCE_KEYS",
    "G1_CONTACT_RAW_SOURCE_SCHEMA",
    "IsaacSim6ContactSensor",
    "PRESS_BUTTON_CONTACT_SCHEMA_VERSION",
    "classify_g1_contact_provenance",
    "evaluate_contact_lifecycle",
    "inspect_g1_contact_stage_authority",
    "normalize_press_button_contact_record",
    "normalize_g1_contact_provenance",
    "validate_press_button_contact_record",
    "validate_press_button_contact_trace",
    "validate_contact_physics_policy",
]

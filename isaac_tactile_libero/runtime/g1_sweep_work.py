"""Import-safe bounded-work and progress authority for G1 sweep proofs."""

from __future__ import annotations

from collections import OrderedDict
from copy import deepcopy
from dataclasses import asdict, dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import time
from typing import Any, Callable, Hashable, Mapping


SWEEP_WORK_SCHEMA_VERSION = "g1.full_robot.sweep_work.v1"
SWEEP_PROGRESS_SCHEMA_VERSION = "g1.full_robot.sweep_progress.v1"


def _json_safe(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, bool) or value is None or isinstance(value, str):
        return value
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("sweep work JSON contains a non-finite number")
        return value
    item = getattr(value, "item", None)
    if callable(item):
        return _json_safe(item())
    tolist = getattr(value, "tolist", None)
    if callable(tolist):
        return _json_safe(tolist())
    raise TypeError(
        f"sweep work JSON contains unsupported type: {type(value).__name__}"
    )


def canonical_sha256(
    value: Mapping[str, Any], *, exclude_fields: tuple[str, ...] = ()
) -> str:
    payload = {
        str(key): item
        for key, item in value.items()
        if str(key) not in set(exclude_fields)
    }
    encoded = json.dumps(
        _json_safe(payload),
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


class G1SweepWorkError(RuntimeError):
    """Structured failure for work exhaustion or cache inconsistency."""

    def __init__(
        self,
        code: str,
        message: str,
        *,
        receipt: Mapping[str, Any] | None = None,
    ) -> None:
        self.code = str(code)
        self.message = str(message)
        self.receipt = None if receipt is None else _json_safe(receipt)
        super().__init__(self.message)


@dataclass(frozen=True)
class SweepWorkLimits:
    """Exact availability limits; exhaustion never creates a safe claim."""

    elapsed_monotonic_ns: int = 1_800_000_000_000
    sweep_requests: int = 7_681
    unique_sweep_evaluations: int = 7_681
    pair_certificate_calls: int = 1_000_000
    interval_evaluations: int = 1_000_000
    interval_evaluations_per_pair: int = 4_096
    body_transform_evaluations: int = 65_536
    gjk_calls: int = 1_000_000
    gjk_iterations: int = 96_000_000
    progress_records: int = 4_096
    transform_cache_entries: int = 65_536
    distance_cache_entries: int = 262_144
    pair_cache_entries: int = 262_144
    sweep_cache_entries: int = 8_192

    def __post_init__(self) -> None:
        for field, value in asdict(self).items():
            if not isinstance(value, int) or isinstance(value, bool) or value < 0:
                raise ValueError(f"sweep work limit {field} must be a nonnegative int")


@dataclass
class _CacheEntry:
    value: Any
    digest: str


class ExactDigestLRU:
    """Bounded LRU whose exact values are verified on every reuse."""

    def __init__(self, *, name: str, maximum_entries: int) -> None:
        if not name or maximum_entries < 1:
            raise ValueError("cache name and positive entry limit are required")
        self.name = str(name)
        self.maximum_entries = int(maximum_entries)
        self._entries: OrderedDict[Hashable, _CacheEntry] = OrderedDict()
        self.hits = 0
        self.misses = 0
        self.evictions = 0

    @staticmethod
    def _digest(value: Any) -> str:
        detached = _json_safe(value)
        encoded = json.dumps(
            detached,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
        return hashlib.sha256(encoded).hexdigest()

    def get(self, key: Hashable) -> Any | None:
        entry = self._entries.get(key)
        if entry is None:
            self.misses += 1
            return None
        if entry.digest != self._digest(entry.value):
            raise G1SweepWorkError(
                "G1_FULL_ROBOT_SWEEP_CACHE_INCONSISTENT",
                f"{self.name} cache value differs from its digest",
            )
        self._entries.move_to_end(key)
        self.hits += 1
        return deepcopy(entry.value)

    def put(self, key: Hashable, value: Any) -> None:
        detached = deepcopy(_json_safe(value))
        digest = self._digest(detached)
        existing = self._entries.get(key)
        if existing is not None and existing.digest != digest:
            raise G1SweepWorkError(
                "G1_FULL_ROBOT_SWEEP_CACHE_INCONSISTENT",
                f"{self.name} cache key was rebound to a different value",
            )
        self._entries[key] = _CacheEntry(value=detached, digest=digest)
        self._entries.move_to_end(key)
        while len(self._entries) > self.maximum_entries:
            self._entries.popitem(last=False)
            self.evictions += 1

    def statistics(self) -> dict[str, int]:
        return {
            "hits": self.hits,
            "misses": self.misses,
            "evictions": self.evictions,
            "entries": len(self._entries),
            "maximum_entries": self.maximum_entries,
        }


class SweepWorkLedger:
    """Scene-owned deterministic counters and fail-closed budget authority."""

    _COUNTER_NAMES = (
        "sweep_requests",
        "unique_sweep_evaluations",
        "pair_certificate_calls",
        "interval_evaluations",
        "body_transform_evaluations",
        "gjk_calls",
        "gjk_iterations",
        "progress_records",
    )

    def __init__(
        self,
        *,
        limits: SweepWorkLimits,
        run_id: str,
        scene_id: str,
        trial_id: str,
        lifecycle_record_sha256: str,
        collision_snapshot_sha256: str,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
        progress_callback: Callable[[Mapping[str, Any]], None] | None = None,
    ) -> None:
        for field, value in (
            ("run_id", run_id),
            ("scene_id", scene_id),
            ("trial_id", trial_id),
        ):
            if not isinstance(value, str) or not value:
                raise ValueError(f"{field} is required")
        for field, value in (
            ("lifecycle_record_sha256", lifecycle_record_sha256),
            ("collision_snapshot_sha256", collision_snapshot_sha256),
        ):
            if (
                not isinstance(value, str)
                or len(value) != 64
                or any(char not in "0123456789abcdef" for char in value)
            ):
                raise ValueError(f"{field} must be a lowercase SHA-256")
        self.limits = limits
        self.run_id = run_id
        self.scene_id = scene_id
        self.trial_id = trial_id
        self.lifecycle_record_sha256 = lifecycle_record_sha256
        self.collision_snapshot_sha256 = collision_snapshot_sha256
        self._monotonic_ns = monotonic_ns
        self._started_ns = int(monotonic_ns())
        self._progress_callback = progress_callback
        self.counters = {name: 0 for name in self._COUNTER_NAMES}
        self._per_pair_intervals: dict[Hashable, int] = {}
        self._caches: dict[str, ExactDigestLRU] = {}
        self.last_class_id: str | None = None
        self.last_command_decimal: str | None = None
        self.last_action_index: int | None = None

    def register_cache(self, cache: ExactDigestLRU) -> None:
        if cache.name in self._caches and self._caches[cache.name] is not cache:
            raise G1SweepWorkError(
                "G1_FULL_ROBOT_SWEEP_CACHE_INCONSISTENT",
                f"duplicate cache authority: {cache.name}",
            )
        self._caches[cache.name] = cache

    def set_action_identity(
        self,
        *,
        class_id: str | None,
        command_decimal: str | None,
        action_index: int | None,
    ) -> None:
        self.last_class_id = None if class_id is None else str(class_id)
        self.last_command_decimal = (
            None if command_decimal is None else str(command_decimal)
        )
        self.last_action_index = (
            None if action_index is None else int(action_index)
        )

    def _failure(self, field: str, observed: int, limit: int) -> None:
        message = f"sweep work budget exceeded: {field}={observed} > {limit}"
        receipt = self.work_record(
            status="BLOCKED",
            failure_code="G1_FULL_ROBOT_SWEEP_WORK_BUDGET_EXCEEDED",
            failure_message=message,
        )
        if self._progress_callback is not None:
            self._progress_callback(receipt)
        raise G1SweepWorkError(
            "G1_FULL_ROBOT_SWEEP_WORK_BUDGET_EXCEEDED",
            message,
            receipt=receipt,
        )

    def check_elapsed(self) -> None:
        observed = int(self._monotonic_ns()) - self._started_ns
        if observed > self.limits.elapsed_monotonic_ns:
            self._failure(
                "elapsed_monotonic_ns",
                observed,
                self.limits.elapsed_monotonic_ns,
            )

    def consume(
        self,
        counter: str,
        amount: int = 1,
        *,
        pair_key: Hashable | None = None,
    ) -> None:
        if counter not in self.counters or amount < 0:
            raise ValueError(f"invalid sweep work counter: {counter}")
        self.check_elapsed()
        observed = self.counters[counter] + int(amount)
        self.counters[counter] = observed
        limit = int(getattr(self.limits, counter))
        if observed > limit:
            self._failure(counter, observed, limit)
        if counter == "interval_evaluations" and pair_key is not None:
            pair_observed = self._per_pair_intervals.get(pair_key, 0) + int(amount)
            self._per_pair_intervals[pair_key] = pair_observed
            if pair_observed > self.limits.interval_evaluations_per_pair:
                self._failure(
                    "interval_evaluations_per_pair",
                    pair_observed,
                    self.limits.interval_evaluations_per_pair,
                )

    def work_record(
        self,
        *,
        status: str,
        failure_code: str | None = None,
        failure_message: str | None = None,
    ) -> dict[str, Any]:
        if status not in {"RUNNING", "COMPLETE", "BLOCKED", "INTERRUPTED"}:
            raise ValueError("invalid sweep work status")
        record = {
            "schema_version": SWEEP_WORK_SCHEMA_VERSION,
            "run_id": self.run_id,
            "scene_id": self.scene_id,
            "trial_id": self.trial_id,
            "lifecycle_record_sha256": self.lifecycle_record_sha256,
            "collision_snapshot_sha256": self.collision_snapshot_sha256,
            "status": status,
            "failure_code": failure_code,
            "failure_message": failure_message,
            "limits": asdict(self.limits),
            "counters": dict(self.counters),
            "cache": {
                name: cache.statistics()
                for name, cache in sorted(self._caches.items())
            },
            "last_class_id": self.last_class_id,
            "last_command_decimal": self.last_command_decimal,
            "last_action_index": self.last_action_index,
            "selected_command_cap_m": None,
            "actuation_performed": False,
            "post_abort_actuation_count": 0,
            "force_vector_valid": False,
            "wrench_valid": False,
            "raw_impulse_used_as_force": False,
        }
        record["record_sha256"] = canonical_sha256(record)
        return record


class C2ASweepProgressJournal:
    """Durable sibling journal that exists before final C2a evidence."""

    def __init__(
        self,
        *,
        output: str | Path,
        repository_commit: str,
        run_id: str,
    ) -> None:
        output_path = Path(output)
        self.sidecar_path = output_path.with_name(
            output_path.name + ".sweep-progress.jsonl"
        )
        if self.sidecar_path.exists():
            raise FileExistsError(
                f"sweep progress sidecar already exists: {self.sidecar_path}"
            )
        self.sidecar_path.parent.mkdir(parents=True, exist_ok=True)
        self.repository_commit = str(repository_commit)
        self.run_id = str(run_id)
        self._records: list[dict[str, Any]] = []
        self.append(event="RUN_STARTED")

    def append(
        self,
        *,
        event: str,
        scene_id: str | None = None,
        trial_id: str | None = None,
        class_id: str | None = None,
        command_decimal: str | None = None,
        action_index: int | None = None,
        work_record: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        allowed = {
            "RUN_STARTED",
            "SNAPSHOT_PREPARED",
            "ROUTE_STARTED",
            "ACTION_MILESTONE",
            "ROUTE_COMPLETED",
            "WORK_BUDGET_EXCEEDED",
            "RUN_COMPLETED",
            "RUN_FAILED",
        }
        if event not in allowed:
            raise ValueError(f"unknown sweep progress event: {event}")
        if len(self._records) >= SweepWorkLimits().progress_records:
            raise G1SweepWorkError(
                "G1_FULL_ROBOT_SWEEP_WORK_BUDGET_EXCEEDED",
                "sweep progress record budget exceeded",
            )
        work_digest = None
        if work_record is not None:
            expected = canonical_sha256(
                work_record, exclude_fields=("record_sha256",)
            )
            if work_record.get("record_sha256") != expected:
                raise G1SweepWorkError(
                    "G1_FULL_ROBOT_SWEEP_CACHE_INCONSISTENT",
                    "sweep progress references an invalid work record",
                )
            work_digest = expected
        record = {
            "schema_version": SWEEP_PROGRESS_SCHEMA_VERSION,
            "sequence": len(self._records),
            "event": event,
            "repository_commit": self.repository_commit,
            "run_id": self.run_id,
            "scene_id": scene_id,
            "trial_id": trial_id,
            "class_id": class_id,
            "command_decimal": command_decimal,
            "action_index": action_index,
            "work_record_sha256": work_digest,
            "work_record": (
                None
                if work_record is None
                else deepcopy(_json_safe(work_record))
            ),
            "previous_record_sha256": (
                self._records[-1]["record_sha256"]
                if self._records
                else None
            ),
            "selected_command_cap_m": None,
            "actuation_performed": False,
            "post_abort_actuation_count": 0,
            "force_vector_valid": False,
            "wrench_valid": False,
            "raw_impulse_used_as_force": False,
        }
        record["record_sha256"] = canonical_sha256(record)
        with self.sidecar_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, sort_keys=True) + "\n")
            stream.flush()
            os.fsync(stream.fileno())
        self._records.append(record)
        return deepcopy(record)

    def snapshot(self) -> list[dict[str, Any]]:
        observed = [
            json.loads(line)
            for line in self.sidecar_path.read_text(encoding="utf-8").splitlines()
        ]
        observed = validate_sweep_progress_records(observed)
        if observed != self._records:
            raise G1SweepWorkError(
                "G1_FULL_ROBOT_SWEEP_CACHE_INCONSISTENT",
                "in-memory and durable sweep progress differ",
            )
        return deepcopy(observed)

    def remove_sidecar(self) -> None:
        self.sidecar_path.unlink(missing_ok=False)


def validate_sweep_work_record(record: Any) -> dict[str, Any]:
    """Validate one final or partial no-claim work record."""

    if not isinstance(record, Mapping):
        raise G1SweepWorkError(
            "G1_FULL_ROBOT_SWEEP_CACHE_INCONSISTENT",
            "sweep work record must be a mapping",
        )
    result = deepcopy(_json_safe(record))
    required = {
        "schema_version",
        "run_id",
        "scene_id",
        "trial_id",
        "lifecycle_record_sha256",
        "collision_snapshot_sha256",
        "status",
        "failure_code",
        "failure_message",
        "limits",
        "counters",
        "cache",
        "last_class_id",
        "last_command_decimal",
        "last_action_index",
        "selected_command_cap_m",
        "actuation_performed",
        "post_abort_actuation_count",
        "force_vector_valid",
        "wrench_valid",
        "raw_impulse_used_as_force",
        "record_sha256",
    }
    digest_fields = ("lifecycle_record_sha256", "collision_snapshot_sha256")
    limit_fields = set(asdict(SweepWorkLimits()))
    counter_fields = set(SweepWorkLedger._COUNTER_NAMES)
    valid = (
        set(result) == required
        and result.get("schema_version") == SWEEP_WORK_SCHEMA_VERSION
        and all(isinstance(result.get(field), str) and result.get(field) for field in ("run_id", "scene_id", "trial_id"))
        and all(
            isinstance(result.get(field), str)
            and len(result[field]) == 64
            and all(char in "0123456789abcdef" for char in result[field])
            for field in digest_fields
        )
        and result.get("status") in {"RUNNING", "COMPLETE", "BLOCKED", "INTERRUPTED"}
        and isinstance(result.get("limits"), Mapping)
        and set(result["limits"]) == limit_fields
        and all(
            isinstance(value, int) and not isinstance(value, bool) and value >= 0
            for value in result["limits"].values()
        )
        and isinstance(result.get("counters"), Mapping)
        and set(result["counters"]) == counter_fields
        and all(
            isinstance(value, int) and not isinstance(value, bool) and value >= 0
            for value in result["counters"].values()
        )
        and isinstance(result.get("cache"), Mapping)
        and result.get("selected_command_cap_m") is None
        and result.get("actuation_performed") is False
        and result.get("post_abort_actuation_count") == 0
        and result.get("force_vector_valid") is False
        and result.get("wrench_valid") is False
        and result.get("raw_impulse_used_as_force") is False
        and result.get("record_sha256")
        == canonical_sha256(result, exclude_fields=("record_sha256",))
    )
    if not valid:
        raise G1SweepWorkError(
            "G1_FULL_ROBOT_SWEEP_CACHE_INCONSISTENT",
            "sweep work record is invalid",
        )
    return result


def validate_sweep_progress_records(
    records: Any,
) -> list[dict[str, Any]]:
    """Independently validate a complete progress digest chain."""

    if not isinstance(records, (list, tuple)) or not records:
        raise G1SweepWorkError(
            "G1_FULL_ROBOT_SWEEP_CACHE_INCONSISTENT",
            "sweep progress records are missing",
        )
    observed = [deepcopy(_json_safe(record)) for record in records]
    previous = None
    for sequence, record in enumerate(observed):
        if (
            record.get("schema_version") != SWEEP_PROGRESS_SCHEMA_VERSION
            or record.get("sequence") != sequence
            or record.get("previous_record_sha256") != previous
            or record.get("record_sha256")
            != canonical_sha256(record, exclude_fields=("record_sha256",))
            or record.get("selected_command_cap_m") is not None
            or record.get("actuation_performed") is not False
            or record.get("post_abort_actuation_count") != 0
            or record.get("force_vector_valid") is not False
            or record.get("wrench_valid") is not False
            or record.get("raw_impulse_used_as_force") is not False
            or (
                record.get("work_record") is None
                and record.get("work_record_sha256") is not None
            )
            or (
                record.get("work_record") is not None
                and not isinstance(record.get("work_record"), Mapping)
            )
            or (
                isinstance(record.get("work_record"), Mapping)
                and (
                    record.get("work_record_sha256")
                    != record["work_record"].get("record_sha256")
                    or validate_sweep_work_record(record["work_record"])
                    != record["work_record"]
                )
            )
        ):
            raise G1SweepWorkError(
                "G1_FULL_ROBOT_SWEEP_CACHE_INCONSISTENT",
                "sweep progress digest chain is invalid",
            )
        previous = record["record_sha256"]
    return observed


__all__ = [
    "C2ASweepProgressJournal",
    "ExactDigestLRU",
    "G1SweepWorkError",
    "SWEEP_PROGRESS_SCHEMA_VERSION",
    "SWEEP_WORK_SCHEMA_VERSION",
    "SweepWorkLedger",
    "SweepWorkLimits",
    "canonical_sha256",
    "validate_sweep_work_record",
    "validate_sweep_progress_records",
]

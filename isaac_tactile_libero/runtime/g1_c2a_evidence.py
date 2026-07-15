"""Import-safe C2a selected-pose evidence and freshness validation.

This module only reads immutable files and validates pure provenance.  It never
imports Isaac Sim, Kit, USD, PhysX, or rendering modules.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Any, Callable, Mapping

from isaac_tactile_libero.runtime.g1_static_pose import (
    C2A_ARTICULATION_JOINT_NAMES,
    C2A_ARM_JOINT_NAMES,
    validate_c2a_offline_record,
)
from isaac_tactile_libero.runtime.g1_tracking import G1ValidationError
from isaac_tactile_libero.tasks.press_button_mechanism import (
    load_press_button_mechanism_config,
)


REFRESH_BLOCKER = "CURRENT_C2A_REFRESH_REQUIRED_AFTER_GEOMETRY_SCHEMA_CHANGE"
_EVIDENCE_REQUIRED = "G1_C1_C2A_EVIDENCE_REQUIRED"
_CHECKSUM_MISMATCH = "G1_C1_C2A_EVIDENCE_CHECKSUM_MISMATCH"
_SELECTED_INVALID = "G1_C1_SELECTED_POSE_INVALID"
_SELECTED_HASH_MISMATCH = "G1_C1_SELECTED_POSE_HASH_MISMATCH"
_PROVENANCE_MISMATCH = "G1_C1_SELECTED_POSE_PROVENANCE_MISMATCH"
_SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
_CHECKSUM_LINE_RE = re.compile(r"^([0-9a-f]{64})  ([^/\\]+)$")
_CANDIDATE_IDS = (
    "task-ready-z-0p55",
    "task-ready-z-0p54",
    "task-ready-z-0p53",
)


@dataclass(frozen=True)
class G1CurrentInputDigests:
    task_config_sha256: str
    robot_config_sha256: str
    fr3_asset_sha256: str
    task_card_sha256: str
    geometry_sha256: str


@dataclass(frozen=True)
class C2ASelectedPoseEvidence:
    evidence_dir: Path
    report: Mapping[str, object]
    candidate_record: Mapping[str, object]
    selected_pose_id: str
    selected_pose_sha256: str
    repository_commit: str


def _fail(code: str, message: str) -> None:
    raise G1ValidationError(code, message)


def _sha256_path(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _canonical_sha256(value: Any) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _require_sha256(value: Any, *, field: str, code: str = _SELECTED_INVALID) -> str:
    if not isinstance(value, str) or _SHA256_RE.fullmatch(value) is None:
        _fail(code, f"selected C2a provenance has invalid {field}")
    return value


def _read_json_mapping(path: Path, *, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        _fail(_SELECTED_INVALID, f"C2a {label} is not valid JSON: {path}: {exc}")
    if not isinstance(value, Mapping):
        _fail(_SELECTED_INVALID, f"C2a {label} must be a mapping: {path}")
    return dict(value)


def _verify_checksums(evidence_dir: Path) -> dict[str, str]:
    checksum_path = evidence_dir / "checksums.sha256"
    if not checksum_path.is_file():
        _fail(_EVIDENCE_REQUIRED, f"C2a checksums are missing: {checksum_path}")
    try:
        lines = checksum_path.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeError) as exc:
        _fail(_CHECKSUM_MISMATCH, f"cannot read C2a checksums: {exc}")
    if not lines:
        _fail(_CHECKSUM_MISMATCH, "C2a checksums file is empty")
    entries: dict[str, str] = {}
    for line_number, line in enumerate(lines, start=1):
        match = _CHECKSUM_LINE_RE.fullmatch(line)
        if match is None:
            _fail(
                _CHECKSUM_MISMATCH,
                f"C2a checksum line {line_number} is malformed",
            )
        expected, name = match.groups()
        if name in entries:
            _fail(_CHECKSUM_MISMATCH, f"C2a checksum entry is duplicated: {name}")
        artifact = evidence_dir / name
        if not artifact.is_file():
            _fail(_EVIDENCE_REQUIRED, f"C2a checksum artifact is missing: {artifact}")
        actual = _sha256_path(artifact)
        if actual != expected:
            _fail(
                _CHECKSUM_MISMATCH,
                f"C2a checksum mismatch for {name}: expected {expected}, got {actual}",
            )
        entries[name] = expected
    for required in ("report.json", "offline_candidates.jsonl", "manifest.json"):
        if required not in entries:
            _fail(_EVIDENCE_REQUIRED, f"C2a checksums omit required artifact: {required}")
    return entries


def _read_candidate_records(path: Path) -> list[dict[str, Any]]:
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeError) as exc:
        _fail(_SELECTED_INVALID, f"cannot read C2a offline candidates: {exc}")
    if not lines:
        _fail(_SELECTED_INVALID, "C2a offline candidate JSONL is empty")
    records: list[dict[str, Any]] = []
    for line_number, line in enumerate(lines, start=1):
        if not line.strip():
            _fail(_SELECTED_INVALID, f"C2a offline candidate line {line_number} is empty")
        try:
            value = json.loads(line)
        except json.JSONDecodeError as exc:
            _fail(
                _SELECTED_INVALID,
                f"C2a offline candidate line {line_number} is invalid JSON: {exc}",
            )
        if not isinstance(value, Mapping):
            _fail(_SELECTED_INVALID, f"C2a candidate line {line_number} is not a mapping")
        records.append(dict(value))
    return records


def _validate_manifest_artifacts(
    *, evidence_dir: Path, manifest: Mapping[str, Any]
) -> None:
    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, list) or not artifacts:
        _fail(_SELECTED_INVALID, "C2a manifest artifact provenance is missing")
    seen: set[str] = set()
    for item in artifacts:
        if not isinstance(item, Mapping):
            _fail(_SELECTED_INVALID, "C2a manifest artifact record is malformed")
        name = item.get("path")
        digest = item.get("sha256")
        if not isinstance(name, str) or "/" in name or "\\" in name or name in seen:
            _fail(_SELECTED_INVALID, "C2a manifest artifact path is invalid or duplicated")
        expected = _require_sha256(digest, field=f"manifest artifact {name}")
        path = evidence_dir / name
        if not path.is_file() or _sha256_path(path) != expected:
            _fail(_CHECKSUM_MISMATCH, f"C2a manifest artifact digest mismatch: {name}")
        seen.add(name)
    if not {"report.json", "offline_candidates.jsonl"}.issubset(seen):
        _fail(_SELECTED_INVALID, "C2a manifest omits report or offline candidates")


def _candidate_digest_map(candidate: Mapping[str, Any]) -> dict[str, str | None]:
    return {
        "task_config_sha256": candidate.get("task_config_sha256"),
        "robot_config_sha256": candidate.get("robot_config_sha256"),
        "fr3_asset_sha256": candidate.get("asset_sha256"),
        "task_card_sha256": candidate.get("task_card_sha256"),
        "geometry_sha256": candidate.get("geometry_sha256"),
    }


def _validate_selected_candidate(candidate: Mapping[str, Any]) -> dict[str, Any]:
    if candidate.get("synthetic_test_double") is not False or candidate.get(
        "real_runtime_truth"
    ) is not True:
        _fail(_SELECTED_INVALID, "selected C2a candidate must be real and non-synthetic")
    if candidate.get("solver_identity") != "isaacsim_lula_fr3":
        _fail(_SELECTED_INVALID, "selected C2a candidate is not a real Lula solve")
    if candidate.get("offline_failure_code") is not None:
        _fail(_SELECTED_INVALID, "rejected C2a candidate cannot be selected")
    if candidate.get("ik_solution_valid") is not True or candidate.get(
        "fk_residual_valid"
    ) is not True:
        _fail(_SELECTED_INVALID, "selected C2a candidate lacks valid IK/FK truth")
    if tuple(candidate.get("solver_joint_names", ())) != C2A_ARM_JOINT_NAMES:
        _fail(_SELECTED_INVALID, "selected C2a solver joint order is invalid")
    if tuple(candidate.get("articulation_joint_names", ())) != C2A_ARTICULATION_JOINT_NAMES:
        _fail(_SELECTED_INVALID, "selected C2a articulation joint order is invalid")
    if (
        candidate.get("solver_frame") != "fr3_hand_tcp"
        or candidate.get("base_frame") != "fr3_link0"
        or candidate.get("ee_frame") != "/World/FR3/fr3_hand_tcp"
    ):
        _fail(_SELECTED_INVALID, "selected C2a frame identity is invalid")
    for field, length in (
        ("solver_joint_values", 7),
        ("articulation_joint_values", 9),
        ("fk_position_world_m", 3),
        ("fk_orientation_xyzw", 4),
    ):
        value = candidate.get(field)
        if (
            not isinstance(value, list)
            or len(value) != length
            or any(not isinstance(item, (int, float)) or not math.isfinite(float(item)) for item in value)
        ):
            _fail(_SELECTED_INVALID, f"selected C2a field is invalid: {field}")
    for field in ("ik_position_residual_m", "ik_orientation_residual_rad"):
        value = candidate.get(field)
        if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
            _fail(_SELECTED_INVALID, f"selected C2a residual is invalid: {field}")
    for field in (
        "asset_sha256",
        "task_config_sha256",
        "robot_config_sha256",
        "orientation_source_sha256",
    ):
        _require_sha256(candidate.get(field), field=field)
    orientation_source = candidate.get("orientation_source")
    if not isinstance(orientation_source, Mapping) or _canonical_sha256(
        orientation_source
    ) != candidate.get("orientation_source_sha256"):
        _fail(_SELECTED_INVALID, "selected C2a orientation provenance digest is invalid")
    if orientation_source.get("asset_sha256") != candidate.get("asset_sha256"):
        _fail(_SELECTED_INVALID, "selected C2a orientation/asset provenance disagrees")
    try:
        return validate_c2a_offline_record(candidate)
    except G1ValidationError as exc:
        _fail(_SELECTED_INVALID, f"selected C2a solver/FK record is invalid: {exc.message}")


def _validate_cross_artifact_provenance(
    *,
    report: Mapping[str, Any],
    manifest: Mapping[str, Any],
    candidate: Mapping[str, Any],
    selected_sha256: str,
) -> None:
    for name, summary in (("report", report), ("manifest", manifest)):
        if summary.get("selected_pose_id") != candidate.get("candidate_id"):
            _fail(_SELECTED_INVALID, f"C2a {name} selected pose ID disagrees with JSONL")
        if summary.get("selected_pose_sha256") != selected_sha256:
            _fail(
                _SELECTED_HASH_MISMATCH,
                f"C2a {name} selected hash does not match recomputed JSONL hash {selected_sha256}",
            )
        metadata = summary.get("runtime_metadata")
        if not isinstance(metadata, Mapping):
            _fail(_SELECTED_INVALID, f"C2a {name} runtime provenance is missing")
        for field in ("task_config_sha256", "robot_config_sha256"):
            if metadata.get(field) != candidate.get(field):
                _fail(_PROVENANCE_MISMATCH, f"C2a {name}/{field} provenance disagrees")
        if metadata.get("asset_sha256") != candidate.get("asset_sha256"):
            _fail(_PROVENANCE_MISMATCH, f"C2a {name}/fr3_asset provenance disagrees")

    candidate_digests = _candidate_digest_map(candidate)
    has_future_digest = any(
        candidate_digests[field] is not None
        for field in ("task_card_sha256", "geometry_sha256")
    )
    if not has_future_digest:
        return
    for field, value in candidate_digests.items():
        _require_sha256(value, field=field)
    for name, summary in (("report", report), ("manifest", manifest)):
        digests = summary.get("current_input_digests")
        if not isinstance(digests, Mapping) or dict(digests) != candidate_digests:
            _fail(_PROVENANCE_MISMATCH, f"C2a {name} five-digest provenance disagrees")
        provenance = summary.get("selected_candidate_provenance")
        expected = {
            "candidate_id": candidate["candidate_id"],
            "candidate_sha256": selected_sha256,
            "solver_joint_names": list(candidate["solver_joint_names"]),
            "articulation_joint_names": list(candidate["articulation_joint_names"]),
            "solver_frame": candidate["solver_frame"],
            "base_frame": candidate["base_frame"],
            "ee_frame": candidate["ee_frame"],
            "solver_identity": candidate["solver_identity"],
        }
        if not isinstance(provenance, Mapping) or dict(provenance) != expected:
            _fail(_PROVENANCE_MISMATCH, f"C2a {name} selected candidate provenance disagrees")


def load_g1_c2a_selected_pose_evidence(
    evidence_dir: Path,
) -> C2ASelectedPoseEvidence:
    """Load and independently verify one immutable C2a selected-pose directory."""

    if evidence_dir is None:
        _fail(_EVIDENCE_REQUIRED, "an explicit C2a evidence directory is required")
    directory = Path(evidence_dir).resolve()
    if not directory.is_dir():
        _fail(_EVIDENCE_REQUIRED, f"C2a evidence directory does not exist: {directory}")
    _verify_checksums(directory)
    report = _read_json_mapping(directory / "report.json", label="report")
    manifest = _read_json_mapping(directory / "manifest.json", label="manifest")
    records = _read_candidate_records(directory / "offline_candidates.jsonl")
    _validate_manifest_artifacts(evidence_dir=directory, manifest=manifest)

    selected_pose_id = report.get("selected_pose_id")
    if not isinstance(selected_pose_id, str) or not selected_pose_id.strip():
        _fail(_SELECTED_INVALID, "C2a report does not identify one selected pose")
    matches = [record for record in records if record.get("candidate_id") == selected_pose_id]
    if len(matches) != 1:
        _fail(
            _SELECTED_INVALID,
            f"C2a evidence must contain exactly one selected candidate; found {len(matches)}",
        )
    ids = [record.get("candidate_id") for record in records]
    if len(ids) != len(set(ids)) or ids != list(_CANDIDATE_IDS):
        _fail(_SELECTED_INVALID, "C2a candidate retention/order is duplicated or incomplete")
    selected_pose_sha256 = _canonical_sha256(matches[0])
    for name, summary in (("report", report), ("manifest", manifest)):
        if summary.get("selected_pose_sha256") != selected_pose_sha256:
            _fail(
                _SELECTED_HASH_MISMATCH,
                f"C2a {name} selected hash {summary.get('selected_pose_sha256')} "
                f"does not match recomputed JSONL hash {selected_pose_sha256}",
            )
    candidate = _validate_selected_candidate(matches[0])
    _validate_cross_artifact_provenance(
        report=report,
        manifest=manifest,
        candidate=matches[0],
        selected_sha256=selected_pose_sha256,
    )
    report_repository = report.get("repository")
    manifest_repository = manifest.get("repository")
    if (
        not isinstance(report_repository, Mapping)
        or dict(report_repository) != dict(manifest_repository or {})
        or report_repository.get("dirty") is not False
    ):
        _fail(_SELECTED_INVALID, "C2a repository provenance is invalid or inconsistent")
    repository_commit = report_repository.get("commit")
    if (
        not isinstance(repository_commit, str)
        or len(repository_commit) != 40
        or any(character not in "0123456789abcdef" for character in repository_commit)
    ):
        _fail(_SELECTED_INVALID, "C2a repository commit provenance is invalid")
    return C2ASelectedPoseEvidence(
        evidence_dir=directory,
        report=report,
        candidate_record=candidate,
        selected_pose_id=selected_pose_id,
        selected_pose_sha256=selected_pose_sha256,
        repository_commit=repository_commit,
    )


def compute_g1_current_input_digests(
    *,
    task_config_path: Path,
    robot_config_path: Path,
    fr3_asset_path: Path,
    task_card_path: Path,
) -> G1CurrentInputDigests:
    """Hash current tracked inputs and the parsed geometry contract."""

    paths = {
        "task_config": Path(task_config_path).resolve(),
        "robot_config": Path(robot_config_path).resolve(),
        "fr3_asset": Path(fr3_asset_path).resolve(),
        "task_card": Path(task_card_path).resolve(),
    }
    for family, path in paths.items():
        if not path.is_file():
            _fail(_PROVENANCE_MISMATCH, f"current {family} input is missing: {path}")
    try:
        mechanism = load_press_button_mechanism_config(paths["task_config"])
    except Exception as exc:
        _fail(_PROVENANCE_MISMATCH, f"current parsed geometry is invalid: {exc}")
    contract = mechanism.geometry_contract
    if contract is None or not mechanism.route_validation_input_eligible:
        _fail(_PROVENANCE_MISMATCH, "current formal PressButton geometry is unavailable")
    return G1CurrentInputDigests(
        task_config_sha256=_sha256_path(paths["task_config"]),
        robot_config_sha256=_sha256_path(paths["robot_config"]),
        fr3_asset_sha256=_sha256_path(paths["fr3_asset"]),
        task_card_sha256=_sha256_path(paths["task_card"]),
        geometry_sha256=contract.geometry_sha256,
    )


def validate_g1_c2a_current_input_provenance(
    evidence: C2ASelectedPoseEvidence,
    current: G1CurrentInputDigests,
) -> None:
    """Reject historical or stale selected-pose evidence before runtime creation."""

    current_values = {
        "task_config_sha256": current.task_config_sha256,
        "robot_config_sha256": current.robot_config_sha256,
        "fr3_asset_sha256": current.fr3_asset_sha256,
        "task_card_sha256": current.task_card_sha256,
        "geometry_sha256": current.geometry_sha256,
    }
    for field, value in current_values.items():
        _require_sha256(value, field=f"current {field}", code=_PROVENANCE_MISMATCH)
    candidate_values = _candidate_digest_map(evidence.candidate_record)
    missing_migrated = [
        field
        for field in ("task_card_sha256", "geometry_sha256")
        if candidate_values.get(field) is None
    ]
    if missing_migrated:
        families = ", ".join(missing_migrated)
        _fail(
            REFRESH_BLOCKER,
            f"historical C2a evidence {evidence.evidence_dir} lacks current {families}; "
            "a separately approved fresh C2a run is required after the geometry schema "
            "change; the historical evidence was not modified",
        )
    mismatches = [
        field
        for field, current_value in current_values.items()
        if candidate_values.get(field) != current_value
    ]
    if mismatches:
        _fail(
            _PROVENANCE_MISMATCH,
            "selected C2a evidence disagrees with current input digest families: "
            + ", ".join(mismatches),
        )


def prepare_g1_c2a_tracking_inputs(
    *,
    c2a_evidence_dir: Path | None,
    task_config_path: Path,
    robot_config_path: Path,
    fr3_asset_path: Path,
    task_card_path: Path,
    factory_builder: Callable[[], Any],
) -> dict[str, Any]:
    """Complete Task 9 validation before invoking the supplied factory builder."""

    if c2a_evidence_dir is None:
        _fail(_EVIDENCE_REQUIRED, "an explicit C2a evidence directory is required")
    evidence = load_g1_c2a_selected_pose_evidence(Path(c2a_evidence_dir))
    current = compute_g1_current_input_digests(
        task_config_path=Path(task_config_path),
        robot_config_path=Path(robot_config_path),
        fr3_asset_path=Path(fr3_asset_path),
        task_card_path=Path(task_card_path),
    )
    validate_g1_c2a_current_input_provenance(evidence, current)
    factory = factory_builder()
    return {"evidence": evidence, "current_input_digests": current, "factory": factory}


__all__ = [
    "C2ASelectedPoseEvidence",
    "G1CurrentInputDigests",
    "REFRESH_BLOCKER",
    "compute_g1_current_input_digests",
    "load_g1_c2a_selected_pose_evidence",
    "prepare_g1_c2a_tracking_inputs",
    "validate_g1_c2a_current_input_provenance",
]

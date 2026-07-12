#!/usr/bin/env python3
"""Import-safe C2a offline/static preliminary runner.

Real Isaac execution remains approval-gated; all runtime objects are injected.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import shlex
from typing import Any, Callable, Mapping, Sequence

from isaac_tactile_libero.robots.fr3_static_pose_diagnostic import (
    author_c2a_joint_state_before_play,
)
from isaac_tactile_libero.runtime.g1_static_pose import (
    validate_c2a_readiness_sample,
)


C2A_SEED = 1701
C2A_READINESS_ACTIONS = 64
C2A_PHYSICS_SUBSTEPS = 3


def _jsonable(value: Any) -> Any:
    if hasattr(value, "tolist"):
        return value.tolist()
    if isinstance(value, Mapping):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


def _write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(_jsonable(value), indent=2, sort_keys=True) + "\n", encoding="utf-8")


def author_c2a_pose_before_play(**kwargs: Any) -> dict[str, Any]:
    """Expose the injected authoring seam used by fake and real stages."""

    return author_c2a_joint_state_before_play(**kwargs)


def _default_test_double_readiness_sample(
    *,
    candidate: Mapping[str, Any],
    scene_id: str,
    fresh_scene_token: str,
    action_index: int,
) -> dict[str, Any]:
    target = list(candidate["articulation_joint_values"])
    return {
        "schema_version": "g1.c2a.static.v1",
        "candidate_id": candidate["candidate_id"],
        "scene_id": scene_id,
        "fresh_scene_token": fresh_scene_token,
        "seed": C2A_SEED,
        "readiness_action_index": action_index,
        "requested_vector_m": [0.0, 0.0, 0.0],
        "physics_substeps": C2A_PHYSICS_SUBSTEPS,
        "target_before": target.copy(),
        "target_after": target.copy(),
        "send_result": True,
        "contact": False,
        "raw_contact_count": 0,
        "collision": False,
        "penetration_m": 0.0,
        "penetration_provenance_valid": True,
        "collision_monitor_error": None,
        "button_released": True,
        "button_reset": True,
        "force_vector_valid": False,
        "wrench_valid": False,
        "raw_impulse_used_as_force": False,
        "finite": True,
        "post_abort_actuation_count": 0,
        "synthetic_test_double": True,
    }


def run_c2a_static_qualification(
    *,
    candidate_records: Sequence[Mapping[str, Any]],
    scene_factory: Callable[..., Any],
    nonzero_sender: Callable[..., Any] | None = None,
) -> dict[str, Any]:
    """Run three fresh, fixed-zero static scenes per retained candidate."""

    del nonzero_sender  # Deliberately unreachable: C2a has no non-zero action path.
    static_scenes: list[dict[str, Any]] = []
    tokens: list[str] = []
    for candidate in candidate_records:
        for scene_index in range(3):
            scene_id = f"{candidate['candidate_id']}-scene-{scene_index}"
            token = f"c2a-{candidate['candidate_id']}-{scene_index}-{C2A_SEED}"
            scene = scene_factory(
                candidate_id=candidate["candidate_id"],
                candidate_record=dict(candidate),
                scene_id=scene_id,
                fresh_scene_token=token,
                scene_index=scene_index,
                seed=C2A_SEED,
            )
            tokens.append(token)
            step = getattr(scene, "run_zero_readiness_action", None)
            samples: list[dict[str, Any]] = []
            failure_code: str | None = None
            for action_index in range(C2A_READINESS_ACTIONS):
                if callable(step):
                    sample = dict(
                        step(
                            requested_vector_m=(0.0, 0.0, 0.0),
                            action_index=action_index,
                            physics_substeps=C2A_PHYSICS_SUBSTEPS,
                        )
                    )
                    sample.update(
                        scene_id=scene_id,
                        fresh_scene_token=token,
                        readiness_action_index=action_index,
                    )
                else:
                    sample = _default_test_double_readiness_sample(
                        candidate=candidate,
                        scene_id=scene_id,
                        fresh_scene_token=token,
                        action_index=action_index,
                    )
                samples.append(sample)
                try:
                    validate_c2a_readiness_sample(sample)
                except Exception as error:
                    failure_code = str(getattr(error, "code", "G1_C2A_NONFINITE"))
                    break
            close = getattr(scene, "close", None)
            if callable(close):
                close()
            real_runtime_truth = bool(samples) and not any(
                sample.get("synthetic_test_double") is True for sample in samples
            )
            static_scenes.append(
                {
                    "schema_version": "g1.c2a.static.v1",
                    "candidate_id": candidate["candidate_id"],
                    "scene_id": scene_id,
                    "fresh_scene_token": token,
                    "stage_object_id": id(scene),
                    "articulation_object_id": id(scene),
                    "seed": C2A_SEED,
                    "readiness_samples": samples,
                    "failure_code": failure_code,
                    "passed": (
                        failure_code is None
                        and len(samples) == C2A_READINESS_ACTIONS
                        and real_runtime_truth
                    ),
                    "claim_eligible": False,
                }
            )
    return {
        "scene_count": len(static_scenes),
        "fresh_scene_tokens": tokens,
        "static_scenes": static_scenes,
        "readiness_sample_count": sum(
            len(scene["readiness_samples"]) for scene in static_scenes
        ),
        "claim_eligible": False,
        "controlled_arrival": False,
        "direct_reset_qualified": False,
        "reset_repeatability_qualified": False,
        "c2_completed": False,
        "selected_command_cap_m": None,
    }


def write_c2a_static_evidence(
    *,
    output: str | Path,
    repository_commit: str,
    command: Sequence[str],
    offline_candidates: Sequence[Mapping[str, Any]],
    static_scenes: Sequence[Mapping[str, Any]],
    readiness_samples: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    """Write one immutable, preliminary, non-claim C2a directory."""

    destination = Path(output)
    destination.mkdir(parents=True, exist_ok=False)
    command_path = destination / "command.log"
    command_path.write_text(shlex.join([str(item) for item in command]) + "\n", encoding="utf-8")
    offline_path = destination / "offline_candidates.jsonl"
    offline_path.write_text(
        "".join(json.dumps(_jsonable(item), sort_keys=True) + "\n" for item in offline_candidates),
        encoding="utf-8",
    )
    scenes_path = destination / "static_scenes.jsonl"
    scenes_path.write_text(
        "".join(json.dumps(_jsonable(item), sort_keys=True) + "\n" for item in static_scenes),
        encoding="utf-8",
    )
    readiness_path = destination / "readiness_samples.jsonl"
    readiness_path.write_text(
        "".join(json.dumps(_jsonable(item), sort_keys=True) + "\n" for item in readiness_samples),
        encoding="utf-8",
    )
    report = {
        "schema_version": "g1.c2a.static.v1",
        "evidence_stage": "preliminary",
        "repository": {"commit": str(repository_commit), "dirty": False},
        "offline_candidate_count": len(offline_candidates),
        "static_scene_count": len(static_scenes),
        "readiness_sample_count": len(readiness_samples),
        "claim_eligible": False,
        "controlled_arrival": False,
        "direct_reset_qualified": False,
        "reset_repeatability_qualified": False,
        "selected_command_cap_m": None,
        "c2_completed": False,
        "gate_status_updated": False,
        "t070_completed": False,
    }
    report_path = destination / "report.json"
    _write_json(report_path, report)
    artifact_paths = (command_path, offline_path, scenes_path, readiness_path, report_path)
    manifest = {
        **report,
        "run_id": destination.name,
        "gate_id": "G1",
        "status": "BLOCKED",
        "command": [str(item) for item in command],
        "artifacts": [
            {"path": path.name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
            for path in artifact_paths
        ],
        "blockers": ["C2A_PRELIMINARY_NOT_GATE_EVIDENCE"],
    }
    manifest_path = destination / "manifest.json"
    _write_json(manifest_path, manifest)
    checksum_paths = (*artifact_paths, manifest_path)
    (destination / "checksums.sha256").write_text(
        "".join(
            f"{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.name}\n"
            for path in checksum_paths
        ),
        encoding="utf-8",
    )
    return report


__all__ = [
    "author_c2a_pose_before_play",
    "run_c2a_static_qualification",
    "write_c2a_static_evidence",
]

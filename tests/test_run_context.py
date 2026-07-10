from __future__ import annotations

from pathlib import Path
import sys

from isaac_tactile_libero.evidence.run_context import RunContext


def test_run_context_records_command_lock_and_runtime_without_isaac_import(tmp_path: Path) -> None:
    lock = tmp_path / "lock.txt"
    lock.write_text("numpy==2.4.2\n", encoding="utf-8")
    before = set(sys.modules)

    context = RunContext.capture(
        command=["python", "-m", "pytest", "-q"],
        dependency_lock=lock,
        isaac_sim="6.0.1",
        gpu="NVIDIA RTX 4090 48GB",
    )

    newly_loaded = set(sys.modules) - before
    assert context.run_id
    assert context.command == ("python", "-m", "pytest", "-q")
    assert len(context.dependency_lock_sha256) == 64
    assert context.python.startswith("3.")
    assert context.isaac_sim == "6.0.1"
    assert context.gpu == "NVIDIA RTX 4090 48GB"
    assert "isaacsim" not in newly_loaded
    assert "omni" not in newly_loaded
    assert "carb" not in newly_loaded


def test_run_context_serialization_is_json_safe(tmp_path: Path) -> None:
    lock = tmp_path / "lock.txt"
    lock.write_text("x", encoding="utf-8")
    payload = RunContext.capture(command=["true"], dependency_lock=lock).as_dict()
    assert payload["command"] == ["true"]
    assert payload["started_at"].endswith("Z")

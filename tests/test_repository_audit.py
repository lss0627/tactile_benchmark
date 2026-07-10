from __future__ import annotations

import json
from pathlib import Path
import subprocess

from isaac_tactile_libero.repository.audit import audit_repository


def _git(root: Path, *args: str) -> None:
    subprocess.run(["git", "-C", str(root), *args], check=True, capture_output=True)


def test_repository_audit_classifies_required_and_generated_paths(tmp_path: Path) -> None:
    _git(tmp_path, "init")
    (tmp_path / ".gitignore").write_text("/generated/\n/ignored/\n", encoding="utf-8")
    (tmp_path / "src").mkdir()
    tracked = tmp_path / "src" / "tracked.py"
    tracked.write_text("VALUE = 1\n", encoding="utf-8")
    _git(tmp_path, "add", ".gitignore", "src/tracked.py")
    _git(
        tmp_path,
        "-c",
        "user.name=Test",
        "-c",
        "user.email=test@example.invalid",
        "commit",
        "-m",
        "baseline",
    )
    tracked.write_text("VALUE = 2\n", encoding="utf-8")
    (tmp_path / "src" / "untracked.py").write_text("VALUE = 3\n", encoding="utf-8")
    (tmp_path / "ignored").mkdir()
    (tmp_path / "ignored" / "required.yaml").write_text("required: true\n", encoding="utf-8")
    (tmp_path / "generated").mkdir()
    (tmp_path / "generated" / "result.json").write_text("{}\n", encoding="utf-8")
    external = tmp_path.parent / "robot.usd"
    external.write_text("#usda 1.0\n", encoding="utf-8")

    report = audit_repository(
        tmp_path,
        required_patterns=("src/*.py", "ignored/*.yaml"),
        generated_patterns=("generated/**",),
        external_assets=(external,),
    )

    assert report["modified"] == ["src/tracked.py"]
    assert report["untracked_required"] == ["src/untracked.py"]
    assert report["ignored_required"] == ["ignored/required.yaml"]
    assert report["generated"] == ["generated/result.json"]
    assert report["external_assets"][0]["path"] == str(external)
    assert report["clean_checkout_ready"] is False


def test_repository_audit_schema_fixture_lists_all_classifications() -> None:
    schema = json.loads(
        Path("tests/fixtures/repository_audit.schema.json").read_text(encoding="utf-8")
    )
    required = set(schema["required"])
    assert {
        "tracked",
        "modified",
        "untracked_required",
        "ignored_required",
        "generated",
        "external_assets",
        "clean_checkout_ready",
    } <= required


def test_repository_audit_supports_git_archive_without_dot_git(tmp_path: Path) -> None:
    (tmp_path / "isaac_tactile_libero").mkdir()
    (tmp_path / "isaac_tactile_libero" / "__init__.py").write_text("", encoding="utf-8")
    report = audit_repository(
        tmp_path,
        required_patterns=("isaac_tactile_libero/**/*.py",),
        generated_patterns=(),
    )
    assert report["source_kind"] == "archive"
    assert report["tracked"] == ["isaac_tactile_libero/__init__.py"]
    assert report["modified"] == []
    assert report["untracked_required"] == []
    assert report["clean_checkout_ready"] is True

from pathlib import Path

import tomllib


def test_formal_baseline_is_python312_and_promoted_isaacsim601_lock() -> None:
    pyproject = tomllib.loads(Path("pyproject.toml").read_text(encoding="utf-8"))
    assert pyproject["project"]["requires-python"] == ">=3.12,<3.13"
    lock = Path("requirements/lock-py312.txt").read_text(encoding="utf-8")
    assert "candidates/lock-py312-isaacsim-6.0.1.txt" in lock
    guide = Path("requirements/isaac-sim-6.0.1.md").read_text(encoding="utf-8")
    assert "GPU_CONTACT_NATIVE_INSTABILITY" in guide
    assert "UNVALIDATED" in guide
    assert Path("requirements/archive/lock-py311-isaacsim-5.1.txt").exists()


def test_first_party_cutover_has_no_removed_or_deprecated_imports() -> None:
    from isaac_tactile_libero.repository.import_scan import scan_first_party_imports

    report = scan_first_party_imports(Path.cwd())
    assert report["errors"] == []
    assert report["warnings"] == []

from __future__ import annotations

from pathlib import Path

from isaac_tactile_libero.repository.import_scan import scan_first_party_imports


def test_cutover_has_no_removed_or_deprecated_isaac_imports() -> None:
    report = scan_first_party_imports(Path("."))
    assert report["errors"] == []
    assert report["warnings"] == []


def test_import_scan_excludes_vendor_and_historical_docs(tmp_path: Path) -> None:
    (tmp_path / "isaac_tactile_libero").mkdir()
    (tmp_path / "isaac_tactile_libero" / "ok.py").write_text(
        "import json\n",
        encoding="utf-8",
    )
    (tmp_path / "vendor").mkdir()
    (tmp_path / "vendor" / "old.py").write_text(
        "import omni.isaac.kit\n",
        encoding="utf-8",
    )
    report = scan_first_party_imports(tmp_path)
    assert report["errors"] == []

import json
import subprocess
import sys


def test_check_backend_config_accepts_default_probe_config(tmp_path):
    output = tmp_path / "backend_config_report.json"

    result = subprocess.run(
        [
            sys.executable,
            "scripts/check_backend_config.py",
            "--config",
            "configs/backend/lightwheel_optional.yaml",
            "--output",
            str(output),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    payload = json.loads(result.stdout)
    assert payload["ok"] is True
    assert payload["backend_enabled"] is False
    assert payload["probe_only"] is True
    assert payload["asset_provenance_ok"] is True
    assert payload["missing_required_fields"] == []


def test_check_backend_config_fails_enabled_runtime_import_without_probe_only(tmp_path):
    config = tmp_path / "bad_backend.yaml"
    config.write_text(
        "\n".join(
            [
                "enabled: true",
                "backend_enabled: true",
                "lightwheel_repo_path: ''",
                "lightwheel_python_package: lightwheel",
                "lightwheel_asset_root: ''",
                "use_lightwheel_assets: true",
                "require_assets: false",
                "allow_noncommercial_assets: true",
                "allow_runtime_import: true",
                "probe_only: false",
                "planned_tasks: [PressButton]",
                "task_import_mode: manifest_only",
                "runtime_status: probe_only_not_connected",
                "asset_manifest: assets/asset_manifest.csv",
            ]
        ),
        encoding="utf-8",
    )

    result = subprocess.run(
        [
            sys.executable,
            "scripts/check_backend_config.py",
            "--config",
            str(config),
        ],
        text=True,
        capture_output=True,
    )

    payload = json.loads(result.stdout)
    assert result.returncode == 1
    assert payload["ok"] is False
    assert any("probe_only" in error for error in payload["errors"])

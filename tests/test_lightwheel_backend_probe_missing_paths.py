import json
import subprocess
import sys


def test_probe_lightwheel_backend_enabled_missing_paths_reports_errors(tmp_path):
    config = tmp_path / "lightwheel_enabled.yaml"
    config.write_text(
        "\n".join(
            [
                "enabled: true",
                "backend_enabled: true",
                "lightwheel_repo_path: /definitely/missing/lightwheel_repo",
                "lightwheel_python_package: lightwheel_missing_pkg_for_probe",
                "lightwheel_asset_root: /definitely/missing/lightwheel_assets",
                "use_lightwheel_assets: false",
                "require_assets: true",
                "allow_noncommercial_assets: true",
                "allow_runtime_import: false",
                "probe_only: true",
                "planned_tasks: [PressButton]",
                "task_import_mode: manifest_only",
                "runtime_status: probe_only_not_connected",
                "asset_manifest: assets/asset_manifest.csv",
            ]
        ),
        encoding="utf-8",
    )
    output = tmp_path / "status.json"

    result = subprocess.run(
        [
            sys.executable,
            "scripts/probe_lightwheel_backend.py",
            "--config",
            str(config),
            "--output",
            str(output),
        ],
        text=True,
        capture_output=True,
    )

    payload = json.loads(result.stdout)
    assert result.returncode == 1
    assert payload["ok"] is False
    assert payload["backend_enabled"] is True
    assert payload["repo_path_exists"] is False
    assert payload["asset_root_exists"] is False
    assert payload["python_import_available"] is False
    assert payload["runtime_import_allowed"] is False
    assert any("repo" in error.lower() for error in payload["errors"])
    assert any("asset" in error.lower() for error in payload["errors"])

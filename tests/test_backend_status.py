import json


def test_backend_status_is_json_serializable_and_has_probe_fields():
    from isaac_tactile_libero.envs.backend_status import BackendStatus

    status = BackendStatus(
        backend_name="lightwheel",
        backend_enabled=False,
        runtime_status="planned_or_disabled",
        repo_path_exists=False,
        asset_root_exists=False,
        python_import_available=False,
        runtime_import_allowed=False,
        runtime_connected=False,
        reset_step_available=False,
        errors=["repo path not configured"],
    )

    payload = status.as_dict()
    encoded = json.dumps(payload, sort_keys=True)

    assert "planned_or_disabled" in encoded
    assert payload["ok"] is False
    assert payload["backend_name"] == "lightwheel"
    assert payload["optional_backend"] is True
    assert payload["downloads_assets"] is False

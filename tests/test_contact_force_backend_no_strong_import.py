import subprocess
import sys


def test_safe_find_spec_does_not_execute_parent_package(tmp_path, monkeypatch):
    package = tmp_path / "sentinel_parent"
    package.mkdir()
    (package / "__init__.py").write_text(
        "raise RuntimeError('parent package executed')\n",
        encoding="utf-8",
    )
    (package / "child.py").write_text("VALUE = 1\n", encoding="utf-8")
    monkeypatch.syspath_prepend(str(tmp_path))
    sys.modules.pop("sentinel_parent", None)
    sys.modules.pop("sentinel_parent.child", None)

    from isaac_tactile_libero.envs.isaacsim_contact_force import safe_find_spec

    result = safe_find_spec("sentinel_parent.child")

    assert result["available"] is True
    assert result["error"] is None
    assert "sentinel_parent" not in sys.modules
    assert "sentinel_parent.child" not in sys.modules


def test_contact_force_backend_import_does_not_import_isaacsim_or_omni():
    code = """
import sys
before = set(sys.modules)
import isaac_tactile_libero.envs.isaacsim_contact_force as contact_force
after = set(sys.modules)
loaded = sorted(name for name in after - before if name == 'isaacsim' or name == 'omni' or name.startswith('omni.') or name == 'carb' or name.startswith('carb.'))
assert loaded == [], loaded
report = contact_force.ContactForceReport.unavailable(method='auto', error='dry-run')
assert report.as_dict()['contact_force_available'] is False
"""
    subprocess.run([sys.executable, "-c", code], check=True, text=True, capture_output=True)

from __future__ import annotations

from pathlib import Path

import yaml

from isaac_tactile_libero.assets.resolver import resolve_external_asset
from isaac_tactile_libero.robots.fr3_articulation_spec import load_fr3_articulation_config


FR3_KEY = "Robots/FrankaRobotics/FrankaFR3/fr3.usd"


def test_asset_key_resolves_from_environment_override(tmp_path: Path, monkeypatch) -> None:
    asset = tmp_path / FR3_KEY
    asset.parent.mkdir(parents=True)
    asset.write_text("#usda 1.0\n", encoding="utf-8")
    monkeypatch.setenv("ISAAC_TACTILE_ASSET_ROOT", str(tmp_path))

    result = resolve_external_asset(FR3_KEY)

    assert result.ok is True
    assert result.path == asset
    assert result.source == "ISAAC_TACTILE_ASSET_ROOT"


def test_missing_asset_reports_every_attempt(tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setenv("ISAAC_TACTILE_ASSET_ROOT", str(tmp_path))

    result = resolve_external_asset("missing.usd", include_defaults=False)

    assert result.ok is False
    assert result.path is None
    assert result.attempted_paths == (tmp_path / "missing.usd",)
    assert "ISAAC_TACTILE_ASSET_ROOT" in result.diagnostic


def test_fr3_config_is_portable_and_uses_exact_isaac6_dof_names(
    tmp_path: Path,
    monkeypatch,
) -> None:
    asset = tmp_path / FR3_KEY
    asset.parent.mkdir(parents=True)
    asset.write_text("#usda 1.0\n", encoding="utf-8")
    monkeypatch.setenv("ISAAC_TACTILE_ASSET_ROOT", str(tmp_path))
    config_path = Path("configs/robots/fr3_real_articulation.yaml")
    raw = config_path.read_text(encoding="utf-8")

    spec = load_fr3_articulation_config(config_path)

    assert "/mnt/data/home/" not in raw
    assert spec.assets.fr3_usd_path == str(asset)
    assert spec.joints.joint_names[-2:] == (
        "fr3_finger_joint1",
        "fr3_finger_joint2",
    )
    data = yaml.safe_load(raw)
    assert data["fr3_usd_key"] == FR3_KEY

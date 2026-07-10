from pathlib import Path
import re

from isaac_tactile_libero.assets.resolver import resolve_external_asset


ABSOLUTE_DEVELOPER_PATH = re.compile(r"(?:/home/|/mnt/|[A-Za-z]:\\\\)")


def test_required_configs_do_not_embed_developer_absolute_paths() -> None:
    offenders = []
    for path in sorted(Path("configs").rglob("*")):
        if path.suffix not in {".yaml", ".yml", ".json"}:
            continue
        if ABSOLUTE_DEVELOPER_PATH.search(path.read_text(encoding="utf-8")):
            offenders.append(str(path))
    assert offenders == []


def test_fr3_asset_resolves_through_documented_key_not_configured_absolute_path() -> None:
    resolution = resolve_external_asset("Robots/FrankaRobotics/FrankaFR3/fr3.usd")
    assert resolution.key == "Robots/FrankaRobotics/FrankaFR3/fr3.usd"
    assert resolution.path is None or resolution.path.is_absolute()
    assert resolution.source in {"environment", "explicit", "default_search", "unresolved"}

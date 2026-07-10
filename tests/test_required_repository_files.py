from pathlib import Path

import yaml


def test_required_repository_inventory_covers_migration_runtime() -> None:
    path = Path("configs/repository/required_files.yaml")
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    patterns = set(data["required_patterns"])
    assert "isaac_tactile_libero/**/*.py" in patterns
    assert "isaac_tactile_libero/schemas/*.schema.json" in patterns
    assert "configs/**/*.yaml" in patterns
    assert "requirements/**/*.txt" in patterns
    assert "specs/001-benchmark-reconstruction/**/*.md" in patterns


def test_every_declared_required_pattern_matches_a_file() -> None:
    data = yaml.safe_load(
        Path("configs/repository/required_files.yaml").read_text(encoding="utf-8")
    )
    missing = [
        pattern
        for pattern in data["required_patterns"]
        if not any(path.is_file() for path in Path(".").glob(pattern))
    ]
    assert missing == []

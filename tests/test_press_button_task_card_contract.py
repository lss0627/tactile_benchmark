from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
PHYSICAL_CONFIG = ROOT / "configs/tasks/press_button_physical.yaml"
TASK_CARD = ROOT / "configs/tasks/cards/press_button.v1.yaml"


def test_task_card_mechanism_version_matches_physical_config() -> None:
    physical = yaml.safe_load(PHYSICAL_CONFIG.read_text(encoding="utf-8"))
    card = yaml.safe_load(TASK_CARD.read_text(encoding="utf-8"))
    physical_version = physical["mechanism"]["mechanism_version"]
    card_version = card["scene"]["mechanism_version"]

    assert physical_version == card_version == "1.1.0"

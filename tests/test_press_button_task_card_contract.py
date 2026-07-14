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


def test_mechanism_migration_preserves_task_and_runtime_semantics() -> None:
    physical = yaml.safe_load(PHYSICAL_CONFIG.read_text(encoding="utf-8"))
    card = yaml.safe_load(TASK_CARD.read_text(encoding="utf-8"))

    assert physical["task_version"] == "1.0.2"
    assert card["task_version"] == "1.0.1"
    assert physical["motion"]["max_translation_per_step_m"] == 0.0005
    assert physical["mechanism"]["joint_axis"] == [0.0, 0.0, -1.0]
    assert physical["mechanism"]["travel_limit_m"] == 0.012
    assert physical["mechanism"]["pressed_threshold_m"] == 0.009
    assert physical["mechanism"]["release_threshold_m"] == 0.001
    assert physical["mechanism"]["reset_tolerance_m"] == 0.0005
    assert card["robot"]["action_semantics"] == [
        "dx",
        "dy",
        "dz",
        "dRx",
        "dRy",
        "dRz",
        "gripper",
    ]
    assert card["task_truth"]["state_source"] == "observed_button_joint_travel"

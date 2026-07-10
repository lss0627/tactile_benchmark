import yaml


def test_fr3_press_button_planned_task_config_is_non_benchmark():
    with open("configs/tasks/press_button_fr3_planned.yaml", "r", encoding="utf-8") as stream:
        cfg = yaml.safe_load(stream)

    assert cfg["task"] == "PressButton"
    assert cfg["robot_mode"] == "real_fr3_articulation_planned"
    assert cfg["controller_connected"] is False
    assert cfg["scripted_policy_planned"] is True
    assert cfg["benchmark_result"] is False
    assert cfg["not_for_paper_claims"] is True

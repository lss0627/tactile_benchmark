import json
import subprocess
import sys


def test_ee_placeholder_runtime_dataset_eval_schema_marks_robot_contract(tmp_path):
    dataset_path = tmp_path / "press_button_ee_placeholder_10ep_runtime_smoke.hdf5"
    output_dir = tmp_path / "eval"
    subprocess.run(
        [
            sys.executable,
            "scripts/collect_press_button_runtime_demos.py",
            "--dry-run",
            "--runtime-config",
            "configs/backend/isaacsim_visual_smoke.yaml",
            "--output",
            str(dataset_path),
            "--num-episodes",
            "10",
            "--seeds",
            *[str(seed) for seed in range(10)],
            "--policy",
            "scripted",
            "--tactile",
            "force_wrench",
            "--robot-mode",
            "ee_placeholder",
            "--robot-config",
            "configs/robots/fr3_ee_placeholder.yaml",
            "--max-steps",
            "4",
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    subprocess.run(
        [
            sys.executable,
            "scripts/evaluate_runtime_dataset.py",
            "--dataset",
            str(dataset_path),
            "--policy",
            "replay",
            "--max-episodes",
            "10",
            "--output",
            str(output_dir),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    metrics = json.loads((output_dir / "metrics.json").read_text(encoding="utf-8"))
    report = json.loads((output_dir / "dataset_eval_report.json").read_text(encoding="utf-8"))
    assert metrics["dataset_kind"] == "runtime_smoke"
    assert metrics["backend"] == "isaacsim_press_button"
    assert metrics["task_name"] == "PressButton"
    assert metrics["num_episodes"] == 10
    assert metrics["robot_mode"] == "ee_placeholder"
    assert metrics["placeholder_robot"] is True
    assert metrics["real_fr3_articulation"] is False
    assert metrics["force_source"] == "unavailable"
    assert metrics["contact_force_available"] is False
    assert metrics["no_fake_force_from_displacement"] is True
    assert metrics["benchmark_result"] is False
    assert metrics["not_for_paper_claims"] is True
    assert report["ok"] is True
    assert report["robot_mode"] == "ee_placeholder"
    assert report["placeholder_robot"] is True
    assert report["real_fr3_articulation"] is False
    assert len(report["episodes"]) == 10
    assert all(ep["robot_mode"] == "ee_placeholder" for ep in report["episodes"])
    assert all(ep["placeholder_robot"] is True for ep in report["episodes"])
    assert all(ep["real_fr3_articulation"] is False for ep in report["episodes"])
    assert all(ep["mask"]["has_force"] is False for ep in report["episodes"])
    assert all(ep["no_fake_force_from_displacement"] is True for ep in report["episodes"])

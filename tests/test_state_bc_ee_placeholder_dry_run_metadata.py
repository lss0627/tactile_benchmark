import json
import subprocess
import sys


def test_state_bc_ee_placeholder_dry_run_summary_records_robot_contract(tmp_path):
    dataset_path = tmp_path / "press_button_ee_placeholder_10ep_runtime_smoke.hdf5"
    output_dir = tmp_path / "state_bc_dry_run"
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
            "scripts/train_bc.py",
            "--config",
            "configs/train/bc_mock.yaml",
            "--dataset",
            str(dataset_path),
            "--policy",
            "state_bc",
            "--output",
            str(output_dir),
            "--dry-run",
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    summary = json.loads((output_dir / "train_summary.json").read_text(encoding="utf-8"))
    checkpoint = json.loads((output_dir / "checkpoint_mock.json").read_text(encoding="utf-8"))
    assert summary["dataset_kind"] == "runtime_smoke"
    assert summary["dataset_episodes"] == 10
    assert summary["robot_mode"] == "ee_placeholder"
    assert summary["placeholder_robot"] is True
    assert summary["real_fr3_articulation"] is False
    assert summary["runtime_smoke"] is True
    assert summary["dry_run"] is True
    assert summary["is_trained"] is False
    assert summary["mock_or_stub"] is True
    assert summary["benchmark_result"] is False
    assert summary["not_for_paper_claims"] is True
    assert checkpoint["dataset_kind"] == "runtime_smoke"
    assert checkpoint["robot_mode"] == "ee_placeholder"
    assert checkpoint["placeholder_robot"] is True
    assert checkpoint["real_fr3_articulation"] is False
    assert checkpoint["dry_run"] is True
    assert checkpoint["is_trained"] is False
    assert checkpoint["mock_or_stub"] is True

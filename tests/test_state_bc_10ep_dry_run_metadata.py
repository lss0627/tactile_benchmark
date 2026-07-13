import json
import subprocess
import sys


def test_state_bc_10ep_runtime_smoke_dry_run_stays_marked_insufficient(tmp_path):
    dataset_path = tmp_path / "runtime_smoke_10ep.hdf5"
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
            "--max-steps",
            "20",
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
    assert summary["num_episodes"] == 10
    assert summary["insufficient_real_episodes"] is True
    assert summary["benchmark_result"] is False
    assert summary["not_for_paper_claims"] is True
    assert checkpoint["dataset_kind"] == "runtime_smoke"
    assert checkpoint["num_episodes"] == 10
    assert checkpoint["insufficient_real_episodes"] is True
    assert checkpoint["benchmark_result"] is False
    assert checkpoint["not_for_paper_claims"] is True

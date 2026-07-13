import subprocess
import sys

import pytest


def test_make_env_rejects_non_press_button_for_isaacsim_press_button_backend():
    from isaac_tactile_libero.envs.make import make_env

    with pytest.raises(ValueError, match="only supports PressButton"):
        make_env(task="PegInsert", backend="isaacsim_press_button")


def test_evaluate_rejects_non_press_button_runtime_backend(tmp_path):
    result = subprocess.run(
        [
            sys.executable,
            "scripts/evaluate.py",
            "--task",
            "PegInsert",
            "--backend",
            "isaacsim_press_button",
            "--policy",
            "scripted",
            "--dry-run-runtime",
            "--output",
            str(tmp_path / "eval"),
        ],
        text=True,
        capture_output=True,
    )

    assert result.returncode != 0
    assert "only supports PressButton" in result.stderr

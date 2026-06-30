import json
import subprocess
import sys

import numpy as np
import pytest


TASKS = ["PressButton", "SoftPress", "PushSlider", "PegInsert", "PlugSocketInsert"]
TACTILE_MODES = ["none", "force_wrench", "visuotactile", "force_plus_visuotactile"]


@pytest.mark.parametrize("task", TASKS)
@pytest.mark.parametrize("tactile", TACTILE_MODES)
def test_make_env_runs_one_mock_episode_for_every_phase_1_combo(task, tactile):
    from isaac_tactile_libero.envs.make import make_env
    from isaac_tactile_libero.schemas.observation import assert_observation_schema

    env = make_env(task=task, tactile=tactile, seed=0, split="test_seen")
    obs = env.reset()
    assert_observation_schema(obs)

    terminated = truncated = False
    info = {}
    for _ in range(5):
        obs, reward, terminated, truncated, info = env.step(np.zeros(7, dtype=np.float32))
        assert_observation_schema(obs)
        assert isinstance(reward, float)
        if terminated or truncated:
            break

    env.close()

    assert terminated or truncated
    assert info["task_name"] == task
    assert info["tactile_mode"] == tactile
    assert info["split"] == "test_seen"
    assert "metrics" in info


def test_list_tasks_script_outputs_registered_tasks():
    result = subprocess.run(
        [sys.executable, "scripts/list_tasks.py"],
        check=True,
        text=True,
        capture_output=True,
    )

    for task in TASKS:
        assert task in result.stdout


def test_smoke_test_script_runs_requested_combo_and_prints_json_summary():
    result = subprocess.run(
        [
            sys.executable,
            "scripts/smoke_test.py",
            "--task",
            "PegInsert",
            "--tactile",
            "force_wrench",
            "--seeds",
            "0",
            "--episodes",
            "1",
        ],
        check=True,
        text=True,
        capture_output=True,
    )
    summary = json.loads(result.stdout)

    assert summary["ok"] is True
    assert summary["num_runs"] == 1
    assert summary["runs"][0]["task_name"] == "PegInsert"
    assert summary["runs"][0]["tactile_mode"] == "force_wrench"


def test_smoke_test_default_is_ci_gate_for_three_seeds():
    result = subprocess.run(
        [sys.executable, "scripts/smoke_test.py"],
        check=True,
        text=True,
        capture_output=True,
    )
    summary = json.loads(result.stdout)

    assert summary["ok"] is True
    assert summary["num_runs"] == 60
    assert {run["seed"] for run in summary["runs"]} == {0, 1, 2}

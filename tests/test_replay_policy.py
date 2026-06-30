import subprocess
import sys

import numpy as np
import pytest


def _make_dataset(path):
    subprocess.run(
        [
            sys.executable,
            "scripts/collect_mock_demos.py",
            "--config",
            "configs/dataset/mock_dataset.yaml",
            "--tasks",
            "PegInsert",
            "--tactile",
            "force_wrench",
            "--seeds",
            "0",
            "--episodes-per-task",
            "1",
            "--output",
            str(path),
        ],
        check=True,
        text=True,
        capture_output=True,
    )


def test_replay_policy_reads_hdf5_actions_by_episode_id(tmp_path):
    from isaac_tactile_libero.policies.replay import ReplayPolicy

    dataset_path = tmp_path / "mock.hdf5"
    _make_dataset(dataset_path)
    episode_id = "mock-PegInsert-force_wrench-seed0-ep0"

    policy = ReplayPolicy(cfg={"dataset": str(dataset_path)})
    policy.reset(task_name="PegInsert", tactile_mode="force_wrench", seed=0, episode_id=episode_id)

    assert policy.episode_id == episode_id
    assert policy.num_steps == 3
    first = policy.act({})
    assert first.shape == (7,)
    assert first.dtype == np.float32
    policy.act({})
    policy.act({})
    with pytest.raises(StopIteration, match="exhausted"):
        policy.act({})


def test_replay_policy_can_select_episode_by_task_tactile_seed(tmp_path):
    from isaac_tactile_libero.policies.replay import ReplayPolicy

    dataset_path = tmp_path / "mock.hdf5"
    _make_dataset(dataset_path)
    policy = ReplayPolicy(cfg={"dataset": str(dataset_path)})
    policy.reset(task_name="PegInsert", tactile_mode="force_wrench", seed=0)

    assert policy.episode_id == "mock-PegInsert-force_wrench-seed0-ep0"
    assert policy.current_step == 0

import json
import subprocess
import sys

import numpy as np
import pytest

pytest.importorskip("torch")


def _train_state_bc(tmp_path):
    dataset_path = tmp_path / "mock_state.hdf5"
    output_dir = tmp_path / "train_state"
    subprocess.run(
        [
            sys.executable,
            "scripts/collect_mock_demos.py",
            "--config",
            "configs/dataset/mock_dataset.yaml",
            "--tasks",
            "PegInsert",
            "--tactile",
            "none",
            "--seeds",
            "0",
            "--episodes-per-task",
            "1",
            "--output",
            str(dataset_path),
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
            "configs/train/state_bc_minimal.yaml",
            "--dataset",
            str(dataset_path),
            "--policy",
            "state_bc",
            "--output",
            str(output_dir),
        ],
        check=True,
        text=True,
        capture_output=True,
    )
    return dataset_path, output_dir


def test_state_bc_loads_real_checkpoint_and_returns_valid_action(tmp_path):
    import isaac_tactile_libero  # noqa: F401
    from isaac_tactile_libero.envs.make import make_env
    from isaac_tactile_libero.registry.policy_registry import POLICY_REGISTRY

    _, output_dir = _train_state_bc(tmp_path)
    env = make_env(task="PegInsert", tactile="none", seed=0, split="test_seen")
    obs = env.reset()
    policy = POLICY_REGISTRY.make("state_bc", cfg={"device": "cpu"})
    policy.load(str(output_dir / "checkpoint.json"))

    action = policy.act(obs)
    env.close()

    assert action.shape == (7,)
    assert action.dtype == np.float32
    assert policy.is_trained is True
    assert policy.mock_or_stub is False
    assert policy.last_action_metadata["is_trained"] is True
    assert policy.last_action_metadata["dataset_is_mock"] is True
    assert policy.last_action_metadata["not_for_paper_claims"] is True
    assert policy.last_action_metadata["checkpoint_is_trained"] is True
    assert policy.last_action_metadata["loaded_checkpoint_mock_or_stub"] is False


def test_inspect_checkpoint_distinguishes_trained_on_mock_dataset(tmp_path):
    _, output_dir = _train_state_bc(tmp_path)

    result = subprocess.run(
        [
            sys.executable,
            "scripts/inspect_checkpoint.py",
            "--checkpoint",
            str(output_dir / "checkpoint.json"),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    payload = json.loads(result.stdout)
    assert payload["ok"] is True
    assert payload["checkpoint_kind"] == "trained_on_mock_dataset"
    assert payload["is_trained"] is True
    assert payload["mock_or_stub"] is False
    assert payload["dataset_is_mock"] is True
    assert payload["not_for_paper_claims"] is True

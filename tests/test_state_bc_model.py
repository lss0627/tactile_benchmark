import pytest

torch = pytest.importorskip("torch")


def test_state_bc_mlp_maps_robot_state_features_to_7d_action():
    from isaac_tactile_libero.training.models import StateBCMLP

    model = StateBCMLP(input_dim=26, hidden_dim=32, action_dim=7)
    batch = torch.zeros((4, 26), dtype=torch.float32)

    action = model(batch)

    assert tuple(action.shape) == (4, 7)
    assert action.dtype == torch.float32

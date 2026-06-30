"""Minimal PyTorch models for the real-training slice.

PyTorch is an optional dependency. Import this module only when the `train`
extra is installed.
"""

from __future__ import annotations

try:
    import torch
    from torch import nn
except ModuleNotFoundError as exc:  # pragma: no cover - exercised when train extra is absent.
    raise ModuleNotFoundError(
        "StateBCMLP requires PyTorch. Install the optional training extra with "
        "`python -m pip install -e '.[train]'`."
    ) from exc


class StateBCMLP(nn.Module):
    """Small state-only behavior cloning MLP.

    This is intentionally narrow: robot state features in, 7D action out. It is
    not a CNN, Transformer, ACT, Diffusion Policy, VLA, or tactile model.
    """

    def __init__(self, *, input_dim: int, hidden_dim: int = 64, action_dim: int = 7):
        super().__init__()
        self.input_dim = int(input_dim)
        self.hidden_dim = int(hidden_dim)
        self.action_dim = int(action_dim)
        self.net = nn.Sequential(
            nn.Linear(self.input_dim, self.hidden_dim),
            nn.ReLU(),
            nn.Linear(self.hidden_dim, self.hidden_dim),
            nn.ReLU(),
            nn.Linear(self.hidden_dim, self.action_dim),
        )

    def forward(self, obs_features: "torch.Tensor") -> "torch.Tensor":
        return self.net(obs_features.float())

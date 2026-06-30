"""Untrained mock/stub BC baseline policy skeletons."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

from isaac_tactile_libero.registry.policy_registry import POLICY_REGISTRY
from isaac_tactile_libero.schemas.action import DEFAULT_ACTION_SCHEMA, validate_action
from isaac_tactile_libero.version import BENCHMARK_VERSION

from .base import BasePolicy
from .baseline_specs import BASELINE_SPECS, BaselinePolicySpec
from .batch_builder import extract_mock_state_features
from .observation_filter import filter_observation


class BaselineBCPolicy(BasePolicy):
    """Base class for untrained BC skeletons.

    The current implementation intentionally does not train or load a model. It
    only validates the 7D action contract and records mock/stub metadata.
    """

    spec: BaselinePolicySpec

    def __init__(self, cfg: dict[str, Any] | None = None):
        super().__init__(cfg=cfg)
        self.policy_name = self.spec.policy_name
        self.name = self.policy_name
        self.required_observation_keys = self.spec.required_observation_keys
        self.allowed_modalities = self.spec.allowed_modalities
        self.forbidden_modalities = self.spec.forbidden_modalities
        self.uses_oracle_state = self.spec.uses_oracle_state
        self.uses_tactile_force = self.spec.uses_tactile_force
        self.uses_visuotactile = self.spec.uses_visuotactile
        self.action_schema_version = self.spec.action_schema_version
        self.is_trainable = self.spec.is_trainable
        self.is_trained = self.spec.is_trained
        self.mock_or_stub = self.spec.mock_or_stub
        self.upper_bound_mock = self.spec.upper_bound_mock
        self.last_filtered_observation: dict[str, Any] | None = None
        self.last_action_metadata = self._metadata(filtered_leakage_free=True)
        self.model = None
        self.model_config: dict[str, Any] | None = None
        self.device = self.cfg.get("device", "cpu")

    def act(self, obs: dict[str, Any]) -> np.ndarray:
        self.last_filtered_observation = filter_observation(obs, self.spec)
        if self.is_trained and self.model is not None and self.policy_name == "state_bc":
            action = self._act_with_state_bc_model(self.last_filtered_observation)
        else:
            action = validate_action(np.zeros(DEFAULT_ACTION_SCHEMA.dim, dtype=np.float32))
        self.last_action_metadata = self._metadata(
            filtered_leakage_free=bool(self.last_filtered_observation["metadata"]["leakage_free"])
        )
        return action

    def load(self, checkpoint: str) -> "BaselineBCPolicy":
        super().load(checkpoint)
        metadata = getattr(self, "checkpoint_metadata", {}) or {}
        if metadata.get("is_trained"):
            if self.policy_name != "state_bc":
                raise NotImplementedError("Only StateBC can load real trained checkpoints in this minimal slice")
            self._load_state_bc_checkpoint(checkpoint, metadata)
        return self

    def _load_state_bc_checkpoint(self, checkpoint: str, metadata: dict[str, Any]) -> None:
        try:
            import torch
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "Loading a trained StateBC checkpoint requires PyTorch. Install the optional train extra with "
                "`python -m pip install -e '.[train]'`."
            ) from exc

        from isaac_tactile_libero.training.models import StateBCMLP

        checkpoint_path = Path(checkpoint)
        weights_path = checkpoint_path.parent / str(metadata.get("weights_path", "checkpoint.pt"))
        payload = torch.load(weights_path, map_location=self.device)
        model_config = dict(metadata.get("model_config") or payload.get("model_config") or {})
        model = StateBCMLP(
            input_dim=int(model_config["input_dim"]),
            hidden_dim=int(model_config.get("hidden_dim", 64)),
            action_dim=int(model_config.get("action_dim", DEFAULT_ACTION_SCHEMA.dim)),
        )
        model.load_state_dict(payload["model_state_dict"])
        model.eval()
        self.model = model
        self.model_config = model_config
        self.is_trained = True
        self.mock_or_stub = bool(metadata.get("mock_or_stub", False))

    def _act_with_state_bc_model(self, filtered_observation: dict[str, Any]) -> np.ndarray:
        import torch

        assert self.model is not None
        features = extract_mock_state_features(filtered_observation)
        with torch.no_grad():
            tensor = torch.as_tensor(features.reshape(1, -1), dtype=torch.float32)
            action = self.model(tensor).cpu().numpy()[0]
        return validate_action(action)

    def _metadata(self, *, filtered_leakage_free: bool) -> dict[str, Any]:
        checkpoint_metadata = getattr(self, "checkpoint_metadata", {}) or {}
        metadata = {
            "policy_name": self.policy_name,
            "policy_type": self.spec.policy_type,
            "required_observation_keys": list(self.required_observation_keys),
            "allowed_modalities": list(self.allowed_modalities),
            "forbidden_modalities": list(self.forbidden_modalities),
            "uses_oracle_state": bool(self.uses_oracle_state),
            "uses_tactile_force": bool(self.uses_tactile_force),
            "uses_visuotactile": bool(self.uses_visuotactile),
            "action_schema_version": self.action_schema_version,
            "is_trainable": bool(self.is_trainable),
            "is_trained": bool(self.is_trained),
            "mock_or_stub": bool(self.mock_or_stub),
            "untrained_mock_policy": not bool(self.is_trained),
            "upper_bound_mock": bool(self.upper_bound_mock),
            "filtered_observation_leakage_free": bool(filtered_leakage_free),
        }
        if checkpoint_metadata:
            metadata.update(
                {
                    "checkpoint_path": getattr(self, "checkpoint", None),
                    "checkpoint_is_trained": bool(checkpoint_metadata.get("is_trained", False)),
                    "loaded_checkpoint_mock_or_stub": bool(checkpoint_metadata.get("mock_or_stub", True)),
                    "checkpoint_dry_run": bool(checkpoint_metadata.get("dry_run", False)),
                    "runtime_env": checkpoint_metadata.get("runtime_env"),
                    "dataset_is_mock": bool(checkpoint_metadata.get("dataset_is_mock", False)),
                    "not_for_paper_claims": bool(checkpoint_metadata.get("not_for_paper_claims", False)),
                }
            )
        return metadata


class StateBC(BaselineBCPolicy):
    spec = BASELINE_SPECS["state_bc"]


class VisionBC(BaselineBCPolicy):
    spec = BASELINE_SPECS["vision_bc"]


class VisionStateBC(BaselineBCPolicy):
    spec = BASELINE_SPECS["vision_state_bc"]


class VisionForceBC(BaselineBCPolicy):
    spec = BASELINE_SPECS["vision_force_bc"]


class VisionVisuoTactileBC(BaselineBCPolicy):
    spec = BASELINE_SPECS["vision_vt_bc"]


class VisionForceVisuoTactileBC(BaselineBCPolicy):
    spec = BASELINE_SPECS["vision_force_vt_bc"]


class OracleStateBC(BaselineBCPolicy):
    spec = BASELINE_SPECS["oracle_state_bc"]


_BASELINE_CLASSES = {
    "state_bc": StateBC,
    "vision_bc": VisionBC,
    "vision_state_bc": VisionStateBC,
    "vision_force_bc": VisionForceBC,
    "vision_vt_bc": VisionVisuoTactileBC,
    "vision_force_vt_bc": VisionForceVisuoTactileBC,
    "oracle_state_bc": OracleStateBC,
}

for _name, _cls in _BASELINE_CLASSES.items():
    _spec = BASELINE_SPECS[_name]
    POLICY_REGISTRY.register(
        _name,
        _cls,
        version=BENCHMARK_VERSION,
        kind="untrained mock/stub BC skeleton",
        is_trainable=True,
        is_trained=False,
        mock_or_stub=True,
        allowed_modalities=_spec.allowed_modalities,
        required_observation_keys=_spec.required_observation_keys,
        uses_oracle_state=_spec.uses_oracle_state,
        upper_bound_mock=_spec.upper_bound_mock,
    )

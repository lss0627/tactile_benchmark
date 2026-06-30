"""BC trainer protocol for dry-run and the minimal StateBC training slice."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import numpy as np

from isaac_tactile_libero.datasets.reader import HDF5DatasetReader
from isaac_tactile_libero.policies.baseline_specs import BASELINE_SPECS, get_baseline_spec
from isaac_tactile_libero.policies.batch_builder import build_mock_baseline_batch, build_state_bc_training_batch
from isaac_tactile_libero.schemas.action import ACTION_DIM
from isaac_tactile_libero.version import SCHEMA_VERSION

from .checkpoint import build_mock_checkpoint, build_real_state_bc_checkpoint, write_checkpoint_metadata, write_mock_checkpoint
from .config import TrainingConfig
from .logger import JsonlTrainLogger


class BCTrainer:
    """Run dry-run training or the minimal real StateBC supervised trainer."""

    def __init__(self, cfg: TrainingConfig):
        self.cfg = cfg
        if cfg.policy_name not in BASELINE_SPECS:
            raise ValueError(f"Unsupported BC skeleton policy for dry-run training: {cfg.policy_name}")

    @property
    def output_dir(self) -> Path:
        return Path(self.cfg.output_dir)

    def run(self) -> dict[str, Any]:
        if self.cfg.dry_run:
            return self._run_dry_run()
        if self.cfg.policy_name != "state_bc":
            raise NotImplementedError("Real BC training is currently implemented only for policy=state_bc")
        return self._run_state_bc_real_training()

    def _run_dry_run(self) -> dict[str, Any]:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        spec = get_baseline_spec(self.cfg.policy_name)
        with HDF5DatasetReader(self.cfg.dataset_path) as reader:
            dataset_schema_version = reader.schema_version
            dataset_info = reader.dataset_info
            batch = build_mock_baseline_batch(reader, spec)

        checks = dict(batch["checks"])
        checks["allowed_modalities_ok"] = list(batch["allowed_modalities"]) == list(spec.allowed_modalities)
        if not all(checks.values()):
            raise ValueError(f"Dry-run batch checks failed: {checks}")

        steps_per_epoch = max(1, math.ceil(max(batch["num_steps"], 1) / max(self.cfg.batch_size, 1)))
        total_steps = self.cfg.num_epochs * steps_per_epoch
        self._write_log(total_steps=total_steps, steps_per_epoch=steps_per_epoch)

        observation_filter_summary = {
            "policy_name": self.cfg.policy_name,
            "allowed_modalities": list(spec.allowed_modalities),
            "required_observation_keys": list(spec.required_observation_keys),
            "dataset_episodes": batch["num_episodes"],
            "dataset_steps": batch["num_steps"],
            "checks": checks,
            "mock_or_stub": True,
        }
        summary = {
            "status": "dry_run_complete",
            "total_steps": total_steps,
            "num_epochs": self.cfg.num_epochs,
            "dataset_episodes": batch["num_episodes"],
            "dataset_steps": batch["num_steps"],
            "policy_name": self.cfg.policy_name,
            "batch_size": self.cfg.batch_size,
            "dry_run": True,
            "is_trained": False,
            "mock_or_stub": True,
            "checkpoint_path": str(self.output_dir / "checkpoint_mock.json"),
            "log_path": str(self.output_dir / "train_log.jsonl"),
        }
        checkpoint = build_mock_checkpoint(
            policy_name=self.cfg.policy_name,
            dataset_path=self.cfg.dataset_path,
            dataset_schema_version=dataset_schema_version,
            dataset_info=dataset_info,
            observation_filter_summary=observation_filter_summary,
            training_config=self.cfg.to_dict(),
        )
        (self.output_dir / "train_summary.json").write_text(
            json.dumps(summary, indent=2, sort_keys=True),
            encoding="utf-8",
        )
        write_mock_checkpoint(self.output_dir / "checkpoint_mock.json", checkpoint)
        return summary

    def _run_state_bc_real_training(self) -> dict[str, Any]:
        try:
            import torch
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "Real StateBC training requires PyTorch. Install the optional train extra with "
                "`python -m pip install -e '.[train]'`."
            ) from exc

        from .models import StateBCMLP

        self.output_dir.mkdir(parents=True, exist_ok=True)
        torch.manual_seed(self.cfg.seed)
        rng = np.random.default_rng(self.cfg.seed)
        device = torch.device(self.cfg.device)

        with HDF5DatasetReader(self.cfg.dataset_path) as reader:
            dataset_schema_version = reader.schema_version
            dataset_info = reader.dataset_info
            batch = build_state_bc_training_batch(reader)

        checks = dict(batch["checks"])
        checks["allowed_modalities_ok"] = list(batch["allowed_modalities"]) == ["robot_state"]
        if not all(checks.values()):
            raise ValueError(f"StateBC training batch checks failed: {checks}")
        if batch["actions"].shape[1] != ACTION_DIM:
            raise ValueError(f"Expected action shape [N, {ACTION_DIM}], got {batch['actions'].shape}")
        if batch["num_steps"] == 0:
            raise ValueError("Cannot train StateBC on an empty dataset batch")

        features = torch.as_tensor(batch["obs_features"], dtype=torch.float32, device=device)
        actions = torch.as_tensor(batch["actions"], dtype=torch.float32, device=device)
        model_config = {
            "model_class": "StateBCMLP",
            "input_dim": int(features.shape[1]),
            "hidden_dim": 64,
            "action_dim": ACTION_DIM,
        }
        model = StateBCMLP(
            input_dim=model_config["input_dim"],
            hidden_dim=model_config["hidden_dim"],
            action_dim=model_config["action_dim"],
        ).to(device)
        optimizer = torch.optim.Adam(model.parameters(), lr=self.cfg.learning_rate)
        loss_fn = torch.nn.MSELoss()

        total_steps = 0
        final_loss = 0.0
        with JsonlTrainLogger(self.output_dir / "train_log.jsonl") as logger:
            for epoch in range(self.cfg.num_epochs):
                indices = rng.permutation(features.shape[0])
                for epoch_step, start in enumerate(range(0, len(indices), self.cfg.batch_size)):
                    batch_indices = torch.as_tensor(indices[start : start + self.cfg.batch_size], dtype=torch.long, device=device)
                    pred = model(features.index_select(0, batch_indices))
                    target = actions.index_select(0, batch_indices)
                    loss = loss_fn(pred, target)
                    optimizer.zero_grad()
                    loss.backward()
                    optimizer.step()
                    final_loss = float(loss.detach().cpu().item())
                    logger.write(
                        {
                            "epoch": epoch,
                            "step": total_steps,
                            "epoch_step": epoch_step,
                            "loss": final_loss,
                            "batch_size": int(target.shape[0]),
                            "policy_name": self.cfg.policy_name,
                            "dry_run": False,
                            "is_trained": True,
                            "mock_or_stub": False,
                            "runtime_env": "mock_dataset",
                            "dataset_is_mock": True,
                            "not_for_paper_claims": True,
                        }
                    )
                    total_steps += 1

        weights_payload = {
            "model_state_dict": model.state_dict(),
            "model_config": model_config,
            "policy_name": self.cfg.policy_name,
            "action_schema_version": SCHEMA_VERSION,
        }
        torch.save(weights_payload, self.output_dir / "checkpoint.pt")

        observation_filter_summary = {
            "policy_name": self.cfg.policy_name,
            "allowed_modalities": list(batch["allowed_modalities"]),
            "required_observation_keys": list(batch["required_observation_keys"]),
            "dataset_episodes": batch["num_episodes"],
            "dataset_steps": batch["num_steps"],
            "state_feature_dim": batch["state_feature_dim"],
            "feature_schema": list(batch["feature_schema"]),
            "feature_extractor": batch["feature_extractor"],
            "checks": checks,
            "mock_or_stub": True,
            "dataset_is_mock": True,
        }
        checkpoint = build_real_state_bc_checkpoint(
            policy_name=self.cfg.policy_name,
            dataset_path=self.cfg.dataset_path,
            dataset_schema_version=dataset_schema_version,
            dataset_info=dataset_info,
            observation_filter_summary=observation_filter_summary,
            training_config=self.cfg.to_dict(),
            model_config=model_config,
            final_loss=final_loss,
        )
        write_checkpoint_metadata(self.output_dir / "checkpoint.json", checkpoint)
        summary = {
            "status": "trained_on_mock_dataset",
            "total_steps": total_steps,
            "num_epochs": self.cfg.num_epochs,
            "dataset_episodes": batch["num_episodes"],
            "dataset_steps": batch["num_steps"],
            "policy_name": self.cfg.policy_name,
            "batch_size": self.cfg.batch_size,
            "learning_rate": self.cfg.learning_rate,
            "device": str(device),
            "dry_run": False,
            "is_trained": True,
            "mock_or_stub": False,
            "runtime_env": "mock_dataset",
            "dataset_is_mock": True,
            "not_for_paper_claims": True,
            "final_loss": final_loss,
            "checkpoint_path": str(self.output_dir / "checkpoint.json"),
            "weights_path": str(self.output_dir / "checkpoint.pt"),
            "log_path": str(self.output_dir / "train_log.jsonl"),
        }
        (self.output_dir / "train_summary.json").write_text(
            json.dumps(summary, indent=2, sort_keys=True),
            encoding="utf-8",
        )
        return summary

    def _write_log(self, *, total_steps: int, steps_per_epoch: int) -> None:
        with JsonlTrainLogger(self.output_dir / "train_log.jsonl") as logger:
            step = 0
            for epoch in range(self.cfg.num_epochs):
                for epoch_step in range(steps_per_epoch):
                    mock_loss = 1.0 / float(step + 1)
                    logger.write(
                        {
                            "epoch": epoch,
                            "step": step,
                            "epoch_step": epoch_step,
                            "total_steps": total_steps,
                            "mock_loss": mock_loss,
                            "batch_size": self.cfg.batch_size,
                            "policy_name": self.cfg.policy_name,
                            "dry_run": True,
                            "is_trained": False,
                            "mock_or_stub": True,
                        }
                    )
                    step += 1

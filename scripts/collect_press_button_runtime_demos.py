#!/usr/bin/env python
"""Collect PressButton runtime-smoke rollouts into the stable HDF5 schema.

Dry-run writes schema-compatible proxy episodes and never starts Isaac Sim.
Non-dry-run uses the optional single-task Isaac Sim PressButton runtime. Both
paths are runtime-smoke artifacts only, not benchmark datasets and not paper
results.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from isaac_tactile_libero.datasets.writer import HDF5DatasetWriter
from isaac_tactile_libero.envs.isaacsim_backend_status import (
    load_isaacsim_visual_smoke_config,
    probe_isaacsim_visual_smoke,
)
from isaac_tactile_libero.envs.isaacsim_press_button_env import (
    INSTRUCTION,
    TASK_NAME,
    IsaacSimPressButtonEnv,
    default_robot_runtime_fields,
    default_press_button_contact_metrics,
    random_press_button_action,
    scripted_press_button_action,
)
from isaac_tactile_libero.schemas.action import DEFAULT_ACTION_SCHEMA, clip_action
from isaac_tactile_libero.schemas.dataset import CONTACT_METRIC_KEYS, DatasetMetadata
from isaac_tactile_libero.schemas.observation import (
    assert_observation_schema,
    default_robot_state,
    make_mock_observation,
)
from isaac_tactile_libero.sensors.config import sensor_config_snapshot
from isaac_tactile_libero.sensors.runtime_tactile_adapter import (
    adapt_press_button_runtime_tactile,
    runtime_tactile_status_fields,
)

SUITE_NAME = "tactile_contact"
RUNTIME_BACKEND = "isaacsim_press_button"
SUPPORTED_POLICIES = ("scripted", "random", "zero")
SUPPORTED_TACTILE_MODES = ("none", "force_wrench")
SUPPORTED_ROBOT_MODES = ("pusher", "ee_placeholder")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime-config", default="configs/backend/isaacsim_visual_smoke.yaml")
    parser.add_argument("--output", default="outputs/press_button_runtime_dataset_smoke/press_button_runtime_smoke.hdf5")
    parser.add_argument("--num-episodes", type=int, default=3)
    parser.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2])
    parser.add_argument("--policy", choices=SUPPORTED_POLICIES, default="scripted")
    parser.add_argument("--tactile", choices=SUPPORTED_TACTILE_MODES, default="force_wrench")
    parser.add_argument("--max-steps", type=int, default=80)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--webrtc", action="store_true")
    parser.add_argument("--save-screenshots", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--robot-mode", choices=SUPPORTED_ROBOT_MODES, default="pusher")
    parser.add_argument("--robot-config", help="Optional robot placeholder YAML config.")
    return parser.parse_args()


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


def _select_action(
    *,
    policy_name: str,
    observation: dict[str, Any],
    step: int,
    max_steps: int,
    rng: np.random.Generator,
) -> np.ndarray:
    if policy_name == "scripted":
        return scripted_press_button_action(observation, step, max_steps)
    if policy_name == "random":
        return random_press_button_action(rng)
    return np.zeros(7, dtype=np.float32)


def _dataset_info(
    *,
    output_path: Path,
    args: argparse.Namespace,
    tactile_snapshot: dict[str, Any],
) -> dict[str, Any]:
    dataset_info = DatasetMetadata(
        dataset_name="Isaac-Tactile-LIBERO-PressButton-Runtime-Smoke",
        dataset_version="0.1.0",
        mock_stub=bool(args.dry_run),
    ).__dict__
    dataset_info.update(
        {
            "dataset_kind": "runtime_smoke",
            "runtime_smoke": True,
            "backend": RUNTIME_BACKEND,
            "task_name": TASK_NAME,
            "suite_name": SUITE_NAME,
            "num_episodes": int(args.num_episodes),
            "runtime_config_path": str(args.runtime_config),
            "tactile_mode": str(args.tactile),
            "force_source": "unavailable",
            "contact_force_available": False,
            "button_displacement_available": True,
            "benchmark_result": False,
            "not_for_paper_claims": True,
            "lightwheel_assets_used": False,
            "dry_run": bool(args.dry_run),
            "policy_name": str(args.policy),
            "robot_mode": str(args.robot_mode),
            "robot_config_path": str(args.robot_config) if args.robot_config else None,
            "placeholder_robot": True,
            "placeholder_pusher": args.robot_mode == "pusher",
            "real_fr3_articulation": False,
            "output_path": str(output_path),
            "tactile_config_snapshot": tactile_snapshot,
        }
    )
    return dataset_info


def _contact_metrics_from_runtime(metrics: dict[str, Any]) -> dict[str, float]:
    contact_duration = float(metrics.get("contact_step_count", 0)) / DEFAULT_ACTION_SCHEMA.control_frequency_hz
    values = {
        "max_contact_force": float(metrics.get("max_contact_force", metrics.get("max_contact_force_norm", 0.0))),
        "mean_contact_force": float(metrics.get("mean_contact_force", metrics.get("mean_contact_force_norm", 0.0))),
        "force_violation_rate": 0.0,
        "contact_duration": contact_duration,
        "contact_loss_count": 0.0,
        "jamming_count": 0.0,
        "insertion_depth": 0.0,
    }
    return {key: float(values.get(key, 0.0)) for key in CONTACT_METRIC_KEYS}


def _runtime_metadata(
    *,
    final_metrics: dict[str, Any],
    tactile_mode: str,
    policy_name: str,
    seed: int,
    num_steps: int,
    success: bool,
    dry_run: bool,
    runtime_config_path: str,
    robot_mode: str,
    robot_config_path: str | None,
    screenshot_path: str | None = None,
) -> dict[str, Any]:
    robot_fields = default_robot_runtime_fields(
        robot_mode=robot_mode,
        robot_config_path=robot_config_path,
        ee_pose=final_metrics.get("ee_pose"),
        gripper_command=float(final_metrics.get("gripper_command", 0.0)),
    )
    tactile_status = runtime_tactile_status_fields(
        adapt_press_button_runtime_tactile(
            {
                "contact_signal_seen": bool(final_metrics.get("contact_signal_seen", success)),
                "contact_proxy_triggered": bool(final_metrics.get("contact_proxy_triggered", success)),
                "contact_force_available": False,
                "physics_contact_available": False,
                "button_displacement_available": bool(final_metrics.get("button_displacement_available", True)),
                "button_displacement": float(final_metrics.get("button_displacement", 0.0)),
                "button_press_depth": float(final_metrics.get("button_press_depth", 0.0)),
                "max_button_press_depth": float(final_metrics.get("max_button_press_depth", 0.0)),
                "using_geometric_fallback": bool(final_metrics.get("using_geometric_fallback", False)),
            },
            tactile_mode=tactile_mode,
        )
    )
    success_source = str(final_metrics.get("success_source", "button_displacement" if success else "none"))
    return {
        "dataset_kind": "runtime_smoke",
        "runtime_smoke": True,
        "backend": RUNTIME_BACKEND,
        "task_name": TASK_NAME,
        "suite_name": SUITE_NAME,
        "policy_name": policy_name,
        "seed": int(seed),
        "tactile_mode": tactile_mode,
        "dry_run": bool(dry_run),
        "runtime_loop_executed": not dry_run,
        "runtime_config_path": str(runtime_config_path),
        "robot_config_path": robot_config_path,
        "num_steps": int(num_steps),
        "success": bool(success),
        "success_source": success_source,
        "force_source": "unavailable",
        "contact_force_available": False,
        "physics_contact_available": False,
        "button_displacement_available": True,
        "geometric_contact_proxy": True,
        "real_tactile_contact": False,
        "placeholder_pusher": True,
        "lightwheel_assets_used": False,
        "benchmark_result": False,
        "not_for_paper_claims": True,
        "screenshot_path": screenshot_path,
        **robot_fields,
        **tactile_status,
    }


def _episode_payload(
    *,
    episode_id: str,
    seed: int,
    tactile_mode: str,
    policy_name: str,
    dry_run: bool,
    runtime_config_path: str,
    robot_mode: str,
    robot_config_path: str | None,
    observations: list[dict[str, Any]],
    actions: list[np.ndarray],
    rewards: list[float],
    success_labels: list[bool],
    final_metrics: dict[str, Any],
    tactile_snapshot: dict[str, Any],
    screenshot_path: str | None = None,
) -> dict[str, Any]:
    success = bool(success_labels[-1]) if success_labels else False
    runtime_metadata = _runtime_metadata(
        final_metrics=final_metrics,
        tactile_mode=tactile_mode,
        policy_name=policy_name,
        seed=seed,
        num_steps=len(actions),
        success=success,
        dry_run=dry_run,
        runtime_config_path=runtime_config_path,
        robot_mode=robot_mode,
        robot_config_path=robot_config_path,
        screenshot_path=screenshot_path,
    )
    return {
        "episode_id": episode_id,
        "task_name": TASK_NAME,
        "suite_name": SUITE_NAME,
        "instruction": INSTRUCTION,
        "seed": int(seed),
        "split": "runtime_smoke",
        "tactile_mode": tactile_mode,
        "observations": observations,
        "actions": actions,
        "rewards": rewards,
        "success": success_labels,
        "contact_metrics": _contact_metrics_from_runtime(final_metrics),
        "metadata": {
            "mock_stub": bool(dry_run),
            "dataset_kind": "runtime_smoke",
            "runtime_smoke": True,
            "backend": RUNTIME_BACKEND,
            "policy_name": policy_name,
            "tactile_mode": tactile_mode,
            "runtime_config_path": str(runtime_config_path),
            "robot_config_path": robot_config_path,
            "robot_mode": runtime_metadata["robot_mode"],
            "robot_name": runtime_metadata["robot_name"],
            "ee_prim_path": runtime_metadata["ee_prim_path"],
            "pusher_prim_path": runtime_metadata["pusher_prim_path"],
            "placeholder_robot": runtime_metadata["placeholder_robot"],
            "placeholder_pusher": runtime_metadata["placeholder_pusher"],
            "real_fr3_articulation": runtime_metadata["real_fr3_articulation"],
            "real_fr3_control": runtime_metadata["real_fr3_control"],
            "ee_pose": runtime_metadata["ee_pose"],
            "gripper_command": runtime_metadata["gripper_command"],
            "action_schema_version": runtime_metadata["action_schema_version"],
            "tactile_config_snapshot": tactile_snapshot,
            "force_source": "unavailable",
            "contact_force_available": False,
            "button_displacement_available": True,
            "geometric_contact_proxy": True,
            "real_tactile_contact": False,
            "lightwheel_assets_used": False,
            "benchmark_result": False,
            "not_for_paper_claims": True,
            "success": success,
            "success_source": runtime_metadata["success_source"],
            "num_steps": len(actions),
            "runtime_metadata": runtime_metadata,
            "final_metrics": _jsonable(final_metrics),
        },
    }


def _dry_run_episode(
    *,
    episode_index: int,
    seed: int,
    tactile_mode: str,
    policy_name: str,
    max_steps: int,
    runtime_config_path: str,
    robot_mode: str,
    robot_config_path: str | None,
    tactile_snapshot: dict[str, Any],
) -> dict[str, Any]:
    rng = np.random.default_rng(seed)
    max_steps = max(1, int(max_steps))
    scripted_success_step = max(1, min(max_steps, int(round(max_steps * 0.7))))
    button_pose = np.array([0.55, 0.0, 0.47], dtype=np.float32)
    pusher_pose = np.array([0.0, 0.0, 0.76], dtype=np.float32)
    observations: list[dict[str, Any]] = []
    actions: list[np.ndarray] = []
    rewards: list[float] = []
    success_labels: list[bool] = []
    final_metrics: dict[str, Any] = default_press_button_contact_metrics()
    robot_fields = default_robot_runtime_fields(robot_mode=robot_mode, robot_config_path=robot_config_path)
    if robot_mode == "ee_placeholder":
        pusher_pose = np.asarray(robot_fields["ee_pose"][:3], dtype=np.float32)
    max_button_press_depth = 0.0
    first_contact_step: int | None = None
    first_success_step: int | None = None
    contact_step_count = 0

    for step in range(max_steps):
        previous_obs = {
            "runtime": {
                "pusher_pose": pusher_pose.copy(),
                "button_pose": button_pose.copy(),
            }
        }
        action = _select_action(
            policy_name=policy_name,
            observation=previous_obs,
            step=step,
            max_steps=max_steps,
            rng=rng,
        )
        action = clip_action(action)
        pusher_pose = (pusher_pose + action[:3]).astype(np.float32)
        contact_seen = step + 1 >= max(1, scripted_success_step - 2)
        success = step + 1 >= scripted_success_step
        if contact_seen:
            contact_step_count += 1
            if first_contact_step is None:
                first_contact_step = step + 1
        if success and first_success_step is None:
            first_success_step = step + 1
        button_press_depth = 0.04 if success else (0.015 if contact_seen else 0.0)
        max_button_press_depth = max(max_button_press_depth, button_press_depth)
        button_displacement = max_button_press_depth
        button_pose[2] = np.float32(0.47 - min(max_button_press_depth, 0.04))
        ee_pose = list(robot_fields["ee_pose"])
        ee_pose[:3] = pusher_pose.astype(np.float32).tolist()
        dynamic_robot_fields = default_robot_runtime_fields(
            robot_mode=robot_mode,
            robot_config_path=robot_config_path,
            ee_pose=ee_pose,
            gripper_command=float(action[6]),
        )
        runtime_status = {
            "contact_signal_seen": contact_seen,
            "contact_proxy_triggered": contact_seen,
            "contact_force_available": False,
            "physics_contact_available": False,
            "contact_force_vector": None,
            "button_displacement_available": True,
            "button_displacement": button_displacement,
            "button_press_depth": button_press_depth,
            "max_button_press_depth": max_button_press_depth,
            "using_geometric_fallback": False,
            **dynamic_robot_fields,
        }
        tactile = adapt_press_button_runtime_tactile(runtime_status, tactile_mode=tactile_mode)
        robot_state = default_robot_state()
        robot_state["ee_pose"][:3] = pusher_pose
        robot_state["ee_pose"][3:] = np.asarray(robot_fields["ee_pose"][3:], dtype=np.float32)
        robot_state["joint_pos"][:3] = pusher_pose
        obs = make_mock_observation(
            language=INSTRUCTION,
            robot_state=robot_state,
            tactile=tactile,
            step=step + 1,
            timestamp=(step + 1) / DEFAULT_ACTION_SCHEMA.control_frequency_hz,
        )
        obs["runtime"] = {
            "pusher_pose": pusher_pose.copy(),
            "button_pose": button_pose.copy(),
            "button_pressed": success,
            "contact_proxy": contact_seen,
            "geometric_contact_proxy": True,
            "placeholder_pusher": True,
            "physics_contact_available": False,
            "contact_signal_seen": contact_seen,
            "contact_force_available": False,
            "button_displacement_available": True,
            "button_displacement": button_displacement,
            "button_press_depth": button_press_depth,
            "max_button_press_depth": max_button_press_depth,
            "using_geometric_fallback": False,
            "success_source": "button_displacement" if success else "none",
            **dynamic_robot_fields,
        }
        assert_observation_schema(obs)
        observations.append(obs)
        actions.append(action)
        rewards.append(float(1.0 if success else 0.0))
        success_labels.append(bool(success))
        final_metrics = {
            "success": bool(success),
            "success_source": "button_displacement" if success else "none",
            "num_steps": step + 1,
            "first_contact_step": first_contact_step,
            "first_success_step": first_success_step,
            "contact_step_count": contact_step_count,
            "completion_time": (step + 1) / DEFAULT_ACTION_SCHEMA.control_frequency_hz,
            "contact_signal_seen": contact_seen,
            "contact_proxy_triggered": contact_seen,
            "contact_force_available": False,
            "physics_contact_available": False,
            "contact_force_norm": 0.0,
            "max_contact_force_norm": 0.0,
            "mean_contact_force_norm": 0.0,
            "contact_force_source": "unavailable",
            "force_source": "unavailable",
            "button_displacement_available": True,
            "button_displacement": button_displacement,
            "button_press_depth": button_press_depth,
            "max_button_press_depth": max_button_press_depth,
            "using_geometric_fallback": False,
            **runtime_tactile_status_fields(tactile),
            **dynamic_robot_fields,
        }
        if success:
            break

    episode_id = f"runtime-smoke-PressButton-{tactile_mode}-seed{seed}-ep{episode_index}"
    return _episode_payload(
        episode_id=episode_id,
        seed=seed,
        tactile_mode=tactile_mode,
        policy_name=policy_name,
        dry_run=True,
        runtime_config_path=runtime_config_path,
        robot_mode=robot_mode,
        robot_config_path=robot_config_path,
        observations=observations,
        actions=actions,
        rewards=rewards,
        success_labels=success_labels,
        final_metrics=final_metrics,
        tactile_snapshot=tactile_snapshot,
    )


def _real_runtime_episodes(
    *,
    args: argparse.Namespace,
    cfg: dict[str, Any],
    output_path: Path,
    tactile_snapshot: dict[str, Any],
) -> tuple[list[dict[str, Any]], IsaacSimPressButtonEnv]:
    readiness = probe_isaacsim_visual_smoke(cfg).as_dict()
    if not readiness.get("ready_for_runtime", False):
        payload = {
            "ok": False,
            "runtime_smoke": True,
            "dry_run": False,
            "benchmark_result": False,
            "not_for_paper_claims": True,
            "errors": list(readiness.get("errors", [])) + list(readiness.get("blocking_conditions", [])),
            "warnings": list(readiness.get("warnings", [])),
        }
        print(json.dumps(payload, indent=2, sort_keys=True))
        raise SystemExit(1)

    env: IsaacSimPressButtonEnv | None = None
    try:
        episodes: list[dict[str, Any]] = []
        env = IsaacSimPressButtonEnv(
            cfg=cfg,
            headless=bool(args.headless),
            webrtc=bool(args.webrtc),
            enable_runtime=True,
            tactile_mode=args.tactile,
            robot_mode=args.robot_mode,
            robot_config_path=args.robot_config,
        )
        env.build()
        for episode_index in range(int(args.num_episodes)):
            seed = int(args.seeds[episode_index % len(args.seeds)])
            rng = np.random.default_rng(seed)
            obs = env.reset(seed=seed)
            observations: list[dict[str, Any]] = []
            actions: list[np.ndarray] = []
            rewards: list[float] = []
            success_labels: list[bool] = []
            final_info: dict[str, Any] = {"success": False, "metrics": default_press_button_contact_metrics()}
            for step in range(max(1, int(args.max_steps))):
                action = _select_action(
                    policy_name=args.policy,
                    observation=obs,
                    step=step,
                    max_steps=args.max_steps,
                    rng=rng,
                )
                next_obs, reward, terminated, truncated, info = env.step(action)
                assert_observation_schema(next_obs)
                observations.append(next_obs)
                actions.append(action)
                rewards.append(float(reward))
                success_labels.append(bool(info.get("success", False)))
                final_info = info
                obs = next_obs
                if terminated or truncated:
                    break
            episode_id = f"runtime-smoke-PressButton-{args.tactile}-seed{seed}-ep{episode_index}"
            screenshot_path = None
            if args.save_screenshots:
                screenshot_path = str(output_path.with_name(f"{episode_id}.png"))
                saved, warning = env.save_screenshot(screenshot_path)
                if not saved and warning:
                    screenshot_path = None
            episodes.append(
                _episode_payload(
                    episode_id=episode_id,
                    seed=seed,
                    tactile_mode=args.tactile,
                    policy_name=args.policy,
                    dry_run=False,
                    runtime_config_path=args.runtime_config,
                    robot_mode=args.robot_mode,
                    robot_config_path=args.robot_config,
                    observations=observations,
                    actions=actions,
                    rewards=rewards,
                    success_labels=success_labels,
                    final_metrics=dict(final_info.get("metrics", {})),
                    tactile_snapshot=tactile_snapshot,
                    screenshot_path=screenshot_path,
                )
            )
        return episodes, env
    except Exception:
        if env is not None:
            env.close()
        raise


def _write_dataset(
    *,
    output_path: Path,
    dataset_info: dict[str, Any],
    creation_config: dict[str, Any],
    episodes: list[dict[str, Any]],
) -> None:
    with HDF5DatasetWriter(output_path, dataset_info=dataset_info, creation_config=creation_config) as writer:
        for episode in episodes:
            writer.write_episode(episode)


def _summary_payload(*, args: argparse.Namespace, output_path: Path, episodes: list[dict[str, Any]]) -> dict[str, Any]:
    final_runtime_metadata = {}
    if episodes:
        final_runtime_metadata = dict(episodes[-1].get("metadata", {}).get("runtime_metadata", {}))
    return {
        "ok": True,
        "runtime_smoke": True,
        "dry_run": bool(args.dry_run),
        "output": str(output_path),
        "num_episodes": len(episodes),
        "task_name": TASK_NAME,
        "backend": RUNTIME_BACKEND,
        "policy_name": str(args.policy),
        "tactile_mode": str(args.tactile),
        "robot_mode": str(args.robot_mode),
        "robot_config_path": str(args.robot_config) if args.robot_config else None,
        "placeholder_robot": bool(final_runtime_metadata.get("placeholder_robot", True)),
        "placeholder_pusher": bool(final_runtime_metadata.get("placeholder_pusher", args.robot_mode == "pusher")),
        "real_fr3_articulation": bool(final_runtime_metadata.get("real_fr3_articulation", False)),
        "seeds": [int(seed) for seed in args.seeds],
        "success": [bool(ep["success"][-1]) if ep["success"] else False for ep in episodes],
        "force_source": "unavailable",
        "contact_force_available": False,
        "button_displacement_available": bool(final_runtime_metadata.get("button_displacement_available", True)),
        "success_source": final_runtime_metadata.get("success_source"),
        "mask": final_runtime_metadata.get("mask", {"has_force": False, "has_wrench": False}),
        "benchmark_result": False,
        "not_for_paper_claims": True,
        "lightwheel_assets_used": False,
    }


def main() -> int:
    args = parse_args()
    if args.num_episodes <= 0:
        raise SystemExit("--num-episodes must be positive.")
    if not args.seeds:
        raise SystemExit("--seeds must contain at least one seed.")

    cfg = load_isaacsim_visual_smoke_config(args.runtime_config)
    output_path = Path(args.output)
    tactile_snapshot = sensor_config_snapshot()
    dataset_info = _dataset_info(output_path=output_path, args=args, tactile_snapshot=tactile_snapshot)
    creation_config = {
        "runtime_smoke": True,
        "backend": RUNTIME_BACKEND,
        "task_name": TASK_NAME,
        "runtime_config_path": str(args.runtime_config),
        "policy_name": str(args.policy),
        "tactile_mode": str(args.tactile),
        "robot_mode": str(args.robot_mode),
        "robot_config_path": str(args.robot_config) if args.robot_config else None,
        "placeholder_robot": True,
        "placeholder_pusher": args.robot_mode == "pusher",
        "real_fr3_articulation": False,
        "num_episodes": int(args.num_episodes),
        "seeds": [int(seed) for seed in args.seeds],
        "max_steps": int(args.max_steps),
        "dry_run": bool(args.dry_run),
        "benchmark_result": False,
        "not_for_paper_claims": True,
        "lightwheel_assets_used": False,
        "tactile_config_snapshot": tactile_snapshot,
    }

    env: IsaacSimPressButtonEnv | None = None
    if args.dry_run:
        episodes = [
            _dry_run_episode(
                episode_index=episode_index,
                seed=int(args.seeds[episode_index % len(args.seeds)]),
                tactile_mode=args.tactile,
                policy_name=args.policy,
                max_steps=args.max_steps,
                runtime_config_path=args.runtime_config,
                robot_mode=args.robot_mode,
                robot_config_path=args.robot_config,
                tactile_snapshot=tactile_snapshot,
            )
            for episode_index in range(int(args.num_episodes))
        ]
    else:
        episodes, env = _real_runtime_episodes(
            args=args,
            cfg=cfg,
            output_path=output_path,
            tactile_snapshot=tactile_snapshot,
        )

    try:
        _write_dataset(
            output_path=output_path,
            dataset_info=dataset_info,
            creation_config=creation_config,
            episodes=episodes,
        )
        summary = _summary_payload(args=args, output_path=output_path, episodes=episodes)
        print(json.dumps(_jsonable(summary), indent=2, sort_keys=True))
        return 0
    finally:
        if env is not None:
            env.close()


if __name__ == "__main__":
    raise SystemExit(main())

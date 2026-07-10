"""Public make_env contract."""

from __future__ import annotations

from typing import Any

from .isaacsim_press_button_env import IsaacSimPressButtonEnv
from .isaacsim_fr3_press_button_env import IsaacSimFR3PressButtonEnv
from .mock_env import MockIsaacTactileLiberoEnv


def make_env(
    task: str,
    robot: str = "fr3_tactile",
    tactile: str = "none",
    split: str = "train",
    seed: int = 0,
    num_envs: int = 1,
    cfg: dict[str, Any] | None = None,
    backend: str = "mock",
    headless: bool = True,
    webrtc: bool = True,
    enable_runtime: bool = False,
) -> MockIsaacTactileLiberoEnv | IsaacSimPressButtonEnv | IsaacSimFR3PressButtonEnv:
    """Create a benchmark environment.

    ``mock`` remains the default CI/backend. ``isaacsim_press_button`` is a
    single-task optional runtime smoke and only supports PressButton.
    """

    cfg = cfg or {}
    if backend == "mock":
        return MockIsaacTactileLiberoEnv(
            task=task,
            robot=robot,
            tactile=tactile,
            split=split,
            seed=seed,
            num_envs=num_envs,
            cfg=cfg,
        )
    if backend == "isaacsim_press_button":
        if task != "PressButton":
            raise ValueError("backend=isaacsim_press_button only supports PressButton.")
        runtime_cfg = cfg.get("runtime_config", cfg)
        robot_mode = cfg.get("robot_mode", runtime_cfg.get("robot_mode", "pusher") if isinstance(runtime_cfg, dict) else "pusher")
        robot_config = cfg.get("robot_config", runtime_cfg.get("robot_config") if isinstance(runtime_cfg, dict) else None)
        robot_config_path = cfg.get(
            "robot_config_path",
            runtime_cfg.get("robot_config_path") if isinstance(runtime_cfg, dict) else None,
        )
        return IsaacSimPressButtonEnv(
            cfg=dict(runtime_cfg or {}),
            headless=headless,
            webrtc=webrtc,
            enable_runtime=enable_runtime,
            tactile_mode=tactile,
            robot_mode=robot_mode,
            robot_config=robot_config,
            robot_config_path=robot_config_path,
        )
    if backend == "isaacsim_fr3_press_button":
        if task != "PressButton":
            raise ValueError("backend=isaacsim_fr3_press_button only supports PressButton.")
        runtime_cfg = cfg.get("runtime_config", cfg)
        return IsaacSimFR3PressButtonEnv(
            cfg=dict(runtime_cfg or {}),
            headless=headless,
            webrtc=webrtc,
            enable_runtime=enable_runtime,
            tactile_mode=tactile,
        )
    raise ValueError(
        f"Unknown backend: {backend}. Available: mock, isaacsim_press_button, isaacsim_fr3_press_button"
    )

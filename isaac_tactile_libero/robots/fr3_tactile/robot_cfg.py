"""FR3-Tactile mock/stub robot config."""

from __future__ import annotations

from copy import deepcopy
from typing import Any

import numpy as np

from isaac_tactile_libero.registry.robot_registry import ROBOT_REGISTRY
from isaac_tactile_libero.version import BENCHMARK_VERSION

from .action_cfg import FR3_TACTILE_ACTION_CFG
from .frames import FR3_TACTILE_FRAMES
from .usd_paths import FR3_TACTILE_USD_PATH


class FR3TactileRobotConfig:
    """Static mock/stub embodiment metadata for the Phase 1 runtime."""

    name = "fr3_tactile"

    def __init__(self, cfg: dict[str, Any] | None = None):
        cfg = cfg or {}
        self.joint_names = cfg.get(
            "joint_names",
            [
                "fr3_joint1",
                "fr3_joint2",
                "fr3_joint3",
                "fr3_joint4",
                "fr3_joint5",
                "fr3_joint6",
                "fr3_joint7",
                "finger_joint1",
                "finger_joint2",
            ],
        )
        self.link_names = cfg.get(
            "link_names",
            [
                "fr3_link0",
                "fr3_link1",
                "fr3_link2",
                "fr3_link3",
                "fr3_link4",
                "fr3_link5",
                "fr3_link6",
                "fr3_link7",
                "fr3_hand",
            ],
        )
        self.frames = deepcopy(FR3_TACTILE_FRAMES)
        self.action_cfg = deepcopy(FR3_TACTILE_ACTION_CFG)
        self.usd_path = FR3_TACTILE_USD_PATH
        self.default_joint_pose = np.zeros(len(self.joint_names), dtype=np.float32)
        self.collision_settings = {"enabled": False, "status": "mock/stub no physics collision"}
        self.articulation_properties = {"status": "mock/stub no Isaac articulation"}

    def as_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "joint_names": list(self.joint_names),
            "link_names": list(self.link_names),
            "frames": deepcopy(self.frames),
            "action_cfg": deepcopy(self.action_cfg),
            "usd_path": self.usd_path,
            "collision_settings": deepcopy(self.collision_settings),
            "articulation_properties": deepcopy(self.articulation_properties),
        }


ROBOT_REGISTRY.register(
    "fr3_tactile",
    FR3TactileRobotConfig,
    version=BENCHMARK_VERSION,
    embodiment="mock/stub FR3-Tactile",
)

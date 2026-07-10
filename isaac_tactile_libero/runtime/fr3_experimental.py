"""Isaac Sim 6 experimental FR3 articulation controller."""

from __future__ import annotations

from typing import Any, Callable

import numpy as np

from isaac_tactile_libero.schemas.action import clip_action


EXPECTED_FR3_DOFS = (
    "fr3_joint1",
    "fr3_joint2",
    "fr3_joint3",
    "fr3_joint4",
    "fr3_joint5",
    "fr3_joint6",
    "fr3_joint7",
    "fr3_finger_joint1",
    "fr3_finger_joint2",
)


def _to_numpy(value: Any) -> np.ndarray:
    if isinstance(value, np.ndarray):
        return value
    if hasattr(value, "numpy"):
        return np.asarray(value.numpy())
    try:
        import warp as wp  # type: ignore

        return np.asarray(wp.to_numpy(value))
    except Exception:
        return np.asarray(value)


class IsaacSim6FR3Controller:
    """Bounded 7D delta-EE controller using experimental articulation Jacobians."""

    def __init__(
        self,
        prim_path: str = "/World/FR3",
        *,
        ee_link_name: str = "fr3_hand",
        damping: float = 0.02,
        max_joint_delta_rad: float = 0.02,
        max_gripper_width_m: float = 0.04,
        articulation_factory: Callable[[str], Any] | None = None,
    ) -> None:
        self.prim_path = str(prim_path)
        self.ee_link_name = str(ee_link_name)
        self.damping = float(damping)
        self.max_joint_delta_rad = float(max_joint_delta_rad)
        self.max_gripper_width_m = float(max_gripper_width_m)
        self._articulation_factory = articulation_factory
        self.articulation: Any | None = None
        self.dof_names: tuple[str, ...] = ()
        self.link_names: tuple[str, ...] = ()
        self._ee_jacobian_index: int | None = None

    def initialize(
        self,
        *,
        step_callback: Callable[[int], None] | None = None,
        timeout_steps: int = 5,
    ) -> None:
        if self._articulation_factory is None:
            from isaacsim.core.experimental.prims import Articulation  # type: ignore

            self._articulation_factory = Articulation
        last_error: Exception | None = None
        for attempt in range(max(0, int(timeout_steps)) + 1):
            articulation = self._articulation_factory(self.prim_path)
            names = tuple(str(name) for name in articulation.dof_names)
            if names != EXPECTED_FR3_DOFS:
                raise RuntimeError(f"FR3 DOF contract mismatch: expected {EXPECTED_FR3_DOFS}, got {names}")
            links = tuple(str(name) for name in articulation.link_names)
            if self.ee_link_name not in links:
                raise RuntimeError(f"FR3 EE link {self.ee_link_name!r} not found in {links}")
            self.articulation = articulation
            self.dof_names = names
            self.link_names = links
            try:
                self._ee_jacobian_index = self._resolve_jacobian_index()
                return
            except AssertionError as exc:
                last_error = exc
                self.articulation = None
                if step_callback is None or attempt >= int(timeout_steps):
                    break
                step_callback(1)
        raise RuntimeError(f"FR3 tensor view did not become ready within {timeout_steps} steps: {last_error}")

    def _resolve_jacobian_index(self) -> int:
        assert self.articulation is not None
        link_index = self.link_names.index(self.ee_link_name)
        jacobian = _to_numpy(self.articulation.get_jacobian_matrices())
        if jacobian.ndim != 4 or jacobian.shape[0] != 1 or jacobian.shape[2] != 6:
            raise RuntimeError(f"Unexpected FR3 Jacobian shape: {jacobian.shape}")
        if jacobian.shape[1] == len(self.link_names) - 1:
            if link_index == 0:
                raise RuntimeError("The fixed articulation root has no Jacobian row")
            return link_index - 1
        if jacobian.shape[1] == len(self.link_names):
            return link_index
        raise RuntimeError(
            f"FR3 Jacobian/link contract mismatch: {jacobian.shape[1]} rows for {len(self.link_names)} links"
        )

    def read_joint_state(self) -> tuple[np.ndarray, np.ndarray]:
        if self.articulation is None:
            raise RuntimeError("FR3 controller is not initialized")
        q = _to_numpy(self.articulation.get_dof_positions()).reshape(-1).astype(np.float32, copy=True)
        qd = _to_numpy(self.articulation.get_dof_velocities()).reshape(-1).astype(np.float32, copy=True)
        if q.shape != (len(EXPECTED_FR3_DOFS),) or qd.shape != q.shape:
            raise RuntimeError(f"Unexpected FR3 state shape: q={q.shape}, qd={qd.shape}")
        if not np.all(np.isfinite(q)) or not np.all(np.isfinite(qd)):
            raise RuntimeError("FR3 joint state contains NaN/Inf")
        return q, qd

    def apply_action(self, action: Any) -> dict[str, Any]:
        if self.articulation is None or self._ee_jacobian_index is None:
            raise RuntimeError("FR3 controller is not initialized")
        bounded = clip_action(action)
        q, _ = self.read_joint_state()
        cartesian_delta = bounded[:6].astype(np.float64)
        zero_action = bool(np.allclose(bounded, 0.0))
        dq = np.zeros_like(q, dtype=np.float64)
        if not np.allclose(cartesian_delta, 0.0):
            jacobians = _to_numpy(self.articulation.get_jacobian_matrices())
            jacobian = np.asarray(jacobians[0, self._ee_jacobian_index, :, :], dtype=np.float64)
            lhs = jacobian @ jacobian.T + self.damping**2 * np.eye(6, dtype=np.float64)
            dq = jacobian.T @ np.linalg.solve(lhs, cartesian_delta)
            dq = np.clip(dq, -self.max_joint_delta_rad, self.max_joint_delta_rad)
        target = q.astype(np.float64) + dq
        gripper_width = (float(bounded[6]) + 1.0) * 0.5 * self.max_gripper_width_m
        if not zero_action:
            target[7:9] = gripper_width
        if not np.all(np.isfinite(target)):
            raise RuntimeError("FR3 target contains NaN/Inf")
        self.articulation.set_dof_position_targets(target.astype(np.float32).reshape(1, -1))
        return {
            "command_sent": True,
            "controller_method": "experimental_jacobian_dls",
            "action_shape": list(bounded.shape),
            "bounded_action": bounded.tolist(),
            "zero_action": zero_action,
            "max_abs_joint_delta": float(np.max(np.abs(dq))) if dq.size else 0.0,
            "force_vector_valid": False,
            "wrench_valid": False,
        }

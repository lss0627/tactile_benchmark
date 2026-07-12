"""Isaac Sim 6 experimental FR3 articulation controller."""

from __future__ import annotations

from typing import Any, Callable

import numpy as np

from isaac_tactile_libero.schemas.action import clip_action
from isaac_tactile_libero.runtime.fr3_target_latch import FR3PositionTargetLatch


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

    target_latch_type = FR3PositionTargetLatch

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
        self._target_latch: FR3PositionTargetLatch | None = None
        self._scene_token: str | None = None
        self._aborted_reason: str | None = None
        self._closed = False

    def initialize(
        self,
        *,
        step_callback: Callable[[int], None] | None = None,
        timeout_steps: int = 5,
    ) -> None:
        self.reset_target_latch()
        self._closed = False
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
                target_reader = getattr(articulation, "get_dof_position_targets", None)
                if not callable(target_reader):
                    raise RuntimeError("FR3 position target API is unavailable for target latch")
                position_targets = target_reader()
                scene_token = f"{self.prim_path}@{id(articulation)}"
                latch = self.target_latch_type(
                    dof_names=names,
                    scene_token=scene_token,
                    prim_path=self.prim_path,
                    articulation_object_id=id(articulation),
                )
                latch.seed(
                    position_targets,
                    dof_names=names,
                    scene_token=scene_token,
                    source="get_dof_position_targets",
                    prim_path=self.prim_path,
                    articulation_object_id=id(articulation),
                )
                self._target_latch = latch
                self._scene_token = scene_token
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
        if self._aborted_reason is not None:
            raise RuntimeError(f"FR3 controller aborted: {self._aborted_reason}")
        if self.articulation is None or self._ee_jacobian_index is None:
            raise RuntimeError("FR3 controller is not initialized")
        if self._target_latch is None or self._scene_token is None:
            raise RuntimeError("FR3 controller target latch is not initialized")
        bounded = clip_action(action)
        cartesian_delta = bounded[:6].astype(np.float64)
        zero_action = bool(np.allclose(bounded, 0.0))
        if zero_action:
            dq = np.zeros(len(EXPECTED_FR3_DOFS), dtype=np.float64)
            target = self._target_latch.resolve_zero_target(
                observed_joint_positions=None,
                scene_token=self._scene_token,
            ).astype(np.float64)
        else:
            q, _ = self.read_joint_state()
            dq = np.zeros_like(q, dtype=np.float64)
            target = q.astype(np.float64)
        if not zero_action and not np.allclose(cartesian_delta, 0.0):
            jacobians = _to_numpy(self.articulation.get_jacobian_matrices())
            jacobian = np.asarray(jacobians[0, self._ee_jacobian_index, :, :], dtype=np.float64)
            lhs = jacobian @ jacobian.T + self.damping**2 * np.eye(6, dtype=np.float64)
            dq = jacobian.T @ np.linalg.solve(lhs, cartesian_delta)
            dq = np.clip(dq, -self.max_joint_delta_rad, self.max_joint_delta_rad)
            target = target + dq
        gripper_width = (float(bounded[6]) + 1.0) * 0.5 * self.max_gripper_width_m
        if not zero_action:
            target[7:9] = gripper_width
        if not np.all(np.isfinite(target)):
            raise RuntimeError("FR3 target contains NaN/Inf")
        target_array = target.astype(np.float32).reshape(1, -1)
        send_result = self.articulation.set_dof_position_targets(target_array)
        command_sent = send_result is not False
        if not zero_action and command_sent:
            self._target_latch.accept_target(
                target_array,
                send_succeeded=True,
                dof_names=self.dof_names,
                scene_token=self._scene_token,
                source="accepted_nonzero_action",
                prim_path=self.prim_path,
                articulation_object_id=id(self.articulation),
            )
        return {
            "command_sent": command_sent,
            "controller_method": "experimental_jacobian_dls",
            "action_shape": list(bounded.shape),
            "bounded_action": bounded.tolist(),
            "zero_action": zero_action,
            "max_abs_joint_delta": float(np.max(np.abs(dq))) if dq.size else 0.0,
            "force_vector_valid": False,
            "wrench_valid": False,
            "target_latch_provenance": self._target_latch.provenance,
        }

    def reset_target_latch(self) -> None:
        if self._target_latch is not None:
            self._target_latch.reset()
        self._target_latch = None
        self._scene_token = None
        self._aborted_reason = None
        self.articulation = None
        self.dof_names = ()
        self.link_names = ()
        self._ee_jacobian_index = None

    def abort(self, reason: str) -> None:
        self._aborted_reason = str(reason)
        if self._target_latch is not None:
            self._target_latch.abort(self._aborted_reason)

    def close(self) -> None:
        if self._closed:
            return
        if self._target_latch is not None:
            self._target_latch.invalidate("controller closed")
        self._target_latch = None
        self._scene_token = None
        self.articulation = None
        self._ee_jacobian_index = None
        self._closed = True

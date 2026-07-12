"""Import-safe accepted position-target state for FR3 controllers."""

from __future__ import annotations

from typing import Any, Sequence

import numpy as np


class FR3PositionTargetLatch:
    """Store one validated scene-local FR3 position target as command state."""

    def __init__(
        self,
        *,
        dof_names: Sequence[str],
        scene_token: str,
        prim_path: str | None = None,
        articulation_object_id: int | None = None,
    ) -> None:
        names = tuple(str(name) for name in dof_names)
        if not names or len(set(names)) != len(names):
            raise RuntimeError("FR3 target latch requires unique ordered DOF names")
        if not str(scene_token):
            raise RuntimeError("FR3 target latch requires a scene token")
        self._dof_names = names
        self._scene_token = str(scene_token)
        self._prim_path = None if prim_path is None else str(prim_path)
        self._articulation_object_id = (
            None if articulation_object_id is None else int(articulation_object_id)
        )
        self._target: np.ndarray | None = None
        self._source: str | None = None
        self._invalidated_reason: str | None = None
        self._aborted_reason: str | None = None

    @property
    def provenance(self) -> dict[str, Any]:
        return {
            "scene_token": self._scene_token,
            "dof_names": list(self._dof_names),
            "source": self._source,
            "prim_path": self._prim_path,
            "articulation_object_id": self._articulation_object_id,
            "seeded": self._target is not None,
            "invalidated_reason": self._invalidated_reason,
            "aborted_reason": self._aborted_reason,
        }

    def _validate_identity(
        self,
        *,
        dof_names: Sequence[str],
        scene_token: str,
        prim_path: str | None,
        articulation_object_id: int | None,
    ) -> None:
        names = tuple(str(name) for name in dof_names)
        if names != self._dof_names:
            raise RuntimeError(
                f"FR3 target latch DOF order mismatch: expected {self._dof_names}, got {names}"
            )
        if str(scene_token) != self._scene_token:
            raise RuntimeError("FR3 target latch scene token mismatch")
        if self._prim_path is not None and str(prim_path) != self._prim_path:
            raise RuntimeError("FR3 target latch prim provenance mismatch")
        if (
            self._articulation_object_id is not None
            and articulation_object_id != self._articulation_object_id
        ):
            raise RuntimeError("FR3 target latch articulation provenance mismatch")

    def _validated_target(self, target: Any) -> np.ndarray:
        array = np.asarray(target, dtype=np.float32)
        valid_shapes = {(len(self._dof_names),), (1, len(self._dof_names))}
        if array.shape not in valid_shapes:
            raise RuntimeError(
                "FR3 position target shape mismatch: "
                f"expected {(1, len(self._dof_names))} or {(len(self._dof_names),)}, got {array.shape}"
            )
        flat = array.reshape(-1).copy()
        if not np.all(np.isfinite(flat)):
            raise RuntimeError("FR3 position target contains NaN/Inf")
        return flat

    def seed(
        self,
        target: Any,
        *,
        dof_names: Sequence[str],
        scene_token: str,
        source: str,
        prim_path: str | None = None,
        articulation_object_id: int | None = None,
    ) -> None:
        if not str(source):
            raise RuntimeError("FR3 target latch source is required")
        self._validate_identity(
            dof_names=dof_names,
            scene_token=scene_token,
            prim_path=prim_path,
            articulation_object_id=articulation_object_id,
        )
        self._target = self._validated_target(target)
        self._target.setflags(write=False)
        self._source = str(source)
        self._invalidated_reason = None
        self._aborted_reason = None

    def resolve_zero_target(
        self,
        *,
        observed_joint_positions: Any | None = None,
        scene_token: str,
    ) -> np.ndarray:
        del observed_joint_positions
        if self._aborted_reason is not None:
            raise RuntimeError(f"FR3 target latch aborted: {self._aborted_reason}")
        if self._invalidated_reason is not None:
            raise RuntimeError(f"FR3 target latch invalidated: {self._invalidated_reason}")
        if str(scene_token) != self._scene_token:
            raise RuntimeError("FR3 target latch scene token mismatch")
        if self._target is None:
            raise RuntimeError("FR3 target latch is unseeded")
        return self._target.copy()

    def accept_target(
        self,
        target: Any,
        *,
        send_succeeded: bool,
        dof_names: Sequence[str],
        scene_token: str,
        source: str,
        prim_path: str | None = None,
        articulation_object_id: int | None = None,
    ) -> bool:
        if self._aborted_reason is not None:
            raise RuntimeError(f"FR3 target latch aborted: {self._aborted_reason}")
        if not send_succeeded:
            return False
        self.seed(
            target,
            dof_names=dof_names,
            scene_token=scene_token,
            source=source,
            prim_path=prim_path,
            articulation_object_id=articulation_object_id,
        )
        return True

    def reset(self) -> None:
        self._target = None
        self._source = None
        self._invalidated_reason = None
        self._aborted_reason = None

    def invalidate(self, reason: str = "invalidated") -> None:
        self._target = None
        self._source = None
        self._invalidated_reason = str(reason)

    def abort(self, reason: str) -> None:
        self._aborted_reason = str(reason)

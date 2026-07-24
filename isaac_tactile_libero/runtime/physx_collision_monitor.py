"""Import-safe adapter over the PhysX contact-report identity boundary."""

from __future__ import annotations

from typing import Any, Sequence

import numpy as np


class PhysXCollisionMonitor:
    """Separate allowlisted task contact from unsafe FR3 collision."""

    def __init__(
        self,
        *,
        interface: Any,
        path_decoder: Any,
        allowed_contact_pairs: Sequence[Sequence[str]],
    ) -> None:
        self.interface = interface
        self.path_decoder = path_decoder
        self.allowed_contact_pairs = [
            tuple(str(item) for item in pair)
            for pair in allowed_contact_pairs
        ]
        if any(len(pair) != 2 for pair in self.allowed_contact_pairs):
            raise ValueError("allowed Contact pairs must contain two paths")
        self.samples = 0

    @staticmethod
    def _path_matches(actual: str, configured: str) -> bool:
        return actual == configured or actual.startswith(
            configured.rstrip("/") + "/"
        )

    def _allowed(self, first: str, second: str) -> bool:
        return any(
            (
                self._path_matches(first, expected_first)
                and self._path_matches(second, expected_second)
            )
            or (
                self._path_matches(first, expected_second)
                and self._path_matches(second, expected_first)
            )
            for expected_first, expected_second in self.allowed_contact_pairs
        )

    def read(self) -> dict[str, Any]:
        try:
            headers, contacts = self.interface.get_contact_report()
        except Exception as exc:
            return {
                "valid": False,
                "unsafe_collision": False,
                "unsafe_pairs": [],
                "max_penetration_m": 0.0,
                "contact_count": 0,
                "error": f"{type(exc).__name__}: {exc}",
            }
        self.samples += 1
        unsafe_pairs: list[list[str]] = []
        maximum_penetration = 0.0
        count = 0
        try:
            for header in headers:
                first = str(self.path_decoder(header.collider0))
                second = str(self.path_decoder(header.collider1))
                contact_count = int(header.num_contact_data)
                count += contact_count
                if (
                    first.startswith("/World/FR3")
                    or second.startswith("/World/FR3")
                ) and not self._allowed(first, second):
                    pair = [first, second]
                    if pair not in unsafe_pairs:
                        unsafe_pairs.append(pair)
                start = int(header.contact_data_offset)
                for index in range(start, start + contact_count):
                    separation = float(contacts[index].separation)
                    if np.isfinite(separation):
                        maximum_penetration = max(
                            maximum_penetration,
                            max(0.0, -separation),
                        )
        except Exception as exc:
            return {
                "valid": False,
                "unsafe_collision": False,
                "unsafe_pairs": unsafe_pairs,
                "max_penetration_m": maximum_penetration,
                "contact_count": count,
                "error": f"{type(exc).__name__}: {exc}",
            }
        return {
            "valid": True,
            "unsafe_collision": bool(unsafe_pairs),
            "unsafe_pairs": unsafe_pairs,
            "max_penetration_m": maximum_penetration,
            "contact_count": count,
            "error": "",
        }


__all__ = ["PhysXCollisionMonitor"]

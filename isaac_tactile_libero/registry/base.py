"""Small typed registry used by the mock/stub benchmark skeleton."""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass, field
from typing import Any, Callable, Generic, TypeVar

T = TypeVar("T")


@dataclass(frozen=True)
class RegistryEntry(Generic[T]):
    """A registered constructor and its benchmark metadata."""

    name: str
    cls: Callable[..., T]
    metadata: dict[str, Any] = field(default_factory=dict)


class Registry(Generic[T]):
    """Register, list, retrieve, and instantiate benchmark components."""

    def __init__(self, kind: str):
        self.kind = kind
        self._entries: "OrderedDict[str, RegistryEntry[T]]" = OrderedDict()

    def register(self, name: str, cls: Callable[..., T], **metadata: Any) -> None:
        if name in self._entries:
            raise ValueError(f"{self.kind} '{name}' is already registered")
        self._entries[name] = RegistryEntry(name=name, cls=cls, metadata=dict(metadata))

    def list(self) -> list[str]:
        return list(self._entries.keys())

    def entries(self) -> list[RegistryEntry[T]]:
        return list(self._entries.values())

    def get(self, name: str) -> RegistryEntry[T]:
        try:
            return self._entries[name]
        except KeyError as exc:
            available = ", ".join(self.list()) or "<none>"
            raise KeyError(f"Unknown {self.kind} '{name}'. Available: {available}") from exc

    def make(self, name: str, cfg: dict[str, Any] | None = None, **kwargs: Any) -> T:
        entry = self.get(name)
        return entry.cls(cfg=cfg or {}, **kwargs)

    def clear(self) -> None:
        self._entries.clear()

"""Isaac-Tactile-LIBERO Phase 1 mock/stub package."""

from .version import BENCHMARK_VERSION


def register_builtins() -> None:
    """Import built-in mock/stub plugins so their registries are populated."""

    from . import policies as _policies  # noqa: F401
    from . import robots as _robots  # noqa: F401
    from . import sensors as _sensors  # noqa: F401
    from . import tasks as _tasks  # noqa: F401


register_builtins()

__all__ = ["BENCHMARK_VERSION", "register_builtins"]

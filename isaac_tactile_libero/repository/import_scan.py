"""AST-only Isaac Sim import policy scanner for first-party Python."""

from __future__ import annotations

import ast
from pathlib import Path
from typing import Iterable


FIRST_PARTY_ROOTS = (
    "isaac_tactile_libero",
    "scripts",
    "tests",
    "configs",
)
REMOVED_PREFIXES = ("omni.isaac",)
DEPRECATED_PREFIXES = (
    "isaacsim.core.api",
    "isaacsim.core.prims",
    "isaacsim.core.utils",
)


def _matches(module: str, prefixes: Iterable[str]) -> str | None:
    for prefix in prefixes:
        if module == prefix or module.startswith(prefix + "."):
            return prefix
    return None


def _imports(path: Path) -> list[tuple[int, str]]:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    imports: list[tuple[int, str]] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imports.extend((node.lineno, alias.name) for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imports.append((node.lineno, node.module))
    return imports


def scan_first_party_imports(root: str | Path) -> dict:
    repository = Path(root)
    errors: list[dict] = []
    warnings: list[dict] = []
    scanned_files = 0
    for relative_root in FIRST_PARTY_ROOTS:
        source_root = repository / relative_root
        if not source_root.is_dir():
            continue
        for path in sorted(source_root.rglob("*.py")):
            if any(part in {"vendor", ".venv", "__pycache__"} for part in path.parts):
                continue
            scanned_files += 1
            for line, module in _imports(path):
                relative = path.relative_to(repository).as_posix()
                removed = _matches(module, REMOVED_PREFIXES)
                if removed:
                    errors.append(
                        {
                            "file": relative,
                            "line": line,
                            "module": module,
                            "rule": f"removed:{removed}",
                        }
                    )
                    continue
                deprecated = _matches(module, DEPRECATED_PREFIXES)
                if deprecated:
                    warnings.append(
                        {
                            "file": relative,
                            "line": line,
                            "module": module,
                            "rule": f"deprecated:{deprecated}",
                        }
                    )
    return {
        "scanned_files": scanned_files,
        "errors": errors,
        "warnings": warnings,
    }

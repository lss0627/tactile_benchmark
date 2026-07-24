"""Complete versioned schema catalog and definition audit."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any

from .base import RecordSchema
from .benchmark import validate_paper_v1_constants
from .collection import COLLECTION_SCHEMAS
from .dataset import GENERALIZATION_DATASET_SCHEMAS
from .evaluation import EVALUATION_SCHEMAS
from .plugin import PLUGIN_SCHEMAS
from .task import TASK_SCHEMAS
from .training import TRAINING_SCHEMAS


ALL_CONTRACT_SCHEMAS: tuple[RecordSchema, ...] = (
    *TASK_SCHEMAS,
    *PLUGIN_SCHEMAS,
    *COLLECTION_SCHEMAS,
    *GENERALIZATION_DATASET_SCHEMAS,
    *TRAINING_SCHEMAS,
    *EVALUATION_SCHEMAS,
)
SCHEMA_CATALOG = {schema.name: schema for schema in ALL_CONTRACT_SCHEMAS}


def schema_catalog_sha256() -> str:
    definitions = [
        schema.as_json_schema()
        for schema in sorted(ALL_CONTRACT_SCHEMAS, key=lambda item: item.name)
    ]
    payload = json.dumps(
        definitions,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def validate_schema_catalog() -> dict[str, Any]:
    errors: list[str] = []
    names = [schema.name for schema in ALL_CONTRACT_SCHEMAS]
    ids = [schema.schema_id for schema in ALL_CONTRACT_SCHEMAS]
    if len(names) != len(set(names)):
        errors.append("schema catalog contains duplicate names")
    if len(ids) != len(set(ids)):
        errors.append("schema catalog contains duplicate schema IDs")
    errors.extend(validate_paper_v1_constants())
    schema_errors: dict[str, list[str]] = {}
    for schema in ALL_CONTRACT_SCHEMAS:
        observed = schema.definition_errors()
        if observed:
            schema_errors[schema.name] = observed
            errors.extend(f"{schema.name}: {error}" for error in observed)
    return {
        "ok": not errors,
        "schema_version": "1.0.0",
        "schema_count": len(ALL_CONTRACT_SCHEMAS),
        "schema_names": sorted(names),
        "schema_catalog_sha256": schema_catalog_sha256(),
        "schema_errors": schema_errors,
        "errors": errors,
    }


def export_schema_catalog(output: str | Path) -> list[Path]:
    destination = Path(output)
    destination.mkdir(parents=True, exist_ok=False)
    written: list[Path] = []
    for schema in sorted(ALL_CONTRACT_SCHEMAS, key=lambda item: item.name):
        path = destination / f"{schema.name}-{schema.version}.schema.json"
        path.write_text(
            json.dumps(schema.as_json_schema(), indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        written.append(path)
    return written

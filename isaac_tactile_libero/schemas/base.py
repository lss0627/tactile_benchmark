"""Dependency-free versioned record schemas for benchmark manifests.

The project intentionally keeps these contract validators import-safe outside
Isaac Sim.  The descriptors use a strict, documented subset of JSON Schema and
can be exported as draft 2020-12 documents for review tooling.
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import json
import math
import re
from typing import Any, Callable, Mapping, Sequence


CONTRACT_SCHEMA_VERSION = "1.0.0"
JSON_SCHEMA_DIALECT = "https://json-schema.org/draft/2020-12/schema"
SEMANTIC_VERSION_PATTERN = re.compile(
    r"^(0|[1-9][0-9]*)[.](0|[1-9][0-9]*)[.](0|[1-9][0-9]*)$"
)
SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")
COMMIT_PATTERN = re.compile(r"^[0-9a-f]{40}$")

Invariant = Callable[[Mapping[str, Any]], Sequence[str]]


def _type_matches(value: Any, expected: str) -> bool:
    if expected == "null":
        return value is None
    if expected == "boolean":
        return isinstance(value, bool)
    if expected == "integer":
        return isinstance(value, int) and not isinstance(value, bool)
    if expected == "number":
        return (
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(float(value))
        )
    if expected == "string":
        return isinstance(value, str)
    if expected == "array":
        return isinstance(value, list)
    if expected == "object":
        return isinstance(value, Mapping)
    raise ValueError(f"unsupported schema type: {expected}")


def _display_types(expected: str | Sequence[str]) -> str:
    values = (expected,) if isinstance(expected, str) else tuple(expected)
    return " or ".join(values)


def _canonical_item(value: Any) -> str:
    return json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    )


def _validate_format(path: str, value: str, format_name: str) -> list[str]:
    if format_name == "semver" and not SEMANTIC_VERSION_PATTERN.fullmatch(value):
        return [f"{path} must be a semantic version (MAJOR.MINOR.PATCH)"]
    if format_name == "sha256" and not SHA256_PATTERN.fullmatch(value):
        return [f"{path} must be a lowercase 64-character SHA-256 digest"]
    if format_name == "commit" and not COMMIT_PATTERN.fullmatch(value):
        return [f"{path} must be a lowercase 40-character commit digest"]
    if format_name not in {"semver", "sha256", "commit"}:
        return [f"{path} uses unsupported format {format_name!r}"]
    return []


def _validate_value(path: str, value: Any, descriptor: Mapping[str, Any]) -> list[str]:
    errors: list[str] = []
    expected = descriptor.get("type")
    if expected is not None:
        expected_types = (expected,) if isinstance(expected, str) else tuple(expected)
        if not any(_type_matches(value, item) for item in expected_types):
            return [f"{path} must be {_display_types(expected_types)}"]

    if "const" in descriptor and value != descriptor["const"]:
        errors.append(f"{path} must equal {descriptor['const']!r}")
    if "enum" in descriptor and value not in descriptor["enum"]:
        allowed = ", ".join(str(item) for item in descriptor["enum"])
        errors.append(f"{path} must be one of: {allowed}")

    if isinstance(value, str):
        minimum_length = descriptor.get("minLength")
        if minimum_length is not None and len(value) < int(minimum_length):
            errors.append(f"{path} must contain at least {minimum_length} character(s)")
        format_name = descriptor.get("format")
        if format_name is not None:
            errors.extend(_validate_format(path, value, str(format_name)))
        pattern = descriptor.get("pattern")
        if pattern is not None and re.fullmatch(str(pattern), value) is None:
            errors.append(f"{path} does not match the required pattern")

    if (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    ):
        if "minimum" in descriptor and value < descriptor["minimum"]:
            if descriptor["minimum"] == 1:
                errors.append(f"{path} must be greater than 0")
            else:
                errors.append(f"{path} must be at least {descriptor['minimum']}")
        if "exclusiveMinimum" in descriptor and value <= descriptor["exclusiveMinimum"]:
            errors.append(f"{path} must be greater than {descriptor['exclusiveMinimum']}")
        if "maximum" in descriptor and value > descriptor["maximum"]:
            errors.append(f"{path} must be at most {descriptor['maximum']}")

    if isinstance(value, list):
        if "minItems" in descriptor and len(value) < int(descriptor["minItems"]):
            errors.append(f"{path} must contain at least {descriptor['minItems']} item(s)")
        if "maxItems" in descriptor and len(value) > int(descriptor["maxItems"]):
            errors.append(f"{path} must contain at most {descriptor['maxItems']} item(s)")
        if descriptor.get("uniqueItems"):
            try:
                canonical = [_canonical_item(item) for item in value]
            except (TypeError, ValueError, OverflowError):
                errors.append(f"{path} items must be JSON serializable")
            else:
                if len(canonical) != len(set(canonical)):
                    errors.append(f"{path} items must be unique")
        item_descriptor = descriptor.get("items")
        if isinstance(item_descriptor, Mapping):
            for index, item in enumerate(value):
                errors.extend(_validate_value(f"{path}[{index}]", item, item_descriptor))

    if isinstance(value, Mapping):
        required_keys = tuple(descriptor.get("required", ()))
        for key in required_keys:
            if key not in value:
                errors.append(f"{path}.{key} is required")
        properties = descriptor.get("properties")
        if isinstance(properties, Mapping):
            for key, child_descriptor in properties.items():
                if key in value:
                    errors.extend(
                        _validate_value(
                            f"{path}.{key}",
                            value[key],
                            child_descriptor,
                        )
                    )
            if descriptor.get("additionalProperties") is False:
                unknown = sorted(set(value).difference(properties))
                for key in unknown:
                    errors.append(f"{path}.{key} is not allowed")
        minimum_properties = descriptor.get("minProperties")
        if minimum_properties is not None and len(value) < int(minimum_properties):
            errors.append(f"{path} must not be empty")

    return errors


@dataclass(frozen=True)
class RecordSchema:
    """One strict versioned record contract plus semantic invariants."""

    name: str
    version: str
    fields: Mapping[str, Mapping[str, Any]]
    required: tuple[str, ...]
    example: Mapping[str, Any]
    invariants: tuple[Invariant, ...] = ()
    allow_additional_properties: bool = False

    @property
    def schema_id(self) -> str:
        return f"https://tactilibero.org/schemas/{self.name}-{self.version}"

    def validate(self, payload: Any) -> list[str]:
        if not isinstance(payload, Mapping):
            return [f"{self.name} must be an object"]

        errors: list[str] = []
        for field in self.required:
            if field not in payload:
                errors.append(f"{field} is required")
        if not self.allow_additional_properties:
            for field in sorted(set(payload).difference(self.fields)):
                errors.append(f"{field} is not allowed by {self.name} {self.version}")
        for field, descriptor in self.fields.items():
            if field in payload:
                errors.extend(_validate_value(field, payload[field], descriptor))
        for invariant in self.invariants:
            try:
                errors.extend(str(error) for error in invariant(payload))
            except (KeyError, TypeError, ValueError):
                # Field/type errors are already emitted above; invariants must
                # never turn malformed user input into an import/runtime crash.
                continue
        return errors

    def require_valid(self, payload: Any) -> None:
        errors = self.validate(payload)
        if errors:
            raise ValueError(f"{self.name} {self.version}: " + "; ".join(errors))

    def as_json_schema(self) -> dict[str, Any]:
        return {
            "$schema": JSON_SCHEMA_DIALECT,
            "$id": self.schema_id,
            "title": self.name,
            "type": "object",
            "additionalProperties": self.allow_additional_properties,
            "required": list(self.required),
            "properties": deepcopy(dict(self.fields)),
        }

    def definition_errors(self) -> list[str]:
        errors: list[str] = []
        if not SEMANTIC_VERSION_PATTERN.fullmatch(self.version):
            errors.append(f"{self.name} schema version is not semantic")
        if self.required != tuple(dict.fromkeys(self.required)):
            errors.append(f"{self.name} required fields contain duplicates")
        unknown_required = sorted(set(self.required).difference(self.fields))
        if unknown_required:
            errors.append(f"{self.name} required fields are undefined: {unknown_required}")
        errors.extend(self.validate(deepcopy(self.example)))
        try:
            json.dumps(self.as_json_schema(), sort_keys=True)
        except (TypeError, ValueError) as exc:
            errors.append(f"{self.name} JSON Schema is not serializable: {exc}")
        return errors


def record_schema(
    name: str,
    *,
    fields: Mapping[str, Mapping[str, Any]],
    required: Sequence[str],
    example: Mapping[str, Any],
    invariants: Sequence[Invariant] = (),
    version: str = CONTRACT_SCHEMA_VERSION,
) -> RecordSchema:
    versioned_fields: dict[str, Mapping[str, Any]] = {
        "schema_version": {"type": "string", "const": version},
        **dict(fields),
    }
    versioned_required = tuple(dict.fromkeys(("schema_version", *required)))
    versioned_example = {"schema_version": version, **dict(example)}
    return RecordSchema(
        name=name,
        version=version,
        fields=versioned_fields,
        required=versioned_required,
        example=versioned_example,
        invariants=tuple(invariants),
    )


def string_field(
    *,
    enum: Sequence[str] | None = None,
    format_name: str | None = None,
    nullable: bool = False,
    min_length: int = 1,
) -> dict[str, Any]:
    descriptor: dict[str, Any] = {
        "type": ["string", "null"] if nullable else "string",
    }
    if not nullable:
        descriptor["minLength"] = min_length
    if enum is not None:
        descriptor["enum"] = list(enum)
    if format_name is not None:
        descriptor["format"] = format_name
        patterns = {
            "semver": SEMANTIC_VERSION_PATTERN.pattern,
            "sha256": SHA256_PATTERN.pattern,
            "commit": COMMIT_PATTERN.pattern,
        }
        if format_name in patterns:
            descriptor["pattern"] = patterns[format_name]
    return descriptor


def integer_field(
    *,
    minimum: int | None = None,
    maximum: int | None = None,
    nullable: bool = False,
) -> dict[str, Any]:
    descriptor: dict[str, Any] = {
        "type": ["integer", "null"] if nullable else "integer",
    }
    if minimum is not None:
        descriptor["minimum"] = minimum
    if maximum is not None:
        descriptor["maximum"] = maximum
    return descriptor


def number_field(
    *,
    minimum: float | None = None,
    exclusive_minimum: float | None = None,
    maximum: float | None = None,
) -> dict[str, Any]:
    descriptor: dict[str, Any] = {"type": "number"}
    if minimum is not None:
        descriptor["minimum"] = minimum
    if exclusive_minimum is not None:
        descriptor["exclusiveMinimum"] = exclusive_minimum
    if maximum is not None:
        descriptor["maximum"] = maximum
    return descriptor


def boolean_field() -> dict[str, Any]:
    return {"type": "boolean"}


def object_field(
    *,
    required: Sequence[str] = (),
    properties: Mapping[str, Mapping[str, Any]] | None = None,
    nonempty: bool = False,
    additional_properties: bool = True,
) -> dict[str, Any]:
    descriptor: dict[str, Any] = {
        "type": "object",
        "required": list(required),
    }
    if nonempty:
        descriptor["minProperties"] = 1
    if properties is not None:
        descriptor["properties"] = dict(properties)
        descriptor["additionalProperties"] = additional_properties
    return descriptor


def array_field(
    *,
    items: Mapping[str, Any] | None = None,
    min_items: int = 0,
    max_items: int | None = None,
    unique: bool = False,
) -> dict[str, Any]:
    descriptor: dict[str, Any] = {
        "type": "array",
        "minItems": min_items,
        "uniqueItems": unique,
    }
    if max_items is not None:
        descriptor["maxItems"] = max_items
    if items is not None:
        descriptor["items"] = dict(items)
    return descriptor

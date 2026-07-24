"""Shared manifest, digest, semantic-version, and registration validation."""

from __future__ import annotations

import hashlib
import json
from copy import deepcopy
from dataclasses import dataclass
from typing import Any, Callable, Generic, Mapping, Sequence, TypeVar

from isaac_tactile_libero.schemas.base import (
    RecordSchema,
    SEMANTIC_VERSION_PATTERN,
    SHA256_PATTERN,
)
from isaac_tactile_libero.schemas.plugin import COMMUNITY_PLUGIN_SCHEMA

from .base import Registry


T = TypeVar("T")

PLUGIN_TYPE_BY_REGISTRY_KIND = {
    "robot": "ROBOT",
    "task": "TASK",
    "sensor": "SENSOR",
    "tactile sensor": "SENSOR",
    "expert": "EXPERT",
    "observation modality": "MODALITY",
}
CONTRACT_KEY_BY_REGISTRY_KIND = {
    "tactile sensor": "sensor",
}


@dataclass(frozen=True)
class RegistryContractDefinition:
    registry_kind: str
    component_type: str
    contract_version: str
    required_capabilities: tuple[str, ...]
    community_plugin: bool


REGISTRY_CONTRACTS = {
    "robot": RegistryContractDefinition(
        "robot", "ROBOT", "1.0.0", ("factory", "public_action"), True
    ),
    "task": RegistryContractDefinition(
        "task", "TASK", "1.0.0", ("factory", "task_state"), True
    ),
    "sensor": RegistryContractDefinition(
        "sensor", "SENSOR", "1.0.0", ("observations", "validity_masks"), True
    ),
    "expert": RegistryContractDefinition(
        "expert", "EXPERT", "1.0.0", ("public_action",), True
    ),
    "observation modality": RegistryContractDefinition(
        "observation modality",
        "MODALITY",
        "1.0.0",
        ("observation_fields", "validity_masks"),
        True,
    ),
    "policy": RegistryContractDefinition(
        "policy", "POLICY", "1.0.0", ("inference", "public_action"), False
    ),
    "training algorithm": RegistryContractDefinition(
        "training algorithm",
        "TRAINING_ALGORITHM",
        "1.0.0",
        ("train", "checkpoint"),
        False,
    ),
}


def parse_semantic_version(version: str) -> tuple[int, int, int]:
    """Parse the supported stable semantic-version form."""

    match = SEMANTIC_VERSION_PATTERN.fullmatch(str(version))
    if match is None:
        raise ValueError(f"invalid semantic version: {version!r}")
    return tuple(int(component) for component in match.groups())  # type: ignore[return-value]


def is_semantic_version_compatible(required: str, supported: str) -> bool:
    """Return true when ``supported`` is same-major and not older."""

    try:
        required_parts = parse_semantic_version(required)
        supported_parts = parse_semantic_version(supported)
    except ValueError:
        return False
    return (
        required_parts[0] == supported_parts[0]
        and supported_parts >= required_parts
    )


def canonical_json_bytes(
    value: Any,
    *,
    exclude_fields: Sequence[str] = (),
) -> bytes:
    """Serialize a manifest deterministically without lossy coercion."""

    payload = value
    if isinstance(value, Mapping) and exclude_fields:
        excluded = set(exclude_fields)
        payload = {key: item for key, item in value.items() if key not in excluded}
    return json.dumps(
        payload,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")


def canonical_sha256(
    value: Any,
    *,
    exclude_fields: Sequence[str] = (),
) -> str:
    return hashlib.sha256(
        canonical_json_bytes(value, exclude_fields=exclude_fields)
    ).hexdigest()


def validate_manifest_digest(
    manifest: Mapping[str, Any],
    *,
    digest_field: str,
) -> list[str]:
    supplied = manifest.get(digest_field)
    if not isinstance(supplied, str) or not SHA256_PATTERN.fullmatch(supplied):
        return [f"{digest_field} must be a lowercase 64-character SHA-256 digest"]
    expected = canonical_sha256(manifest, exclude_fields=(digest_field,))
    if supplied != expected:
        return [
            f"{digest_field} mismatch: expected {expected}, observed {supplied}"
        ]
    return []


def validate_versioned_manifest(
    manifest: Mapping[str, Any],
    *,
    schema: RecordSchema,
    digest_field: str | None = None,
) -> list[str]:
    errors = schema.validate(manifest)
    if digest_field is not None and digest_field in manifest:
        errors.extend(validate_manifest_digest(manifest, digest_field=digest_field))
    return errors


def validate_registration_manifest(
    registry_kind: str,
    manifest: Mapping[str, Any],
    *,
    required_contract_version: str,
    required_capabilities: Sequence[str] = (),
) -> list[str]:
    """Validate a community manifest before its factory becomes callable."""

    errors = COMMUNITY_PLUGIN_SCHEMA.validate(manifest)
    contract_key = CONTRACT_KEY_BY_REGISTRY_KIND.get(registry_kind, registry_kind)
    expected_type = PLUGIN_TYPE_BY_REGISTRY_KIND.get(contract_key)
    if expected_type is None:
        errors.append(f"unknown registry contract: {registry_kind}")
    elif manifest.get("plugin_type") != expected_type:
        errors.append(
            f"{registry_kind} registry requires plugin_type {expected_type}"
        )

    supported = manifest.get("supported_contract_versions")
    if not isinstance(supported, list) or not any(
        isinstance(version, str)
        and is_semantic_version_compatible(required_contract_version, version)
        for version in supported
    ):
        errors.append(
            f"plugin has no compatible contract version for "
            f"{required_contract_version}"
        )

    capabilities = manifest.get("capabilities")
    for capability in required_capabilities:
        if (
            not isinstance(capabilities, Mapping)
            or capabilities.get(capability) is not True
        ):
            errors.append(f"plugin is missing required capability {capability}")
    return errors


def validate_component_registration_manifest(
    registry_kind: str,
    manifest: Mapping[str, Any],
) -> list[str]:
    """Validate non-community policy/training component manifests."""

    contract = REGISTRY_CONTRACTS.get(registry_kind)
    if contract is None:
        return [f"unknown registry contract: {registry_kind}"]
    errors: list[str] = []
    required = (
        "schema_version",
        "component_id",
        "component_type",
        "version",
        "entry_point",
        "supported_contract_versions",
        "capabilities",
        "source_and_license",
        "test_report_sha256",
    )
    for field in required:
        if field not in manifest:
            errors.append(f"{field} is required")
    unknown = sorted(set(manifest).difference(required))
    for field in unknown:
        errors.append(f"{field} is not allowed")
    if manifest.get("schema_version") != "1.0.0":
        errors.append("schema_version must equal 1.0.0")
    for field in ("component_id", "entry_point"):
        if not isinstance(manifest.get(field), str) or not manifest[field]:
            errors.append(f"{field} must be a non-empty string")
    if manifest.get("component_type") != contract.component_type:
        errors.append(
            f"{registry_kind} requires component_type {contract.component_type}"
        )
    try:
        parse_semantic_version(str(manifest.get("version", "")))
    except ValueError:
        errors.append("version must be a semantic version")
    supported = manifest.get("supported_contract_versions")
    if (
        not isinstance(supported, list)
        or not supported
        or any(
            not isinstance(version, str)
            or not SEMANTIC_VERSION_PATTERN.fullmatch(version)
            for version in supported
        )
        or (
            all(isinstance(version, str) for version in supported)
            and len(supported) != len(set(supported))
        )
    ):
        errors.append("supported_contract_versions must contain unique semantic versions")
    if not isinstance(supported, list) or not any(
        isinstance(version, str)
        and is_semantic_version_compatible(contract.contract_version, version)
        for version in supported
    ):
        errors.append(
            f"component has no compatible contract version for "
            f"{contract.contract_version}"
        )
    capabilities = manifest.get("capabilities")
    for capability in contract.required_capabilities:
        if (
            not isinstance(capabilities, Mapping)
            or capabilities.get(capability) is not True
        ):
            errors.append(f"component is missing required capability {capability}")
    source = manifest.get("source_and_license")
    if (
        not isinstance(source, Mapping)
        or not isinstance(source.get("source"), str)
        or not source.get("source")
        or not isinstance(source.get("license"), str)
        or not source.get("license")
    ):
        errors.append("source_and_license requires source and license")
    digest = manifest.get("test_report_sha256")
    if not isinstance(digest, str) or not SHA256_PATTERN.fullmatch(digest):
        errors.append("test_report_sha256 must be a lowercase 64-character SHA-256 digest")
    return errors


def validate_registry_registration(
    registry_kind: str,
    manifest: Mapping[str, Any],
) -> list[str]:
    """Apply the canonical contract for one of the seven registry kinds."""

    contract_key = CONTRACT_KEY_BY_REGISTRY_KIND.get(registry_kind, registry_kind)
    contract = REGISTRY_CONTRACTS.get(contract_key)
    if contract is None:
        return [f"unknown registry contract: {registry_kind}"]
    if contract.community_plugin:
        return validate_registration_manifest(
            contract_key,
            manifest,
            required_contract_version=contract.contract_version,
            required_capabilities=contract.required_capabilities,
        )
    return validate_component_registration_manifest(contract_key, manifest)


def validate_registry_contract_definitions() -> list[str]:
    """Audit the immutable central definitions independently of live entries."""

    errors: list[str] = []
    for key, contract in REGISTRY_CONTRACTS.items():
        if contract.registry_kind != key:
            errors.append(f"{key} registry_kind mismatch")
        try:
            parse_semantic_version(contract.contract_version)
        except ValueError as exc:
            errors.append(str(exc))
        if not contract.required_capabilities:
            errors.append(f"{key} must declare required capabilities")
        if len(contract.required_capabilities) != len(
            set(contract.required_capabilities)
        ):
            errors.append(f"{key} required capabilities contain duplicates")
    return errors


class ContractRegistry(Registry[T], Generic[T]):
    """Registry that validates version/capabilities before registration."""

    def __init__(
        self,
        kind: str,
        *,
        contract_version: str,
        required_capabilities: Sequence[str] = (),
    ):
        parse_semantic_version(contract_version)
        contract_key = CONTRACT_KEY_BY_REGISTRY_KIND.get(kind, kind)
        contract = REGISTRY_CONTRACTS.get(contract_key)
        if contract is None:
            raise ValueError(f"unknown registry contract: {kind}")
        if contract_version != contract.contract_version:
            raise ValueError(
                f"{kind} contract version must equal {contract.contract_version}"
            )
        if tuple(required_capabilities) != contract.required_capabilities:
            raise ValueError(
                f"{kind} required capabilities must equal "
                f"{contract.required_capabilities}"
            )
        super().__init__(kind)
        self.contract_version = contract_version
        self.required_capabilities = tuple(required_capabilities)
        self._manifests: dict[str, dict[str, Any]] = {}

    def register_contract(
        self,
        name: str,
        cls: Callable[..., T],
        *,
        manifest: Mapping[str, Any],
        **metadata: Any,
    ) -> None:
        errors = validate_registry_registration(self.kind, manifest)
        if errors:
            raise ValueError(
                f"invalid {self.kind} registration {name!r}: " + "; ".join(errors)
            )
        frozen_manifest = deepcopy(dict(manifest))
        Registry.register(
            self,
            name,
            cls,
            contract_version=self.contract_version,
            manifest=deepcopy(frozen_manifest),
            **metadata,
        )
        self._manifests[name] = frozen_manifest

    def manifest(self, name: str) -> dict[str, Any]:
        self.get(name)
        return deepcopy(self._manifests[name])

    def register(self, name: str, cls: Callable[..., T], **metadata: Any) -> None:
        raise TypeError(
            f"{self.kind} uses register_contract(..., manifest=...); "
            "unvalidated register() is disabled"
        )

    def clear(self) -> None:
        super().clear()
        self._manifests.clear()

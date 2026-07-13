import json
from pathlib import Path


CONTRACTS = Path("specs/001-benchmark-reconstruction/contracts")
RUNTIME = Path("isaac_tactile_libero/schemas")


def test_gate_and_manifest_runtime_schemas_match_canonical_contracts() -> None:
    for name in (
        "compatibility-report.schema.json",
        "evidence-manifest.schema.json",
        "gate-status.schema.json",
    ):
        assert (RUNTIME / name).read_bytes() == (CONTRACTS / name).read_bytes()


def test_compatibility_schema_keeps_formal_gate_status_contract_unchanged() -> None:
    schema = json.loads((RUNTIME / "compatibility-report.schema.json").read_text(encoding="utf-8"))
    status = schema["properties"]["status"]["enum"]
    assert status == ["PASS_SMOKE", "BLOCKED"]
    assert schema["properties"]["claim_class"]["const"] == "runtime_smoke"
    assert "BLOCKED_DRIVER" not in json.dumps(schema)
    assert "PASS_UNSUPPORTED_RUNTIME" not in json.dumps(schema)
    assert "PASS_PHYSICAL" not in json.dumps(schema)

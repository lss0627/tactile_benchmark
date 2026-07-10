import runpy


def test_review_gate_rejects_smoke_as_g0_benchmark_pass() -> None:
    module = runpy.run_path("scripts/review_gate.py")
    errors = module["review_manifest"](
        {
            "gate_id": "G0",
            "status": "PASS_SMOKE",
            "claim_class": "runtime_smoke",
        },
        expected_gate="G0",
        validate_schema=False,
        validate_freshness=False,
    )["errors"]
    assert "G0 requires PASS_BENCHMARK" in errors

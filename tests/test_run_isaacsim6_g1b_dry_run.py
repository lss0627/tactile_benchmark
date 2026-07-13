from argparse import Namespace
import runpy


def test_g1b_runner_dry_run_contract_is_non_benchmark() -> None:
    module = runpy.run_path("scripts/run_isaacsim6_g1b.py")
    report = module["run_dry"](Namespace(cycles=100, steps=500))
    assert report["ok"] is True
    assert report["dry_run"] is True
    assert report["reset_cycles_requested"] == 100
    assert report["rollout_steps_requested"] == 500
    assert report["claim_class"] == "runtime_smoke"
    assert report["benchmark_result"] is False
    assert report["gpu_contact_blocker"] == "GPU_CONTACT_NATIVE_INSTABILITY"

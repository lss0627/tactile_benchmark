from pathlib import Path

import yaml


def test_isaacsim6_fr3_backend_pins_cpu_contact_and_acceptance_windows() -> None:
    path = Path("configs/backend/isaacsim_fr3_press_button.yaml")
    assert path.exists()
    cfg = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert cfg["simulator_version"] == "6.0.1"
    assert cfg["python_version"] == "3.12"
    assert cfg["physics_device"] == "cpu"
    assert cfg["rendering_device"] == "cuda:0"
    assert cfg["gpu_dynamics"] is False
    assert cfg["gpu_contact_status"] == "BLOCKED_UNVALIDATED"
    assert cfg["sensor_ready_timeout_steps"] == 5
    assert cfg["contact_onset_tolerance_steps"] == 2
    assert cfg["contact_release_timeout_steps"] == 5
    assert cfg["contact_stable_steps"] == 3
    assert cfg["force_zero_epsilon"] == 1.0e-4
    assert cfg["claim_class"] == "runtime_smoke"
    assert cfg["benchmark_result"] is False

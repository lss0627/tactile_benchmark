from __future__ import annotations

import builtins
import hashlib
import importlib.util
import inspect
from pathlib import Path
import sys
from typing import Any

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
TARGET = ROOT / "isaac_tactile_libero/tasks/press_button_mechanism.py"
PHYSICAL_CONFIG = ROOT / "configs/tasks/press_button_physical.yaml"
HISTORICAL_ATTEMPT_02_TASK_CONFIG_SHA256 = (
    "507c1684d45cf17dda41bbcd690e03850c55a8a4444edc076f47e9bd6eb8008a"
)


def _target():
    assert TARGET.is_file(), "T055 missing movable PressButton mechanism module"
    spec = importlib.util.spec_from_file_location("press_button_mechanism_t055", TARGET)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _config(module):
    return module.PressButtonMechanismConfig(
        joint_name="button_joint",
        rest_position_m=0.0,
        travel_limit_m=0.012,
        pressed_threshold_m=0.009,
        release_threshold_m=0.001,
        reset_tolerance_m=0.0005,
        reset_noise_m=0.0002,
        collision_enabled=True,
    )


def _formal_payload() -> dict[str, object]:
    payload = yaml.safe_load(PHYSICAL_CONFIG.read_text(encoding="utf-8"))
    mechanism = payload["mechanism"]
    mechanism.update(
        {
            "mechanism_version": "1.1.0",
            "base_orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
            "geometry": {
                "frame": "mechanism_root",
                "units": "m",
                "button": {
                    "primitive": "capped_cylinder",
                    "center_local_m": [0.0, 0.0, 0.0],
                    "axis_token": "Z",
                    "radius_m": 0.035,
                    "half_height_m": 0.009,
                },
                "housing": {
                    "primitive": "oriented_box",
                    "center_local_m": [0.0, 0.0, -0.025],
                    "half_extents_m": [0.045, 0.045, 0.010],
                },
            },
            "contact_exclusion": {
                "schema_version": "1.0.0",
                "subject": "fr3_hand_tcp_point",
                "obstacle_ids": ["button", "housing"],
                "required_clearance_m": 0.005,
                "distance_metric": "conservative_closed_solid_clearance_v1",
                "route_validation": "continuous_line_segment",
                "boundary_policy": "equality_allowed",
            },
        }
    )
    return payload


def _write_payload(tmp_path: Path, payload: dict[str, object]) -> Path:
    path = tmp_path / "press_button.yaml"
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    return path


def _assert_exact_failure(call, code: str) -> None:
    with pytest.raises(Exception) as caught:
        call()
    assert getattr(caught.value, "code", None) == code
    assert isinstance(getattr(caught.value, "message", None), str)
    assert caught.value.message.strip()


class RecordingPressButtonStageAuthoringAdapter:
    def __init__(self) -> None:
        self.calls: list[tuple[Any, ...]] = []

    def author_root(
        self,
        *,
        root_path: str,
        position_m: tuple[float, float, float],
        orientation_xyzw: tuple[float, float, float, float],
    ) -> None:
        self.calls.append(("root", position_m, orientation_xyzw))

    def author_capped_cylinder(
        self,
        *,
        path: str,
        center_local_m: tuple[float, float, float],
        axis_token: str,
        radius_m: float,
        height_m: float,
    ) -> None:
        self.calls.append(
            ("button", center_local_m, axis_token, radius_m, height_m)
        )

    def author_oriented_box(
        self,
        *,
        path: str,
        center_local_m: tuple[float, float, float],
        half_extents_m: tuple[float, float, float],
    ) -> None:
        self.calls.append(("housing", center_local_m, half_extents_m))


def _require_stage_authoring_capability(module) -> None:
    protocol = getattr(module, "PressButtonStageAuthoringAdapter", None)
    real_adapter = getattr(module, "UsdPressButtonStageAuthoringAdapter", None)
    assert protocol is not None, "missing approved PressButton stage-authoring capability"
    assert real_adapter is not None, "missing approved lazy USD stage-authoring adapter"
    assert "authoring_adapter" in inspect.signature(
        module.PressButtonMechanism.build_stage
    ).parameters


def test_mechanism_declares_real_joint_travel_limits_and_collision() -> None:
    module = _target()
    contract = module.PressButtonMechanism(_config(module)).scene_contract()

    assert contract["joint_type"] == "prismatic"
    assert contract["joint_name"] == "button_joint"
    assert contract["lower_limit_m"] == pytest.approx(0.0)
    assert contract["upper_limit_m"] == pytest.approx(0.012)
    assert contract["collision_enabled"] is True
    assert contract["movable"] is True
    assert contract["body0_path"].endswith("/Housing")
    assert contract["body1_path"].endswith("/Button")
    assert contract["body0_kinematic"] is True
    assert contract["local_pos0_m"] == [0.0, 0.0, 0.025]
    assert contract["local_pos1_m"] == [0.0, 0.0, 0.0]


@pytest.mark.parametrize(
    ("position_m", "rest", "pressed", "released", "reset"),
    [
        (0.0, True, False, True, True),
        (0.0008, False, False, True, False),
        (0.009, False, True, False, False),
        (0.012, False, True, False, False),
    ],
)
def test_observed_joint_position_defines_button_state(
    position_m: float,
    rest: bool,
    pressed: bool,
    released: bool,
    reset: bool,
) -> None:
    module = _target()
    state = module.PressButtonMechanism(_config(module)).observe_joint_position(position_m)

    assert state.travel_m == pytest.approx(position_m)
    assert state.at_rest is rest
    assert state.pressed is pressed
    assert state.released is released
    assert state.reset is reset


def test_seeded_reset_is_deterministic_and_inside_reset_tolerance() -> None:
    module = _target()
    mechanism = module.PressButtonMechanism(_config(module))

    first = mechanism.sample_reset_position(seed=1701)
    second = mechanism.sample_reset_position(seed=1701)

    assert first == second
    assert abs(first - mechanism.config.rest_position_m) <= mechanism.config.reset_tolerance_m


def test_vertical_button_return_drive_is_preloaded_against_gravity() -> None:
    module = _target()
    config = module.load_press_button_mechanism_config(
        PHYSICAL_CONFIG
    )
    contract = module.PressButtonMechanism(config).scene_contract()

    expected_gravity_load_n = config.button_mass_kg * 9.81
    assert config.return_preload_n >= expected_gravity_load_n
    assert contract["gravity_load_along_travel_n"] == pytest.approx(expected_gravity_load_n)
    assert contract["drive_target_position_m"] < config.lower_limit_m


def test_observation_outside_joint_limits_is_rejected() -> None:
    module = _target()
    mechanism = module.PressButtonMechanism(_config(module))

    with pytest.raises(ValueError, match="joint travel"):
        mechanism.observe_joint_position(0.013)


def test_mechanism_stage_builder_is_injected_before_fr3_runtime_starts() -> None:
    from isaac_tactile_libero.robots.fr3_differential_ik import FR3DifferentialIKRuntime

    def stage_builder(_stage) -> None:
        return None

    runtime = FR3DifferentialIKRuntime(
        simulation_app=object(),
        fr3_usd_path="/external/fr3.usd",
        stage_builder=stage_builder,
    )

    controller = runtime.ik_runtime.ee_controller.controller
    assert controller._stage_builder is stage_builder


def test_legacy_mechanism_1_0_is_state_only_and_ineligible_for_formal_build() -> None:
    module = _target()
    config = _config(module)

    assert getattr(config, "geometry_contract_available", None) is False
    assert getattr(config, "tcp_route_exclusion_qualified", None) is False
    assert getattr(config, "benchmark_cap_eligible", None) is False
    assert getattr(config, "runtime_stage_build_eligible", None) is False
    assert getattr(config, "route_validation_input_eligible", None) is False


def test_legacy_mechanism_formal_build_fails_with_required_geometry_code() -> None:
    module = _target()
    source = inspect.getsource(module.PressButtonMechanism.build_stage)

    assert "G1_PRESS_BUTTON_FORMAL_GEOMETRY_REQUIRED" in source
    assert "runtime_stage_build_eligible" in source


def test_tracked_physical_config_loads_strict_formal_mechanism_1_1() -> None:
    module = _target()
    config = module.load_press_button_mechanism_config(PHYSICAL_CONFIG)

    assert config.mechanism_version == "1.1.0"
    assert config.base_position_m == (0.55, 0.0, 0.47)
    assert config.base_orientation_xyzw == (0.0, 0.0, 0.0, 1.0)
    assert config.geometry_contract is not None
    assert config.geometry_contract_available is True
    assert config.runtime_stage_build_eligible is True
    assert config.route_validation_input_eligible is True


def test_formal_mechanism_schema_never_grants_benchmark_cap_eligibility() -> None:
    module = _target()
    config = module.load_press_button_mechanism_config(PHYSICAL_CONFIG)
    mechanism = module.PressButtonMechanism(config)

    assert getattr(config, "benchmark_cap_eligible", False) is False
    assert getattr(mechanism, "benchmark_cap_eligible", False) is False
    contract = getattr(config, "geometry_contract", None)
    assert contract is not None
    assert getattr(contract, "benchmark_cap_eligible", False) is False


@pytest.mark.parametrize(
    "missing_path",
    [
        "base_orientation_xyzw",
        "geometry",
        "geometry.button",
        "geometry.housing",
        "contact_exclusion",
    ],
)
def test_formal_mechanism_1_1_rejects_missing_geometry_fields_without_fallback(
    tmp_path: Path, missing_path: str
) -> None:
    module = _target()
    payload = _formal_payload()
    mechanism = payload["mechanism"]
    if "." in missing_path:
        parent, child = missing_path.split(".")
        mechanism[parent].pop(child)
    else:
        mechanism.pop(missing_path)
    path = _write_payload(tmp_path, payload)

    _assert_exact_failure(
        lambda: module.load_press_button_mechanism_config(path),
        "G1_C1_CONTACT_EXCLUSION_SCHEMA_INVALID",
    )


@pytest.mark.parametrize(
    "orientation",
    [[0.0, 0.0, 1.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, float("nan"), 1.0]],
)
def test_formal_mechanism_1_1_rejects_invalid_root_orientation(
    tmp_path: Path, orientation: list[float]
) -> None:
    module = _target()
    payload = _formal_payload()
    payload["mechanism"]["base_orientation_xyzw"] = orientation
    path = _write_payload(tmp_path, payload)

    _assert_exact_failure(
        lambda: module.load_press_button_mechanism_config(path),
        "G1_C1_CONTACT_EXCLUSION_GEOMETRY_INVALID",
    )


@pytest.mark.parametrize("mechanism_version", ["1.1.1", "1.2.0", "2.0.0", "legacy"])
def test_unknown_mechanism_versions_fail_closed(
    tmp_path: Path, mechanism_version: str
) -> None:
    module = _target()
    payload = _formal_payload()
    payload["mechanism"]["mechanism_version"] = mechanism_version
    path = _write_payload(tmp_path, payload)

    _assert_exact_failure(
        lambda: module.load_press_button_mechanism_config(path),
        "G1_C1_CONTACT_EXCLUSION_SCHEMA_INVALID",
    )


@pytest.mark.parametrize("mechanism_version", ["1.0.0", "1.0.1", "1.0.9"])
def test_explicit_legacy_1_0_x_remains_state_only(
    mechanism_version: str,
) -> None:
    module = _target()
    payload = _formal_payload()["mechanism"]
    payload["mechanism_version"] = mechanism_version
    for field in ("base_orientation_xyzw", "geometry", "contact_exclusion"):
        payload.pop(field)
    config = module.PressButtonMechanismConfig.from_mapping(payload)
    state = module.PressButtonMechanism(config).observe_joint_position(0.0)

    assert state.reset is True
    assert state.released is True
    assert getattr(config, "geometry_contract_available", None) is False
    assert getattr(config, "tcp_route_exclusion_qualified", None) is False
    assert getattr(config, "benchmark_cap_eligible", None) is False
    assert getattr(config, "runtime_stage_build_eligible", None) is False
    assert getattr(config, "route_validation_input_eligible", None) is False


def test_legacy_formal_build_blocks_before_pxr_import(monkeypatch: pytest.MonkeyPatch) -> None:
    module = _target()
    mechanism = module.PressButtonMechanism(_config(module))
    real_import = builtins.__import__

    def guarded_import(name, *args, **kwargs):
        assert not name.startswith("pxr"), "legacy formal build attempted a pxr import"
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", guarded_import)
    _assert_exact_failure(
        lambda: mechanism.build_stage(object()),
        "G1_PRESS_BUTTON_FORMAL_GEOMETRY_REQUIRED",
    )


def test_tracked_config_digest_changes_from_attempt_02_without_inventing_provenance() -> None:
    module = _target()
    current_digest = hashlib.sha256(PHYSICAL_CONFIG.read_bytes()).hexdigest()
    historical_provenance = {
        "task_config_sha256": HISTORICAL_ATTEMPT_02_TASK_CONFIG_SHA256,
    }

    assert current_digest != HISTORICAL_ATTEMPT_02_TASK_CONFIG_SHA256
    assert "task_card_sha256" not in historical_provenance
    assert "geometry_sha256" not in historical_provenance
    config = module.load_press_button_mechanism_config(PHYSICAL_CONFIG)
    assert config.geometry_contract.task_config_sha256 == current_digest


def test_stage_authoring_adapter_records_root_housing_button_order_and_exact_values() -> None:
    module = _target()
    _require_stage_authoring_capability(module)
    config = module.load_press_button_mechanism_config(PHYSICAL_CONFIG)
    adapter = RecordingPressButtonStageAuthoringAdapter()

    module.PressButtonMechanism(config).build_stage(
        object(), authoring_adapter=adapter
    )

    assert adapter.calls[:3] == [
        ("root", (0.55, 0.0, 0.47), (0.0, 0.0, 0.0, 1.0)),
        ("housing", (0.0, 0.0, -0.025), (0.045, 0.045, 0.010)),
        ("button", (0.0, 0.0, 0.0), "Z", 0.035, 0.018),
    ]


def test_stage_authoring_uses_the_loaded_contract_and_matching_digests() -> None:
    module = _target()
    _require_stage_authoring_capability(module)
    config = module.load_press_button_mechanism_config(PHYSICAL_CONFIG)
    contract = config.geometry_contract
    source = inspect.getsource(module.PressButtonMechanism.build_stage)

    assert "cfg.geometry_contract" in source
    assert "parse_press_button_geometry_contract" not in source
    scene = module.PressButtonMechanism(config).scene_contract()
    assert scene["geometry_sha256"] == contract.geometry_sha256
    assert (
        scene["world_from_mechanism_root_sha256"]
        == contract.world_from_mechanism_root_sha256
    )
    assert config.geometry_contract is contract


def test_stage_authoring_module_and_fake_path_are_import_safe() -> None:
    forbidden = ("pxr", "omni", "isaacsim")
    before = {name for name in sys.modules if name.split(".", 1)[0] in forbidden}
    module = _target()
    _require_stage_authoring_capability(module)
    adapter = RecordingPressButtonStageAuthoringAdapter()

    module.PressButtonMechanism(
        module.load_press_button_mechanism_config(PHYSICAL_CONFIG)
    ).build_stage(object(), authoring_adapter=adapter)

    after = {name for name in sys.modules if name.split(".", 1)[0] in forbidden}
    assert after == before


def test_real_usd_authoring_adapter_keeps_pxr_import_lazy_and_uses_full_dimensions() -> None:
    module = _target()
    _require_stage_authoring_capability(module)
    adapter_type = module.UsdPressButtonStageAuthoringAdapter
    source = inspect.getsource(adapter_type)

    assert "CreateAxisAttr(axis_token)" in source
    assert "2.0 *" in source
    assert "orientation_xyzw" in source
    assert "half_extents_m" in source
    assert "from pxr import" in source


def test_formal_stage_builder_contains_no_geometry_authority_literals() -> None:
    module = _target()
    _require_stage_authoring_capability(module)
    source = inspect.getsource(module.PressButtonMechanism.build_stage)

    for literal in ("0.035", "0.018", "0.09", "0.025"):
        assert literal not in source


def test_adapter_injection_cannot_skip_complete_stage_physics_semantics() -> None:
    module = _target()
    _require_stage_authoring_capability(module)
    source = inspect.getsource(module.PressButtonMechanism.build_stage)
    required_stage_semantics = (
        "CollisionAPI",
        "RigidBodyAPI",
        "MassAPI",
        "PrismaticJoint",
        "CreateBody0Rel",
        "CreateBody1Rel",
        "CreateLowerLimitAttr",
        "CreateUpperLimitAttr",
        "CreateTargetPositionAttr",
        "CreateStiffnessAttr",
        "CreateDampingAttr",
    )

    assert all(marker in source for marker in required_stage_semantics)
    assert "if fake" not in source
    assert "type(authoring_adapter)" not in source
    assert "authoring_adapter.__class__" not in source

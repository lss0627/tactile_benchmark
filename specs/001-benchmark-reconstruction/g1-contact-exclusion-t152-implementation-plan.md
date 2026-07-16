# G1 Contact Exclusion and T152 Recovery Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> superpowers:executing-plans to implement this plan task-by-task. Every
> physical/runtime execution remains separately approval-gated.

**Goal:** Migrate PressButton to the versioned shared analytic geometry contract,
complete the real pose-conditioned T152 CLI integration, and prepare a fresh C2a
refresh without running Isaac Sim during implementation.

**Architecture:** The mechanism YAML is the only geometry/root-pose authority.
An import-safe analytic geometry module validates continuous TCP segments against
the declared capped cylinder and OBB. The real runner consumes verified C2a
evidence, derives six routes, authors the selected pose pre-Play, executes the
multiclass plan, and writes immutable preliminary evidence.

**Tech Stack:** Python 3.12, dataclasses, NumPy, PyYAML, USD/PhysX lazy runtime
adapters, pytest, Spec Kit evidence manifests.

---

## Execution contract

The approved behavior baseline is the existing 84 T152 assertion REDs plus four
GREEN controls. Actual execution starts only from a clean HEAD that already
contains the final human-reviewed version of this plan. Task 1 dynamically
records the local branch, tracking ref, live origin, and Draft PR #2 head SHAs;
all four and `git rev-parse HEAD` must be identical. The plan never embeds its
own future revision SHA. The PR must be OPEN and Draft with base `main`. T150
must be `[x]`; T151, T152, and T070 must remain `[ ]`; and attempt-04 must remain
`ATTEMPT_04_PROHIBITED`.

No task in this plan permits Isaac Sim. No task permits C2a, attempt-04, or a
PressButton episode. The sole physical command in Task 12 is prepared text for a
later, separately approved, one-run operation and must not be invoked while
executing this plan.

The invariant values are:

- Command matrix: `0.0`, `0.00025`, `0.00035`, `0.00040`, `0.00045` m.
- Exact observed-action hard limit: `0.0005` m.
- TCP static contact-exclusion clearance: exactly `0.005` m.
- Static scope: `TCP_POINT_VS_DECLARED_MECHANISM_SOLIDS` only.
- A static pass may set `tcp_route_exclusion_qualified=true`, but always records
  `full_robot_static_collision_exclusion_qualified=false`.
- Full-robot runtime safety: per-action CPU Contact, collision, penetration, and
  post-action checks remain mandatory.
- Force truth: `force_vector_valid=false`, `wrench_valid=false`, and
  `raw_impulse_used_as_force=false`.
- Physics: `physics_device=cpu`, `broadphase_type=MBP`, and GPU dynamics
  disabled.

Every real runtime sample must still prove
`contact_valid=true`, `in_contact=false`, `raw_contact_count=0`,
`collision_report_valid=true`, `unsafe_collision=false`,
`penetration_provenance_valid=true`, and `post_abort_actuation_count=0`; static
analytic records cannot synthesize those values.

The first commit that changes geometry/config/parser behavior activates
`CURRENT_C2A_REFRESH_REQUIRED_AFTER_GEOMETRY_SCHEMA_CHANGE`. From that commit
onward, attempt-02 is historical preliminary evidence only and cannot satisfy a
current-input check for T152, T151, or attempt-04.

## Task 1: Freeze the baseline and create the RED migration manifest

**Files**

- Add: `tests/fixtures/g1_t152_baseline_inventory.json`
- Add: `tests/fixtures/g1_t152_red_migration_manifest.json`
- Add: `tests/test_g1_t152_red_migration_manifest.py`
- Add: `tests/run_g1_node_inventory.py`
- Read only: `configs/repository/intentional-future-red-nodeids.txt`
- Read only: `tests/test_g1_pose_conditioned_tracking_cli.py`

**Public test contract**

Add these node IDs:

- `tests/test_g1_t152_red_migration_manifest.py::test_t152_baseline_inventory_distinguishes_behavior_source_and_execution_start_commits`
- `tests/test_g1_t152_red_migration_manifest.py::test_t152_migration_manifest_maps_every_retired_node_exactly_once`
- `tests/test_g1_t152_red_migration_manifest.py::test_t152_migration_manifest_preserves_each_safety_behavior`
- `tests/test_g1_t152_red_migration_manifest.py::test_t152_inventory_keeps_future_red_separate`

The inventory JSON separately records:

- `behavior_source_commit`: the commit that last changed the approved T152 test
  behavior, obtained with
  `git log -1 --format=%H -- tests/test_g1_pose_conditioned_tracking_cli.py`;
- `execution_start_commit`: the clean, final-plan HEAD obtained with
  `git rev-parse HEAD` at Task 1 preflight.

It also records every node ID, outcome, and classification for 84 T152 expected
REDs, four T152 GREEN controls, 748 original GREEN nodes, 125 intentional
future-RED nodes, the four exact-hard-limit nodes, and the deprecated API scan
result. A SHA-256 covers each sorted node-ID list; outcome order is retained
separately so accidental reordering is visible. The executor verifies that the
T152 test blob at `behavior_source_commit` is byte-identical to the blob at
`execution_start_commit`; otherwise the approved behavior inventory is stale.
Frozen node IDs are stored under
`selections.<selection_name>.node_ids`, including separate
`selections.original_green.node_ids` and
`selections.t152_green_controls.node_ids` arrays. Task 11 may read those arrays
for historical classification, but it may not add later nodes to them.

`tests/run_g1_node_inventory.py` is a test-side command for immutable named
selections that already exist at Task 1: `t152_expected_red`,
`t152_green_controls`, `original_green`, `intentional_future_red`, and
`exact_hard_limit`. It accepts `--inventory`, one of those exact selections,
expected pass/fail outcome, and optional expected classification counts. It
executes exactly the frozen node IDs through pytest, parses the JUnit outcome,
and rejects any missing, extra, reordered, or misclassified node. It is not
imported by production code. It must not expose a selection that purports to
contain GREEN node IDs introduced later by Tasks 2–10; dynamic current-GREEN
verification belongs exclusively to Task 11.

The migration manifest has one row per sphere-specific node expansion:

```json
{
  "old_node_id": "tests/test_g1_pose_conditioned_tracking_cli.py::test_t152_route_builder_derives_all_six_records_from_pose_geometry_and_current_inputs",
  "replacement_node_ids": [
    "tests/test_g1_pose_conditioned_tracking_cli.py::test_t152_route_builder_derives_all_six_records_from_declared_solids"
  ],
  "retained_behavior": [
    "routes_are_derived_from_current_inputs",
    "six_canonical_classes_are_complete",
    "caller_validity_flags_are_not_authoritative"
  ],
  "replacement_reason": "The approved authority is a capped cylinder plus an OBB, not a single sphere."
}
```

Required mapping rows are:

| Old expansion | Replacement | Retained behavior |
|---|---|---|
| `test_t152_route_builder_derives_all_six_records_from_pose_geometry_and_current_inputs` | `test_t152_route_builder_derives_all_six_records_from_declared_solids` | derived six-route construction and digest binding |
| `test_t152_route_derivation_changes_digest_or_blocks_when_geometry_changes[selected_pose]` | same parameter on the replacement node | pose changes alter records/digest or block |
| `...[task_geometry]` | `...[mechanism_root]` and `...[declared_solids]` | authoritative task geometry mutation is never ignored |
| `...[workspace]` | same parameter on the replacement node | workspace changes alter records/digest or block |
| `...[contact_exclusion]` | `...[contact_exclusion_policy]` | clearance/policy changes alter records/digest or block |
| `test_t152_route_derivation_ignores_caller_claimed_true_flags[workspace]` | same parameter on replacement node | caller workspace booleans are ignored |
| `...[contact_exclusion]` | same parameter on replacement node | caller contact-exclusion booleans are ignored |

Splitting one old expansion into multiple replacement nodes is allowed only when
all replacements name the same retained behavior. An old node with no mapping,
a duplicate old mapping, or a replacement that is neither collected nor RED at
the schema-correction checkpoint is an immediate stop.

**Steps**

- [ ] Capture `behavior_source_commit`, `execution_start_commit`, branch and PR
  metadata, task states, and prohibition state in the inventory metadata.
- [ ] Fail before collection unless local HEAD, local branch SHA, tracking SHA,
  live origin SHA, and PR head SHA all equal `execution_start_commit` and the
  worktree is clean.
- [ ] Run the existing T152 file once and record exactly `84 failed, 4 passed`,
  with zero errors, skips, collection failures, or Isaac imports.
- [ ] Run the original GREEN selection with both the T152 file and all 125
  intentional future-RED nodes excluded; record exactly 748 passing node IDs.
- [ ] Run the 125 intentional future-RED node IDs directly and record every node
  and failure classification: C2 78, C3 29, freshness 10, Task 9 8.
- [ ] Run the four hard-limit nodes and record `4 passed`.
- [ ] Run the deprecated Isaac API scan and record `0 errors, 0 warnings`.
- [ ] Add the two JSON manifests and their schema/coverage tests without touching
  production code.
- [ ] Rerun only the four manifest tests; all four must pass.

**Commands and expected results**

```bash
EXECUTION_START_COMMIT=$(git rev-parse HEAD)
BEHAVIOR_SOURCE_COMMIT=$(
  git log -1 --format=%H -- tests/test_g1_pose_conditioned_tracking_cli.py
)
LOCAL_BRANCH_SHA=$(git rev-parse refs/heads/codex/g1-press-button-safety)
TRACKING_SHA=$(git rev-parse @{u})
LIVE_ORIGIN_SHA=$(
  git ls-remote --heads origin codex/g1-press-button-safety | awk '{print $1}'
)
PR_HEAD_SHA=$(gh api repos/lss0627/tactile_benchmark/pulls/2 --jq .head.sha)
test -z "$(git status --porcelain)"
test "$EXECUTION_START_COMMIT" = "$LOCAL_BRANCH_SHA"
test "$EXECUTION_START_COMMIT" = "$TRACKING_SHA"
test "$EXECUTION_START_COMMIT" = "$LIVE_ORIGIN_SHA"
test "$EXECUTION_START_COMMIT" = "$PR_HEAD_SHA"
test "$(git rev-parse "$BEHAVIOR_SOURCE_COMMIT:tests/test_g1_pose_conditioned_tracking_cli.py")" = \
     "$(git rev-parse "$EXECUTION_START_COMMIT:tests/test_g1_pose_conditioned_tracking_cli.py")"
# Expected: every command exits 0; both commit identities are written to the inventory.

python -m pytest -q tests/test_g1_pose_conditioned_tracking_cli.py \
  --junitxml=/tmp/g1-t152-start.xml
# Expected: 84 failed, 4 passed; every failure is an approved assertion RED.

mapfile -t FUTURE_RED < configs/repository/intentional-future-red-nodeids.txt
DESELECT=()
for node in "${FUTURE_RED[@]}"; do DESELECT+=(--deselect "$node"); done
python -m pytest -q --ignore=tests/test_g1_pose_conditioned_tracking_cli.py \
  "${DESELECT[@]}" --junitxml=/tmp/g1-green-start.xml
# Expected: exactly 748 passed.

python -m pytest -q "${FUTURE_RED[@]}" --junitxml=/tmp/g1-future-red-start.xml
# Expected: exactly 125 failed, no pass/error/skip, with the approved 78/29/10/8 split.

python -m pytest -q \
  tests/test_fr3_runtime_safety.py::test_observed_public_action_displacement_equal_to_exact_hard_limit_passes \
  tests/test_fr3_runtime_safety.py::test_nextafter_above_exact_observed_hard_limit_aborts_without_epsilon \
  tests/test_fr3_runtime_safety.py::test_observed_hard_limit_comparison_source_has_no_epsilon_or_isclose \
  tests/test_fr3_runtime_safety.py::test_physical_safety_config_requires_exact_observed_hard_limit
# Expected: 4 passed.

python scripts/check_isaacsim6_imports.py --deprecated-as-error
# Expected: 0 errors, 0 warnings.

python -m pytest -q tests/test_g1_t152_red_migration_manifest.py
# Expected after manifest creation: 4 passed.
```

**RED/GREEN checkpoint**

No production RED is introduced here. The expected RED is still the frozen set
of 84 T152 nodes. GREEN means the four inventory/manifest nodes pass and their
recorded lists reproduce the observed baseline exactly.

**Stop conditions**

Stop on any SHA inequality/dirty worktree, behavior-source blob mismatch,
non-assertion T152 failure, baseline count/node drift, unmapped sphere-specific
node, intentional future-RED classification change, or deprecated scan
diagnostic.

**Commit**

`test(g1): freeze T152 red migration inventory`

Before commit, rerun the manifest test and the T152 file. After commit, verify the
diff contains only the four test/manifest files. This task depends only on the
starting preflight; Task 2 depends on this manifest. Isaac Sim is not allowed.

## Task 2: Correct the RED schema fixtures

**Files**

- Modify: `tests/test_g1_pose_conditioned_tracking_cli.py`
- Modify: `tests/test_press_button_mechanism.py`
- Add: `tests/test_press_button_geometry_contract.py`
- Add: `tests/test_g1_contact_exclusion_geometry.py`
- Add: `tests/test_press_button_task_card_contract.py`
- Modify: `tests/fixtures/g1_t152_red_migration_manifest.json`

Replace the temporary sphere input with a canonical fixture containing
`mechanism.base_position_m`, `mechanism.base_orientation_xyzw`, button capped
cylinder, housing OBB, and the approved contact-exclusion policy. There is no
`axis_local` configuration field and no caller-owned `workspace_valid` or
`contact_exclusion_valid` authority.

```python
@pytest.fixture
def press_button_geometry_1_1_mapping() -> dict[str, object]:
    return {
        "mechanism_version": "1.1.0",
        "base_position_m": [0.55, 0.0, 0.47],
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
```

**New RED node IDs**

Schema and root:

- `test_geometry_1_1_requires_every_top_level_contract_field`
- `test_geometry_1_1_rejects_unknown_top_level_contract_field`
- `test_mechanism_root_requires_finite_position_shape_three`
- `test_mechanism_root_requires_finite_xyzw_shape_four`
- `test_mechanism_root_rejects_zero_norm_and_ambiguous_quaternion_order`
- `test_mechanism_root_normalizes_xyzw_and_uses_canonical_sign`
- `test_world_transform_digest_changes_with_position_or_orientation`

Button and housing:

- `test_capped_cylinder_accepts_exact_axis_token[X]`
- `test_capped_cylinder_accepts_exact_axis_token[Y]`
- `test_capped_cylinder_accepts_exact_axis_token[Z]`
- `test_capped_cylinder_rejects_unknown_or_noncanonical_axis_token`
- `test_capped_cylinder_rejects_axis_token_and_axis_local_together`
- `test_capped_cylinder_axis_must_be_parallel_or_antiparallel_to_joint_axis`
- `test_geometry_dimensions_must_be_finite_and_strictly_positive[button-radius]`
- `test_geometry_dimensions_must_be_finite_and_strictly_positive[button-half-height]`
- `test_geometry_dimensions_must_be_finite_and_strictly_positive[housing-x]`
- `test_geometry_dimensions_must_be_finite_and_strictly_positive[housing-y]`
- `test_geometry_dimensions_must_be_finite_and_strictly_positive[housing-z]`
- `test_geometry_requires_exact_frame_units_and_approved_primitives`

Policy, digest, version, and truth boundary:

- `test_contact_exclusion_requires_exact_ordered_unique_obstacle_ids`
- `test_contact_exclusion_requires_exact_clearance_0p005`
- `test_contact_exclusion_rejects_unknown_metric_route_or_boundary_policy`
- `test_geometry_contract_rejects_each_missing_or_unknown_nested_field`
- `test_geometry_digest_binds_root_solids_policy_and_derived_axis`
- `test_geometry_digest_mutation_is_detected`
- `test_legacy_mechanism_1_0_is_state_only_and_ineligible_for_formal_build`
- `test_legacy_mechanism_formal_build_fails_with_required_geometry_code`
- `test_task_card_mechanism_version_matches_physical_config`
- `test_geometry_contract_digests_bind_config_and_task_card`
- `test_static_exclusion_scope_is_tcp_point_only`
- `test_static_exclusion_pass_cannot_set_cap_gate_c1_c2_or_g1_pass`
- `test_runtime_contact_collision_penetration_truth_remains_required_after_static_pass`

`tests/test_press_button_task_card_contract.py` initially contributes the exact
assertion RED
`test_task_card_mechanism_version_matches_physical_config`: it requires both the
physical config and task card to declare formal mechanism `1.1.0` and to match.
This node must be observed RED in Task 2; Task 6 is not its first execution.

At the end of Task 2,
`tests/test_g1_contact_exclusion_geometry.py` contains exactly two fixture-only
GREEN controls and zero analytic behavior REDs:

- `test_declared_geometry_fixture_contains_cylinder_obb_and_exact_policy`
- `test_declared_geometry_fixture_has_stable_test_side_digest`

Task 3 adds the analytic behavior REDs to that file. The Task 1 inventory and
migration manifest classify the task-card node as a new schema-migration RED and
the two geometry-fixture nodes as new GREEN controls; none is inserted into the
original 84 RED or original four-GREEN lists.

Use full pytest paths when writing the tests, for example
`tests/test_press_button_geometry_contract.py::test_geometry_1_1_requires_every_top_level_contract_field`.
Parameterized expansions must be separately present in `--collect-only` output.
Before production symbols exist, tests resolve them with an assertion-based
capability helper: `find_spec()` is checked for non-null, then `getattr()` is
checked for the named callable/type. A missing module or symbol therefore creates
the intended assertion RED, never ImportError or collection failure.

**Steps**

- [ ] Update the one-to-one migration manifest before removing the sphere
  fixture; verify every old expansion has one or more replacement expansions.
- [ ] Introduce the shared `press_button_geometry_1_1_mapping` fixture under the
  existing import-safe test boundary.
- [ ] Add every schema/version/truth node above. Use a positive parsing fixture
  only to prove the test data are internally coherent; production APIs must
  remain missing at this checkpoint.
- [ ] Add and observe the task-card/config `1.1.0` synchronization assertion RED.
- [ ] Add exactly the two named geometry-fixture GREEN controls and record their
  separate inventory classification.
- [ ] Modify the existing seven sphere-dependent expansions to consume declared
  solids while retaining their original behavioral assertions.
- [ ] Collect the new files and assert zero import, fixture, syntax, path, or
  Isaac errors.
- [ ] Run the replacement selection and observe assertion REDs caused only by
  missing contract/parser/validator behavior.

**Commands and expected results**

```bash
python -m pytest --collect-only -q \
  tests/test_press_button_geometry_contract.py \
  tests/test_g1_contact_exclusion_geometry.py \
  tests/test_press_button_mechanism.py \
  tests/test_press_button_task_card_contract.py \
  tests/test_g1_pose_conditioned_tracking_cli.py
# Expected: collection succeeds; the geometry file contributes exactly two
# fixture controls; no Isaac modules are imported.

python -m pytest -q \
  tests/test_press_button_geometry_contract.py \
  tests/test_g1_contact_exclusion_geometry.py \
  tests/test_press_button_mechanism.py \
  tests/test_press_button_task_card_contract.py \
  tests/test_g1_pose_conditioned_tracking_cli.py
# Expected RED: new schema nodes fail by assertion because strict 1.1.0 parsing,
# digesting, legacy gating, and declared-solid route construction do not exist.
# The task-card synchronization node is assertion-RED. The two geometry-fixture
# controls and existing four T152 controls pass.

python -m pytest -q tests/test_g1_t152_red_migration_manifest.py
# Expected: all manifest nodes pass and no old sphere expansion is unmapped.
```

**Stop conditions**

Stop if a RED is caused by collection/import/fixture/path errors, if a schema
field must be guessed, if a caller validity flag remains authoritative, or if
any old safety behavior loses its mapped replacement.

**Commit**

`test(g1): replace spherical contact exclusion fixtures`

Before and after commit, run the exact collect-only and five-file focused
commands above plus the migration-manifest test. The schema and task-card nodes
must remain assertion RED; the manifest, two new fixture controls, and four
existing T152 controls must be GREEN. Task 2 depends on Task 1;
Task 3 depends on the canonical fixture established here. Isaac Sim is not
allowed.

## Task 3: Define RED continuous analytic geometry contracts

**Files**

- Modify: `tests/test_g1_contact_exclusion_geometry.py`
- Modify: `tests/test_g1_pose_conditioned_tracking_cli.py`
- Modify: `tests/fixtures/g1_t152_red_migration_manifest.json` only if a mapped
  replacement gains parameterized expansions.

Use pure tuple/NumPy inputs. The test module must not import `pxr`, `omni`,
`isaacsim`, or launch an application. Every result asserts a non-empty exact code
and message. Schema failure uses `G1_C1_CONTACT_EXCLUSION_SCHEMA_INVALID`, solid
failure uses `G1_C1_CONTACT_EXCLUSION_GEOMETRY_INVALID`, route failure uses
`G1_C1_CONTACT_EXCLUSION_ROUTE_INVALID`, an analytically undecidable/nonfinite
case uses `G1_C1_CONTACT_EXCLUSION_CLEARANCE_UNPROVEN`, and digest disagreement
uses `G1_C1_CONTACT_EXCLUSION_DIGEST_MISMATCH`.

The test file obtains the future validator through this import-safe RED seam:

```python
def _geometry_capability(name: str):
    spec = importlib.util.find_spec(
        "isaac_tactile_libero.runtime.g1_contact_exclusion"
    )
    assert spec is not None, "missing approved analytic-geometry capability"
    module = importlib.import_module(spec.name)
    value = getattr(module, name, None)
    assert callable(value), f"missing approved capability: {name}"
    return value
```

The assertion on `spec` is the initial RED. The test must not execute the import
branch until a module exists, so no module-not-found exception can occur.

**OBB RED node IDs**

- `test_expanded_obb_rejects_continuous_interior_crossing`
- `test_expanded_obb_allows_tangent_boundary_contact`
- `test_expanded_obb_allows_parallel_segment_strictly_outside`
- `test_expanded_obb_allows_boundary_coincident_segment`
- `test_expanded_obb_rejects_endpoint_inside`
- `test_expanded_obb_checks_zero_length_segment_as_point`
- `test_expanded_obb_uses_rotated_root_and_box_frame`
- `test_expanded_obb_nonfinite_or_unordered_interval_is_unproven[nonfinite]`
- `test_expanded_obb_nonfinite_or_unordered_interval_is_unproven[unordered]`

**Finite capped-cylinder RED node IDs**

- `test_expanded_cylinder_rejects_radial_interior_crossing`
- `test_expanded_cylinder_rejects_cap_interior_crossing`
- `test_expanded_cylinder_allows_radial_tangent`
- `test_expanded_cylinder_allows_cap_tangent`
- `test_expanded_cylinder_solves_axis_parallel_segment`
- `test_expanded_cylinder_allows_boundary_coincident_segment`
- `test_expanded_cylinder_rejects_endpoint_inside`
- `test_expanded_cylinder_checks_zero_length_segment_as_point`
- `test_expanded_cylinder_maps_axis_token[X]`
- `test_expanded_cylinder_maps_axis_token[Y]`
- `test_expanded_cylinder_maps_axis_token[Z]`
- `test_expanded_cylinder_uses_rotated_mechanism_root`
- `test_expanded_cylinder_handles_radial_quadratic_degeneration[linear]`
- `test_expanded_cylinder_handles_radial_quadratic_degeneration[constant-inside]`
- `test_expanded_cylinder_handles_radial_quadratic_degeneration[constant-outside]`
- `test_expanded_cylinder_nonfinite_solver_state_is_unproven[coefficient]`
- `test_expanded_cylinder_nonfinite_solver_state_is_unproven[discriminant]`
- `test_expanded_cylinder_nonfinite_solver_state_is_unproven[root]`
- `test_expanded_cylinder_nonfinite_solver_state_is_unproven[interval]`

**Clearance/evidence RED node IDs**

- `test_clearance_exactly_0p005_allows_expanded_boundary_touch`
- `test_clearance_strictly_below_0p005_fails_without_tolerance`
- `test_continuous_validation_rejects_midsegment_intersection_with_safe_endpoints`
- `test_conservative_pass_records_only_approved_lower_bound`
- `test_design_time_0p021_is_never_emitted_as_runtime_route_minimum`
- `test_route_result_records_obstacle_segment_expansion_and_provenance`
- `test_route_digest_covers_ordered_segments_geometry_and_policy`
- `test_digest_mismatch_fails_closed_before_scene_acquisition`

The strict-low fixture must use
`math.nextafter(0.005, 0.0)` as an actual clearance, and source assertions must
forbid `epsilon`, `isclose`, fixed-step loops, and endpoint-only predicates in
the validator.

```python
def assert_exact_blocker(result, code: str) -> None:
    assert result.clearance_passed is False
    assert result.code == code
    assert isinstance(result.message, str) and result.message.strip()

def test_continuous_validation_rejects_midsegment_intersection_with_safe_endpoints():
    validate_segment_against_expanded_obb = _geometry_capability(
        "validate_segment_against_expanded_obb"
    )
    result = validate_segment_against_expanded_obb(
        start_world_m=(-0.1, 0.0, 0.0),
        end_world_m=(0.1, 0.0, 0.0),
        world_from_obstacle=np.eye(4),
        center_local_m=(0.0, 0.0, 0.0),
        half_extents_m=(0.01, 0.01, 0.01),
        required_clearance_m=0.005,
    )
    assert_exact_blocker(result, "G1_C1_CONTACT_EXCLUSION_ROUTE_INVALID")
```

**Steps**

- [ ] Add all OBB open-interior slab cases, including parallel and zero-length
  branches.
- [ ] Add all finite-cylinder radial-quadratic and axial-open-interval cases,
  including tangencies and degeneracies.
- [ ] Add exact boundary and strict-nextafter tests with source guards against
  tolerance and sampling.
- [ ] Assert evidence serialization includes segment endpoints/digest, expanded
  solid, interior/boundary decisions, lower bound or null, and provenance.
- [ ] Assert an unproven result never guesses a finite lower bound.
- [ ] Run collection and then the focused selection to observe assertion RED.

**Commands and expected results**

```bash
python -m pytest --collect-only -q tests/test_g1_contact_exclusion_geometry.py
# Expected: all named expansions collect with no import or fixture errors.

python -m pytest -q tests/test_g1_contact_exclusion_geometry.py
# Expected RED: every analytic behavior node fails by assertion because the
# import-safe continuous validator and result types do not yet exist at their
# approved import seam; any GREEN fixture-only checks are listed explicitly.
```

**Stop conditions**

Stop on collection errors, tolerance/sampling requirements, missing non-empty
messages, a finite cylinder replaced by a sphere/infinite cylinder, or any test
that cannot distinguish open interior from boundary contact.

**Commit**

`test(g1): define analytic contact exclusion contracts`

Before and after commit, run the Task 3 collect-only and focused analytic-file
commands plus the migration-manifest test. All analytic behavior nodes remain
expected assertion RED; fixture controls pass. Task 3 depends on Task
2; Tasks 4 and 5 depend on these RED contracts. Isaac Sim is not allowed.

## Task 4: Implement import-safe geometry contract types

**Files**

- Add: `isaac_tactile_libero/tasks/press_button_geometry.py`
- Modify: `tests/test_press_button_geometry_contract.py`
- Do not modify YAML yet.

**Types and signatures**

```python
@dataclass(frozen=True)
class MechanismRootPose:
    position_m: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]

@dataclass(frozen=True)
class CappedCylinderGeometry:
    center_local_m: tuple[float, float, float]
    axis_token: str
    radius_m: float
    half_height_m: float

@dataclass(frozen=True)
class OrientedBoxGeometry:
    center_local_m: tuple[float, float, float]
    half_extents_m: tuple[float, float, float]

@dataclass(frozen=True)
class ContactExclusionPolicy:
    schema_version: str
    subject: str
    obstacle_ids: tuple[str, ...]
    required_clearance_m: float
    distance_metric: str
    route_validation: str
    boundary_policy: str

@dataclass(frozen=True)
class PressButtonGeometryContract:
    root_pose: MechanismRootPose
    button: CappedCylinderGeometry
    housing: OrientedBoxGeometry
    contact_exclusion: ContactExclusionPolicy
    geometry_sha256: str

@dataclass(frozen=True)
class PressButtonWorldGeometry:
    world_from_mechanism_root: tuple[tuple[float, ...], ...]
    world_from_mechanism_root_sha256: str
    button_center_world_m: tuple[float, float, float]
    button_axis_world: tuple[float, float, float]
    housing_center_world_m: tuple[float, float, float]
    housing_world_from_local: tuple[tuple[float, ...], ...]
```

```python
def parse_press_button_geometry_contract(
    mechanism: Mapping[str, object],
    *,
    joint_axis: Sequence[float],
    task_config_sha256: str,
) -> PressButtonGeometryContract: ...

def canonicalize_xyzw(value: Sequence[float]) -> tuple[float, float, float, float]: ...
def axis_token_to_local(axis_token: str) -> tuple[float, float, float]: ...
def derive_press_button_world_geometry(
    contract: PressButtonGeometryContract,
) -> PressButtonWorldGeometry: ...
```

The parser compares exact key sets at every mapping level. Missing or unknown
keys raise an exception carrying `G1_C1_CONTACT_EXCLUSION_SCHEMA_INVALID`; there
are no defaults. Numeric arrays are shape-checked and finite. Quaternion input is
xyzw only, normalized after a nonzero finite norm check, and canonicalized so
`w > 0`; when `w == 0`, the first nonzero x/y/z component is positive. This
canonical sign is included in the digest.

```python
AXIS_TOKEN_TO_LOCAL = {
    "X": (1.0, 0.0, 0.0),
    "Y": (0.0, 1.0, 0.0),
    "Z": (0.0, 0.0, 1.0),
}

axis = np.asarray(axis_token_to_local(button.axis_token), dtype=np.float64)
joint = np.asarray(joint_axis, dtype=np.float64)
joint /= np.linalg.norm(joint)
if np.linalg.norm(np.cross(axis, joint)) > 1e-8 or abs(abs(axis @ joint) - 1.0) > 1e-8:
    fail("G1_C1_CONTACT_EXCLUSION_GEOMETRY_INVALID", "button axis and joint_axis are not collinear")
```

Canonical digest input is canonical JSON with sorted mapping keys and compact
separators, while array order, `obstacle_ids`, and xyzw ordering remain semantic.
It includes normalized root pose, `axis_token`, derived `axis_local`, dimensions,
policy, task-config digest, and transform provenance. World centers and axes are
computed by one homogeneous transform; the validator never assumes identity
orientation and never reads a USD stage back to recover missing input.

The module imports only standard-library modules and NumPy. It must contain no
Isaac, `pxr`, `omni`, or application import.

**Steps**

- [ ] Add the immutable dataclasses and exact blocker exception/result plumbing.
- [ ] Implement strict mapping helpers, finite shape checks, exact enums, and
  positive dimensions.
- [ ] Implement xyzw normalization/canonical sign and token mapping.
- [ ] Implement collinearity validation and stable canonical digest.
- [ ] Implement root transform and world geometry derivation.
- [ ] Run the Task 2 schema file and make only Task 4-owned nodes GREEN.
- [ ] Run import-boundary tests to prove no Isaac import occurs.

**Commands and expected results**

```bash
python -m pytest -q tests/test_press_button_geometry_contract.py
# Expected before implementation: Task 4-owned schema assertions fail.
# Expected after implementation: all nodes in this file pass.

python -m pytest -q \
  tests/test_g1_pose_conditioned_tracking_cli.py::test_t152_positive_runner_and_fake_factory_import_boundary_never_starts_isaac
# Expected before and after: 1 passed.

python -c 'import isaac_tactile_libero.tasks.press_button_geometry'
# Expected after implementation: exit 0 without Isaac/pxr imports.
```

**Stop conditions**

Stop if parsing needs a default or guessed field, if root orientation cannot be
derived before runtime, if token and vector become dual authorities, if the
digest is not deterministic, or if the module requires Isaac.

**Commit**

`feat(g1): define PressButton geometry contract`

Before commit, the schema file and import check must pass while Task 3 analytic
nodes remain RED. After commit, rerun them and verify no YAML or mechanism builder
changed. Task 4 depends on Tasks 2 and 3; Task 5 depends on its immutable types.
Isaac Sim is not allowed.

## Task 5: Implement the continuous segment/solid validator

**Files**

- Add: `isaac_tactile_libero/runtime/g1_contact_exclusion.py`
- Modify: `tests/test_g1_contact_exclusion_geometry.py`

**Types and signatures**

```python
@dataclass(frozen=True)
class OpenInterval:
    lower: float
    upper: float
    lower_attained: bool = False
    upper_attained: bool = False

@dataclass(frozen=True)
class SegmentClearanceResult:
    clearance_passed: bool
    intersects_expanded_interior: bool
    touches_expanded_boundary: bool
    minimum_clearance_lower_bound_m: float | None
    required_clearance_m: float
    conservative_rejection_possible: bool
    code: str | None
    message: str | None
    evidence: Mapping[str, object]

@dataclass(frozen=True)
class ContactExclusionRouteResult:
    tcp_route_exclusion_qualified: bool
    contact_exclusion_scope: str
    full_robot_static_collision_exclusion_qualified: bool
    class_results: tuple[Mapping[str, object], ...]
    code: str | None
    message: str | None

def validate_segment_against_expanded_obb(
    *,
    start_world_m: Sequence[float],
    end_world_m: Sequence[float],
    world_from_obstacle: Sequence[Sequence[float]],
    center_local_m: Sequence[float],
    half_extents_m: Sequence[float],
    required_clearance_m: float,
) -> SegmentClearanceResult: ...

def validate_segment_against_expanded_capped_cylinder(
    *,
    start_world_m: Sequence[float],
    end_world_m: Sequence[float],
    world_from_obstacle: Sequence[Sequence[float]],
    axis_token: str,
    radius_m: float,
    half_height_m: float,
    required_clearance_m: float,
) -> SegmentClearanceResult: ...

def validate_contact_exclusion_routes(
    *,
    ordered_routes: Sequence[Mapping[str, object]],
    contract: PressButtonGeometryContract,
    world_geometry: PressButtonWorldGeometry,
    current_input_digests: Mapping[str, str],
) -> ContactExclusionRouteResult: ...
```

`OpenInterval` represents strict solution sets. Intersection with `[0, 1]`
must retain whether the result contains a real open span; a single endpoint or
tangent is not an interior intersection. Use a helper whose return is
`OpenInterval | None`, where `None` means an empty proven set, not an unproven
calculation. Unproven state is raised separately with the exact blocker.

**OBB algorithm**

Transform `u` and `v` into OBB local coordinates and set
`E = half_extents + clearance`. For each axis solve the strict slab inequality.

```python
def _strict_linear_band(a: float, b: float, radius: float) -> OpenInterval | None:
    # Solve -radius < a + b*t < radius over real t.
    require_finite(a, b, radius)
    if b == 0.0:
        return OpenInterval(-math.inf, math.inf) if -radius < a < radius else None
    t0 = (-radius - a) / b
    t1 = ( radius - a) / b
    lo, hi = (t0, t1) if t0 < t1 else (t1, t0)
    return OpenInterval(lo, hi)

inside = _open_intersection(
    OpenInterval(0.0, 1.0, lower_attained=True, upper_attained=True),
    *(_strict_linear_band(p0[i] - center[i], d[i], E[i]) for i in range(3)),
)
```

If `inside` contains any t in the closed segment domain, the segment intersects
the expanded OBB open interior and fails. Evaluate closed-box membership at the
candidate slab endpoints/tangent parameters to distinguish boundary touch from
strict exterior. A zero-length segment uses the same exact point predicates.

**Finite capped-cylinder algorithm**

Transform the segment to a cylinder-centered coordinate system whose axial
coordinate follows `axis_token`. With `R = radius + clearance`,
`H = half_height + clearance`, radial coordinates `r0` and `rd`, solve
`a*t*t + b*t + c < 0`, where:

```python
a = float(rd @ rd)
b = float(2.0 * (r0 @ rd))
c = float(r0 @ r0 - R * R)
```

Use explicit branches:

```python
def _strict_quadratic_negative(a: float, b: float, c: float) -> OpenInterval | None:
    require_finite(a, b, c)
    if a == 0.0:
        if b == 0.0:
            return OpenInterval(-math.inf, math.inf) if c < 0.0 else None
        root = -c / b
        return OpenInterval(-math.inf, root) if b > 0.0 else OpenInterval(root, math.inf)
    disc = b * b - 4.0 * a * c
    require_finite(disc)
    if disc <= 0.0:
        # disc == 0 is a boundary tangent for a >= 0 and contains no open radial interior.
        return None
    sqrt_disc = math.sqrt(disc)
    q = -0.5 * (b + math.copysign(sqrt_disc, b))
    if q == 0.0:
        roots = sorted(((-b - sqrt_disc) / (2.0 * a),
                        (-b + sqrt_disc) / (2.0 * a)))
    else:
        roots = sorted((q / a, c / q))
    require_finite(*roots)
    return OpenInterval(roots[0], roots[1])
```

Solve `-H < axial0 + axial_delta*t < H` with `_strict_linear_band`; intersect
the radial interval, axial interval, and `[0,1]`. A nonempty open-interior set is
FAIL. A discriminant of exactly zero, cap tangent, radial tangent, or
boundary-coincident segment passes when no open-interior t exists. Nonfinite or
unorderable intermediates return
`G1_C1_CONTACT_EXCLUSION_CLEARANCE_UNPROVEN`; they never guess a result.

This axial-plus-radial expansion and the coordinate-wise OBB expansion are
conservative supersets of the Euclidean clearance neighborhood. They can reject
a geometrically safe corner route, but cannot accept a route whose Euclidean
clearance is below `0.005`. A proven conservative pass records only
`minimum_clearance_lower_bound_m=0.005` unless a separately identified exact
algorithm proves more. It never emits the design-time `0.021` value.

Evidence contains the ordered obstacle ID, primitive, segment/class ID,
endpoints and digest, expanded dimensions, interior and boundary decisions,
lower bound or null, exact decision/code/message, root/geometry/config digests,
scope, and `conservative_rejection_possible=true`.

**Steps**

- [ ] Add result/interval dataclasses, finite guards, transforms, and canonical
  serialization helpers.
- [ ] Implement OBB strict slab intersection and exact boundary classification.
- [ ] Implement finite-cylinder radial quadratic, axial interval, and every
  degeneration branch.
- [ ] Implement ordered route/obstacle aggregation with fail-closed digest
  checks and TCP-only truth fields.
- [ ] Run every Task 3 RED node; make all analytic nodes GREEN without changing
  their assertions.
- [ ] Inspect source to confirm there is no sampling, SciPy, Isaac import,
  `epsilon`, or `isclose`.

**Commands and expected results**

```bash
python -m pytest -q tests/test_g1_contact_exclusion_geometry.py
# Before implementation: Task 3 behavior nodes fail by assertion.
# After implementation: all collected analytic nodes pass.

python -c 'import isaac_tactile_libero.runtime.g1_contact_exclusion'
# Expected after implementation: exit 0 with no Isaac/pxr import.

rg -n 'isclose|epsilon|linspace|arange|sample' \
  isaac_tactile_libero/runtime/g1_contact_exclusion.py
# Expected: no validator implementation match.
```

**Stop conditions**

Stop if numerical tolerance is needed to make equality pass, if continuous
coverage becomes sampled/endpoint-only, if a degeneration cannot be proven, if
a conservative rejection becomes an acceptance, or if any evidence field is
invented.

**Commit**

`feat(g1): validate continuous TCP clearance`

Before and after commit, run the analytic file and import check. After commit all
Task 3 nodes are GREEN while mechanism/T152 integration REDs remain RED. Task 5
depends on Task 4 types and Task 3 tests; Tasks 7 and 8 depend on this validator.
Isaac Sim is not allowed.

## Task 6: Migrate the mechanism parser and tracked consumers to 1.1.0

**Files**

- Modify: `isaac_tactile_libero/tasks/press_button_mechanism.py`
- Modify: `configs/tasks/press_button_physical.yaml`
- Modify: `configs/tasks/cards/press_button.v1.yaml`
- Modify: `tests/test_press_button_mechanism.py`
- Modify: `tests/test_press_button_geometry_contract.py`
- Modify: `tests/test_press_button_task_card_contract.py`

**Version behavior**

Extend `PressButtonMechanismConfig` with a parsed geometry contract and explicit
eligibility properties:

```python
@dataclass(frozen=True)
class PressButtonMechanismConfig:
    # Existing state/reset/release fields remain unchanged.
    mechanism_version: str
    base_position_m: tuple[float, float, float]
    base_orientation_xyzw: tuple[float, float, float, float] | None
    geometry_contract: PressButtonGeometryContract | None

    @property
    def geometry_contract_available(self) -> bool: ...
    @property
    def runtime_stage_build_eligible(self) -> bool: ...
    @property
    def route_validation_input_eligible(self) -> bool: ...
```

For exact `1.1.0`, orientation, geometry, and contact-exclusion are required.
Successful strict parsing sets `geometry_contract_available=true`, permits
formal stage construction through `runtime_stage_build_eligible=true`, and
permits the declared values to enter TCP route validation through
`route_validation_input_eligible=true`. These are input/schema capabilities,
not benchmark results.

`PressButtonMechanismConfig` must not expose a true
`benchmark_cap_eligible` property. That field belongs to real runtime samples
and can become true only after the shared qualifying runtime kernel, complete
sample truth, multiclass aggregation, and tested-only cap selection succeed.
Static analytic success records only
`tcp_route_exclusion_qualified=true|false` and
`full_robot_static_collision_exclusion_qualified=false`; it cannot set cap
eligibility.

Explicit legacy `1.0.x` may still parse for state/reset/release classification,
but returns:

```yaml
geometry_contract_available: false
tcp_route_exclusion_qualified: false
benchmark_cap_eligible: false
runtime_stage_build_eligible: false
route_validation_input_eligible: false
```

If a legacy caller requests `build_stage()` or route validation, raise
`G1_PRESS_BUTTON_FORMAL_GEOMETRY_REQUIRED` with a non-empty message. Do not copy
the old dimensions into a fallback. Unknown mechanism major/minor versions fail
closed.

Change the physical YAML to mechanism 1.1.0 and add the exact approved root,
geometry, and policy mappings. Keep
`press_button_physical.yaml::task_version: "1.0.2"`. Change only
`configs/tasks/cards/press_button.v1.yaml::scene.mechanism_version` to `"1.1.0"`;
retain the card's existing task-version semantics.

This task is the first behavior/config migration. It proves that the new
task/config/card/geometry digests differ from attempt-02 and therefore classifies
attempt-02 as historical/stale. This activates the condition
`CURRENT_C2A_REFRESH_REQUIRED_AFTER_GEOMETRY_SCHEMA_CHANGE`, but Task 6 does not
implement the complete T152 evidence-loader/orchestration blocker. Task 9 owns
that blocker and its report/manifest/exit/close lifecycle.

**Steps**

- [ ] Add strict formal-versus-legacy parsing without changing state/reset/release
  calculations.
- [ ] Add the three schema/input eligibility properties and the formal-build
  blocker; prove the mechanism config cannot grant benchmark cap eligibility.
- [ ] Insert the approved 1.1.0 mappings in the physical config and synchronize
  the task-card mechanism version.
- [ ] Remove geometry defaults from the formal path; retain no second authority.
- [ ] Assert physical task version and card task-version meanings are unchanged.
- [ ] Make Task 2 version/config nodes GREEN.
- [ ] Prove the migrated current task/card/geometry digests differ from the
  immutable attempt-02 provenance and classify it stale without invoking the
  T152 CLI loader.
- [ ] Run state/reset/release tests to prove legacy classification still works
  and formal runtime build is blocked.

**Commands and expected results**

```bash
python -m pytest -q \
  tests/test_press_button_geometry_contract.py \
  tests/test_press_button_mechanism.py \
  tests/test_press_button_task_card_contract.py
# Before migration: version/config nodes are RED.
# After migration: all selected schema/version/digest nodes pass and attempt-02
# is proven stale by digest comparison. The T152 loader/orchestration exact
# refresh-blocker nodes remain assertion RED until Task 9.

python -m pytest --collect-only -q tests/test_press_button_mechanism.py
# Expected: no collection/import/Isaac error.
```

**Stop conditions**

Stop if task card and config versions diverge, if a legacy path can construct a
formal stage, if a formal geometry default remains, if task success/reset/action
semantics change, if schema/static geometry can grant benchmark cap eligibility,
or if the migrated digests do not classify attempt-02 as stale.

**Commit**

`feat(g1): migrate PressButton mechanism 1.1`

Before commit run geometry-contract, mechanism, and actual collected task-card
contract files. After commit rerun them, prove old/current digest inequality,
and confirm the T152 loader/orchestration refresh-blocker nodes remain RED for
Task 9. Task 6 depends on Task 4; Task 7 depends on this versioned config. Isaac
Sim is not allowed.

## Task 7: Separate geometry-only receipt authoring from complete USD stage build

**Approved design**

Follow
[`g1-press-button-geometry-authoring-receipt-design.md`](g1-press-button-geometry-authoring-receipt-design.md).
The former combined adapter/build contract is replaced by an import-safe
geometry-only receipt seam plus a real complete `build_stage()` path. Task 7 is
split into RED correction and GREEN implementation; neither stage authorizes
Isaac Sim.

### Task 7A: Correct the RED contracts for the approved seam

**Files**

- Modify: `tests/test_press_button_mechanism.py`
- Do not modify production code, YAML, configs, evidence, or task status.

**Corrected RED ownership**

Correct the six historical Task 7 RED nodes one-for-one instead of weakening or
deleting them:

| Historical behavior | Corrected RED behavior |
|---|---|
| exact root/housing/button calls | call `author_declared_geometry()` with a recording declared-geometry adapter and verify `root -> housing -> button` plus exact arguments |
| shared object and digests | receipt retains `config.geometry_contract` by identity and copies both contract digests exactly |
| import-safe fake path | importing the module and running the geometry-only seam loads none of `pxr`, `omni`, or `isaacsim` |
| lazy real USD adapter | `UsdPressButtonDeclaredGeometryAuthoringAdapter` imports `pxr` only when the real adapter is constructed or called |
| geometry-literal guard | the real geometry adapter and complete builder contain none of the formal geometry authority literals |
| complete physical semantics | `build_stage()` has no adapter-injection parameter and still contains collision, rigid body, mass, joint, relationships, anchors, rotations, limits, and drive behavior |

The corrected or additional node IDs are:

- `test_declared_geometry_seam_records_root_housing_button_order_and_exact_values`
- `test_geometry_authoring_receipt_reuses_loaded_contract_and_matching_digests`
- `test_declared_geometry_seam_and_recording_fake_are_import_safe`
- `test_real_usd_declared_geometry_adapter_keeps_pxr_import_lazy_and_uses_full_dimensions`
- `test_formal_stage_builder_and_geometry_adapter_contain_no_geometry_authority_literals`
- `test_complete_build_stage_has_no_adapter_injection_and_preserves_physics_semantics`
- `test_geometry_authoring_receipt_is_geometry_only_and_never_benchmark_eligible`
- `test_declared_geometry_seam_rejects_legacy_before_adapter_call`
- `test_complete_build_stage_signature_accepts_only_real_stage`

**Task 7A baseline classification**

Task 7A does not manufacture a uniform RED result. The current complete builder
already has two behaviors that are preservation controls:

- `test_complete_build_stage_signature_accepts_only_real_stage` is GREEN because
  the current signature already accepts only `self` and `stage`.
- The existing collision, rigid body, mass, prismatic joint, Body0/Body1,
  anchors, rotations, limits, and drive source assertions are GREEN within the
  complete-builder contract.

The missing geometry receipt, geometry-only seam, real declared-geometry
adapter, receipt/contract digest binding, and complete-builder use of the new
seam are RED. When the complete-builder node checks both old and new behavior,
its existing physics/joint assertions must execute and pass before the assertion
that the builder calls `author_declared_geometry()` fails. Tests may not call a
generic missing-capability helper first when doing so would mask an already
implemented signature or physical-authoring behavior.

The RED receipt assertions require:

```text
schema_version = g1.press_button.geometry_authoring_receipt.v1
receipt.contract is config.geometry_contract
geometry_only = true
complete_stage = false
benchmark_cap_eligible = false
```

Legacy rejection must use `G1_PRESS_BUTTON_FORMAL_GEOMETRY_REQUIRED` with a
non-empty message before any adapter call. The complete builder signature must
accept `self` and `stage` only. A geometry receipt cannot satisfy complete-stage,
C1, cap, gate, or runtime truth. The complete-builder source guard also rejects
branching or reflection based on adapter identity, caller identity, class name,
or module name.

**Task 7A commands and expected result**

```bash
python -m pytest --collect-only -q tests/test_press_button_mechanism.py
# Expected: all corrected and additional node IDs collect; 0 import, fixture,
# path, syntax, or collection errors.

python -m pytest -q tests/test_press_button_mechanism.py \
  -k 'declared_geometry or geometry_authoring_receipt or complete_build_stage or formal_stage_builder'
# Expected before production changes: receipt/seam/real-adapter/shared-contract
# assertions are missing-capability RED; the signature node and existing
# physics/joint preservation assertions are genuinely GREEN; 0 errors and 0
# skips. A combined builder node is RED only at its missing seam-integration
# assertion after its preservation assertions pass.

python -m pytest -q tests/test_press_button_mechanism.py \
  -k 'not declared_geometry and not geometry_authoring_receipt and not complete_build_stage and not formal_stage_builder'
# Expected: all Task 6 and prior mechanism nodes remain GREEN.
```

**Task 7A stop conditions**

Stop if a corrected node disappears without a one-to-one behavior mapping, a
failure is caused by import/fixture/collection/environment setup, an existing
signature or physics behavior is masked behind a missing-capability assertion,
a receipt is treated as complete stage success, a fake calls the complete
builder, or an approved Task 6 assertion is changed.

**Task 7A commit**

`test(g1): correct geometry-only authoring contracts`

This commit contains tests only. Observe the RED partition before committing
and rerun the same collection and selections after committing.

### Task 7B: Implement the receipt seam and real complete builder

**Files**

- Modify: `isaac_tactile_libero/tasks/press_button_mechanism.py`
- Use the corrected `tests/test_press_button_mechanism.py` without weakening its
  assertions.
- Do not modify geometry/config values, physics policy, task status, or evidence.

**Interfaces**

```python
class PressButtonDeclaredGeometryAuthoringAdapter(Protocol):
    def author_root(
        self, *, root_path: str, position_m: tuple[float, float, float],
        orientation_xyzw: tuple[float, float, float, float],
    ) -> None: ...
    def author_oriented_box(
        self, *, path: str, center_local_m: tuple[float, float, float],
        half_extents_m: tuple[float, float, float],
    ) -> None: ...
    def author_capped_cylinder(
        self, *, path: str, center_local_m: tuple[float, float, float],
        axis_token: str, radius_m: float, height_m: float,
    ) -> None: ...

@dataclass(frozen=True)
class PressButtonGeometryAuthoringReceipt:
    schema_version: str
    mechanism_version: str
    contract: PressButtonGeometryContract
    geometry_sha256: str
    world_from_mechanism_root_sha256: str
    root_prim_path: str
    housing_prim_path: str
    button_prim_path: str
    geometry_only: bool
    complete_stage: bool
    benchmark_cap_eligible: bool

def author_declared_geometry(
    self,
    *,
    authoring_adapter: PressButtonDeclaredGeometryAuthoringAdapter,
) -> PressButtonGeometryAuthoringReceipt: ...

class UsdPressButtonDeclaredGeometryAuthoringAdapter:
    """Lazy USD adapter for root and declared solids only."""

def build_stage(self, stage: Any) -> dict[str, Any]: ...
```

**GREEN phase 1: receipt and import-safe seam**

- [ ] Define the three-method declared-geometry protocol.
- [ ] Define the frozen receipt with the exact schema version and no-claim
  fields.
- [ ] Require formal 1.1.0 and reuse the exact `config.geometry_contract`.
- [ ] Call root, housing, and button in that order with contract-derived values.
- [ ] Return the receipt only after all calls succeed.
- [ ] Keep this seam free of `pxr`, `omni`, and `isaacsim` imports.
- [ ] Fail legacy calls before the first adapter call.

Run:

```bash
python -m pytest -q tests/test_press_button_mechanism.py \
  -k 'declared_geometry or geometry_authoring_receipt or legacy'
# Expected: receipt/seam/legacy nodes GREEN; complete real-builder nodes remain
# assertion RED; no Isaac import or startup.
```

Commit:

`feat(g1): add declared geometry authoring receipt`

**GREEN phase 2: real complete stage**

- [ ] Implement the lazy `UsdPressButtonDeclaredGeometryAuthoringAdapter(stage)`.
- [ ] Convert xyzw root orientation to USD quaternion ordering.
- [ ] Derive full cylinder height and Cube dimensions from the contract.
- [ ] Keep the complete builder signature limited to the real stage.
- [ ] Call the geometry-only seam, validate authored prims, then apply every
  existing collision, rigid body, mass, prismatic joint, relationship, anchor,
  rotation, limit, and drive operation.
- [ ] Derive the housing-side joint anchor from button/housing contract centers.
- [ ] Remove formal geometry authority literals from the adapter and builder.
- [ ] Return a complete scene contract only after all physical authoring
  succeeds.

Run:

```bash
python -m pytest -q tests/test_press_button_mechanism.py
python -m pytest -q tests/test_press_button_geometry_contract.py
python -m pytest -q tests/test_g1_contact_exclusion_geometry.py
python -m pytest -q tests/test_press_button_task_card_contract.py
# Expected: all Task 4–7 nodes GREEN; no application startup.
```

Commit:

`refactor(g1): share declared geometry with complete USD stage`

**Task 7B stop conditions**

Stop if the geometry-only seam imports Isaac/USD, the receipt claims a complete
stage or benchmark eligibility, persistent provenance uses object identity,
the real builder accepts an injected fake/custom adapter, complete physical
authoring is skipped, root/solid/joint geometry requires a source literal,
analytic and authoring paths do not share the same contract/digests, or any unit
test needs Isaac Sim.

Task 7 depends on Tasks 4 and 6. Task 8 depends on Task 7B GREEN plus the Task 5
continuous validator. Isaac Sim is not allowed in Task 7A or Task 7B.

## Task 8: Derive and validate the command-bound six-route bundle

**Approved architecture**

Follow
[`g1-task8-command-bound-route-bundle-design.md`](g1-task8-command-bound-route-bundle-design.md).
Task 8 produces one import-safe pure-Python bundle for five canonical commands
and six canonical classes. It does not build a plan, construct a factory, acquire
a scene, execute an action, write evidence, aggregate a cap, or close a runtime.

### Task 8A: Correct RED ownership and bundle schema

**Files**

- Modify: `tests/test_g1_pose_conditioned_tracking_cli.py`
- Add or modify only test-side helpers/fixtures needed for the exact command-bound
  bundle contract.
- Do not modify production code, YAML, evidence, task status, or the intentional
  future-RED inventory.

**RED correction**

- [ ] Replace raw formal `mechanism_geometry` input with a real parsed
  `PressButtonGeometryContract` fixture.
- [ ] Replace handwritten A/P/R inputs with the canonical task-route geometry
  function contract.
- [ ] Add `selected_pose_sha256` and the exact five current digest fields:
  `task_config_sha256`, `task_card_sha256`, `robot_config_sha256`,
  `fr3_asset_sha256`, and `geometry_sha256`.
- [ ] Change the six invalid-route expansions to invoke the pure bundle validator
  directly; they do not construct a factory or call orchestration.
- [ ] Change the three local and three continuous schedule expansions to consume
  command routes from the pure bundle rather than a Task 10 plan.
- [ ] Keep the Task 10 plan-motif node, six executor-motif expansions, and seven
  orchestration-route expansions unchanged and assertion RED.
- [ ] Keep the existing six-route and motif fixture controls GREEN.

The existing Task 8 node families remain observable:

- `test_t152_all_six_complete_routes_are_required_before_scene_acquisition`
  (six direct-validator expansions);
- `test_t152_local_class_executes_plus16_minus32_plus16_in_every_window`
  (three pure-bundle expansions);
- `test_t152_continuous_class_consumes_decimal_endpoint_reflection_schedule`
  (three pure-bundle expansions);
- `test_t152_motif_digest_exact_scalar_and_float64_materialization_cross_check`;
- `test_t152_route_builder_derives_all_six_records_from_declared_solids`;
- `test_t152_declared_route_derivation_changes_digest_or_blocks` (five
  expansions); and
- `test_t152_declared_route_derivation_ignores_caller_true_flags` (two
  expansions).

Add command-bound RED contracts with dedicated `test_task8_` names:

- `test_task8_command_authority_is_exact_decimal_bound_and_strictly_ordered`;
- `test_task8_task_route_geometry_is_canonical_and_digest_bound`;
- `test_task8_selected_candidate_hash_fk_and_frame_are_bundle_bound`;
- `test_task8_bundle_is_exact_six_classes_by_five_commands_in_order`;
- `test_task8_zero_command_routes_are_256_action_immutable_holds`;
- `test_task8_each_command_route_records_schedule_endpoints_and_segments`;
- `test_task8_current_digests_are_complete_lowercase_and_contract_bound`;
- `test_task8_command_matrix_mutation_fails_closed`;
- `test_task8_bundle_class_command_motif_and_segment_digests_recompute`;
- `test_task8_digest_mutation_fails_closed` (bundle, class, command, motif,
  segment, task geometry, workspace, geometry, and policy expansions); and
- `test_task8_runner_reexports_pure_route_bundle_functions_without_copying`.

The Task 8 requirement-to-test mapping is fixed:

| Task 8 requirement | RED contract |
|---|---|
| exact five-command Decimal/float64 authority | `test_task8_command_authority_is_exact_decimal_bound_and_strictly_ordered` |
| single A/P/R and press-axis authority | `test_task8_task_route_geometry_is_canonical_and_digest_bound` |
| pure selected-candidate hash, measured FK, and frame binding | `test_task8_selected_candidate_hash_fk_and_frame_are_bundle_bound` |
| exact 6 classes x 5 commands and zero hold | `test_task8_bundle_is_exact_six_classes_by_five_commands_in_order`, `test_task8_zero_command_routes_are_256_action_immutable_holds` |
| local and continuous action schedules | the three local and three continuous existing expansions |
| ordered endpoints and full segments | `test_task8_each_command_route_records_schedule_endpoints_and_segments` |
| exact five current digests and parsed contract binding | `test_task8_current_digests_are_complete_lowercase_and_contract_bound` |
| command matrix cannot drift | `test_task8_command_matrix_mutation_fails_closed` |
| bundle/class/command/motif/segment digest closure | `test_task8_bundle_class_command_motif_and_segment_digests_recompute`, `test_task8_digest_mutation_fails_closed` expansions |
| missing/reordered/partial/nonfinite/workspace/contact fail closed | six existing direct-validator expansions |
| selected pose/root/solid/workspace/policy mutations | five existing declared-solid mutation expansions |
| caller truth flags ignored | two existing caller-flag expansions |
| runner owns no copied algorithm | `test_task8_runner_reexports_pure_route_bundle_functions_without_copying` |
| Task 10 remains separate | existing plan-motif, six executor-motif, and seven orchestration-route expansions stay RED |

**Historical selection boundary**

The former broad expression selected 29 assertion RED and 2 controls. Fifteen
failures belong to Task 8; fourteen belong to Task 10: one plan-motif, six
executor-motif, and seven orchestration-route expansions. The broad expression
is retained only as historical ownership information and is not run as a Task 8
checkpoint.

**Task 8A command**

Use exact existing function node IDs and the dedicated new prefix:

```bash
python -m pytest -q \
  tests/test_g1_pose_conditioned_tracking_cli.py::test_t152_all_six_complete_routes_are_required_before_scene_acquisition \
  tests/test_g1_pose_conditioned_tracking_cli.py::test_t152_local_class_executes_plus16_minus32_plus16_in_every_window \
  tests/test_g1_pose_conditioned_tracking_cli.py::test_t152_continuous_class_consumes_decimal_endpoint_reflection_schedule \
  tests/test_g1_pose_conditioned_tracking_cli.py::test_t152_motif_digest_exact_scalar_and_float64_materialization_cross_check \
  tests/test_g1_pose_conditioned_tracking_cli.py::test_t152_route_builder_derives_all_six_records_from_declared_solids \
  tests/test_g1_pose_conditioned_tracking_cli.py::test_t152_declared_route_derivation_changes_digest_or_blocks \
  tests/test_g1_pose_conditioned_tracking_cli.py::test_t152_declared_route_derivation_ignores_caller_true_flags

python -m pytest -q tests/test_g1_pose_conditioned_tracking_cli.py \
  -k 'test_task8_'

python -m pytest -q \
  tests/test_g1_pose_conditioned_tracking_cli.py::test_t152_positive_six_route_fixture_is_complete_and_canonical \
  tests/test_g1_pose_conditioned_tracking_cli.py::test_t152_positive_motif_fixtures_are_canonical_and_self_consistent

python -m pytest -q \
  tests/test_g1_pose_conditioned_tracking_cli.py::test_t152_plan_carries_consumable_canonical_motif_not_only_class_label \
  tests/test_g1_pose_conditioned_tracking_cli.py::test_t152_executor_consumes_exact_class_motif_schedule \
  tests/test_g1_pose_conditioned_tracking_cli.py::test_t152_orchestration_route_failure_blocks_factory_plan_and_success_evidence
```

The final command must remain exactly 14 Task 10 assertion RED. Do not use a
keyword expression containing generic `route` or `motif`.
Before production changes, every corrected/new behavior node must be an
assertion RED for the missing command-bound capability, both fixture controls
must remain GREEN, and Task 10 ownership must remain unchanged. Collection,
import, fixture, path, syntax, and Isaac errors must all be zero.

**Task 8A stop conditions**

Stop if a Task 10 node is changed, a test still supplies raw formal geometry or
handwritten A/P/R, a current digest family is omitted, caller truth becomes an
authority, or any failure is not the intended assertion RED.

**Task 8A commit**

`test(g1): correct Task 8 route bundle contracts`

This commit is tests-only. Task 8B may begin only after the corrected RED
partition is reviewed.

### Task 8B: Implement the pure command-bound bundle

**Production files**

- Modify: `isaac_tactile_libero/runtime/g1_tracking.py`
- Modify: `isaac_tactile_libero/runtime/g1_contact_exclusion.py`
- Modify: `scripts/run_g1_tracking_envelope.py` for direct import-safe re-export
  only.
- Do not implement any Task 9 evidence loader or Task 10 plan, executor,
  orchestration, factory, scene, evidence, aggregation, close, or shutdown API.

**Public interfaces**

```python
def derive_g1_pose_conditioned_routes(
    *,
    selected_candidate: Mapping[str, object],
    selected_pose_sha256: str,
    class_definitions: Sequence[Mapping[str, object]],
    task_route_geometry: Mapping[str, object],
    command_matrix_m: Sequence[float],
    workspace_limits: Mapping[str, object],
    geometry_contract: PressButtonGeometryContract,
    current_input_digests: Mapping[str, str],
) -> Mapping[str, object]: ...

def validate_g1_pose_conditioned_routes(
    *,
    route_bundle: Mapping[str, object],
    geometry_contract: PressButtonGeometryContract,
    workspace_limits: Mapping[str, object],
    current_input_digests: Mapping[str, str],
) -> ContactExclusionRouteResult: ...
```

**Task 8B.1: authorities and derivation**

- [ ] Add `G1_TRACKING_COMMANDS_M` and
  `G1_TRACKING_COMMAND_DECIMAL_STRINGS` as the one co-located five-command
  authority; make `build_g1_multiclass_tracking_plan()` consume it.
- [ ] Add `g1_press_button_task_route_geometry()` with world-frame A/P/R, press
  axis, schema, and canonical digest.
- [ ] Derive `S` only from finite selected-candidate measured FK.
- [ ] Construct all six classes by five commands, with a 256-action zero hold,
  four exact local windows, and existing Decimal phase-reflected schedules.
- [ ] Materialize every ordered endpoint and continuous segment without dropping
  crossings, reversals, or zero-length holds.
- [ ] Bind selected pose, class definition, command, task geometry, workspace,
  geometry, policy, root transform, and all five current digests.

Focused corrected RED should become GREEN for command authority, task geometry,
bundle shape, schedules, endpoints, and construction digests while validation
failure nodes remain RED.

Commit:

`feat(g1): derive command-bound pose routes`

**Task 8B.2: independent validation**

- [ ] Validate schema, selected pose, exact class/command order, all digests,
  finite endpoints, and the continuous world-frame workspace proof.
- [ ] Invoke `validate_contact_exclusion_routes()` once per canonical command;
  reuse its OBB and capped-cylinder segment validators and do not copy their
  analytic algorithms.
- [ ] Transpose the five lower-level results into six command-bound class
  records while retaining every obstacle and segment result.
- [ ] Ignore caller `workspace_valid`, `contact_exclusion_valid`,
  `route_complete`, and `finite` claims.
- [ ] Raise exact non-empty `G1ValidationError` codes:
  `G1_C1_ROUTE_PROVENANCE_INVALID`,
  `G1_C1_CONTACT_EXCLUSION_ROUTE_INVALID`, or
  `G1_C1_CONTACT_EXCLUSION_DIGEST_MISMATCH`.
- [ ] Return qualification true only when every one of the 30 command routes
  passes; full-robot static qualification remains false.
- [ ] Re-export the two public pure functions from the runner without an
  algorithm wrapper.

After this commit, all exact Task 8 node IDs are GREEN, Task 10 nodes remain
assertion RED, and `tests/test_g1_contact_exclusion_geometry.py` remains fully
GREEN without simulator startup.

Commit:

`feat(g1): validate command-bound declared-solid routes`

**Task 8B stop conditions**

Stop if implementation trusts caller flags, accepts raw formal geometry,
duplicates A/P/R or analytic solid algorithms, changes the five commands or
clearance, uses a design-only minimum as a result, omits any action segment,
adds any Task 9/10 partial API, imports or starts Isaac, or changes the existing
Task 10 RED outcome.

Task 8 depends on Task 5 and Task 7B GREEN. Task 9 depends on the validated
bundle contract. Task 10 consumes the Task 8 bundle plus the Task 9 verified
selected pose. Isaac Sim is not allowed in Task 8A or Task 8B.

## Task 9: Verify selected C2a evidence and current-input freshness

Task 9 begins only after the Task 8 command-bound bundle contract is GREEN. It
owns evidence/current-input freshness and does not rebuild routes. Task 10 later
consumes both the validated Task 8 bundle and the Task 9 verified selected pose.

**Files**

- Modify: `scripts/run_g1_tracking_envelope.py`
- Modify: `scripts/run_g1_static_pose_qualification.py`
- Modify: `isaac_tactile_libero/robots/fr3_static_pose_runtime.py`
- Modify: `tests/test_g1_pose_conditioned_tracking_cli.py`
- Modify: `tests/test_g1_static_pose_qualification.py`
- Modify: `tests/test_g1_static_pose_runtime_cli.py`

**Types and signatures**

Keep the pure loader import-safe in `scripts/run_g1_tracking_envelope.py`, or move
it to a pure runtime module and re-export it from the script if script size makes
that boundary clearer. The public seam remains:

```python
@dataclass(frozen=True)
class G1CurrentInputDigests:
    task_config_sha256: str
    robot_config_sha256: str
    fr3_asset_sha256: str
    task_card_sha256: str
    geometry_sha256: str

@dataclass(frozen=True)
class C2ASelectedPoseEvidence:
    evidence_dir: Path
    report: Mapping[str, object]
    candidate_record: Mapping[str, object]
    selected_pose_id: str
    selected_pose_sha256: str
    repository_commit: str

def load_g1_c2a_selected_pose_evidence(
    evidence_dir: Path,
) -> C2ASelectedPoseEvidence: ...

def validate_g1_c2a_current_input_provenance(
    evidence: C2ASelectedPoseEvidence,
    current: G1CurrentInputDigests,
) -> None: ...
```

Add a required CLI argument:

```python
parser.add_argument("--c2a-evidence", type=Path, required=True)
parser.add_argument(
    "--task-card",
    type=Path,
    default=Path("configs/tasks/cards/press_button.v1.yaml"),
)
```

Before constructing a runtime factory, the loader verifies directory existence,
`checksums.sha256`, `report.json`, `offline_candidates.jsonl`, exactly one
selected real candidate, no duplicate/synthetic candidate, selected ID agreement,
and an independently recomputed hash of the JSONL record. A hash string copied
from the report is not evidence. It validates joint order, frame, Lula identity,
asset/config provenance, and non-null valid solver/FK fields.

Current digests are freshly computed from the task config, robot config, resolved
FR3 asset, task card, and parsed geometry. Candidate/report agreement is
insufficient when any current input differs. Once Task 6 changes config/code,
attempt-02 must produce this exact top-level blocker before factory creation:

```text
CURRENT_C2A_REFRESH_REQUIRED_AFTER_GEOMETRY_SCHEMA_CHANGE
```

The report/manifest records the non-empty explanation and historical evidence
path, returns exit code 1, and performs one close after blocker evidence is
durably written. It does not edit, copy, or rehash attempt-02.

Task 9 is the first checkpoint at which the complete freshness behavior becomes
GREEN: evidence loading and checksum validation, candidate-hash recomputation,
all five current-input digest families, pre-factory exact blocker emission, and
report/manifest/exit/unique-close propagation. Task 6 supplies the stale digest
fact only; it cannot satisfy these orchestration contracts.

Extend new C2a candidate/report/manifest provenance to carry
`task_card_sha256` and `geometry_sha256`. Add the task-card path to the static
qualification CLI and `C2ARealSceneFactory` input so the separately approved
fresh run can establish current provenance. The retained attempt-02 files remain
immutable.

**Owned RED nodes**

- `test_t152_cli_requires_explicit_c2a_evidence_directory`
- `test_t152_main_loads_c2a_evidence_before_factory_builder`
- `test_t152_loader_reads_checksums_report_candidates_and_recomputes_selected_hash`
- `test_t152_loader_returns_jsonl_candidate_instead_of_hardcoded_pose`
- `test_t152_missing_evidence_argument_stops_before_factory_builder`
- Eight `test_t152_invalid_c2a_evidence_stops_before_factory_builder[...]`
  expansions: missing directory, checksums, report, candidates, checksum
  mismatch, report tamper, candidate tamper, duplicate candidate.
- Three `test_t152_current_input_digest_mismatch_stops_before_runtime_creation[...]`
  expansions: task config, robot config, FR3 asset; extend the same contract to
  task card and geometry as named new expansions.
- Existing pose/hash/joint/frame/asset/config identity mismatch expansions.

**Steps**

- [ ] Add checksum parsing and candidate hash recomputation from actual evidence
  files; do not use test constants or embedded joint values.
- [ ] Enforce candidate uniqueness, real-runtime truth, and exact provenance.
- [ ] Compute all current digests before factory construction and emit the
  freshness blocker for attempt-02 after Task 6.
- [ ] Extend future C2a static evidence with task-card and geometry digests while
  retaining its candidate-local rejection behavior.
- [ ] Add required CLI paths and thread them through pure orchestration before
  any lazy Isaac factory is invoked.
- [ ] Make all Task 9-owned RED nodes GREEN and prove factory call count is zero
  on every failure.

**Commands and expected results**

```bash
python -m pytest -q tests/test_g1_pose_conditioned_tracking_cli.py \
  -k 'c2a_evidence or selected_candidate or selected_pose or current_input or provenance'
# Before implementation: loader/current-input assertions fail.
# After implementation: all selected nodes pass; attempt-02 is rejected as stale
# under the migrated current config with the exact refresh blocker.

python -m pytest -q \
  tests/test_g1_static_pose_qualification.py \
  tests/test_g1_static_pose_runtime_cli.py
# Expected after implementation: all existing GREEN nodes pass and new digest
# propagation tests pass; no Isaac startup occurs because fakes are injected.

python scripts/run_g1_tracking_envelope.py --help
python scripts/run_g1_static_pose_qualification.py --help
# Expected: both exit 0 without importing/starting Isaac.
```

**Stop conditions**

Stop if evidence can be supplied without a directory, if report strings replace
JSONL hash recomputation, if any current digest is omitted, if attempt-02 is
accepted after migration, if retained evidence changes, or if factory creation
precedes validation.

**Commit**

`feat(g1): verify current C2a pose evidence`

Before and after commit run the loader selection, static fake/evidence files, and
both help commands. After commit every Task 9 node is GREEN and attempt-02 is
explicitly historical/stale. Task 9 depends on Tasks 6 and 8; Task 10 depends on
its verified pose record. Isaac Sim is not allowed.

## Task 10: Wire pre-Play pose authoring and the multiclass real CLI

**Files**

- Modify: `scripts/run_g1_tracking_envelope.py`
- Modify: `isaac_tactile_libero/runtime/g1_tracking.py` only where existing pure
  multiclass APIs need a narrow extension.
- Modify: `isaac_tactile_libero/robots/fr3_static_pose_diagnostic.py` only if a
  reusable protocol must be exported; do not duplicate its algorithm.
- Modify: `tests/test_g1_pose_conditioned_tracking_cli.py`
- Modify directly affected runner/evidence tests, including
  `tests/test_g1_tracking_envelope.py` and
  `tests/test_g1_press_button_runner_evidence.py` when collected names show they
  own the affected schema.

**Orchestration interfaces**

```python
def build_g1_pose_conditioned_tracking_plan(
    *,
    selected_pose: C2ASelectedPoseEvidence,
    validated_routes: ContactExclusionRouteResult,
) -> tuple[Mapping[str, object], ...]: ...

def build_g1_pose_conditioned_runtime_preplay(
    *,
    stage_builder: Callable[..., object],
    selected_candidate: Mapping[str, object],
    joint_order: Sequence[str],
    preplay_adapter: UsdPhysxC2APrePlayAdapter,
) -> object: ...

def execute_g1_pose_conditioned_tracking_trial(
    scene: object,
    trial: Mapping[str, object],
) -> Mapping[str, object]: ...

def run_g1_pose_conditioned_tracking_plan(
    *,
    plan: Sequence[Mapping[str, object]],
    factory_builder: Callable[..., object],
) -> Mapping[str, object]: ...

def write_g1_pose_conditioned_tracking_evidence(
    *, output_dir: Path, result: Mapping[str, object]
) -> None: ...

def orchestrate_g1_pose_conditioned_tracking(
    *, args: argparse.Namespace,
    factory_builder: Callable[..., object],
    writer_builder: Callable[..., object],
) -> int: ...
```

Add `_PoseConditionedIsaacTrackingScene` as the real lazy-runtime scene and keep
the script's `_IsaacSceneFactory` name as the tested factory boundary; change
that factory to construct the new scene. During the existing stage-builder callback, before
`FR3DifferentialIKRuntime.build()` completes Play startup, call the existing
`author_c2a_joint_state_before_play()` with an existing
`UsdPhysxC2APrePlayAdapter` (or an exactly equivalent exported protocol). Author
position, zero velocity, and matching drive targets. Validate authored values
before allowing build completion. Post-Play authoring, active-runtime teleport,
and nonzero pre-position action are blockers.

The CLI `main()` must call the pose-conditioned orchestration and
`build_g1_multiclass_tracking_plan()`; it must not call legacy
`build_g1_tracking_plan()`. The plan is exactly 5 commands × 6 canonical classes
× 3 fresh scenes = 90 trials, ordered command ascending, canonical class order,
then scene 0/1/2. Every scene has distinct scene token, stage, articulation,
latch, and instance identity while using the same verified joint order/values.

Each trial executes exactly 64 immutable zero-readiness actions, three physics
substeps per action, then exactly 256 motif actions in four ordered contiguous
64-action windows. It consumes the canonical schedule rather than a class label.
Every nonzero measurement calls the existing shared qualifying Lula kernel;
`compatibility_smoke`/Jacobian results are never cap-eligible.

Use the existing `run_g1_multiclass_tracking_plan()` and
`aggregate_g1_multiclass_tracking_envelope()` for candidate-local stop-tail,
class completeness, tested-only cap selection, and no interpolation/upward
rounding. Do not copy those algorithms into the script.

Evidence writer cross-records pose ID/hash, class ID/version, motif/schedule
digest, command, scene identities, route/geometry/config/card/asset/joint/frame
provenance, readiness/measurement/window counts, real/synthetic truth, CPU
physics policy, contact/collision/penetration fields, force truth, and
post-abort actuation. JSONL counts must equal report/manifest counts. Checksums
finish before exactly one `close(exit_code)`. Writer failure emits
`G1_C1_EVIDENCE_WRITE_FAILED`, cannot leave an acceptable manifest, and closes
with exit 1.

**Owned existing T152 RED behavior**

- `main` selects multiclass and exact 90-trial order.
- Fresh-scene identity and pre-Play authoring/lifecycle seam nodes.
- 64 readiness, 256 measurement, and four 64-window nodes.
- Six executor-motif expansions and shared Lula kernel node.
- Compatibility exclusion and multiclass aggregation nodes.
- Full evidence-provenance node.
- Stop-tail/systemic exit, checksum-before-close, writer failure, and four
  post-abort/force-truth expansions.
- All Task 9/8 outputs are consumed without weakening their checks.

**Steps**

- [ ] Replace the legacy `main()` path with verified evidence → geometry/routes
  → multiclass plan → lazy factory → evidence orchestration.
- [ ] Add the real pre-Play stage-builder seam by reusing the C2a authoring
  contract.
- [ ] Execute exact readiness and motif schedules with shared kernel and fresh
  identity checks.
- [ ] Call existing multiclass runner/aggregator and preserve stop-tail.
- [ ] Upgrade trial/sample/report/manifest writers and unique close lifecycle.
- [ ] Make all remaining T152 RED nodes GREEN without editing approved
  assertions except the Task 2 documented schema migration.
- [ ] Prove source/import boundaries do not start Isaac in tests or `--help`.

**Commands and expected results**

```bash
python -m pytest -q tests/test_g1_pose_conditioned_tracking_cli.py
# Before Task 10: remaining lifecycle/orchestration/evidence nodes are RED.
# After Task 10: every collected node in the file passes, including the four
# original GREEN controls and all migrated/new contracts.

python -m pytest -q \
  tests/test_g1_tracking_envelope.py \
  tests/test_g1_press_button_runner_evidence.py \
  tests/test_g1_static_pose_runtime_cli.py \
  tests/test_g1_static_pose_qualification.py
# Expected after implementation: all non-intentional nodes pass with fakes; no
# application startup.

python scripts/run_g1_tracking_envelope.py --help
# Expected: exit 0 without Isaac import/startup.
```

**Stop conditions**

Stop if the legacy plan/aggregator remains in `main`, if pose authoring occurs
after Play, if any count/order differs, if an algorithm is copied instead of
reused, if compatibility output becomes cap evidence, if close precedes
checksums, if post-abort actuation is nonzero, or if a unit test needs Isaac.

**Commit**

`feat(g1): wire pose-conditioned multiclass CLI`

Before commit, the T152 file must be wholly GREEN and focused runner/evidence
files must pass. After commit rerun both selections and the help command. Task 10
depends on Tasks 8 and 9; Task 11 validates the complete chain. Isaac Sim is not
allowed.

## Task 11: Complete GREEN verification and close T152

### D3 corrective design checkpoint

`V1=7ef680b0a5d062c682a2d1715539e7b32f09b538` completed the first
verification-infrastructure implementation, but its retained
`/tmp/g1-t152-pre-projection` run stopped with exactly four portable failures
after collecting 965 selected archive nodes. The failures were caused by an
empty archive-local `.git` directory without a HEAD/index/history, not by
PressButton runtime or physical behavior. The failed directory is immutable
and must not be deleted, overwritten, or reused.

This documentation-only `D3` approves:

```text
SYNTHETIC_CLEAN_GIT_CONTEXT
+ PORTABLE_BLOB_ATTESTATION_WITH_MAIN_CHECKOUT_HISTORY_VERIFICATION
```

After `git archive "$VERIFY_COMMIT" | tar -x -C "$EXPORT_ROOT"`, and before
archive collection or pytest, the later corrective commit `W` implements and
uses the single helper `prepare_portable_git_context(export_root)`. The helper
operates strictly inside the extracted archive, rejects a pre-existing `.git`,
runs `git init`, configures the fixed identity `Portable Verification
<portable-verification@example.invalid>`, sets `portable.archive=true`, runs
`git add -f --all`, and creates exactly one no-GPG commit with both author and
committer dates fixed to `2000-01-01T00:00:00Z` and message
`portable verification archive snapshot`. It requires a resolvable `HEAD`, an
empty `git status --porcelain`, and a true portable marker. It does not copy or
inject any history object, ref, pack, bundle, alternate, graft, replacement,
worktree file, evidence, or output from the main checkout. Exported source
bytes remain exactly the bytes produced by `git archive`; only `.git` metadata
is added.

The existing historical inventory node remains one node with two explicit,
fail-closed modes. In the main checkout it continues to resolve real
`behavior_source_commit:path` and `execution_start_commit:path` objects and
fails if required history is missing. Only when
`git config --bool --get portable.archive` is true may it instead require the
fixture's approved behavior/execution commit strings, require the two fixture
blob IDs to be equal. D4 below supersedes the former requirement that the
current archive source equal those historical blobs. There is no catch-all
missing-history pass.

`W` is RED-to-GREEN and may modify only
`scripts/check_clean_checkout.py`, allowed bodies/helpers/assertions of
existing nodes in `tests/test_clean_checkout_cli.py` and
`tests/test_g1_t152_red_migration_manifest.py`, and this Task 11 plan. It may
not add, delete, rename, or re-parameterize a test node. The focused RED must
be an assertion failure that proves the missing synthetic-Git contract before
implementation. `W` must preserve the frozen counts and digests: full 1091,
main current GREEN 966, portable current GREEN 965, external historical 1,
intentional future RED 125, collection-order digest
`1c8e6a8e9b09da6b06435ea6c75191c5fb4b3c3fa7e1b97161951e65249d45ad`,
and sorted digest
`00a6e84c5d2e1f623f2211db8272ca95859e8050417f7c25cbfeef9afd84efc7`.
The external manifest remains exactly the attempt-02 node.

The report and manifest must record:

```yaml
portable_archive_reads_original_worktree: false
portable_git_context: synthetic_clean_repository
portable_history_objects_injected: false
portable_source_bytes_equal_git_archive: true
```

After the focused RED/GREEN cycle, `W` reruns the four original failures,
clean-checkout tests, migration-manifest tests, all 113 T152 nodes, Task 8's 40
nodes, Task 9's 32 nodes, 80 static-qualification nodes, 108 Tasks 4-7 nodes,
the original 748 GREEN inventory, current GREEN 966, future RED 125 with the
78/29/10/8 classification, exact hard-limit 4, contact analytics 38, the
deprecated-API scan, CLI help/import boundaries, and full collection. Every
frozen node-ID file and both digests must remain unchanged.

The D3 topology and parent invariants, superseded by D4 below, were:

```text
E_impl -> D1 -> D2 -> V1 -> D3 -> W -> P_t152
V1 = 7ef680b0a5d062c682a2d1715539e7b32f09b538
D3^ = V1
W^ = D3
P_t152^ = W
```

Under D3, `W` would have used `/tmp/g1-t152-pre-projection-w`. Any
failure in focused RED/GREEN, full verification, counts, digests, source
isolation, synthetic-Git provenance, blob attestation, or topology stops the
workflow without `P_t152` or G0. This D3 section supersedes older `V`/`P`
wording below wherever it conflicts.

### D4 historical/current blob separation checkpoint

`D4` is a documentation-and-fixture-only child of
`D3=66551b9f55729b920adb5fda64f9b52a9852b8f7`. It adds exactly one baseline
inventory metadata field:

```json
"portable_current_source_blob_git": "2839e2ff67864c692f1bdb9ae5dc64e2dea34f91"
```

It leaves the two historical fields unchanged:

```json
"behavior_source_test_blob_git": "b9864a8b8eea289fa61eb7e3e41633c35947c5ef"
"execution_start_test_blob_git": "b9864a8b8eea289fa61eb7e3e41633c35947c5ef"
```

The historical blobs prove what was approved at the two historical commits;
the portable-current blob proves what the current archive actually executes.
Both proof classes are mandatory and independent.

In the real main checkout, the existing inventory node must run these exact
proofs without fallback:

```bash
git rev-parse \
  d5fdac8dc109adfd23946bdff5352a26d7081302:tests/test_g1_pose_conditioned_tracking_cli.py
git rev-parse \
  46c771e0b83ab81479f0a87629e0d2709f56aac0:tests/test_g1_pose_conditioned_tracking_cli.py
git hash-object tests/test_g1_pose_conditioned_tracking_cli.py
```

The first two results must equal their respective historical fixture fields
and `b9864a8b8eea289fa61eb7e3e41633c35947c5ef`. The third result must be
recomputed and equal `portable_current_source_blob_git` and
`2839e2ff67864c692f1bdb9ae5dc64e2dea34f91`. Missing history, a wrong commit
or path, or any blob mismatch fails closed. A non-portable checkout never
falls back to portable semantics.

In the synthetic archive, `git config --bool --get portable.archive` must
return exactly `true`; HEAD must resolve; and `git status --porcelain` must be
empty. The inventory node recomputes `git hash-object` for the same current
source and compares it to the independent portable-current field. It also
requires both approved historical commit strings and historical blob fields
to be present unchanged, while explicitly recording that the historical
objects were not injected and were not reverified in the archive. It must not
call or fabricate a successful historical `commit:path` lookup.

The current blob must never be written into a historical field, and a
historical blob must never be written into the portable-current field. The
current blob is recomputed rather than derived from another fixture field.
The archive never receives history, bundles, alternates, replacement refs,
attempt-02 material, outputs, or evidence and never reads the original
worktree.

`W` must add these report/manifest values:

```yaml
portable_archive_reads_original_worktree: false
portable_git_context: synthetic_clean_repository
portable_history_objects_injected: false
portable_source_bytes_equal_git_archive: true
portable_current_source_blob_git: 2839e2ff67864c692f1bdb9ae5dc64e2dea34f91
historical_behavior_blob_git: b9864a8b8eea289fa61eb7e3e41633c35947c5ef
historical_execution_start_blob_git: b9864a8b8eea289fa61eb7e3e41633c35947c5ef
historical_objects_verified_in_main_checkout: true
historical_objects_verified_in_portable_archive: false
```

The active topology is:

```text
E_impl -> D1 -> D2 -> V1 -> D3 -> D4 -> W -> P_t152
D4^ = D3
W^ = D4
P_t152^ = W
```

The W pre-projection directory is the new immutable
`/tmp/g1-t152-pre-projection-d4-w`. The failed
`/tmp/g1-t152-pre-projection` remains untouched. D4 changes no Python test,
node ID, parameterization, task state, production code, runtime config,
evidence, threshold, or command matrix. This section supersedes D3 and older
Task 11 wording wherever blob semantics, topology, parentage, or directory
names conflict.

**Files**

- Design authority:
  `specs/001-benchmark-reconstruction/g1-task11-portable-verification-closure-design.md`.
- Documentation revision `D4` modifies only that design authority, this Task
  11 plan, and `tests/fixtures/g1_t152_baseline_inventory.json`. It does not
  implement verification or modify Python tests.
- Corrective verification-infrastructure commit `W` may modify
  `scripts/check_clean_checkout.py`, only bodies, non-parametrizing
  fixtures/helpers, and assertions of existing nodes in
  `tests/test_clean_checkout_cli.py` and
  `tests/test_g1_t152_red_migration_manifest.py`, and this Task 11 verification
  helper/plan. It may not add, delete, or rename a test function or add,
  remove, rename, or alter a parameterized expansion.
- Projection commit `P_t152` may modify only
  `specs/001-benchmark-reconstruction/tasks.md`, changing T152 from `[ ]` to
  `[x]`, and this plan, recording the already-known implementation, design
  checkpoints, and verification SHAs.
- Produce a commit-bound external-verification attestation and final G0
  repository-integrity evidence at `P_t152`. Neither is runtime or physical
  evidence.

**Commit identities and no-self-reference rule**

- `E_impl=aa47af3946f2f9f934147b4b263affe345a9d450` is the last production
  implementation commit after Tasks 1–10.
- `D1=d561f3be49b3ba059286818e325adc81b5b0b269` is the first approved
  portable-closure design checkpoint and satisfies `D1^=E_impl`.
- `D2=6d234a4bf8d8420fbd58d771e9828af2f9d0efa6` is the second design
  checkpoint and satisfies `D2^=D1`.
- `V1=7ef680b0a5d062c682a2d1715539e7b32f09b538` is the first verification
  implementation and satisfies `V1^=D2`; its failed projection is retained.
- `D3=66551b9f55729b920adb5fda64f9b52a9852b8f7` is the synthetic-Git
  design checkpoint and satisfies `D3^=V1`.
- `D4=ac84bc39cf70d1e45c95ccf3e9e4fdf0ff77cac8` is the approved
  historical/current-blob separation revision and satisfies `D4^=D3`.
- `W` is the later corrective RED-to-GREEN verification-infrastructure commit
  and must satisfy `W^=D4`.
- Before editing for `W`, capture clean-`D4` full/current node-ID lists. After
  `W`, compare all four lists byte-for-byte and stop on any drift.
- Run the complete pre-projection verification suite only at clean `W`, using
  `/tmp/g1-t152-pre-projection-d4-w`.
- After it passes, edit only `tasks.md` and this plan, record the literal
  already-known `E_impl`, `D1`, `D2`, `V1`, `D3`, `D4`, and `W`, and create the
  unique projection/status commit `P_t152` with message
  `docs(g1): complete T152 geometry integration`.
- Do not write `P_t152`'s SHA into either tracked file. Define
  `FINAL_E2=P_t152` only
  after commit creation with `FINAL_E2=$(git rev-parse HEAD)`.
- Verify `P_t152^=W`, rerun the complete suite at `P_t152`, compare its four node-ID
  lists, two current-GREEN digests, counts, and normalized JUnit outcomes with
  the clean `W` snapshot, create the `P_t152`-bound external-verification
  attestation, then create G0 evidence bound to `P_t152`.
- After `P_t152`, no tracked file may change. Any tracked change invalidates final
  verification/freshness and requires a new reviewed projection commit.

The required topology is therefore:

```text
E_impl = aa47af3946f2f9f934147b4b263affe345a9d450
→ D1 = d561f3be49b3ba059286818e325adc81b5b0b269
→ D2 = 6d234a4bf8d8420fbd58d771e9828af2f9d0efa6
→ V1 = 7ef680b0a5d062c682a2d1715539e7b32f09b538
→ D3 = 66551b9f55729b920adb5fda64f9b52a9852b8f7
→ D4 = ac84bc39cf70d1e45c95ccf3e9e4fdf0ff77cac8
→ W = corrective verification infrastructure
→ P_t152 = final projection/status
→ FINAL_E2 = P_t152
```

**Reusable verification suite**

The function below was established by `V1` and is corrected by `W`, then
executed once at clean `W` and again at `P_t152`. Its ordered checks are:
geometry schema, analytic
geometry, mechanism/task card, T152, affected runtime/safety, exact hard limit,
original 748 GREEN, dynamic main-checkout current GREEN, intentional future-RED
125, full portable/external/future classification, dual list/digest snapshots,
deprecated/import checks, and a tracked-only portable archive. Dynamic current
GREEN is derived from the current complete collection minus the exact frozen
125 future-RED IDs; it is not an immutable Task 1 selection.

The four authoritative files are:

```text
all-nodeids.collection.txt       # 1091, pytest collection order
all-nodeids.sorted.txt           # 1091, sort/comm authority only
current-green.collection.txt     # 966, collection order retained
current-green.sorted.txt         # 966, classification authority only
```

The two approved current-GREEN digests are independent:

```text
collection-order SHA-256 = 1c8e6a8e9b09da6b06435ea6c75191c5fb4b3c3fa7e1b97161951e65249d45ad
sorted SHA-256           = 00a6e84c5d2e1f623f2211db8272ca95859e8050417f7c25cbfeef9afd84efc7
```

The external-evidence manifest contains exactly this one node:

```text
tests/test_g1_pose_conditioned_tracking_cli.py::test_t152_attempt02_is_historical_and_emits_exact_refresh_blocker_before_close
```

**Fixed V1/W test-node contract**

At clean `D2`, `tests/test_clean_checkout_cli.py` has nine test functions and
exactly 12 collected node IDs because one existing function has four frozen
parameterized expansions. `V1` preserved and `W` must preserve these exact
node IDs byte-for-byte:

```text
tests/test_clean_checkout_cli.py::test_clean_checkout_green_command_deselects_only_manifest_nodes
tests/test_clean_checkout_cli.py::test_clean_checkout_parses_future_red_junit_without_calling_failures_passes
tests/test_clean_checkout_cli.py::test_clean_checkout_plan_has_required_isolated_steps
tests/test_clean_checkout_cli.py::test_clean_checkout_rejects_duplicate_manifest_node
tests/test_clean_checkout_cli.py::test_clean_checkout_rejects_invalid_future_red_junit_outcomes[0-changes0-return code]
tests/test_clean_checkout_cli.py::test_clean_checkout_rejects_invalid_future_red_junit_outcomes[1-changes1-unexpected passes]
tests/test_clean_checkout_cli.py::test_clean_checkout_rejects_invalid_future_red_junit_outcomes[1-changes2-errors]
tests/test_clean_checkout_cli.py::test_clean_checkout_rejects_invalid_future_red_junit_outcomes[1-changes3-skipped]
tests/test_clean_checkout_cli.py::test_clean_checkout_rejects_missing_manifest_node
tests/test_clean_checkout_cli.py::test_clean_checkout_rejects_uncollected_future_node
tests/test_clean_checkout_cli.py::test_clean_checkout_report_records_future_red_count_and_digest
tests/test_clean_checkout_cli.py::test_future_red_manifest_has_exact_unique_nodeids
```

The ten approved D1 RED contracts use only that fixed set:

| D1 RED contract | Exact existing node ID or IDs |
|---|---|
| 1. Exact external-manifest count, ordering, uniqueness, and spelling | `tests/test_clean_checkout_cli.py::test_future_red_manifest_has_exact_unique_nodeids` |
| 2. Collection membership of the external node | `tests/test_clean_checkout_cli.py::test_clean_checkout_rejects_uncollected_future_node` |
| 3. Disjoint external and future-RED sets | `tests/test_clean_checkout_cli.py::test_clean_checkout_rejects_duplicate_manifest_node` |
| 4. Portable deselection of exactly 125 future-RED plus one external node | `tests/test_clean_checkout_cli.py::test_clean_checkout_green_command_deselects_only_manifest_nodes` |
| 5. Portable selection and PASS count of exactly 965 | `tests/test_clean_checkout_cli.py::test_clean_checkout_parses_future_red_junit_without_calling_failures_passes`<br>`tests/test_clean_checkout_cli.py::test_clean_checkout_rejects_invalid_future_red_junit_outcomes[0-changes0-return code]`<br>`tests/test_clean_checkout_cli.py::test_clean_checkout_rejects_invalid_future_red_junit_outcomes[1-changes1-unexpected passes]`<br>`tests/test_clean_checkout_cli.py::test_clean_checkout_rejects_invalid_future_red_junit_outcomes[1-changes2-errors]`<br>`tests/test_clean_checkout_cli.py::test_clean_checkout_rejects_invalid_future_red_junit_outcomes[1-changes3-skipped]` |
| 6. Complete classification of exactly 1091 nodes | `tests/test_clean_checkout_cli.py::test_clean_checkout_rejects_missing_manifest_node` |
| 7. Required G0 report/manifest fields and counts | `tests/test_clean_checkout_cli.py::test_clean_checkout_report_records_future_red_count_and_digest` |
| 8. Portable archive isolation and no external-evidence projection into the archive | `tests/test_clean_checkout_cli.py::test_clean_checkout_plan_has_required_isolated_steps` |
| 9. Generation and comparison of both current-GREEN list views and digests | `tests/test_clean_checkout_cli.py::test_clean_checkout_report_records_future_red_count_and_digest`; `tests/test_clean_checkout_cli.py::test_future_red_manifest_has_exact_unique_nodeids` |
| 10. P_t152-bound attestation validation, exact stale blocker/factory-zero contract, and attempt checksum preservation | `tests/test_clean_checkout_cli.py::test_clean_checkout_plan_has_required_isolated_steps`<br>`tests/test_clean_checkout_cli.py::test_clean_checkout_report_records_future_red_count_and_digest`<br>`tests/test_clean_checkout_cli.py::test_clean_checkout_rejects_invalid_future_red_junit_outcomes[0-changes0-return code]`<br>`tests/test_clean_checkout_cli.py::test_clean_checkout_rejects_invalid_future_red_junit_outcomes[1-changes1-unexpected passes]`<br>`tests/test_clean_checkout_cli.py::test_clean_checkout_rejects_invalid_future_red_junit_outcomes[1-changes2-errors]`<br>`tests/test_clean_checkout_cli.py::test_clean_checkout_rejects_invalid_future_red_junit_outcomes[1-changes3-skipped]` |

Reuse is intentional: an existing node may assert more than one related
contract, but no new node or expansion is permitted. Full collection remains
exactly 1091, current GREEN remains exactly 966, and both approved digests
remain unchanged.

```bash
set -euo pipefail

capture_t152_nodeids() {
  SNAPSHOT_COMMIT=$1
  SNAPSHOT_DIR=$2
  test "$(git rev-parse HEAD)" = "$SNAPSHOT_COMMIT"
  test -z "$(git status --porcelain)"
  test ! -e "$SNAPSHOT_DIR"
  mkdir -p "$SNAPSHOT_DIR"

  awk 'NF && $1 !~ /^#/' \
    configs/repository/intentional-future-red-nodeids.txt | sort -u \
    > "$SNAPSHOT_DIR/intentional-future-red-nodeids.txt"
  test "$(wc -l < "$SNAPSHOT_DIR/intentional-future-red-nodeids.txt")" -eq 125
  python -m pytest --collect-only -q > "$SNAPSHOT_DIR/full-collection.log"
  awk '/^tests\// && /::/' "$SNAPSHOT_DIR/full-collection.log" \
    > "$SNAPSHOT_DIR/all-nodeids.collection.txt"
  sort -u "$SNAPSHOT_DIR/all-nodeids.collection.txt" \
    > "$SNAPSHOT_DIR/all-nodeids.sorted.txt"
  test "$(wc -l < "$SNAPSHOT_DIR/all-nodeids.collection.txt")" -eq 1091
  test "$(wc -l < "$SNAPSHOT_DIR/all-nodeids.sorted.txt")" -eq 1091
  python - \
    "$SNAPSHOT_DIR/all-nodeids.collection.txt" \
    "$SNAPSHOT_DIR/intentional-future-red-nodeids.txt" \
    "$SNAPSHOT_DIR/current-green.collection.txt" <<'PY'
import sys

with open(sys.argv[2], encoding="utf-8") as stream:
    future = {line.rstrip("\n") for line in stream if line.strip()}
with open(sys.argv[1], encoding="utf-8") as stream:
    collection = [line.rstrip("\n") for line in stream if line.strip()]
current = [node for node in collection if node not in future]
with open(sys.argv[3], "w", encoding="utf-8") as stream:
    stream.write("\n".join(current) + "\n")
PY
  sort -u "$SNAPSHOT_DIR/current-green.collection.txt" \
    > "$SNAPSHOT_DIR/current-green.sorted.txt"
  test "$(wc -l < "$SNAPSHOT_DIR/current-green.collection.txt")" -eq 966
  test "$(wc -l < "$SNAPSHOT_DIR/current-green.sorted.txt")" -eq 966
  test "$(sha256sum "$SNAPSHOT_DIR/current-green.collection.txt" | awk '{print $1}')" = \
    1c8e6a8e9b09da6b06435ea6c75191c5fb4b3c3fa7e1b97161951e65249d45ad
  test "$(sha256sum "$SNAPSHOT_DIR/current-green.sorted.txt" | awk '{print $1}')" = \
    00a6e84c5d2e1f623f2211db8272ca95859e8050417f7c25cbfeef9afd84efc7
}

run_t152_verification() {
  VERIFY_COMMIT=$1
  VERIFY_LABEL=$2
  EXPECTED_VERIFY_DIR=${3-}
  PRE_W_NODEID_DIR=${4-}
  test "$(git rev-parse HEAD)" = "$VERIFY_COMMIT"
  test -z "$(git status --porcelain)"

  VERIFY_DIR="/tmp/g1-t152-${VERIFY_LABEL}"
  test ! -e "$VERIFY_DIR"
  mkdir -p "$VERIFY_DIR"

  ATTEMPT02_CHECKSUMS=outputs/evidence/G1/c2a-static-preliminary-0ace57ce7169-attempt-02/checksums.sha256
  ATTEMPT02_SHA_BEFORE=$(sha256sum "$ATTEMPT02_CHECKSUMS" | awk '{print $1}')
  test "$ATTEMPT02_SHA_BEFORE" = \
    cc53c4b4bc3cefdc7a2363c6446741e3abfc65e768ac0db71123aa593be528ed

  FUTURE_NODEIDS="$VERIFY_DIR/intentional-future-red-nodeids.txt"
  awk 'NF && $1 !~ /^#/' \
    configs/repository/intentional-future-red-nodeids.txt | sort -u \
    > "$FUTURE_NODEIDS"
  test "$(wc -l < "$FUTURE_NODEIDS")" -eq 125
  FOCUSED_DESELECT=()
  while IFS= read -r node; do
    FOCUSED_DESELECT+=(--deselect "$node")
  done < "$FUTURE_NODEIDS"

  python -m pytest -q tests/test_press_button_geometry_contract.py
  python -m pytest -q tests/test_g1_contact_exclusion_geometry.py
  python -m pytest -q \
    tests/test_press_button_mechanism.py \
    tests/test_press_button_task_card_contract.py
  python -m pytest -q tests/test_g1_pose_conditioned_tracking_cli.py
  python -m pytest -q \
    tests/test_g1_tracking_envelope.py \
    tests/test_g1_press_button_runner_evidence.py \
    tests/test_g1_static_pose_runtime_cli.py \
    tests/test_g1_static_pose_qualification.py \
    tests/test_fr3_runtime_safety.py \
    "${FOCUSED_DESELECT[@]}"
  python -m pytest -q \
    tests/test_fr3_runtime_safety.py::test_observed_public_action_displacement_equal_to_exact_hard_limit_passes \
    tests/test_fr3_runtime_safety.py::test_nextafter_above_exact_observed_hard_limit_aborts_without_epsilon \
    tests/test_fr3_runtime_safety.py::test_observed_hard_limit_comparison_source_has_no_epsilon_or_isclose \
    tests/test_fr3_runtime_safety.py::test_physical_safety_config_requires_exact_observed_hard_limit
  python tests/run_g1_node_inventory.py \
    --inventory tests/fixtures/g1_t152_baseline_inventory.json \
    --selection original_green --expect-pass 748

  EXTERNAL_NODEIDS="$VERIFY_DIR/external-evidence-nodeids.txt"
  awk 'NF && $1 !~ /^#/' \
    configs/repository/external-evidence-nodeids.txt \
    > "$EXTERNAL_NODEIDS"
  test "$(wc -l < "$EXTERNAL_NODEIDS")" -eq 1
  test "$(sort -u "$EXTERNAL_NODEIDS" | wc -l)" -eq 1
  test "$(cat "$EXTERNAL_NODEIDS")" = \
    'tests/test_g1_pose_conditioned_tracking_cli.py::test_t152_attempt02_is_historical_and_emits_exact_refresh_blocker_before_close'
  sha256sum configs/repository/external-evidence-nodeids.txt \
    > "$VERIFY_DIR/external-evidence-manifest.sha256"
  comm -12 "$FUTURE_NODEIDS" "$EXTERNAL_NODEIDS" \
    > "$VERIFY_DIR/future-external-overlap.txt"
  test ! -s "$VERIFY_DIR/future-external-overlap.txt"

  ALL_COLLECTION_LOG="$VERIFY_DIR/full-collection.log"
  ALL_COLLECTION_NODEIDS="$VERIFY_DIR/all-nodeids.collection.txt"
  ALL_SORTED_NODEIDS="$VERIFY_DIR/all-nodeids.sorted.txt"
  python -m pytest --collect-only -q > "$ALL_COLLECTION_LOG"
  awk '/^tests\// && /::/' "$ALL_COLLECTION_LOG" \
    > "$ALL_COLLECTION_NODEIDS"
  sort -u "$ALL_COLLECTION_NODEIDS" > "$ALL_SORTED_NODEIDS"
  test "$(wc -l < "$ALL_COLLECTION_NODEIDS")" -eq 1091
  test "$(wc -l < "$ALL_SORTED_NODEIDS")" -eq 1091
  comm -23 "$FUTURE_NODEIDS" "$ALL_SORTED_NODEIDS" \
    > "$VERIFY_DIR/missing-future-red-nodeids.txt"
  test ! -s "$VERIFY_DIR/missing-future-red-nodeids.txt"
  comm -23 "$EXTERNAL_NODEIDS" "$ALL_SORTED_NODEIDS" \
    > "$VERIFY_DIR/missing-external-evidence-nodeids.txt"
  test ! -s "$VERIFY_DIR/missing-external-evidence-nodeids.txt"

  CURRENT_COLLECTION_NODEIDS="$VERIFY_DIR/current-green.collection.txt"
  CURRENT_SORTED_NODEIDS="$VERIFY_DIR/current-green.sorted.txt"
  python - "$ALL_COLLECTION_NODEIDS" "$FUTURE_NODEIDS" \
    "$CURRENT_COLLECTION_NODEIDS" <<'PY'
import sys

with open(sys.argv[2], encoding="utf-8") as stream:
    future = {line.rstrip("\n") for line in stream if line.strip()}
with open(sys.argv[1], encoding="utf-8") as stream:
    collection = [line.rstrip("\n") for line in stream if line.strip()]
current = [node for node in collection if node not in future]
with open(sys.argv[3], "w", encoding="utf-8") as stream:
    stream.write("\n".join(current) + "\n")
PY
  sort -u "$CURRENT_COLLECTION_NODEIDS" > "$CURRENT_SORTED_NODEIDS"
  test "$(wc -l < "$CURRENT_COLLECTION_NODEIDS")" -eq 966
  test "$(wc -l < "$CURRENT_SORTED_NODEIDS")" -eq 966
  COLLECTION_DIGEST=$(sha256sum "$CURRENT_COLLECTION_NODEIDS" | awk '{print $1}')
  SORTED_DIGEST=$(sha256sum "$CURRENT_SORTED_NODEIDS" | awk '{print $1}')
  test "$COLLECTION_DIGEST" = \
    1c8e6a8e9b09da6b06435ea6c75191c5fb4b3c3fa7e1b97161951e65249d45ad
  test "$SORTED_DIGEST" = \
    00a6e84c5d2e1f623f2211db8272ca95859e8050417f7c25cbfeef9afd84efc7
  printf '1091\n' > "$VERIFY_DIR/all-nodeids-count.txt"
  printf '966\n' > "$VERIFY_DIR/current-green.collection.count.txt"
  printf '966\n' > "$VERIFY_DIR/current-green.sorted.count.txt"
  printf '%s\n' "$COLLECTION_DIGEST" \
    > "$VERIFY_DIR/current-green.collection.sha256.txt"
  printf '%s\n' "$SORTED_DIGEST" \
    > "$VERIFY_DIR/current-green.sorted.sha256.txt"

  PORTABLE_NODEIDS="$VERIFY_DIR/portable-current-green.sorted.txt"
  comm -23 "$CURRENT_SORTED_NODEIDS" "$EXTERNAL_NODEIDS" > "$PORTABLE_NODEIDS"
  test "$(wc -l < "$PORTABLE_NODEIDS")" -eq 965
  comm -23 "$EXTERNAL_NODEIDS" "$CURRENT_SORTED_NODEIDS" \
    > "$VERIFY_DIR/external-not-current-green.txt"
  test ! -s "$VERIFY_DIR/external-not-current-green.txt"
  comm -12 "$PORTABLE_NODEIDS" "$EXTERNAL_NODEIDS" \
    > "$VERIFY_DIR/portable-external-overlap.txt"
  comm -12 "$PORTABLE_NODEIDS" "$FUTURE_NODEIDS" \
    > "$VERIFY_DIR/portable-future-overlap.txt"
  test ! -s "$VERIFY_DIR/portable-external-overlap.txt"
  test ! -s "$VERIFY_DIR/portable-future-overlap.txt"
  cat "$PORTABLE_NODEIDS" "$EXTERNAL_NODEIDS" "$FUTURE_NODEIDS" | sort -u \
    > "$VERIFY_DIR/classified-nodeids.txt"
  cmp "$ALL_SORTED_NODEIDS" "$VERIFY_DIR/classified-nodeids.txt"

  EXTERNAL_VERIFY_DIR="$VERIFY_DIR/external-verification"
  mkdir "$EXTERNAL_VERIFY_DIR"
  printf '%s\n' "$VERIFY_COMMIT" \
    > "$EXTERNAL_VERIFY_DIR/verification-commit.txt"
  cp configs/repository/external-evidence-nodeids.txt \
    "$EXTERNAL_VERIFY_DIR/external-evidence-nodeids.txt"
  cmp configs/repository/external-evidence-nodeids.txt \
    "$EXTERNAL_VERIFY_DIR/external-evidence-nodeids.txt"
  sha256sum configs/repository/external-evidence-nodeids.txt | awk '{print $1}' \
    > "$EXTERNAL_VERIFY_DIR/external-evidence-manifest.sha256"
  printf '%s\n' "$ATTEMPT02_SHA_BEFORE" \
    > "$EXTERNAL_VERIFY_DIR/attempt02-checksum-before.txt"
  DESELECT=()
  while IFS= read -r node; do DESELECT+=(--deselect "$node"); done \
    < "$FUTURE_NODEIDS"
  CURRENT_GREEN_JUNIT="$VERIFY_DIR/current-green.xml"
  python -m pytest -q "${DESELECT[@]}" --junitxml="$CURRENT_GREEN_JUNIT"

  python - "$CURRENT_GREEN_JUNIT" \
    "$VERIFY_DIR/current-green-junit-totals.json" 966 <<'PY'
import json
import sys
import xml.etree.ElementTree as ET

root = ET.parse(sys.argv[1]).getroot()
suites = [root] if root.tag == "testsuite" else list(root.iter("testsuite"))
totals = {
    key: sum(int(suite.attrib.get(key, "0")) for suite in suites)
    for key in ("tests", "failures", "errors", "skipped")
}
expected_tests = int(sys.argv[3])
if totals != {
    "tests": expected_tests,
    "failures": 0,
    "errors": 0,
    "skipped": 0,
}:
    raise SystemExit(f"unexpected current-GREEN JUnit totals: {totals}")
with open(sys.argv[2], "w", encoding="utf-8") as stream:
    json.dump(totals, stream, sort_keys=True, separators=(",", ":"))
    stream.write("\n")
PY

  EXTERNAL_PYTEST_TMP="$VERIFY_DIR/external-pytest-tmp"
  python -m pytest -q \
    'tests/test_g1_pose_conditioned_tracking_cli.py::test_t152_attempt02_is_historical_and_emits_exact_refresh_blocker_before_close' \
    --basetemp="$EXTERNAL_PYTEST_TMP" \
    --junitxml="$EXTERNAL_VERIFY_DIR/external-evidence.xml"
  python - "$EXTERNAL_VERIFY_DIR/external-evidence.xml" \
    "$EXTERNAL_VERIFY_DIR/external-evidence-junit-totals.json" <<'PY'
import json
import sys
import xml.etree.ElementTree as ET

root = ET.parse(sys.argv[1]).getroot()
suites = [root] if root.tag == "testsuite" else list(root.iter("testsuite"))
totals = {
    key: sum(int(suite.attrib.get(key, "0")) for suite in suites)
    for key in ("tests", "failures", "errors", "skipped")
}
if totals != {"tests": 1, "failures": 0, "errors": 0, "skipped": 0}:
    raise SystemExit(f"unexpected external-evidence JUnit totals: {totals}")
with open(sys.argv[2], "w", encoding="utf-8") as stream:
    json.dump(totals, stream, sort_keys=True, separators=(",", ":"))
    stream.write("\n")
PY

  mapfile -t BLOCKER_REPORTS < <(
    find "$EXTERNAL_PYTEST_TMP" -type f -name report.json -print
  )
  test "${#BLOCKER_REPORTS[@]}" -eq 1
  ATTEMPT02_SHA_AFTER=$(sha256sum "$ATTEMPT02_CHECKSUMS" | awk '{print $1}')
  test "$ATTEMPT02_SHA_AFTER" = "$ATTEMPT02_SHA_BEFORE"
  test "$ATTEMPT02_SHA_AFTER" = \
    cc53c4b4bc3cefdc7a2363c6446741e3abfc65e768ac0db71123aa593be528ed
  printf '%s\n' "$ATTEMPT02_SHA_AFTER" \
    > "$EXTERNAL_VERIFY_DIR/attempt02-checksum-after.txt"
  cmp "$EXTERNAL_VERIFY_DIR/attempt02-checksum-before.txt" \
    "$EXTERNAL_VERIFY_DIR/attempt02-checksum-after.txt"

  python - "$VERIFY_COMMIT" "${BLOCKER_REPORTS[0]}" \
    "$EXTERNAL_VERIFY_DIR/attempt02-checksum-before.txt" \
    "$EXTERNAL_VERIFY_DIR/attempt02-checksum-after.txt" \
    "$EXTERNAL_VERIFY_DIR/blocker.json" <<'PY'
import json
import sys

commit, source_path, before_path, after_path, output_path = sys.argv[1:]
nodeid = (
    "tests/test_g1_pose_conditioned_tracking_cli.py::"
    "test_t152_attempt02_is_historical_and_emits_exact_refresh_blocker_before_close"
)
with open(source_path, encoding="utf-8") as stream:
    source = json.load(stream)
code = source.get("systemic_failure_code")
message = source.get("systemic_failure_message")
if code != "CURRENT_C2A_REFRESH_REQUIRED_AFTER_GEOMETRY_SCHEMA_CHANGE":
    raise SystemExit(f"unexpected external blocker code: {code!r}")
if not isinstance(message, str) or not message.strip():
    raise SystemExit("external blocker message is empty")
with open(before_path, encoding="utf-8") as stream:
    checksum_before = stream.read().strip()
with open(after_path, encoding="utf-8") as stream:
    checksum_after = stream.read().strip()
approved_checksum = (
    "cc53c4b4bc3cefdc7a2363c6446741e3abfc65e768ac0db71123aa593be528ed"
)
if checksum_before != approved_checksum or checksum_after != approved_checksum:
    raise SystemExit("external blocker attempt-02 checksum mismatch")
record = {
    "verification_commit": commit,
    "node_id": nodeid,
    "systemic_failure_code": code,
    "systemic_failure_message": message,
    # The exact focused node passes only after asserting no factory calls.
    "factory_call_count": 0,
    "attempt02_checksum_before": checksum_before,
    "attempt02_checksum_after": checksum_after,
}
with open(output_path, "w", encoding="utf-8") as stream:
    json.dump(record, stream, sort_keys=True, separators=(",", ":"))
    stream.write("\n")
PY

  (
    cd "$EXTERNAL_VERIFY_DIR"
    sha256sum \
      verification-commit.txt \
      external-evidence-nodeids.txt \
      external-evidence-manifest.sha256 \
      external-evidence.xml \
      external-evidence-junit-totals.json \
      attempt02-checksum-before.txt \
      attempt02-checksum-after.txt \
      blocker.json \
      > checksums.sha256
    sha256sum -c checksums.sha256
  )

  ORIGINAL_CURRENT_NODEIDS="$VERIFY_DIR/original-current-green-nodeids.txt"
  python - tests/fixtures/g1_t152_baseline_inventory.json \
    "$ORIGINAL_CURRENT_NODEIDS" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    inventory = json.load(stream)
original = inventory["selections"]["original_green"]["node_ids"]
controls = inventory["selections"]["t152_green_controls"]["node_ids"]
if len(original) != 748 or len(controls) != 4:
    raise SystemExit("immutable original GREEN inventory count mismatch")
node_ids = sorted(set(original) | set(controls))
if len(node_ids) != 752:
    raise SystemExit("duplicate immutable original GREEN node ID")
with open(sys.argv[2], "w", encoding="utf-8") as stream:
    stream.write("\n".join(node_ids) + "\n")
PY
  comm -23 "$ORIGINAL_CURRENT_NODEIDS" "$CURRENT_SORTED_NODEIDS" \
    > "$VERIFY_DIR/missing-original-green-nodeids.txt"
  test ! -s "$VERIFY_DIR/missing-original-green-nodeids.txt"
  MIGRATED_NEW_GREEN_NODEIDS="$VERIFY_DIR/migrated-new-current-green-nodeids.txt"
  comm -23 "$CURRENT_SORTED_NODEIDS" "$ORIGINAL_CURRENT_NODEIDS" \
    > "$MIGRATED_NEW_GREEN_NODEIDS"

  python tests/run_g1_node_inventory.py \
    --inventory tests/fixtures/g1_t152_baseline_inventory.json \
    --selection intentional_future_red --expect-fail 125 \
    --expect-classification C2=78 --expect-classification C3=29 \
    --expect-classification freshness=10 --expect-classification task9=8
  python scripts/check_isaacsim6_imports.py --deprecated-as-error
  python scripts/run_g1_tracking_envelope.py --help
  python scripts/run_g1_static_pose_qualification.py --help

  CLEAN_DIR=$(mktemp -d "/tmp/g1-t152-${VERIFY_LABEL}-XXXXXX")
  git archive "$VERIFY_COMMIT" | tar -x -C "$CLEAN_DIR"
  python - "$CLEAN_DIR" <<'PY'
import sys

from scripts.check_clean_checkout import prepare_portable_git_context

prepare_portable_git_context(sys.argv[1])
PY
  (
    cd "$CLEAN_DIR"
    test "$(git config --bool --get portable.archive)" = true
    test -n "$(git rev-parse --verify HEAD)"
    test -z "$(git status --porcelain)"
    test "$(git rev-list --count HEAD)" -eq 1
    python -m pytest --collect-only -q > "$VERIFY_DIR/archive-collection.log"
    awk '/^tests\// && /::/' "$VERIFY_DIR/archive-collection.log" \
      > "$VERIFY_DIR/archive-nodeids.collection.txt"
    cmp "$ALL_COLLECTION_NODEIDS" "$VERIFY_DIR/archive-nodeids.collection.txt"
    ARCHIVE_DESELECT=()
    while IFS= read -r node; do
      ARCHIVE_DESELECT+=(--deselect "$node")
    done < configs/repository/intentional-future-red-nodeids.txt
    while IFS= read -r node; do
      ARCHIVE_DESELECT+=(--deselect "$node")
    done < configs/repository/external-evidence-nodeids.txt
    python -m pytest -q "${ARCHIVE_DESELECT[@]}" \
      --junitxml="$VERIFY_DIR/portable-current-green.xml"
  )
  python - "$VERIFY_DIR/portable-current-green.xml" \
    "$VERIFY_DIR/portable-current-green-junit-totals.json" 965 <<'PY'
import json
import sys
import xml.etree.ElementTree as ET

root = ET.parse(sys.argv[1]).getroot()
suites = [root] if root.tag == "testsuite" else list(root.iter("testsuite"))
totals = {
    key: sum(int(suite.attrib.get(key, "0")) for suite in suites)
    for key in ("tests", "failures", "errors", "skipped")
}
if totals != {"tests": int(sys.argv[3]), "failures": 0, "errors": 0, "skipped": 0}:
    raise SystemExit(f"unexpected portable-GREEN JUnit totals: {totals}")
with open(sys.argv[2], "w", encoding="utf-8") as stream:
    json.dump(totals, stream, sort_keys=True, separators=(",", ":"))
    stream.write("\n")
PY

  if test -n "$PRE_W_NODEID_DIR"; then
    for name in \
      all-nodeids.collection.txt \
      all-nodeids.sorted.txt \
      current-green.collection.txt \
      current-green.sorted.txt; do
      cmp "$PRE_W_NODEID_DIR/$name" "$VERIFY_DIR/$name"
    done
  fi

  if test -n "$EXPECTED_VERIFY_DIR"; then
    for name in \
      all-nodeids.collection.txt \
      all-nodeids.sorted.txt \
      current-green.collection.txt \
      current-green.sorted.txt \
      all-nodeids-count.txt \
      current-green.collection.count.txt \
      current-green.sorted.count.txt \
      current-green.collection.sha256.txt \
      current-green.sorted.sha256.txt \
      current-green-junit-totals.json \
      portable-current-green-junit-totals.json; do
      cmp "$EXPECTED_VERIFY_DIR/$name" "$VERIFY_DIR/$name"
    done
    for name in \
      external-evidence-nodeids.txt \
      external-evidence-manifest.sha256 \
      external-evidence-junit-totals.json \
      attempt02-checksum-before.txt \
      attempt02-checksum-after.txt; do
      cmp "$EXPECTED_VERIFY_DIR/external-verification/$name" \
        "$EXTERNAL_VERIFY_DIR/$name"
    done
    python - \
      "$EXPECTED_VERIFY_DIR/external-verification/blocker.json" \
      "$EXTERNAL_VERIFY_DIR/blocker.json" "$VERIFY_COMMIT" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    before = json.load(stream)
with open(sys.argv[2], encoding="utf-8") as stream:
    after = json.load(stream)
expected_commit = sys.argv[3]
if after.get("verification_commit") != expected_commit:
    raise SystemExit("final blocker is not bound to the verification commit")
for key in (
    "node_id",
    "systemic_failure_code",
    "systemic_failure_message",
    "factory_call_count",
    "attempt02_checksum_before",
    "attempt02_checksum_after",
):
    if after.get(key) != before.get(key):
        raise SystemExit(f"external attestation drift for {key}")
PY
  fi
}
```

Expected at both `W` and `P_t152`: all focused/current GREEN nodes pass; the exact
original 748 remain GREEN; exactly 125 intentional future-REDs retain the
78/29/10/8 classifications with no unexpected pass/error/skip; all collection
and import/help checks succeed; exact hard limit remains `0.0005`; and the
tracked-only archive passes exactly 965 portable current-GREEN nodes. The main
checkout passes all 966 current-GREEN nodes, including the one external-evidence
node, with attempt-02 unchanged and factory call count zero. The external node
is neither future-RED nor skipped. Every collected node belongs to exactly one
of the 965 portable GREEN, one external-evidence GREEN, or exact 125 intentional
future-RED sets. The partition is `965+1+125=1091`; there are no unclassified
nodes. The external-verification directory contains exactly the nine required
files, is bound to the verified commit, records the exact 1/0/0/0 JUnit totals,
exact blocker code, non-empty message, zero factory calls, and equal approved
attempt checksums, and is not runtime or physical evidence. No attempt-02 file
is copied or modified. Portable archive tests do not read the original
worktree. No command launches Isaac Sim.

**Steps**

- [x] Commit only the three authorized `D4` files with message
  `docs(g0): separate historical and portable source blobs`. Verify its parent
  is `D3`, all three independent Git proofs, JSON shape, and unchanged
  collection/digests; then capture the immutable pre-W node-ID snapshot before
  any `W` edit:

  ```bash
  E_IMPL=aa47af3946f2f9f934147b4b263affe345a9d450
  D1=d561f3be49b3ba059286818e325adc81b5b0b269
  D2=6d234a4bf8d8420fbd58d771e9828af2f9d0efa6
  V1=7ef680b0a5d062c682a2d1715539e7b32f09b538
  D3=66551b9f55729b920adb5fda64f9b52a9852b8f7
  D4=$(git rev-parse HEAD)
  test "$(git rev-parse "${D4}^")" = "$D3"
  test "$(git rev-parse "${D3}^")" = "$V1"
  test "$(git rev-parse "${V1}^")" = "$D2"
  test "$(git rev-parse "${D2}^")" = "$D1"
  test "$(git rev-parse "${D1}^")" = "$E_IMPL"
  PRE_W_NODEID_DIR=/tmp/g1-t152-pre-d4-w-nodeids
  capture_t152_nodeids "$D4" "$PRE_W_NODEID_DIR"
  ```

- [x] Implement `W` with one focused assertion RED in an existing test node,
  then GREEN, for the exact archive-local synthetic repository and two-mode
  historical/blob attestation contract. Reuse
  `prepare_portable_git_context(export_root)` from both G0 and this verification
  helper. Do not add/delete/rename test functions, change a parameterized
  expansion, copy history, read the original worktree from the archive, or run
  Isaac Sim. Commit with `fix(g0): make portable archive git-aware`.

  The focused RED was exactly two assertion failures and one passing
  historical/blob control: the missing portable-Git helper and missing report
  provenance fields failed without collection, import, fixture, or environment
  error. The focused GREEN passed 3/3; the full clean-checkout and migration
  files passed 12/12 and 4/4. In a tracked-only synthetic checkout, the four
  retained V1 failures passed 4/4, including the structured C2a output-exists
  error and repository ignore-rule controls.

  A pre-push code review then found that inherited Git common-directory,
  template, config, hook/filter, replacement-ref, and identity environment
  could weaken the absolute isolation claim. A second focused RED reproduced
  the external-common-directory write. The corrected helper constructs a
  minimal Git environment, uses an empty template and disabled hooks, fixes
  both author and committer identity/date, disables replacement objects,
  rejects alternates/grafts/packs/unexpected refs, and proves that every object
  is in the one synthetic HEAD closure. Main historical proof also rejects
  replacement refs/indirection and checks both approved objects are commits.
  The hermetic focused contracts passed 16/16 and the four archive controls
  passed 4/4.
- [ ] Verify `W^=D4`, bind `VERIFY_IMPL=$(git rev-parse HEAD)`, require a clean
  worktree, run the complete suite into the new immutable directory, and
  compare every post-W node-ID list byte-for-byte with clean `D4`:

  ```bash
  VERIFY_IMPL=$(git rev-parse HEAD)
  test "$(git rev-parse "${VERIFY_IMPL}^")" = "$D4"
  run_t152_verification "$VERIFY_IMPL" pre-projection-d4-w-hermetic "" \
    "$PRE_W_NODEID_DIR"
  PREPROJECTION_VERIFY_DIR=/tmp/g1-t152-pre-projection-d4-w-hermetic
  ```

  The earlier `/tmp/g1-t152-pre-projection-d4-w` passed before this pre-push
  review correction and remains immutable but superseded. It is not valid
  evidence for the amended `W`; only the new hermetic directory is authoritative.
- [ ] After the suite passes, change only T152 to `[x]` in `tasks.md`; leave T151
  and T070 `[ ]` and attempt-04 prohibited.
- [ ] Record the literal, already-known `E_impl`, `D1`, `D2`, `V1`, `D3`, `D4`,
  and `W` SHAs in this plan without attempting to record the not-yet-created
  projection SHA.
- [ ] Run `git diff --check`, stage only the two Markdown files, verify their
  diff, and create the unique projection commit:

  ```bash
  git add specs/001-benchmark-reconstruction/tasks.md \
    specs/001-benchmark-reconstruction/g1-contact-exclusion-t152-implementation-plan.md
  git diff --cached --check
  git commit -m "docs(g1): complete T152 geometry integration"
  ```

- [ ] Dynamically bind and verify final `E2=P_t152`, then rerun the complete
  suite:

  ```bash
  FINAL_E2=$(git rev-parse HEAD)
  test "$(git rev-parse "${FINAL_E2}^")" = "$VERIFY_IMPL"
  run_t152_verification "$FINAL_E2" final-projection-p \
    "$PREPROJECTION_VERIFY_DIR" "$PRE_W_NODEID_DIR"
  ```

- [ ] Confirm the final attestation is the exact `P_t152`-bound directory generated
  by the final-projection run. G0 consumes only this directory; it does not
  rerun the external node or read attempt-02.
- [ ] Produce final G0 evidence bound only to `FINAL_E2=P_t152`:

  ```bash
  FINAL_E2_SHORT=${FINAL_E2:0:12}
  G0_OUTPUT="outputs/evidence/G0/t152-geometry-${FINAL_E2_SHORT}"
  EXTERNAL_VERIFICATION=/tmp/g1-t152-final-projection-p/external-verification
  test ! -e "$G0_OUTPUT"
  test "$(cat "$EXTERNAL_VERIFICATION/verification-commit.txt")" = "$FINAL_E2"
  python scripts/check_clean_checkout.py \
    --output "$G0_OUTPUT" \
    --external-verification \
      /tmp/g1-t152-final-projection-p/external-verification
  python scripts/review_gate.py --gate G0 \
    --evidence "$G0_OUTPUT/manifest.json"
  (cd "$G0_OUTPUT" && sha256sum -c checksums.sha256)
  python - "$G0_OUTPUT/report.json" "$G0_OUTPUT/manifest.json" \
    configs/repository/external-evidence-nodeids.txt "$FINAL_E2" <<'PY'
import hashlib
import json
import sys

expected = {
    "total_collected": 1091,
    "current_green_total": 966,
    "portable_green_selected_count": 965,
    "portable_green_passed_count": 965,
    "external_evidence_count": 1,
    "intentional_future_red_count": 125,
    "portable_archive_reads_original_worktree": False,
    "portable_git_context": "synthetic_clean_repository",
    "portable_history_objects_injected": False,
    "portable_source_bytes_equal_git_archive": True,
    "portable_current_source_blob_git": "2839e2ff67864c692f1bdb9ae5dc64e2dea34f91",
    "historical_behavior_blob_git": "b9864a8b8eea289fa61eb7e3e41633c35947c5ef",
    "historical_execution_start_blob_git": "b9864a8b8eea289fa61eb7e3e41633c35947c5ef",
    "historical_objects_verified_in_main_checkout": True,
    "historical_objects_verified_in_portable_archive": False,
    "external_verification_attestation_consumed": True,
    "external_verification_commit": sys.argv[4],
    "external_verification_junit": {
        "tests": 1,
        "failures": 0,
        "errors": 0,
        "skipped": 0,
    },
}
external_node = "tests/test_g1_pose_conditioned_tracking_cli.py::test_t152_attempt02_is_historical_and_emits_exact_refresh_blocker_before_close"
with open(sys.argv[3], "rb") as stream:
    external_manifest_sha256 = hashlib.sha256(stream.read()).hexdigest()
for path in sys.argv[1:3]:
    with open(path, encoding="utf-8") as stream:
        record = json.load(stream)
    for key, value in expected.items():
        if record.get(key) != value:
            raise SystemExit(f"unexpected G0 {path} {key}: {record.get(key)!r}")
    if record.get("external_evidence_nodeids") != [external_node]:
        raise SystemExit(f"unexpected G0 {path} external node IDs")
    if record.get("external_evidence_manifest_sha256") != external_manifest_sha256:
        raise SystemExit(f"unexpected G0 {path} external manifest SHA-256")
PY
  test -z "$(git status --porcelain --untracked-files=no)"
  ```

  Before archive creation, `check_clean_checkout.py` must reject a missing,
  extra, non-regular, symlinked, checksum-invalid, wrong-commit, wrong-manifest,
  wrong-node, non-1/0/0/0-JUnit, wrong-blocker, empty-message, nonzero-factory,
  or unequal/unapproved attempt-checksum attestation. In particular, it
  validates `blocker.json` fields `attempt02_checksum_before` and
  `attempt02_checksum_after` against the corresponding checksum text files and
  the approved SHA. G0 may copy or summarize attestation content and checksums,
  but it labels them external-verification metadata rather than portable test
  output.

- [ ] Freeze `FINAL_E2=P_t152`: do not modify any tracked file after the final suite
  and G0 evidence. A separately approved fresh C2a must bind this SHA, not
  `E_impl`.

**Stop conditions**

Stop on any test-function or parameter-expansion add/delete/rename/change, any
byte of pre-W/post-W node-ID drift, any full count other than 1091, any
current-GREEN count other than 966, unexpected pass/error/skip,
hard-limit/clearance/matrix/truth change, deprecated diagnostic, help-time
Isaac startup, loss of either approved current-GREEN digest, sorting before the
collection-order digest, external/future overlap, an external node that is
absent or does not pass in the main checkout, attempt-02 checksum drift or
copy/modification, archive selection other than 965, portable archive tests
reading the original worktree, a partition other than `965+1+125=1091`,
malformed/checksum-invalid/wrong-commit/wrong-node/wrong-JUnit/wrong-blocker/
empty-message/nonzero-factory/unapproved-attempt external attestation,
`D1^!=E_impl`, `D2^!=D1`, `V1^!=D2`, `D3^!=V1`, `D4^!=D3`,
`W^!=D4`, `P_t152^!=W`, tracked change after `P_t152`, G0
checksum failure, an old G0 invocation without `--external-verification`, or
pressure to run Task 12 or advance T151/T070/attempt-04.

**Commit**

`docs(g1): complete T152 geometry integration`

This is the sole projection/status commit `P_t152` and final `E2`. Its parent
is `W`; tracked files record only the already-existing `E_impl`, `D1`, `D2`,
`V1`, `D3`, `D4`, and `W` SHAs and never record `P_t152`'s SHA. Task 11 depends on
Tasks 1–10 plus all approved design checkpoints and verification
infrastructure. Isaac Sim,
fresh C2a, Task 12 execution, attempt-04, and PressButton episodes are not
allowed.

## Task 12: Prepare the separately approved fresh C2a refresh; do not execute it

**Files**

- No file changes are required.
- Read only: final `E2=P_t152` commit, the C2a script/configs, and the future output
  parent.

This task prepares a review card only. It does not run the command, create the
directory, accept an attempt number without approval, or assume a selected pose.

**Preparation checklist**

- [ ] Bind `FINAL_E2=$(git rev-parse HEAD)` only after Task 11's clean-`W`
  pre-projection verification, `P_t152^=W` projection commit, final verification
  replay, and G0 review.
- [ ] Obtain a separate user authorization containing the one permitted attempt
  ID and exact output path.
- [ ] Verify worktree clean and local/tracking/live-origin/PR heads equal
  `FINAL_E2=P_t152`.
- [ ] Verify Draft PR #2 remains OPEN, Draft, base `main`.
- [ ] Verify the approved output directory does not exist.
- [ ] Verify prior C2a evidence and checksums remain immutable.
- [ ] Record driver `550.144.03 / UNVALIDATED`, Isaac Sim version,
  `physics_device=cpu`, `broadphase_type=MBP`, GPU dynamics disabled, headless
  mode, seed 1701, and EULA authorization.
- [ ] Do not preset the selected pose to `task-ready-z-0p55`; selection must come
  from the new evidence.

**Prepared command — forbidden until separately authorized**

```bash
FINAL_E2=$(git rev-parse HEAD)
FINAL_E2_SHORT=${FINAL_E2:0:12}
: "${ATTEMPT_ID:?set the exact separately approved attempt ID}"
OMNI_KIT_ACCEPT_EULA=YES \
/mnt/data/home/lss/miniconda3/envs/isaac6/bin/python \
scripts/run_g1_static_pose_qualification.py \
  --output "outputs/evidence/G1/c2a-static-preliminary-${FINAL_E2_SHORT}-attempt-${ATTEMPT_ID}" \
  --config configs/tasks/press_button_physical.yaml \
  --task-card configs/tasks/cards/press_button.v1.yaml \
  --robot-config configs/robots/fr3_press_button_safe.yaml \
  --headless \
  --seed 1701
```

Do not pipe through `tee`, wrap in a retry, create a second attempt, or rerun for
any exit code. After the one process exits, stop. A later read-only review checks
`command.log`, `offline_candidates.jsonl`, `static_scenes.jsonl`,
`readiness_samples.jsonl`, `report.json`, `manifest.json`, and
`checksums.sha256`; runs `sha256sum -c checksums.sha256` from the output
directory; verifies current `FINAL_E2=P_t152`/config/card/asset provenance; recomputes
any selected candidate hash; reports truthful rejected candidates and actual
counts; verifies observed `physics_device=cpu`, `broadphase_type=MBP`, and GPU
dynamics disabled; and verifies unique shutdown/exit consistency. That review
also retains the preliminary/no-claim boundary.

**Expected result and stop conditions**

There is no implementation-stage result because the command is not executed.
When separately approved, either a selected pose or an exact non-empty blocker is
valid evidence; neither outcome authorizes a retry. Stop immediately if the
directory exists, heads differ, worktree is dirty, evidence provenance is stale,
the driver/physics policy differs, or an approval does not bind exactly one run.

**Commit and dependency**

No commit. Task 12 depends on final `E2=P_t152` from the
`E_impl -> D1 -> D2 -> V1 -> D3 -> D4 -> W -> P_t152` Task 11 chain and the
standing user authorization.
Isaac Sim is explicitly forbidden during plan implementation and T152 GREEN;
it is permitted only by that later one-run authorization.

## Commit and checkpoint map

Each commit is independently reviewable. A focused command is run immediately
before and after each commit; the post-commit outcome must equal the pre-commit
GREEN/RED partition stated below.

| Task | Commit | Focused command | Required checkpoint |
|---|---|---|---|
| 1 | `test(g1): freeze T152 red migration inventory` | `python -m pytest -q tests/test_g1_t152_red_migration_manifest.py` | 4 manifest tests pass; frozen T152 remains 84 RED + 4 GREEN |
| 2 | `test(g1): replace spherical contact exclusion fixtures` | `python -m pytest -q tests/test_press_button_geometry_contract.py tests/test_g1_contact_exclusion_geometry.py tests/test_press_button_mechanism.py tests/test_press_button_task_card_contract.py tests/test_g1_pose_conditioned_tracking_cli.py` | schema/migrated/task-card nodes assertion-RED; two new fixture controls plus original four controls GREEN; no collection error |
| 3 | `test(g1): define analytic contact exclusion contracts` | `python -m pytest -q tests/test_g1_contact_exclusion_geometry.py` | all behavior nodes assertion-RED; fixture checks GREEN |
| 4 | `feat(g1): define PressButton geometry contract` | `python -m pytest -q tests/test_press_button_geometry_contract.py` | all schema/type/digest nodes GREEN; analytic nodes remain RED |
| 5 | `feat(g1): validate continuous TCP clearance` | `python -m pytest -q tests/test_g1_contact_exclusion_geometry.py` | all analytic nodes GREEN |
| 6 | `feat(g1): migrate PressButton mechanism 1.1` | `python -m pytest -q tests/test_press_button_geometry_contract.py tests/test_press_button_mechanism.py tests/test_press_button_task_card_contract.py` | schema/version/current-digest nodes GREEN and attempt-02 classified stale; complete CLI refresh-blocker nodes remain RED for Task 9 |
| 7A | `test(g1): correct geometry-only authoring contracts` | `python -m pytest -q tests/test_press_button_mechanism.py -k 'declared_geometry or geometry_authoring_receipt or complete_build_stage or formal_stage_builder'` | corrected receipt/seam/complete-builder nodes are exact assertion RED; prior mechanism nodes GREEN |
| 7B.1 | `feat(g1): add declared geometry authoring receipt` | `python -m pytest -q tests/test_press_button_mechanism.py -k 'declared_geometry or geometry_authoring_receipt or legacy'` | import-safe geometry receipt/seam nodes GREEN; complete real-builder nodes remain RED |
| 7B.2 | `refactor(g1): share declared geometry with complete USD stage` | `python -m pytest -q tests/test_press_button_mechanism.py tests/test_press_button_geometry_contract.py tests/test_g1_contact_exclusion_geometry.py tests/test_press_button_task_card_contract.py` | all Task 4–7 nodes GREEN; full physical authoring source contract retained without Isaac startup |
| 8 design | `docs(g1): define command-bound route bundle ownership` | documentation consistency scans only | approved Option B, exact Task 8/10 ownership, and no production/test/config change |
| 8A | `test(g1): correct Task 8 route bundle contracts` | exact Task 8 node IDs plus dedicated `test_task8_` nodes | schema-correct command-bound behavior is assertion RED; two fixture controls GREEN; Task 10 nodes unchanged |
| 8B.1 | `feat(g1): derive command-bound pose routes` | exact Task 8 authority, bundle-shape, schedule, endpoint, and construction-digest nodes | pure 6-class x 5-command derivation GREEN; validation failures remain RED; no Task 10 API added |
| 8B.2 | `feat(g1): validate command-bound declared-solid routes` | all exact Task 8 node IDs plus `tests/test_g1_contact_exclusion_geometry.py` | Task 8 bundle/validation/re-export GREEN; Task 10 plan/executor/orchestration nodes remain RED |
| 9 | `feat(g1): verify current C2a pose evidence` | `python -m pytest -q tests/test_g1_pose_conditioned_tracking_cli.py -k 'c2a_evidence or selected_candidate or selected_pose or current_input or provenance'` | all loader/freshness nodes GREEN; attempt-02 rejected as historical |
| 10 | `feat(g1): wire pose-conditioned multiclass CLI` | `python -m pytest -q tests/test_g1_pose_conditioned_tracking_cli.py` | complete file GREEN; no legacy real-main path |
| 11 design D1 | `docs(g1): define portable T152 verification closure` | documentation consistency checks only | `D1=d561f3be49b3ba059286818e325adc81b5b0b269`, `D1^=E_impl`, dual digest and external classification fixed, T152/T151/T070 unchanged, attempt-04 prohibited |
| 11 design D2 | `docs(g1): close portable verification attestation gaps` | documentation consistency checks only | `D2^=D1`, fixed 12-node V1 inventory, P_t152-bound external attestation and G0 consumption fixed, no implementation |
| 11 verification V1 | `fix(g0): verify portable external evidence` | exact existing-node mapping, D2/V1 list comparison, external manifest, P/V1-bound attestation, dual digest, portable 965, full 1091 classification, G0 schema | `V1^=D2`; retained projection collected 965 and exposed four empty-Git-context failures |
| 11 design D3 | `docs(g0): define portable synthetic git context` | documentation consistency checks only | `D3^=V1`, synthetic clean Git and main-history/portable-blob modes fixed, no implementation |
| 11 design D4 | `docs(g0): separate historical and portable source blobs` | JSON and documentation consistency, three independent Git blob proofs, unchanged collection/digests | `D4^=D3`; historical blobs remain `b9864a8...`, portable current source is `2839e2f...`, no implementation/test-node change |
| 11 correction W | `fix(g0): make portable archive git-aware` | focused existing-node RED/GREEN, D4/W list comparison, helper reuse, synthetic provenance, independent historical/current blob attestation, full verification in `/tmp/g1-t152-pre-projection-d4-w` | `W^=D4`; main 966 GREEN, portable 965 GREEN, external 1 GREEN, future-RED 125 |
| 11 projection | `docs(g1): complete T152 geometry integration` | complete suite at `W`, projection commit `P_t152`, identical suite, P_t152-bound attestation, and G0 at `P_t152` | `P_t152^=W`, `FINAL_E2=P_t152`, tracked files record `E_impl`/`D1`/`D2`/`V1`/`D3`/`D4`/`W` only, T152 `[x]`, T151/T070 `[ ]`, attempt-04 prohibited, no tracked change after `P_t152` |

Tasks 2, 3, 7A, and 8A are deliberately RED-only commits. No production change may
be included with them. Tasks 4–10 may not modify approved RED assertions except
the one-to-one schema-specific migration committed in Task 2 and the
architecture-approved geometry-only seam correction in Task 7A and the
command-bound ownership correction in Task 8A.

## Approved design-section coverage

| Architecture review section | Implementing tasks | Verification |
|---|---|---|
| 1. Scope and truth boundary | 2, 5, 8, 10 | TCP-only fields plus mandatory runtime truth nodes |
| 2. One authoritative geometry source | 4, 6, 7A, 7B | strict YAML parser, geometry-only receipt identity/digests, complete-builder literal-source guard |
| 3. Geometry parsing and validation | 2, 4, 6 | schema/dimension/token/quaternion/digest tests |
| 4. Distance, clearance, continuous mathematics | 3, 5 | slab/quadratic/boundary/degenerate analytic tests |
| 5. Six-route design-time assessment | 8 | command-bound continuous routes recomputed from current inputs; the design table is not copied into results |
| 6. Derived route evidence schema | 5, 8, 10 | obstacle/segment/root/config provenance cross-recording |
| 7. Fail-closed taxonomy | 2–10 | exact code and non-empty-message assertions at each boundary |
| 8. RED fixture correction | 1–3 | one-to-one migration manifest and observed schema RED |
| 9. Version migration/consumer synchronization | 6, 7A, 7B, 9 | formal 1.1.0, state-only legacy, card/config/digest synchronization |
| 10. Freshness/C2a invalidation | 6, 9, 11, 12 | Task 6 proves stale digests, Task 9 propagates the blocker, Task 11 verifies attempt-02 externally in the main checkout while the portable archive excludes it explicitly, and a fresh run is separately gated at final `E2=P_t152` |
| 11. Non-goals/fixed invariants | every task | matrix, limits, clearance, physics, truth, and task-state stop checks |
| 12. Architecture conclusion/task state | 11, 12 | only T152 closes after GREEN; T151/T070 and attempt-04 stay blocked |

## All 31 approved RED contracts

This table is normative: no row may be omitted or merged into an unobservable
claim.

| # | Approved contract | RED owner | GREEN owner |
|---:|---|---|---|
| 1 | Exact 1.1.0 YAML root/geometry/contact fields required | Task 2 schema nodes | Tasks 4 and 6 |
| 2 | Quaternion shape/order/finite/nonzero normalization, canonical sign, transform/digest | Task 2 root nodes | Task 4 |
| 3 | Exact X/Y/Z token mapping, dual-field rejection, joint-axis collinearity | Task 2 button nodes | Task 4 |
| 4 | Every missing/unknown/nonfinite/malformed/invalid field rejected | Task 2 nested-field parameters | Tasks 4 and 6 |
| 5 | geometry-only seam transfers the parsed contract and real `build_stage()` completes physical authoring | Task 2 mechanism nodes plus Task 7A corrected receipt/builder nodes | Task 7B |
| 6 | Root-plus-local transform derives centers/axis/OBB orientation | Task 2 transform nodes | Task 4 |
| 7 | Capped-cylinder point/segment distance correct | Task 3 cylinder nodes | Task 5 |
| 8 | OBB point/segment distance correct | Task 3 OBB nodes | Task 5 |
| 9 | Full continuous segment, not endpoints, controls clearance | Task 3 midsegment node | Task 5 |
| 10 | Exact 0.005 m equality passes | Task 3 equality node | Task 5 |
| 11 | Strict value below 0.005 m fails | Task 3 nextafter node | Task 5 |
| 12 | Caller true flags ignored | Task 2 migrated flag parameters | Task 8 |
| 13 | Root/geometry/transform/segment/route/card/current digests detect mutation | Tasks 2, 3, 9 digest nodes | Tasks 4, 5, 8, 9 |
| 14 | Six canonical route records retained in order | Task 2 migrated route node | Task 8 |
| 15 | Static truth scope remains TCP-point-only | Task 2 scope node | Tasks 5 and 8 |
| 16 | Per-action Contact/collision/penetration/post-action checks remain mandatory | Task 2 runtime-truth node | Task 10 |
| 17 | Static pass cannot create C1/C2/G1/cap/gate PASS | Task 2 no-claim node | Tasks 8 and 10 |
| 18 | OBB interior crossing fails | Task 3 OBB crossing | Task 5 |
| 19 | OBB tangent passes boundary-only | Task 3 OBB tangent | Task 5 |
| 20 | OBB parallel-outside passes | Task 3 OBB parallel | Task 5 |
| 21 | OBB boundary-coincident passes only with empty open-interior intersection | Task 3 OBB boundary | Task 5 |
| 22 | Cylinder radial crossing fails | Task 3 radial crossing | Task 5 |
| 23 | Cylinder cap crossing fails | Task 3 cap crossing | Task 5 |
| 24 | Cylinder radial tangent passes boundary-only | Task 3 radial tangent | Task 5 |
| 25 | Cylinder cap tangent passes boundary-only | Task 3 cap tangent | Task 5 |
| 26 | Cylinder axis-parallel exercises constant-radial branch | Task 3 axis-parallel | Task 5 |
| 27 | Zero-length segment uses exact point predicate | Task 3 OBB/cylinder point nodes | Task 5 |
| 28 | Nonfinite/unorderable coefficients or intervals fail unproven | Task 3 unproven parameters | Task 5 |
| 29 | Conservative pass without exact-minimum proof records 0.005 only | Task 3 lower-bound/21 mm nodes | Task 5 |
| 30 | Explicit legacy 1.0.x is state-only and blocks formal build/route | Task 2 legacy nodes | Tasks 6 and 7 |
| 31 | Physical config, task card, parser, runtime, and tests migrate together | Task 2 synchronization node | Tasks 6–10 |

## Existing 84 T152 RED ownership

Task 1 stores every fully expanded ID. This grouped index proves one-to-one task
ownership while retaining the exact manifest as the source for automated
node-ID comparison.

| Existing RED group | Expansions | Owning task |
|---|---:|---|
| Real main selects multiclass, not legacy | 1 | 10 |
| Invalid selected candidate: missing/duplicate/synthetic/malformed | 4 | 9 |
| Independent selected hash recomputation | 1 | 9 |
| Pose/joint/frame/asset/task/robot identity mismatch | 6 | 9 |
| Exact 90-trial plan and ordering | 1 | 10 |
| Six-route pre-acquisition invalid states | 6 | 8 |
| Plan carries canonical motif | 1 | 10 |
| Local class schedules | 3 | 8 |
| Continuous class schedules | 3 | 8 |
| Motif digest/schedule/float64 consistency | 1 | 8 |
| Fresh scene identities | 1 | 10 |
| Pre-Play authoring success | 1 | 10 |
| Post-Play/teleport/nonzero pre-position rejection | 3 | 10 |
| Exact readiness count | 1 | 10 |
| Exact measurement/windows | 1 | 10 |
| Executor consumes each class motif | 6 | 10 |
| Shared qualifying Lula kernel | 1 | 10 |
| Compatibility path excluded | 1 | 10 |
| Multiclass aggregation | 1 | 10 |
| Evidence cross-provenance | 1 | 10 |
| Stop-tail/systemic exit | 1 | 10 |
| Checksums before close | 1 | 10 |
| Writer failure lifecycle | 1 | 10 |
| Post-abort/force/wrench/raw-impulse truth | 4 | 10 |
| Explicit C2a evidence directory | 1 | 9 |
| Evidence loaded before factory | 1 | 9 |
| Checksum/report/candidates/hash validation | 1 | 9 |
| Selected record comes from JSONL | 1 | 9 |
| Missing evidence argument blocks factory | 1 | 9 |
| Invalid evidence artifact/tamper/duplicate parameters | 8 | 9 |
| Current task/robot/asset digest mismatches | 3 | 9 |
| Orchestration invalid route parameters | 7 | 10 |
| Route builder derives inputs | 1 | 8 |
| Pose/task/workspace/contact geometry mutations | 4 historical nodes mapped to 5 declared-solid replacements | 8 |
| Caller true flags ignored | 2 | 8 |
| Real scene source pre-Play contract | 1 | 10 |
| Injected lifecycle order seam | 1 | 10 |
| Post-Play lifecycle seam rejection | 1 | 10 |
| **Total** | **84** | Tasks 8–10 |

The four original controls remain owned by Task 1 inventory and must stay GREEN:
selected candidate hash recomputation, complete canonical six-route fixture,
canonical motif fixtures, and fake-factory import safety.

## Type and function consistency index

| Symbol | Defined | Consumed |
|---|---|---|
| `MechanismRootPose` | Task 4, `press_button_geometry.py` | Tasks 6–8 |
| `CappedCylinderGeometry` | Task 4 | Tasks 5–8 |
| `OrientedBoxGeometry` | Task 4 | Tasks 5–8 |
| `ContactExclusionPolicy` | Task 4 | Tasks 5, 8 |
| `PressButtonGeometryContract` | Task 4 | Tasks 5–9 |
| `PressButtonWorldGeometry` | Task 4 | Tasks 5, 7, 8 |
| `parse_press_button_geometry_contract` | Task 4 | Tasks 6–9 |
| `derive_press_button_world_geometry` | Task 4 | Tasks 5, 7, 8 |
| `SegmentClearanceResult` | Task 5 | Tasks 5, 8, 10 evidence |
| `ContactExclusionRouteResult` | Task 5 | Tasks 8–10 |
| `validate_segment_against_expanded_obb` | Task 5 | Task 8 route validation |
| `validate_segment_against_expanded_capped_cylinder` | Task 5 | Task 8 route validation |
| `validate_contact_exclusion_routes` | Task 5 | Task 8 orchestration validator |
| `PressButtonDeclaredGeometryAuthoringAdapter` | Task 7B | Task 7A recording tests and Task 7B real USD geometry adapter |
| `PressButtonGeometryAuthoringReceipt` | Task 7B | Task 7A receipt truth tests and Task 7B complete builder |
| `author_declared_geometry` | Task 7B | Task 7A recording tests and Task 7B real complete builder |
| `UsdPressButtonDeclaredGeometryAuthoringAdapter` | Task 7B | Task 7A lazy-import/source tests and Task 7B complete builder |
| `G1_TRACKING_COMMANDS_M` | Task 8B, `g1_tracking.py` | Task 8 bundle derivation and existing multiclass plan; Task 10 plan integration |
| `G1_TRACKING_COMMAND_DECIMAL_STRINGS` | Task 8B, `g1_tracking.py` | Task 8 command and bundle digests |
| `g1_press_button_task_route_geometry` | Task 8B, `g1_tracking.py` | Task 8 bundle derivation and Task 10 provenance recording |
| `derive_g1_pose_conditioned_routes` | Task 8B, `g1_contact_exclusion.py` | Task 8 pure tests and Task 10 plan input |
| `validate_g1_pose_conditioned_routes` | Task 8B, `g1_contact_exclusion.py` | Task 8 pure tests and Task 10 pre-factory orchestration boundary |
| `G1CurrentInputDigests` | Task 9 | Tasks 9–10 |
| `C2ASelectedPoseEvidence` | Task 9 | Tasks 9–10 |
| `load_g1_c2a_selected_pose_evidence` | Task 9 | Task 10 `main` |
| `validate_g1_c2a_current_input_provenance` | Task 9 | Task 10 `main` |
| `build_g1_pose_conditioned_tracking_plan` | Task 10 | Task 10 orchestration |
| `build_g1_pose_conditioned_runtime_preplay` | Task 10 | Task 10 real factory |
| `execute_g1_pose_conditioned_tracking_trial` | Task 10 | Task 10 plan runner |
| `run_g1_pose_conditioned_tracking_plan` | Task 10 | Task 10 orchestration |
| `write_g1_pose_conditioned_tracking_evidence` | Task 10 | Task 10 orchestration |
| `orchestrate_g1_pose_conditioned_tracking` | Task 10 | Task 10 `main` |

## Global stop conditions

Stop the executing session immediately, preserve diagnostics, and request review
if any condition occurs:

- A RED fails through ImportError, NameError, fixture/path/collection error, or
  Isaac environment startup rather than the missing target assertion.
- The geometry parser needs to guess a field, apply a silent default, or recover
  root orientation from an authored USD stage.
- USD geometry and analytic geometry cannot consume the same parsed object and
  digest.
- Continuous validation becomes sampled, fixed-step, endpoint-only, or relies on
  epsilon/`isclose` to implement equality.
- Task-card and physical-config mechanism versions differ.
- Legacy 1.0.x can build a formal stage, qualify a route, or become cap-eligible.
- Attempt-02 is accepted as current after config/code/schema migration.
- Any of the original 748 GREEN node IDs or 125 future-RED node IDs/outcome
  classifications drifts.
- Exact `0.0005`, exact `0.005`, the command matrix, CPU physics, Contact,
  collision/penetration, or force/wrench truth changes.
- Any implementation unit test requires Isaac Sim to become GREEN.
- T151, T070, attempt-04, a PressButton episode, or a physical claim advances.
- A Task 2 schema-specific assertion is changed without a manifest mapping, or
  another approved RED assertion is modified/deleted to manufacture GREEN.

## Plan-author self-review

- [x] All 12 approved architecture sections map to tasks and verification.
- [x] All 31 approved RED contracts have explicit RED and GREEN ownership.
- [x] All 84 existing T152 RED expansions have a grouped owner and an exact
  machine-readable inventory/migration requirement.
- [x] All introduced class/function names are consistent across producer and
  consumer tasks.
- [x] Every implementation task includes a concrete code fragment, exact paths,
  node IDs, commands, expected RED/GREEN outcome, stop conditions, commit, and
  dependency.
- [x] Every task explicitly forbids Isaac Sim; Task 12 prepares but does not run
  the separately gated command.
- [x] Freshness ordering is schema/config migration → T152 GREEN at `E_impl` →
  portable-closure design `D1` → documentation revision `D2` → verification
  implementation `V1` → synthetic-Git design `D3` → blob-separation design
  `D4` → clean-`D4` pre-W node
  snapshot → corrective verification infrastructure `W` → clean-`W`
  pre-projection verification → projection/final `E2` commit `P_t152` → final
  verification, P_t152-bound external attestation, and G0 evidence at `P_t152` →
  separately approved fresh C2a at `P_t152` → evidence review → T151 review →
  separately approved attempt-04.
- [x] T150 remains `[x]`; T151, T152, and T070 remain `[ ]` while this plan is
  authored; attempt-04 remains `ATTEMPT_04_PROHIBITED`.

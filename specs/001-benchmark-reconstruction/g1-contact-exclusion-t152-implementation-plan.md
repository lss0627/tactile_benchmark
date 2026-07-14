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

## Task 8: Derive and validate all six T152 routes

**Files**

- Modify: `tests/test_g1_pose_conditioned_tracking_cli.py`
- Modify: `isaac_tactile_libero/runtime/g1_contact_exclusion.py`
- Modify: `isaac_tactile_libero/runtime/g1_tracking.py` only for canonical
  route/motif data that belongs with existing multiclass capability.
- Modify: `scripts/run_g1_tracking_envelope.py`

**Functions and signatures**

```python
def derive_g1_pose_conditioned_routes(
    *,
    selected_candidate: Mapping[str, object],
    selected_pose_sha256: str,
    class_definitions: Sequence[Mapping[str, object]],
    workspace_limits: Mapping[str, object],
    geometry_contract: PressButtonGeometryContract,
    current_input_digests: Mapping[str, str],
) -> tuple[Mapping[str, object], ...]: ...

def validate_g1_pose_conditioned_routes(
    *,
    ordered_routes: Sequence[Mapping[str, object]],
    geometry_contract: PressButtonGeometryContract,
    workspace_limits: Mapping[str, object],
    current_input_digests: Mapping[str, str],
) -> ContactExclusionRouteResult: ...
```

The script may re-export these names to preserve the approved test import seam,
but owns no duplicate algorithm. Construction consumes selected measured FK,
frame, pose/hash, current task/config/card/asset digests, workspace, the parsed
contract, and the canonical six class definitions. It accepts no trusted
`workspace_valid` or `contact_exclusion_valid` field.

Route order is exactly the existing `G1_TRAJECTORY_CLASS_IDS`. Each local
round-trip is a complete ordered `+16/-32/+16` segment chain at the current
command. Each continuous class materializes the existing Decimal-derived
phase-reflected scalar schedule into float64 and records both forms plus motif
digest. All route segments, including crossings back through the start, are
validated continuously against both solids and workspace. One missing,
reordered, partial, nonfinite, workspace-invalid, contact-invalid, or
digest-mismatched route blocks before `factory_builder`.

Each route record includes selected pose ID/hash, class ID/version, motif digest,
exact scalar schedule, float64 materialization, ordered endpoints/segment
digests, root transform, geometry/config/task-card/robot/asset digests,
TCP-only scope, required clearance, per-obstacle results, and full-robot static
qualification false. The design-time 21 mm table is never a runtime field.

**Owned RED nodes**

- Replacement seven sphere-dependent expansions from Task 1's manifest.
- Existing route parameters:
  `test_t152_all_six_complete_routes_are_required_before_scene_acquisition`
  for `missing`, `reordered`, `partial`, `nonfinite`, `workspace`, and
  `contact_exclusion`.
- Existing orchestration parameters:
  `test_t152_orchestration_route_failure_blocks_factory_plan_and_success_evidence`
  for those six cases plus `digest_mismatch`.
- Existing motif nodes for three local and three continuous class expansions,
  digest, scalar schedule, and float64 equality.

**Steps**

- [ ] Build segments from canonical motifs and current inputs; remove the sphere
  data shape from the formal path.
- [ ] Validate all workspace and contact-exclusion segments before any factory
  or runner call.
- [ ] Bind ordered route/motif/root/config/card digests and revalidate them at
  the orchestration boundary.
- [ ] Preserve exact class and command order without changing the command matrix.
- [ ] Make all Task 8-owned T152 RED nodes GREEN; leave loader/lifecycle/runner
  integration nodes RED.
- [ ] Confirm invalid-route parameter cases report zero factory, runner, scene,
  and successful-evidence calls.

**Commands and expected results**

```bash
python -m pytest -q tests/test_g1_pose_conditioned_tracking_cli.py \
  -k 'route or motif or scalar_schedule or float64'
# Before integration: Task 8-owned assertions fail.
# After integration: all selected route/motif nodes pass; unrelated T152 REDs remain.

python -m pytest -q tests/test_g1_contact_exclusion_geometry.py
# Expected after integration: all pass.
```

**Stop conditions**

Stop if route construction trusts caller flags, omits any class/segment, changes
the matrix/clearance, uses the design-time 21 mm value, allows a factory call on
invalid input, or revives the sphere shape as formal authority.

**Commit**

`feat(g1): derive pose-conditioned exclusion routes`

Before and after commit run the two focused commands and the migration manifest.
All mapped replacement nodes and Task 8-owned nodes must be GREEN; all remaining
T152 nodes keep their recorded RED ownership. Task 8 depends on Task 5 and Task 7B GREEN;
Task 9 depends on validated pre-runtime routes. Isaac Sim is not allowed.

## Task 9: Verify selected C2a evidence and current-input freshness

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

**Files**

- Modify: `specs/001-benchmark-reconstruction/tasks.md` only after every
  pre-projection check passes, changing T152 from `[ ]` to `[x]`.
- Modify: `specs/001-benchmark-reconstruction/g1-contact-exclusion-t152-implementation-plan.md`
  in the same projection commit to record only `E_impl`, the already-existing
  parent implementation SHA.
- Produce final G0 repository-integrity evidence at projection commit `P`; this
  is repository evidence, not runtime/physical evidence.

**Commit identities and no-self-reference rule**

- `E_impl` is the last production implementation commit after Tasks 1–10.
- Run the complete pre-projection verification suite at clean `E_impl`.
- After it passes, edit only `tasks.md` and this plan, record the literal
  already-known `E_impl`, and create the one projection/status commit `P` with
  message `docs(g1): complete T152 geometry integration`.
- Do not write `P`'s SHA into either tracked file. Define final `E2=P` only after
  commit creation with `FINAL_E2=$(git rev-parse HEAD)`.
- Verify `P^ == E_impl`, rerun the complete suite at `P`, and create G0 evidence
  bound to `P`.
- After `P`, no tracked file may change. Any tracked change invalidates final
  verification/freshness and requires a new reviewed projection commit.

**Reusable verification suite**

The function below is executed once at `E_impl` and again at `P`. Its 12
ordered checks are: geometry schema, analytic geometry, mechanism/task card,
T152, affected runtime/safety, exact hard limit, original 748 GREEN, dynamically
collected current GREEN, intentional future-RED 125, full classification,
deprecated/import checks, and detached clean-checkout. Dynamic current GREEN is
derived from the current complete collection minus the exact frozen 125
future-RED IDs; it is not an immutable Task 1 selection.

```bash
set -euo pipefail

run_t152_verification() {
  VERIFY_COMMIT=$1
  VERIFY_LABEL=$2
  EXPECTED_CURRENT_GREEN_NODEIDS=${3-}
  EXPECTED_CURRENT_GREEN_DIGEST=${4-}
  EXPECTED_CURRENT_GREEN_COUNT=${5-}
  test "$(git rev-parse HEAD)" = "$VERIFY_COMMIT"
  test -z "$(git status --porcelain)"

  VERIFY_DIR="/tmp/g1-t152-${VERIFY_LABEL}"
  test ! -e "$VERIFY_DIR"
  mkdir -p "$VERIFY_DIR"

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
    tests/test_fr3_runtime_safety.py
  python -m pytest -q \
    tests/test_fr3_runtime_safety.py::test_observed_public_action_displacement_equal_to_exact_hard_limit_passes \
    tests/test_fr3_runtime_safety.py::test_nextafter_above_exact_observed_hard_limit_aborts_without_epsilon \
    tests/test_fr3_runtime_safety.py::test_observed_hard_limit_comparison_source_has_no_epsilon_or_isclose \
    tests/test_fr3_runtime_safety.py::test_physical_safety_config_requires_exact_observed_hard_limit
  python tests/run_g1_node_inventory.py \
    --inventory tests/fixtures/g1_t152_baseline_inventory.json \
    --selection original_green --expect-pass 748

  FUTURE_NODEIDS="$VERIFY_DIR/intentional-future-red-nodeids.txt"
  awk 'NF && $1 !~ /^#/' \
    configs/repository/intentional-future-red-nodeids.txt | sort -u \
    > "$FUTURE_NODEIDS"
  test "$(wc -l < "$FUTURE_NODEIDS")" -eq 125

  ALL_COLLECTION_LOG="$VERIFY_DIR/full-collection.log"
  ALL_NODEIDS="$VERIFY_DIR/all-nodeids.txt"
  python -m pytest --collect-only -q > "$ALL_COLLECTION_LOG"
  awk '/^tests\// && /::/' "$ALL_COLLECTION_LOG" > "$VERIFY_DIR/all-nodeids.raw"
  sort -u "$VERIFY_DIR/all-nodeids.raw" > "$ALL_NODEIDS"
  test "$(wc -l < "$VERIFY_DIR/all-nodeids.raw")" \
    -eq "$(wc -l < "$ALL_NODEIDS")"
  comm -23 "$FUTURE_NODEIDS" "$ALL_NODEIDS" \
    > "$VERIFY_DIR/missing-future-red-nodeids.txt"
  test ! -s "$VERIFY_DIR/missing-future-red-nodeids.txt"

  CURRENT_GREEN_NODEIDS="$VERIFY_DIR/current-green-nodeids.txt"
  comm -23 "$ALL_NODEIDS" "$FUTURE_NODEIDS" > "$CURRENT_GREEN_NODEIDS"
  DESELECT=()
  while IFS= read -r node; do DESELECT+=(--deselect "$node"); done \
    < "$FUTURE_NODEIDS"
  CURRENT_GREEN_JUNIT="$VERIFY_DIR/current-green.xml"
  python -m pytest -q "${DESELECT[@]}" --junitxml="$CURRENT_GREEN_JUNIT"

  CURRENT_GREEN_COUNT=$(wc -l < "$CURRENT_GREEN_NODEIDS")
  python - "$CURRENT_GREEN_JUNIT" "$CURRENT_GREEN_COUNT" <<'PY'
import sys
import xml.etree.ElementTree as ET

root = ET.parse(sys.argv[1]).getroot()
suites = [root] if root.tag == "testsuite" else list(root.iter("testsuite"))
totals = {
    key: sum(int(suite.attrib.get(key, "0")) for suite in suites)
    for key in ("tests", "failures", "errors", "skipped")
}
expected_tests = int(sys.argv[2])
if totals != {
    "tests": expected_tests,
    "failures": 0,
    "errors": 0,
    "skipped": 0,
}:
    raise SystemExit(f"unexpected current-GREEN JUnit totals: {totals}")
PY

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
  comm -23 "$ORIGINAL_CURRENT_NODEIDS" "$CURRENT_GREEN_NODEIDS" \
    > "$VERIFY_DIR/missing-original-green-nodeids.txt"
  test ! -s "$VERIFY_DIR/missing-original-green-nodeids.txt"
  MIGRATED_NEW_GREEN_NODEIDS="$VERIFY_DIR/migrated-new-current-green-nodeids.txt"
  comm -23 "$CURRENT_GREEN_NODEIDS" "$ORIGINAL_CURRENT_NODEIDS" \
    > "$MIGRATED_NEW_GREEN_NODEIDS"
  cat "$ORIGINAL_CURRENT_NODEIDS" "$MIGRATED_NEW_GREEN_NODEIDS" \
    "$FUTURE_NODEIDS" | sort -u > "$VERIFY_DIR/classified-nodeids.txt"
  cmp "$ALL_NODEIDS" "$VERIFY_DIR/classified-nodeids.txt"

  CURRENT_GREEN_DIGEST=$(sha256sum "$CURRENT_GREEN_NODEIDS" | awk '{print $1}')
  printf '%s\n' "$CURRENT_GREEN_COUNT" > "$VERIFY_DIR/current-green-count.txt"
  printf '%s\n' "$CURRENT_GREEN_DIGEST" > "$VERIFY_DIR/current-green-sha256.txt"
  if test -n "$EXPECTED_CURRENT_GREEN_NODEIDS"; then
    cmp "$EXPECTED_CURRENT_GREEN_NODEIDS" "$CURRENT_GREEN_NODEIDS"
    test "$EXPECTED_CURRENT_GREEN_DIGEST" = "$CURRENT_GREEN_DIGEST"
    test "$EXPECTED_CURRENT_GREEN_COUNT" = "$CURRENT_GREEN_COUNT"
  fi

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
  (cd "$CLEAN_DIR" && python -m pytest -q \
    tests/test_press_button_geometry_contract.py \
    tests/test_g1_contact_exclusion_geometry.py \
    tests/test_press_button_mechanism.py \
    tests/test_press_button_task_card_contract.py \
    tests/test_g1_pose_conditioned_tracking_cli.py)
}
```

Expected at both commits: all focused/current GREEN nodes pass; the exact
original 748 remain GREEN; exactly 125 intentional future-REDs retain the
78/29/10/8 classifications with no unexpected pass/error/skip; all collection
and import/help checks succeed; exact hard limit remains `0.0005`; and the clean
archive passes from tracked files only. The dynamic current-GREEN JUnit must
record failures=0, errors=0, skipped=0, and its node-ID list, count, digest, and
outcomes at `P` must equal `E_impl`. Every collected node is classified as one
of the 752 immutable original GREEN/control IDs, a migrated/new current GREEN
ID, or one of the exact 125 intentional future-RED IDs. No command launches
Isaac Sim.

**Steps**

- [ ] Bind `E_IMPL=$(git rev-parse HEAD)` after Task 10, verify the worktree is
  clean, run `run_t152_verification "$E_IMPL" pre-projection`, and retain the
  dynamic current-GREEN snapshot:

  ```bash
  EIMPL_CURRENT_GREEN_NODEIDS=/tmp/g1-t152-pre-projection/current-green-nodeids.txt
  EIMPL_CURRENT_GREEN_DIGEST=$(cat \
    /tmp/g1-t152-pre-projection/current-green-sha256.txt)
  EIMPL_CURRENT_GREEN_COUNT=$(cat \
    /tmp/g1-t152-pre-projection/current-green-count.txt)
  ```
- [ ] After the suite passes, change only T152 to `[x]` in `tasks.md`; leave T151
  and T070 `[ ]` and attempt-04 prohibited.
- [ ] Record the literal `E_impl` SHA in this plan without attempting to record
  the not-yet-created projection SHA.
- [ ] Run `git diff --check`, stage only the two Markdown files, verify their
  diff, and create the unique projection commit:

  ```bash
  git add specs/001-benchmark-reconstruction/tasks.md \
    specs/001-benchmark-reconstruction/g1-contact-exclusion-t152-implementation-plan.md
  git diff --cached --check
  git commit -m "docs(g1): complete T152 geometry integration"
  ```

- [ ] Dynamically bind and verify final E2/P, then rerun the complete suite:

  ```bash
  FINAL_E2=$(git rev-parse HEAD)
  test "$(git rev-parse "${FINAL_E2}^")" = "$E_IMPL"
  run_t152_verification "$FINAL_E2" final-projection \
    "$EIMPL_CURRENT_GREEN_NODEIDS" \
    "$EIMPL_CURRENT_GREEN_DIGEST" \
    "$EIMPL_CURRENT_GREEN_COUNT"
  ```

- [ ] Produce final G0 evidence bound only to `FINAL_E2=P`:

  ```bash
  FINAL_E2_SHORT=${FINAL_E2:0:12}
  G0_OUTPUT="outputs/evidence/G0/t152-geometry-${FINAL_E2_SHORT}"
  test ! -e "$G0_OUTPUT"
  python scripts/check_clean_checkout.py --output "$G0_OUTPUT"
  python scripts/review_gate.py --gate G0 \
    --evidence "$G0_OUTPUT/manifest.json"
  (cd "$G0_OUTPUT" && sha256sum -c checksums.sha256)
  test -z "$(git status --porcelain --untracked-files=no)"
  ```

- [ ] Freeze `FINAL_E2=P`: do not modify any tracked file after the final suite
  and G0 evidence. A separately approved fresh C2a must bind this SHA, not
  `E_impl`.

**Stop conditions**

Stop on any node-ID/classification drift, unexpected pass/error/skip,
hard-limit/clearance/matrix/truth change, deprecated diagnostic, help-time Isaac
startup, dirty archive dependency, projection parent mismatch, tracked change
after `P`, G0 checksum failure, or pressure to advance T151/T070/attempt-04.

**Commit**

`docs(g1): complete T152 geometry integration`

This is the sole projection/status commit `P` and final `E2`. Its parent is
`E_impl`; tracked files contain only the parent implementation SHA. Task 11
depends on Tasks 1–10. Isaac Sim is not allowed.

## Task 12: Prepare the separately approved fresh C2a refresh; do not execute it

**Files**

- No file changes are required.
- Read only: final `E2=P` commit, the C2a script/configs, and the future output
  parent.

This task prepares a review card only. It does not run the command, create the
directory, accept an attempt number without approval, or assume a selected pose.

**Preparation checklist**

- [ ] Bind `FINAL_E2=$(git rev-parse HEAD)` only after Task 11's projection
  commit, final verification replay, and G0 review.
- [ ] Obtain a separate user authorization containing the one permitted attempt
  ID and exact output path.
- [ ] Verify worktree clean and local/tracking/live-origin/PR heads equal
  `FINAL_E2=P`.
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
directory; verifies current `FINAL_E2=P`/config/card/asset provenance; recomputes
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

No commit. Task 12 depends on final `E2=P` from Task 11 and a new user
authorization. Isaac Sim is explicitly forbidden during plan implementation and
T152 GREEN; it is permitted only by that later one-run authorization.

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
| 8 | `feat(g1): derive pose-conditioned exclusion routes` | `python -m pytest -q tests/test_g1_pose_conditioned_tracking_cli.py -k 'route or motif or scalar_schedule or float64'` | all route/motif-owned nodes GREEN; factory remains untouched on invalid route |
| 9 | `feat(g1): verify current C2a pose evidence` | `python -m pytest -q tests/test_g1_pose_conditioned_tracking_cli.py -k 'c2a_evidence or selected_candidate or selected_pose or current_input or provenance'` | all loader/freshness nodes GREEN; attempt-02 rejected as historical |
| 10 | `feat(g1): wire pose-conditioned multiclass CLI` | `python -m pytest -q tests/test_g1_pose_conditioned_tracking_cli.py` | complete file GREEN; no legacy real-main path |
| 11 | `docs(g1): complete T152 geometry integration` | complete suite at `E_impl`, projection commit `P`, complete suite and G0 at `P` | `P^=E_impl`, `E2=P`, T152 `[x]`, T151/T070 `[ ]`, attempt-04 prohibited, no tracked change after `P` |

Tasks 2, 3, and 7A are deliberately RED-only commits. No production change may
be included with them. Tasks 4–10 may not modify approved RED assertions except
the one-to-one schema-specific migration committed in Task 2 and the
architecture-approved geometry-only seam correction in Task 7A.

## Approved design-section coverage

| Architecture review section | Implementing tasks | Verification |
|---|---|---|
| 1. Scope and truth boundary | 2, 5, 8, 10 | TCP-only fields plus mandatory runtime truth nodes |
| 2. One authoritative geometry source | 4, 6, 7A, 7B | strict YAML parser, geometry-only receipt identity/digests, complete-builder literal-source guard |
| 3. Geometry parsing and validation | 2, 4, 6 | schema/dimension/token/quaternion/digest tests |
| 4. Distance, clearance, continuous mathematics | 3, 5 | slab/quadratic/boundary/degenerate analytic tests |
| 5. Six-route design-time assessment | 8 | six canonical continuous routes; no runtime 21 mm claim |
| 6. Derived route evidence schema | 5, 8, 10 | obstacle/segment/root/config provenance cross-recording |
| 7. Fail-closed taxonomy | 2–10 | exact code and non-empty-message assertions at each boundary |
| 8. RED fixture correction | 1–3 | one-to-one migration manifest and observed schema RED |
| 9. Version migration/consumer synchronization | 6, 7A, 7B, 9 | formal 1.1.0, state-only legacy, card/config/digest synchronization |
| 10. Freshness/C2a invalidation | 6, 9, 11, 12 | Task 6 proves stale digests, Task 9 propagates the blocker, and a fresh run is separately gated at final `E2=P` |
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
| Plan carries canonical motif | 1 | 8 |
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
| Orchestration invalid route parameters | 7 | 8 |
| Route builder derives inputs | 1 | 8 |
| Pose/task/workspace/contact geometry mutations | 4 | 8 |
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
| `derive_g1_pose_conditioned_routes` | Task 8 | Tasks 8–10 script orchestration |
| `validate_g1_pose_conditioned_routes` | Task 8 | Tasks 8–10 script orchestration |
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
  pre-projection verification → projection/final `E2` commit `P` → final
  verification and G0 evidence at `P` → separately approved fresh C2a at `P` →
  evidence review → T151 review → separately approved attempt-04.
- [x] T150 remains `[x]`; T151, T152, and T070 remain `[ ]` while this plan is
  authored; attempt-04 remains `ATTEMPT_04_PROHIBITED`.

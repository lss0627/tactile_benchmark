"""Frozen paper-v1 benchmark scale and protocol constants."""

from __future__ import annotations


PAPER_V1_SUITE_TASKS = {
    "precision": (
        "peg_insertion",
        "usb_like_insertion",
        "key_insertion_turn",
        "pin_socket_alignment",
    ),
    "articulation": (
        "button_press_release",
        "switch_actuation",
        "drawer_motion",
        "cap_knob_twist",
    ),
    "surface_interaction": (
        "sliding",
        "wiping",
        "scraping",
        "surface_following",
    ),
    "deformable_contact": (
        "soft_pressing",
        "sponge_compression",
        "fabric_pull_place",
        "cable_soft_part_seating",
    ),
}

PAPER_V1_PROTOCOLS = {
    "GP-01": "object_geometry",
    "GP-02": "contact_material_physics",
    "GP-03": "sensor_observation",
}

MIN_ACCEPTED_TRAINING_DEMONSTRATIONS_PER_TASK = 50
MIN_TOTAL_ACCEPTED_TRAINING_DEMONSTRATIONS = 800
PAPER_V1_POLICY_SEED_COUNT = 3
MIN_EVALUATION_EPISODES_PER_TASK_CONDITION_PER_SEED = 20


def validate_paper_v1_constants() -> list[str]:
    errors: list[str] = []
    if len(PAPER_V1_SUITE_TASKS) != 4:
        errors.append("paper-v1 must contain exactly four suites")
    task_ids = [
        task_id
        for suite_task_ids in PAPER_V1_SUITE_TASKS.values()
        for task_id in suite_task_ids
    ]
    if any(len(task_ids_for_suite) != 4 for task_ids_for_suite in PAPER_V1_SUITE_TASKS.values()):
        errors.append("every paper-v1 suite must contain exactly four tasks")
    if len(task_ids) != 16 or len(set(task_ids)) != 16:
        errors.append("paper-v1 must contain exactly 16 unique task identifiers")
    if tuple(PAPER_V1_PROTOCOLS) != ("GP-01", "GP-02", "GP-03"):
        errors.append("paper-v1 must contain exactly GP-01, GP-02, and GP-03")
    if MIN_ACCEPTED_TRAINING_DEMONSTRATIONS_PER_TASK < 50:
        errors.append("paper-v1 requires at least 50 accepted demonstrations per task")
    if MIN_TOTAL_ACCEPTED_TRAINING_DEMONSTRATIONS < (
        len(task_ids) * MIN_ACCEPTED_TRAINING_DEMONSTRATIONS_PER_TASK
    ):
        errors.append("paper-v1 requires at least 800 accepted demonstrations")
    if PAPER_V1_POLICY_SEED_COUNT != 3:
        errors.append("paper-v1 requires exactly three policy seeds")
    if MIN_EVALUATION_EPISODES_PER_TASK_CONDITION_PER_SEED < 20:
        errors.append("paper-v1 requires at least 20 evaluation episodes per condition and seed")
    return errors


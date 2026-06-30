def _episode(success=True, steps=3, max_force=2.0, mean_force=1.0, jamming=0, depth=0.03):
    return {
        "task_name": "PegInsert",
        "suite_name": "tactile_assembly",
        "tactile_mode": "force_wrench",
        "seed": 0,
        "num_steps": steps,
        "max_steps": 10,
        "control_frequency_hz": 20.0,
        "success": success,
        "metrics": {
            "max_contact_force": max_force,
            "mean_contact_force": mean_force,
            "force_violation_rate": 0.25,
            "contact_duration": 0.2,
            "contact_loss_count": 1,
            "jamming_count": jamming,
            "insertion_depth": depth,
        },
    }


def test_mock_success_and_contact_metrics_are_computed_from_episode_records():
    from isaac_tactile_libero.metrics.assembly import insertion_depth, jamming_count
    from isaac_tactile_libero.metrics.contact import (
        contact_duration,
        contact_loss_count,
        force_violation_rate,
        max_contact_force,
        mean_contact_force,
    )
    from isaac_tactile_libero.metrics.success import completion_time, success_rate, trajectory_length

    episodes = [_episode(success=True, steps=3), _episode(success=False, steps=10)]

    assert success_rate(episodes) == 0.5
    assert completion_time(episodes[0]) == 0.15
    assert trajectory_length(episodes[1]) == 10
    assert max_contact_force(episodes[0]) == 2.0
    assert mean_contact_force(episodes[0]) == 1.0
    assert force_violation_rate(episodes[0]) == 0.25
    assert contact_duration(episodes[0]) == 0.2
    assert contact_loss_count(episodes[0]) == 1
    assert jamming_count(episodes[0]) == 0
    assert insertion_depth(episodes[0]) == 0.03


def test_mock_metric_aggregation_groups_by_task_and_tactile_mode():
    from isaac_tactile_libero.metrics.aggregation import aggregate_by, aggregate_episodes

    episodes = [
        _episode(success=True, steps=3, max_force=2.0, mean_force=1.0, jamming=0, depth=0.03),
        _episode(success=False, steps=10, max_force=4.0, mean_force=3.0, jamming=2, depth=0.01),
    ]

    overall = aggregate_episodes(episodes)
    assert overall["num_episodes"] == 2
    assert overall["success_rate"] == 0.5
    assert overall["completion_time"] == 0.325
    assert overall["trajectory_length"] == 6.5
    assert overall["max_contact_force"] == 4.0
    assert overall["mean_contact_force"] == 2.0
    assert overall["force_violation_rate"] == 0.25
    assert overall["contact_duration"] == 0.2
    assert overall["contact_loss_count"] == 1.0
    assert overall["jamming_count"] == 1.0
    assert overall["insertion_depth"] == 0.02

    grouped = aggregate_by(episodes, ["task_name", "tactile_mode"])
    assert grouped == [
        {
            "task_name": "PegInsert",
            "tactile_mode": "force_wrench",
            **overall,
        }
    ]

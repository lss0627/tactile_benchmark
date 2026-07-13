import runpy


def test_clean_checkout_plan_has_required_isolated_steps() -> None:
    module = runpy.run_path("scripts/check_clean_checkout.py")
    plan = module["build_plan"]("python", "outputs/evidence/G0/clean-checkout")
    assert plan["uses_git_archive"] is True
    assert plan["builds_wheel"] is True
    assert plan["installs_wheel_in_venv"] is True
    assert plan["runs_no_simulator_tests"] is True
    assert plan["reads_original_worktree"] is False
    assert plan["gate"] == "G0"

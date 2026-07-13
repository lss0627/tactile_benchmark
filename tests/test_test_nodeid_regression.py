from scripts.check_test_nodeids import compare_nodeids


def test_nodeid_regression_requires_all_original_nodes() -> None:
    report = compare_nodeids(["a::one", "b::two"], ["a::one", "b::two", "c::new"])
    assert report["ok"] is True
    assert report["missing"] == []
    assert report["added"] == ["c::new"]


def test_nodeid_regression_reports_missing_nodes() -> None:
    report = compare_nodeids(["a::one", "b::two"], ["a::one"])
    assert report["ok"] is False
    assert report["missing"] == ["b::two"]

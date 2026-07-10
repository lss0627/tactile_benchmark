def test_contact_force_method_report_schema_for_unavailable_and_synthetic_available():
    from isaac_tactile_libero.envs.isaacsim_contact_force import ContactForceReport

    unavailable = ContactForceReport.unavailable(method="contact_sensor", error="module missing").as_dict()
    assert unavailable["method"] == "contact_sensor"
    assert unavailable["contact_probe_method"] == "unavailable"
    assert unavailable["contact_force_available"] is False
    assert unavailable["contact_force_norm"] == 0.0
    assert unavailable["contact_force_unit"] == "N"
    assert unavailable["contact_api_error"] == "module missing"
    assert unavailable["benchmark_result"] is False

    available = ContactForceReport.available(
        method="physx_contact_report",
        force_vector=[0.0, 0.0, 4.0],
        source="physx_contact_report",
        contact_signal_seen=True,
    ).as_dict()
    assert available["method"] == "physx_contact_report"
    assert available["contact_probe_method"] == "physx_contact_report"
    assert available["contact_force_available"] is True
    assert available["contact_force_norm"] == 4.0
    assert available["max_contact_force_norm"] == 4.0
    assert available["mean_contact_force_norm"] == 4.0
    assert available["contact_force_vector"] == [0.0, 0.0, 4.0]
    assert available["contact_force_source"] == "physx_contact_report"
    assert available["benchmark_result"] is False

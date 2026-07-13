from isaac_tactile_libero.robots.fr3_introspection import select_frame_candidates


def test_fr3_joint_frame_candidate_parsing_prefers_fr3_names():
    names = [
        "/World/FR3/fr3_link0",
        "/World/FR3/fr3_link7",
        "/World/FR3/fr3_hand",
        "/World/FR3/fr3_leftfinger",
        "/World/FR3/random_mesh",
    ]

    candidates = select_frame_candidates(names)

    assert "/World/FR3/fr3_link0" in candidates["base_frame_candidates"]
    assert "/World/FR3/fr3_hand" in candidates["ee_frame_candidates"]
    assert "/World/FR3/fr3_hand" in candidates["gripper_frame_candidates"]
    assert "/World/FR3/fr3_leftfinger" in candidates["finger_link_candidates"]

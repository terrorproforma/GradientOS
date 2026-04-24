import json

from gradient_os.absolute_encoder_anchors import (
    ANCHORS_ENV_VAR,
    invalidate_absolute_encoder_anchors,
    load_absolute_encoder_anchors,
    save_absolute_encoder_anchor,
)


def _seed_anchors(tmp_path, monkeypatch, *, robot_id: str, num_joints: int):
    anchors_path = tmp_path / "anchors.json"
    monkeypatch.setenv(ANCHORS_ENV_VAR, str(anchors_path))
    for joint_i in range(num_joints):
        save_absolute_encoder_anchor(
            robot_id,
            num_joints=num_joints,
            logical_joint_index=joint_i,
            home_anchor_rad=float(joint_i) * 0.25,
            source="unit_test",
            axis_indices=[joint_i],
            actor="unit_test",
        )
    return anchors_path


def test_invalidate_absolute_encoder_anchors_clears_only_target_joints(tmp_path, monkeypatch):
    anchors_path = _seed_anchors(
        tmp_path, monkeypatch, robot_id="test_robot", num_joints=6
    )
    seeded = load_absolute_encoder_anchors("test_robot", num_joints=6)
    assert all(entry is not None for entry in seeded)

    saved = invalidate_absolute_encoder_anchors(
        "test_robot",
        num_joints=6,
        logical_joint_indices=[2, 4, 5],
        actor="unit_test",
    )

    # Return value mirrors save_absolute_encoder_anchors shape.
    assert isinstance(saved, dict)
    stored_entries = saved.get("logical_joint_absolute_home_anchors")
    assert isinstance(stored_entries, list)

    # Only J3/J5/J6 (0-indexed 2/4/5) should be cleared.
    loaded = load_absolute_encoder_anchors("test_robot", num_joints=6)
    assert loaded[0] is not None
    assert loaded[1] is not None
    assert loaded[2] is None
    assert loaded[3] is not None
    assert loaded[4] is None
    assert loaded[5] is None

    # On-disk file is pretty-printed JSON; sanity check it parsed.
    payload = json.loads(anchors_path.read_text())
    robot_entry = payload["robots"]["test_robot"]
    assert robot_entry["updated_by"] == "unit_test"


def test_invalidate_absolute_encoder_anchors_ignores_out_of_range_indices(tmp_path, monkeypatch):
    _seed_anchors(tmp_path, monkeypatch, robot_id="test_robot", num_joints=3)
    # Mix of valid, negative, and out-of-range - none should raise.
    invalidate_absolute_encoder_anchors(
        "test_robot",
        num_joints=3,
        logical_joint_indices=[-1, 0, 99, "not_an_int", 2],
        actor="unit_test",
    )
    loaded = load_absolute_encoder_anchors("test_robot", num_joints=3)
    assert loaded[0] is None  # cleared
    assert loaded[1] is not None  # untouched
    assert loaded[2] is None  # cleared


def test_invalidate_absolute_encoder_anchors_empty_list_round_trips(tmp_path, monkeypatch):
    """When the startup preflight calls this with an empty list (no
    encoder-retention faults observed), the anchor store must round
    trip unchanged but still persist an ``updated_at`` stamp so
    operators can see the preflight touched the file.
    """
    _seed_anchors(tmp_path, monkeypatch, robot_id="test_robot", num_joints=3)
    seeded = load_absolute_encoder_anchors("test_robot", num_joints=3)
    invalidate_absolute_encoder_anchors(
        "test_robot",
        num_joints=3,
        logical_joint_indices=[],
        actor="unit_test",
    )
    after = load_absolute_encoder_anchors("test_robot", num_joints=3)
    assert [entry is None for entry in seeded] == [entry is None for entry in after]
    assert all(entry is not None for entry in after)

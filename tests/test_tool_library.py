from gradient_os import tool_library


def test_default_library_contains_identity_and_tig(monkeypatch, tmp_path):
    monkeypatch.setenv(tool_library.TOOL_LIBRARY_ENV_VAR, str(tmp_path))
    payload = tool_library.load_tool_library()
    tool_ids = {item["tool_id"] for item in payload["tools"]}
    assert "identity" in tool_ids
    assert "tig-torch-65deg" in tool_ids


def test_upsert_and_filter(monkeypatch, tmp_path):
    monkeypatch.setenv(tool_library.TOOL_LIBRARY_ENV_VAR, str(tmp_path))
    tool_library.upsert_tool(
        {
            "tool_id": "fixture-probe",
            "display_name": "Fixture Probe",
            "tool_type": "probe",
            "keywords": ["probe", "fixture"],
            "compatible_robot_ids": ["gradient-05"],
            "offset": {
                "position_mm": {"x": 5, "y": 0, "z": 10},
                "rotation_deg": {"x": 0, "y": 0, "z": 0},
            },
        },
        actor="pytest",
    )
    filtered = tool_library.list_tools(robot_id="gradient-05", tool_type="probe", query="fixture")
    assert len(filtered) == 1
    assert filtered[0]["tool_id"] == "fixture-probe"
    assert (tmp_path / "fixture-probe" / "tool.json").exists()


def test_resolve_active_tool_fallback(monkeypatch, tmp_path):
    monkeypatch.setenv(tool_library.TOOL_LIBRARY_ENV_VAR, str(tmp_path))
    resolved = tool_library.resolve_active_tool(robot_id="gradient-05", requested_tool_id="unknown-tool")
    assert resolved["tool_id"] == "identity"


def test_mesh_string_shorthand_and_mesh_transform(monkeypatch, tmp_path):
    monkeypatch.setenv(tool_library.TOOL_LIBRARY_ENV_VAR, str(tmp_path))
    tool_library.upsert_tool(
        {
            "tool_id": "torch-mesh-test",
            "display_name": "Torch Mesh Test",
            "tool_type": "tig_torch",
            "offset": {
                "position_mm": {"x": 0, "y": 0, "z": 100},
                "rotation_deg": {"x": 0, "y": 45, "z": 0},
            },
            "mesh": "mesh.stl",
        },
        actor="pytest",
    )
    tool = tool_library.get_tool("torch-mesh-test")
    assert tool["mesh"]["asset_path"] == "torch-mesh-test/mesh.stl"
    assert tool["mesh"]["scale"] == 1.0
    assert tool["mesh"]["position_mm"] == {"x": 0.0, "y": 0.0, "z": 0.0}
    assert tool["mesh"]["rotation_deg"] == {"x": 0.0, "y": 0.0, "z": 0.0}

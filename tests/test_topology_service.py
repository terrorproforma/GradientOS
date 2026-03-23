from __future__ import annotations

from gradient_os.cad import topology_service


def test_missing_occ_dependency_message_mentions_cad_extra():
    message = topology_service._missing_occ_dependency_message()

    assert "uv pip install -e '.[cad]'" in message
    assert "cadquery-ocp" in message
    assert "pythonocc-core" in message

from scripts.cartesian_jog_diagnostic_capture import _extract_jog_payload, analyze_python_records


def _record(
    *,
    branch_delta=None,
    near_singular=False,
    sigma_min=0.1,
    drift=0.0,
    gate_reason=None,
):
    ab_compare = None
    if branch_delta is not None:
        ab_compare = {"ik_minus_jacobian_max_abs_rad": branch_delta}
    return {
        "ik_debug": {
            "ab_compare": ab_compare,
            "jacobian_diagnostics": {
                "jacobian_near_singular": near_singular,
                "jacobian_sigma_min": sigma_min,
            },
            "command_drift_norm_rad": drift,
            "gate_result": "rejected" if gate_reason else "accepted",
            "gate_reason": gate_reason,
        }
    }


def test_analyze_python_records_clean_session():
    summary = analyze_python_records([_record(drift=0.0), _record(drift=0.01)])

    assert summary["branch_flips"] == []
    assert summary["singularity_spans"] == []
    assert summary["gate_rejections"] == {}
    assert summary["max_command_drift_rad"] == 0.01


def test_analyze_python_records_branch_flip():
    summary = analyze_python_records([_record(branch_delta=0.1), _record(branch_delta=1.2)])

    assert len(summary["branch_flips"]) == 1
    assert summary["branch_flips"][0]["index"] == 1
    assert summary["branch_flips"][0]["delta_rad"] == 1.2


def test_analyze_python_records_singularity_span_and_gate():
    summary = analyze_python_records(
        [
            _record(near_singular=False),
            _record(near_singular=True, sigma_min=0.01),
            _record(near_singular=True, sigma_min=0.02, gate_reason="JOG_COMMAND_DRIFT_EXCEEDED"),
            _record(near_singular=False),
        ]
    )

    assert summary["singularity_spans"] == [
        {"start_index": 1, "start_sigma_min": 0.01, "end_index": 2}
    ]
    assert summary["gate_rejections"] == {"JOG_COMMAND_DRIFT_EXCEEDED": 1}


def test_analyze_python_records_monotonic_drift():
    summary = analyze_python_records([_record(drift=0.0), _record(drift=0.1), _record(drift=0.2)])

    assert summary["monotonic_command_drift"] is True


def test_extract_jog_payload_from_debug_performance_shape():
    payload = {
        "controller": {
            "jog": {
                "ik_debug": {"using_jacobian": True},
            }
        }
    }

    assert _extract_jog_payload(payload) == {"ik_debug": {"using_jacobian": True}}

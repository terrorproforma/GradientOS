from gradient_os.telemetry.terminal_dashboard import (
    TerminalDashboardState,
    process_service_log_line,
)


def test_terminal_dashboard_filters_noisy_api_access_log_lines():
    state = TerminalDashboardState()
    emitted = process_service_log_line(
        "api",
        'INFO:     127.0.0.1:50710 - "GET /info/joints-detailed HTTP/1.1" 200 OK\n',
        state,
    )
    assert emitted == []


def test_terminal_dashboard_filters_noisy_controller_joint_state_lines():
    state = TerminalDashboardState()
    emitted = process_service_log_line(
        "controller",
        "[Controller] Received: 'GET_JOINT_STATE' from ('127.0.0.1', 50710)\n",
        state,
    )
    assert emitted == []


def test_terminal_dashboard_filters_other_read_only_controller_request_lines():
    state = TerminalDashboardState()
    emitted = process_service_log_line(
        "controller",
        "[Controller] Received: 'GET_RUNTIME_CONFIG' from ('127.0.0.1', 50710)\n",
        state,
    )
    assert emitted == []


def test_terminal_dashboard_emits_transition_when_canonical_truth_becomes_unavailable():
    state = TerminalDashboardState()
    emitted = process_service_log_line(
        "controller",
        "[Controller] Canonical joint truth monitor: UNAVAILABLE reason=absolute_home_anchor_missing axes=[0, 1, 2, 3, 4, 5] joints=[1, 2, 3, 4, 5, 6] read_source=cached_fallback\n",
        state,
    )
    assert emitted == [
        "// LIVE STATE // canonical truth: UNAVAILABLE (absolute_home_anchor_missing)"
    ]
    assert state.canonical_truth_available is False
    assert state.canonical_truth_reason == "absolute_home_anchor_missing"


def test_terminal_dashboard_only_emits_canonical_truth_transition_once_per_state():
    state = TerminalDashboardState(
        canonical_truth_available=False,
        canonical_truth_reason="absolute_home_anchor_missing",
    )
    emitted = process_service_log_line(
        "controller",
        "[Controller] Canonical joint truth monitor: UNAVAILABLE reason=absolute_home_anchor_missing axes=[0, 1, 2, 3, 4, 5] joints=[1, 2, 3, 4, 5, 6] read_source=cached_fallback\n",
        state,
    )
    assert emitted == []


def test_terminal_dashboard_filters_noisy_api_runtime_requests():
    state = TerminalDashboardState()
    emitted = process_service_log_line(
        "api",
        'INFO:     127.0.0.1:33234 - "GET /info/runtime-config HTTP/1.1" 200 OK\n',
        state,
    )
    assert emitted == []


def test_terminal_dashboard_filters_blank_and_position_error_lines():
    state = TerminalDashboardState()
    assert process_service_log_line("controller", "\n", state) == []
    emitted = process_service_log_line(
        "controller",
        "[Pi] ERROR: Could not fetch current position joints: Canonical joint truth unavailable (axes=[0, 1], joints=[1, 2])\n",
        state,
    )
    assert emitted == []

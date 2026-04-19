from datetime import datetime, timezone

from gradient_os.telemetry.terminal_dashboard import (
    DASHBOARD_LABEL,
    TerminalDashboardState,
    format_log_entry,
    format_timestamp,
    log_palette_from_env,
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
        (
            DASHBOARD_LABEL,
            "// LIVE STATE // canonical truth: UNAVAILABLE (absolute_home_anchor_missing)",
        )
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


def test_terminal_dashboard_passes_through_regular_service_lines_with_label_preserved():
    state = TerminalDashboardState()
    emitted = process_service_log_line(
        "controller",
        "[Backend Registry] Registered backend: ethercat_rtcore\n",
        state,
    )
    assert emitted == [
        ("controller", "[Backend Registry] Registered backend: ethercat_rtcore")
    ]


def test_format_timestamp_has_launcher_shape():
    fixed = datetime(2026, 4, 17, 9, 23, 30, tzinfo=timezone.utc)
    assert format_timestamp(fixed) == "2026-04-17 09:23:30+0000"


def test_format_log_entry_emits_plain_ascii_when_palette_empty():
    fixed = datetime(2026, 4, 17, 9, 23, 30, tzinfo=timezone.utc)
    rendered = format_log_entry(
        "controller",
        "[Controller] RTCore ready: startup_ready=1",
        now=fixed,
        palette={"muted": "", "label": "", "reset": ""},
    )
    assert rendered == (
        "[2026-04-17 09:23:30+0000] [controller] "
        "[Controller] RTCore ready: startup_ready=1"
    )


def test_format_log_entry_wraps_timestamp_and_label_in_ansi_codes_when_palette_set():
    fixed = datetime(2026, 4, 17, 9, 23, 30, tzinfo=timezone.utc)
    rendered = format_log_entry(
        "api",
        "INFO: Uvicorn running on http://0.0.0.0:4400",
        now=fixed,
        palette={"muted": "\x1b[38;5;244m", "label": "\x1b[1;38;5;45m", "reset": "\x1b[0m"},
    )
    assert rendered == (
        "\x1b[38;5;244m[2026-04-17 09:23:30+0000]\x1b[0m "
        "\x1b[1;38;5;45m[api]\x1b[0m "
        "INFO: Uvicorn running on http://0.0.0.0:4400"
    )


def test_log_palette_from_env_reads_launcher_style_vars():
    env = {
        "GRADIENT_STACK_STYLE_MUTED": "\x1b[38;5;244m",
        "GRADIENT_STACK_STYLE_LABEL": "\x1b[1;38;5;45m",
        "GRADIENT_STACK_STYLE_RESET": "\x1b[0m",
    }
    palette = log_palette_from_env(env)
    assert palette == {
        "muted": "\x1b[38;5;244m",
        "label": "\x1b[1;38;5;45m",
        "reset": "\x1b[0m",
    }


def test_log_palette_from_env_defaults_to_empty_strings_when_unset():
    assert log_palette_from_env({}) == {"muted": "", "label": "", "reset": ""}

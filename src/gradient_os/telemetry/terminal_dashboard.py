from __future__ import annotations

import re
from dataclasses import dataclass

_NOISY_API_ACCESS_RE = re.compile(
    r'GET /(info/joints-detailed|info/joints|health|control/motion-status|info/runtime-config|info/robots|tools/library|robot-program/list|monitor|info/pose)\b'
)
_NOISY_CONTROLLER_REQUEST_RE = re.compile(r"Received: 'GET_JOINT_STATE'")
_NOISY_CONTROLLER_REQUEST_EXTRA_RE = re.compile(
    r"Received: '(GET_STATUS|GET_RUNTIME_CONFIG|GET_POSITION)'"
)
_NOISY_CONTROLLER_WARNING_RE = re.compile(
    r"joint snapshot live read failed: Canonical joint truth unavailable"
)
_NOISY_CONTROLLER_POSITION_ERROR_RE = re.compile(
    r"Could not fetch current position joints: Canonical joint truth unavailable"
)
_CANONICAL_MONITOR_RE = re.compile(
    r"Canonical joint truth monitor:\s+"
    r"(?P<status>AVAILABLE|UNAVAILABLE)"
    r"(?:\s+reason=(?P<reason>[A-Za-z0-9_\-]+))?"
)


@dataclass
class TerminalDashboardState:
    canonical_truth_available: bool | None = None
    canonical_truth_reason: str | None = None


def _format_canonical_truth_transition(
    available: bool,
    reason: str | None,
) -> str:
    if available:
        return "// LIVE STATE // canonical truth: AVAILABLE"
    if reason:
        return f"// LIVE STATE // canonical truth: UNAVAILABLE ({reason})"
    return "// LIVE STATE // canonical truth: UNAVAILABLE"


def process_service_log_line(
    label: str,
    line: str,
    state: TerminalDashboardState,
) -> list[str]:
    text = line.rstrip("\n")
    lower_label = str(label or "").strip().lower()

    if not text.strip():
        return []

    if lower_label == "api" and _NOISY_API_ACCESS_RE.search(text):
        return []

    if lower_label == "controller":
        if _NOISY_CONTROLLER_REQUEST_RE.search(text):
            return []
        if _NOISY_CONTROLLER_REQUEST_EXTRA_RE.search(text):
            return []
        if _NOISY_CONTROLLER_WARNING_RE.search(text):
            return []
        if _NOISY_CONTROLLER_POSITION_ERROR_RE.search(text):
            return []

        match = _CANONICAL_MONITOR_RE.search(text)
        if match:
            available = match.group("status") == "AVAILABLE"
            reason = match.group("reason")
            if (
                state.canonical_truth_available != available
                or state.canonical_truth_reason != reason
            ):
                state.canonical_truth_available = available
                state.canonical_truth_reason = reason
                return [_format_canonical_truth_transition(available, reason)]
            return []

    return [f"[{label}] {text}"]

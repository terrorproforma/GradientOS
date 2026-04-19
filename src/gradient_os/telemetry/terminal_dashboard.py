from __future__ import annotations

import os
import re
from dataclasses import dataclass
from datetime import datetime

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

# Label used for launcher-side dashboard/state transition lines so every emitted
# log entry carries a consistent `[label]` tag alongside the start-stack logger.
DASHBOARD_LABEL = "dashboard"

# Env vars that let start-stack.sh hand its live color palette down into the
# Python helpers so the decorated log lines match the rest of the launcher UI.
_STYLE_ENV_MUTED = "GRADIENT_STACK_STYLE_MUTED"
_STYLE_ENV_LABEL = "GRADIENT_STACK_STYLE_LABEL"
_STYLE_ENV_RESET = "GRADIENT_STACK_STYLE_RESET"


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
) -> list[tuple[str, str]]:
    """Classify a raw service log line for the launcher dashboard.

    Returns a list of ``(label, message)`` tuples that the caller should format
    for display.  Filtered-out lines return an empty list.  Canonical-truth
    state transitions are re-labelled as ``dashboard`` so they render with the
    same ``[timestamp] [dashboard] ...`` chrome as every other launcher line.
    """

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
                return [
                    (
                        DASHBOARD_LABEL,
                        _format_canonical_truth_transition(available, reason),
                    )
                ]
            return []

    return [(str(label), text)]


def log_palette_from_env(env: dict[str, str] | None = None) -> dict[str, str]:
    """Load the ANSI style palette from env vars exported by start-stack.sh.

    Returns a dict with ``muted``, ``label``, and ``reset`` keys.  Missing or
    empty env vars resolve to empty strings, which collapses the formatter
    back to a plain ASCII ``[ts] [label] message`` rendering.
    """

    source = env if env is not None else os.environ
    return {
        "muted": source.get(_STYLE_ENV_MUTED, "") or "",
        "label": source.get(_STYLE_ENV_LABEL, "") or "",
        "reset": source.get(_STYLE_ENV_RESET, "") or "",
    }


def format_timestamp(now: datetime | None = None) -> str:
    """Return a launcher-style timestamp matching ``date '+%Y-%m-%d %H:%M:%S%z'``."""

    moment = now if now is not None else datetime.now().astimezone()
    if moment.tzinfo is None:
        moment = moment.astimezone()
    return moment.strftime("%Y-%m-%d %H:%M:%S%z")


def format_log_entry(
    label: str,
    message: str,
    *,
    now: datetime | None = None,
    palette: dict[str, str] | None = None,
) -> str:
    """Format a service log line to match start-stack.sh's print_log_line.

    Produces ``[timestamp] [label] message`` with optional ANSI styling so that
    lines emitted from Python tailers and interactive-console helpers line up
    with the bash-emitted launcher output.
    """

    ts = format_timestamp(now)
    active_palette = palette if palette is not None else log_palette_from_env()
    muted = active_palette.get("muted", "") or ""
    label_style = active_palette.get("label", "") or ""
    reset = active_palette.get("reset", "") or ""

    if muted or reset:
        ts_part = f"{muted}[{ts}]{reset}"
    else:
        ts_part = f"[{ts}]"

    if label_style or reset:
        label_part = f"{label_style}[{label}]{reset}"
    else:
        label_part = f"[{label}]"

    return f"{ts_part} {label_part} {message}"

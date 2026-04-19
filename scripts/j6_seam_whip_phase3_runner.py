#!/usr/bin/env python3
"""Phase 3 of the J6 seam-whip verification plan.

Executes the slow-speed seam-crossing proof sequence end-to-end against a
RUNNING stack, assuming the RTCore fast_trace drop-in is active (1 kHz
JSONL under /run/gradient-rt-motion/j6-fast-trace.jsonl).

This script is deliberately read-and-act-step-by-step so the operator can
abort between phases (Ctrl-C) if the arm behaves unexpectedly.

Workflow:
  1. Home J6 via /control/home-joint-native {"joint": 6}
  2. /control/power-up and verify probe shows armed+J6 sw=0x9637
  3. Save snapshot 'phase3-after-powerup'
  4. Pre-position J6 from canonical 0 deg -> +175 deg at max_motor_rpm=100
     (about 2.9 s, no seam crossing)
  5. Wait for idle (timeout 15 s)
  6. Save snapshot 'phase3-pre-seam-slow'
  7. SLOW SEAM CROSSING: +175 deg -> +185 deg at max_motor_rpm=1
     (about 17 s, crosses the 6064=0/RM seam at +180 deg canonical)
  8. Wait for idle (timeout 40 s)
  9. Save snapshot 'phase3-post-seam-slow'

Any HTTP error, UDP error, fault, or idle timeout aborts the sequence. The
script never calls stop --hard itself; operator decides when to stop. The
autosave hook on stop --hard will capture the full 1 kHz trace of the
entire session.

Plan reference: /home/pi/.cursor/plans/j6_seam_whip_verification_b8c230f3.plan.md Phase 3.
"""
from __future__ import annotations

import argparse
import base64
import json
import math
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

API_URL = "http://127.0.0.1:4400"
CONTROLLER_HOST = "127.0.0.1"
CONTROLLER_PORT = 3000
J6_INDEX = 5  # zero-based; target_joint_indices uses zero-based indexing
DEFAULT_SAVE_SCRIPT = Path(__file__).resolve().parent / "j6_multiturn_fast_capture.py"


def log(msg: str) -> None:
    stamp = time.strftime("%H:%M:%S", time.localtime())
    print(f"[phase3 {stamp}] {msg}", flush=True)


def http_request(method: str, path: str, *, body: dict | None = None, timeout_s: float = 5.0) -> dict:
    url = f"{API_URL}{path}"
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(url, data=data, method=method)
    if data is not None:
        req.add_header("Content-Type", "application/json")
    with urllib.request.urlopen(req, timeout=timeout_s) as resp:
        raw = resp.read().decode("utf-8")
    try:
        return json.loads(raw)
    except json.JSONDecodeError:
        return {"raw": raw}


def fetch_joint_state() -> dict:
    return http_request("GET", "/info/joints", timeout_s=3.0)


def home_j6() -> dict:
    log("Homing J6 via /control/home-joint-native {\"joint\": 6}")
    return http_request("POST", "/control/home-joint-native", body={"joint": 6}, timeout_s=45.0)


def power_up() -> dict:
    log("Powering up all axes via /control/power-up")
    return http_request("POST", "/control/power-up", timeout_s=10.0)


def wait_for_idle(timeout_s: float) -> dict:
    log(f"Waiting for idle (timeout {timeout_s:.0f} s)")
    return http_request("POST", "/control/wait-for-idle", body={"timeout_s": timeout_s},
                        timeout_s=timeout_s + 5.0)


def send_apply_joint_setpoint(arm_angles_rad: list[float], *, max_motor_rpm: float,
                              target_joint_indices: list[int]) -> dict:
    payload = {
        "arm_angles_rad": [float(x) for x in arm_angles_rad],
        "max_motor_rpm": float(max_motor_rpm),
        "target_joint_indices": [int(x) for x in target_joint_indices],
    }
    b64 = base64.urlsafe_b64encode(json.dumps(payload).encode("utf-8")).decode("ascii")
    msg = f"APPLY_JOINT_SETPOINT,{b64}".encode("ascii")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(10.0)
    try:
        sock.sendto(msg, (CONTROLLER_HOST, CONTROLLER_PORT))
        reply, _ = sock.recvfrom(65535)
    finally:
        sock.close()
    text = reply.decode("utf-8", "replace").strip()
    log(f"controller reply head: {text[:160]}")
    if text.startswith("ERROR"):
        raise RuntimeError(f"controller rejected APPLY_JOINT_SETPOINT: {text}")
    return {"raw": text}


def save_trace_snapshot(label: str) -> None:
    """Invoke the companion save subcommand. Non-fatal on failure."""
    argv = [sys.executable, str(DEFAULT_SAVE_SCRIPT), "save",
            "--if-exists", "--label", label]
    try:
        proc = subprocess.run(argv, capture_output=True, text=True, check=False)
    except Exception as exc:
        log(f"save snapshot '{label}' failed to invoke: {exc}")
        return
    if proc.stdout:
        for line in proc.stdout.splitlines():
            log(f"save/{label}: {line}")
    if proc.stderr:
        for line in proc.stderr.splitlines():
            log(f"save/{label} STDERR: {line}")


def probe_axis_state_j6() -> dict:
    """Cheap probe of J6's statusword via the API's motion-status + joints
    endpoint. Used before issuing a motion command to confirm we're OP."""
    state = fetch_joint_state()
    return state


def fetch_joint_state_with_all_joints_ready(
    *,
    timeout_s: float = 10.0,
    poll_interval_s: float = 0.25,
) -> dict:
    """Poll /info/joints until arm_rad (or arm_display_rad) has 6 non-None
    entries. The `absolute_home_anchor_stale` diagnostic can blank several
    joints for up to a few seconds after any motion + diagnostics run.
    """
    deadline = time.time() + float(timeout_s)
    last = None
    attempts = 0
    while time.time() < deadline:
        attempts += 1
        last = fetch_joint_state()
        arm_rad = last.get("arm_rad") or []
        disp_rad = last.get("arm_display_rad") or []
        ok = True
        for i in range(6):
            a = arm_rad[i] if i < len(arm_rad) else None
            d = disp_rad[i] if i < len(disp_rad) else None
            if a is None and d is None:
                ok = False
                break
        if ok:
            if attempts > 1:
                log(f"joint state settled after {attempts} polls "
                    f"(~{attempts * poll_interval_s:.2f} s)")
            return last
        time.sleep(poll_interval_s)
    raise RuntimeError(
        f"joint state never settled within {timeout_s:.1f} s (final: {last})"
    )


def build_preposition_payload_from_joints(current_joints: dict, *, j6_target_deg: float) -> list[float]:
    """Return the full 6-entry arm_angles_rad list, keeping J1..J5 at their
    current live values and J6 at `j6_target_deg` (converted to rad).

    Falls back to `arm_display_rad` when `arm_rad` is None/empty for a joint
    (the `absolute_home_anchor_stale` diagnostic blanks `arm_rad` briefly
    during and after any motion).
    """
    arm_rad = current_joints.get("arm_rad") or [None] * 6
    disp_rad = current_joints.get("arm_display_rad") or [None] * 6
    if len(arm_rad) < 6 and len(disp_rad) < 6:
        raise RuntimeError(f"neither arm_rad nor arm_display_rad usable in: {current_joints}")
    out: list[float] = []
    for i in range(6):
        v = arm_rad[i] if i < len(arm_rad) and arm_rad[i] is not None else None
        if v is None and i < len(disp_rad):
            v = disp_rad[i]
        if v is None:
            raise RuntimeError(
                f"joint {i} has no arm_rad nor arm_display_rad; joints={current_joints}"
            )
        out.append(float(v))
    out[J6_INDEX] = math.radians(j6_target_deg)
    return out


def _safe_j6_deg(joints: dict) -> str:
    """Pretty print J6 canonical, tolerating empty/None lists."""
    for key in ("arm_deg", "arm_display_deg"):
        values = joints.get(key)
        if isinstance(values, list) and len(values) > J6_INDEX:
            v = values[J6_INDEX]
            if v is not None:
                return f"{float(v):.3f} ({key})"
    return "unavailable"


def _is_already_armed_and_homed() -> bool:
    """Quick probe of /info/runtime-config + motion-status doesn't return
    armed/homing info cleanly; use raw UDP STATUS instead."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(2.0)
        sock.sendto(b"STATUS", (CONTROLLER_HOST, CONTROLLER_PORT))
        reply, _ = sock.recvfrom(65535)
        sock.close()
    except Exception:
        return False
    text = reply.decode("utf-8", "replace")
    # STATUS reply includes "armed=..." or similar; simpler: ask probe via motion-status
    return "armed=1" in text or "armed:1" in text or "arm=1" in text


def run(args: argparse.Namespace) -> int:
    log("Phase 3 runner starting. Operator can Ctrl-C any time to abort.")

    if args.skip_home_power_up:
        log("--skip-home-power-up set; assuming stack is already armed+homed.")
    else:
        # 1. Home J6.
        try:
            reply = home_j6()
        except urllib.error.HTTPError as exc:
            log(f"home-joint-native HTTP {exc.code}: {exc.read().decode('utf-8', 'replace')}")
            return 1
        log(f"home reply: status={reply.get('status')} "
            f"disarmed_after_home={reply.get('disarmed_after_home')}")

        # 2. Power up.
        try:
            reply = power_up()
        except urllib.error.HTTPError as exc:
            log(f"power-up HTTP {exc.code}: {exc.read().decode('utf-8', 'replace')}")
            return 2
        log(f"power-up reply status: {reply.get('status')}")
        # Small settle to let the drive land in OperationEnabled and for the
        # first /info/joints fetch to populate arm_deg.
        time.sleep(1.5)

    # The `absolute_home_anchor_stale` diagnostic blanks several arm_rad /
    # arm_display_rad entries for seconds after power-up or any motion.
    # Poll until a clean 6-joint snapshot is available before building
    # any motion payload.
    try:
        joints = fetch_joint_state_with_all_joints_ready(timeout_s=15.0)
    except RuntimeError as exc:
        log(f"could not get clean joint state: {exc}")
        return 10
    log(f"J6 canonical before preposition: {_safe_j6_deg(joints)}")
    # Cache the pre-motion joint state for J1..J5. Those joints will not move
    # during this sequence (target_joint_indices=[5]), so their pre-motion
    # values stay valid even while the diagnostic blanks post-motion
    # snapshots. For J6 we always overwrite the target anyway.
    cached_joints = joints
    save_trace_snapshot(f"phase3-after-powerup-{args.label_suffix}"
                        if args.label_suffix != "slow" else "phase3-after-powerup")

    suffix = args.label_suffix
    pre_rpm = float(args.preposition_max_motor_rpm)
    seam_rpm = float(args.seam_max_motor_rpm)
    pre_deg = float(args.preposition_deg)
    seam_deg = float(args.seam_target_deg)
    # Expected motion duration for the J6 A6-EC drive:
    #   output_rpm = motor_rpm / gear_ratio
    #   output_deg_per_s = output_rpm * 360 / 60 = output_rpm * 6
    #   time_s = |delta_deg_output| / output_deg_per_s
    #         = |delta_deg_output| * gear_ratio / (motor_rpm * 6)
    gear_ratio = 10.0  # J6
    j6_live_deg = None
    for key in ("arm_deg", "arm_display_deg"):
        values = cached_joints.get(key)
        if isinstance(values, list) and len(values) > J6_INDEX and values[J6_INDEX] is not None:
            j6_live_deg = float(values[J6_INDEX])
            break
    expected_pre_s = (
        abs(pre_deg - (j6_live_deg or 0.0)) * gear_ratio / (pre_rpm * 6.0)
        if pre_rpm > 0
        else 15.0
    )
    expected_seam_s = (
        abs(seam_deg - pre_deg) * gear_ratio / (seam_rpm * 6.0)
        if seam_rpm > 0
        else 30.0
    )
    pre_idle_timeout = max(10.0, 3.0 * expected_pre_s + 5.0)
    seam_idle_timeout = (
        float(args.seam_idle_timeout_s) if args.seam_idle_timeout_s > 0
        else max(10.0, 3.0 * expected_seam_s + 5.0)
    )
    log(f"plan: preposition J6 -> {pre_deg:+.2f} deg at {pre_rpm:.1f} motor RPM "
        f"(expected ~{expected_pre_s:.1f} s, idle timeout {pre_idle_timeout:.0f} s)")
    log(f"plan: seam cross J6 -> {seam_deg:+.2f} deg at {seam_rpm:.1f} motor RPM "
        f"(expected ~{expected_seam_s:.1f} s, idle timeout {seam_idle_timeout:.0f} s)")

    # 3. Pre-position.
    try:
        payload = build_preposition_payload_from_joints(cached_joints, j6_target_deg=pre_deg)
    except RuntimeError as exc:
        log(f"could not build preposition payload: {exc}")
        return 3
    log(f"Sending pre-position: J6 -> {pre_deg:+.2f} deg at max_motor_rpm={pre_rpm:.1f}")
    try:
        send_apply_joint_setpoint(payload, max_motor_rpm=pre_rpm, target_joint_indices=[J6_INDEX])
    except Exception as exc:
        log(f"preposition APPLY_JOINT_SETPOINT failed: {exc}")
        return 4
    try:
        idle = wait_for_idle(timeout_s=pre_idle_timeout)
    except urllib.error.HTTPError as exc:
        log(f"wait-for-idle HTTP {exc.code}: {exc.read().decode('utf-8', 'replace')}")
        return 5
    state = idle.get("state") or idle.get("status")
    log(f"preposition idle state: {state}")
    if state not in {"completed", "ok"}:
        log(f"preposition did not reach completed state: {idle}")
        return 6

    # Longer settle: the host's multi_turn_anchor_inconsistent_with_live_6064
    # gate has a 16-count tolerance and after a 175 deg pre-position at 100
    # motor RPM the live 6064 can take a few seconds to settle within that
    # tolerance of the anchor-implied expected value. A 1 s settle is not
    # enough; 5 s works in practice.
    log("Settling 5 s for multi-turn anchor consistency to re-converge")
    time.sleep(5.0)
    joints_after_pre = fetch_joint_state()
    log(f"J6 canonical after preposition: {_safe_j6_deg(joints_after_pre)}")
    save_trace_snapshot(f"phase3-pre-seam-{suffix}")

    # 4. THE CRITICAL SEAM CROSSING: preposition -> seam target at configured RPM.
    try:
        payload = build_preposition_payload_from_joints(cached_joints, j6_target_deg=seam_deg)
    except RuntimeError as exc:
        log(f"could not build seam-cross payload: {exc}")
        return 7
    log("--- SEAM CROSSING ---")
    log(f"Sending seam cross: J6 -> {seam_deg:+.2f} deg at max_motor_rpm={seam_rpm:.1f} "
        f"(expected ~{expected_seam_s:.1f} s)")
    log("Watch J6 physically. If it whips, Ctrl-C immediately and run stop --hard.")
    t0 = time.time()
    try:
        send_apply_joint_setpoint(payload, max_motor_rpm=seam_rpm, target_joint_indices=[J6_INDEX])
    except Exception as exc:
        log(f"seam-cross APPLY_JOINT_SETPOINT failed: {exc}")
        return 8
    try:
        idle = wait_for_idle(timeout_s=seam_idle_timeout)
    except urllib.error.HTTPError as exc:
        log(f"wait-for-idle HTTP {exc.code}: {exc.read().decode('utf-8', 'replace')}")
        return 9
    elapsed = time.time() - t0
    state = idle.get("state") or idle.get("status")
    log(f"seam cross idle state: {state} (elapsed {elapsed:.1f} s)")

    # 5. Final state + snapshot.
    joints_final = fetch_joint_state()
    log(f"J6 canonical after seam: {_safe_j6_deg(joints_final)}")
    save_trace_snapshot(f"phase3-post-seam-{suffix}")

    log("Phase 3 motion sequence complete. Run `./start-stack.sh stop --hard` to preserve "
        "the full trace via the autosave hook, then analyze with analyze-rtcore.")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--skip-home-power-up",
        action="store_true",
        help="Skip the home+power-up steps (assume the stack is already armed+homed).",
    )
    parser.add_argument(
        "--preposition-deg", type=float, default=+175.0,
        help="Canonical J6 angle for the pre-position move (default +175.0).",
    )
    parser.add_argument(
        "--seam-target-deg", type=float, default=+185.0,
        help="Canonical J6 angle for the seam-crossing move (default +185.0).",
    )
    parser.add_argument(
        "--preposition-max-motor-rpm", type=float, default=100.0,
        help="max_motor_rpm for the pre-position move (default 100).",
    )
    parser.add_argument(
        "--seam-max-motor-rpm", type=float, default=1.0,
        help="max_motor_rpm for the seam-crossing move (default 1 = slow).",
    )
    parser.add_argument(
        "--label-suffix", default="slow",
        help="Suffix appended to the snapshot labels (default 'slow'). Use e.g. '10rpm-positive' "
             "or '1rpm-negative' to disambiguate repeat runs.",
    )
    parser.add_argument(
        "--seam-idle-timeout-s", type=float, default=0.0,
        help="Override the wait-for-idle timeout after the seam move (default 0 = auto-derive "
             "from 3 * expected_duration + 5).",
    )
    args = parser.parse_args()
    try:
        return run(args)
    except KeyboardInterrupt:
        log("ABORTED by operator (Ctrl-C).")
        return 130


if __name__ == "__main__":
    raise SystemExit(main())

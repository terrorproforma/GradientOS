from __future__ import annotations

import argparse
import datetime as dt
import json
import os
from pathlib import Path
import platform
import re
import shutil
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request
from typing import Any

_REPO_ROOT = Path(__file__).resolve().parents[3]
_DEFAULT_OUTPUT_DIR = _REPO_ROOT / "logs" / "diagnostics"
_KERNEL_HINT_PATTERNS = (
    re.compile(r"out of memory", re.IGNORECASE),
    re.compile(r"oom[-_ ]killer", re.IGNORECASE),
    re.compile(r"oom_reaper", re.IGNORECASE),
    re.compile(r"killed process", re.IGNORECASE),
    re.compile(r"under-?voltage", re.IGNORECASE),
    re.compile(r"throttl", re.IGNORECASE),
    re.compile(r"\bv3d\b", re.IGNORECASE),
    re.compile(r"\bvc4\b", re.IGNORECASE),
    re.compile(r"\bgpu\b", re.IGNORECASE),
    re.compile(r"\bhang(?:ed|ing)?\b", re.IGNORECASE),
)
_PROCESS_GROUPS: dict[str, tuple[str, ...]] = {
    "browser": ("chromium", "chromium-browser", "google-chrome"),
    "ide": ("cursor-server", ".cursor-server", "/cursor "),
    "web": ("vite", "run-web.sh", "web-ui"),
    "api": ("run-api.sh", "gradient_os.api.main", "uvicorn"),
    "controller": ("run.sh", "run-sim.sh", "gradient_os.run_controller"),
    "stack": ("start-stack.sh",),
}
_THROTTLED_FLAG_BITS = {
    0: "under_voltage_now",
    1: "arm_frequency_capped_now",
    2: "throttled_now",
    3: "soft_temperature_limit_now",
    16: "under_voltage_occurred",
    17: "arm_frequency_capped_occurred",
    18: "throttled_occurred",
    19: "soft_temperature_limit_occurred",
}


def _read_text(path: Path) -> str | None:
    try:
        return path.read_text(encoding="utf-8").strip()
    except Exception:
        return None


def _read_meminfo() -> dict[str, int]:
    raw = _read_text(Path("/proc/meminfo"))
    if not raw:
        return {}
    result: dict[str, int] = {}
    for line in raw.splitlines():
        key, _, value = line.partition(":")
        parts = value.strip().split()
        if not parts:
            continue
        try:
            result[key] = int(parts[0]) * 1024
        except ValueError:
            continue
    return result


def _read_uptime_seconds() -> float | None:
    raw = _read_text(Path("/proc/uptime"))
    if not raw:
        return None
    try:
        return float(raw.split()[0])
    except (IndexError, ValueError):
        return None


def _read_proc_status(pid: int) -> dict[str, str]:
    raw = _read_text(Path("/proc") / str(pid) / "status")
    if not raw:
        return {}
    result: dict[str, str] = {}
    for line in raw.splitlines():
        key, _, value = line.partition(":")
        result[key.strip()] = value.strip()
    return result


def _read_cmdline(pid: int) -> str:
    try:
        raw = (Path("/proc") / str(pid) / "cmdline").read_bytes()
    except Exception:
        return ""
    if not raw:
        return ""
    return raw.replace(b"\x00", b" ").decode("utf-8", "replace").strip()


def _parse_status_bytes(value: str | None) -> int | None:
    if not value:
        return None
    token = value.split()[0]
    try:
        return int(token) * 1024
    except ValueError:
        return None


def _safe_int(text: str | None) -> int | None:
    if text is None:
        return None
    try:
        return int(text)
    except (TypeError, ValueError):
        return None


def _run_command(command: list[str], timeout_s: float = 2.0) -> dict[str, Any]:
    started = time.perf_counter()
    try:
        completed = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=timeout_s,
            check=False,
        )
    except Exception as exc:
        return {
            "ok": False,
            "error": str(exc),
            "elapsed_ms": round((time.perf_counter() - started) * 1000.0, 2),
            "command": command,
        }
    return {
        "ok": completed.returncode == 0,
        "returncode": completed.returncode,
        "stdout": completed.stdout.strip(),
        "stderr": completed.stderr.strip(),
        "elapsed_ms": round((time.perf_counter() - started) * 1000.0, 2),
        "command": command,
    }


def _collect_host_summary(project_root: Path) -> dict[str, Any]:
    loadavg: tuple[float, float, float] | None
    try:
        loadavg = os.getloadavg()
    except OSError:
        loadavg = None
    return {
        "hostname": socket.gethostname(),
        "platform": platform.platform(),
        "python": sys.version.split()[0],
        "pid": os.getpid(),
        "cwd": os.getcwd(),
        "project_root": str(project_root),
        "boot_id": _read_text(Path("/proc/sys/kernel/random/boot_id")),
        "uptime_s": _read_uptime_seconds(),
        "loadavg": {
            "1m": loadavg[0] if loadavg else None,
            "5m": loadavg[1] if loadavg else None,
            "15m": loadavg[2] if loadavg else None,
        },
    }


def _collect_memory_summary() -> dict[str, Any]:
    meminfo = _read_meminfo()
    mem_total = meminfo.get("MemTotal")
    mem_available = meminfo.get("MemAvailable")
    swap_total = meminfo.get("SwapTotal")
    swap_free = meminfo.get("SwapFree")
    mem_used = (
        mem_total - mem_available if mem_total is not None and mem_available is not None else None
    )
    swap_used = swap_total - swap_free if swap_total is not None and swap_free is not None else None
    return {
        "memory": {
            "total_bytes": mem_total,
            "available_bytes": mem_available,
            "used_bytes": mem_used,
            "used_percent": (round((mem_used / mem_total) * 100.0, 2) if mem_used is not None and mem_total else None),
        },
        "swap": {
            "total_bytes": swap_total,
            "free_bytes": swap_free,
            "used_bytes": swap_used,
            "used_percent": (round((swap_used / swap_total) * 100.0, 2) if swap_used is not None and swap_total else None),
        },
    }


def _classify_process(command: str, name: str) -> str | None:
    haystack = f"{name} {command}".lower()
    for group, tokens in _PROCESS_GROUPS.items():
        if any(token in haystack for token in tokens):
            return group
    return None


def _summarize_process(pid: int) -> dict[str, Any] | None:
    status = _read_proc_status(pid)
    if not status:
        return None
    command = _read_cmdline(pid)
    name = status.get("Name", "")
    return {
        "pid": pid,
        "name": name,
        "state": status.get("State"),
        "ppid": _safe_int(status.get("PPid")),
        "threads": _safe_int(status.get("Threads")),
        "vm_rss_bytes": _parse_status_bytes(status.get("VmRSS")),
        "vm_size_bytes": _parse_status_bytes(status.get("VmSize")),
        "oom_score": _safe_int(_read_text(Path("/proc") / str(pid) / "oom_score")),
        "group": _classify_process(command, name),
        "command": command,
    }


def _collect_process_summary() -> dict[str, Any]:
    current = _summarize_process(os.getpid())
    interesting: dict[str, list[dict[str, Any]]] = {group: [] for group in _PROCESS_GROUPS}
    top_rss: list[dict[str, Any]] = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        proc = _summarize_process(int(entry.name))
        if not proc:
            continue
        if proc["group"]:
            interesting[proc["group"]].append(proc)
        top_rss.append(proc)
    for group in interesting:
        interesting[group] = sorted(
            interesting[group],
            key=lambda item: item.get("vm_rss_bytes") or 0,
            reverse=True,
        )[:8]
    top_rss_sorted = sorted(top_rss, key=lambda item: item.get("vm_rss_bytes") or 0, reverse=True)[:10]
    return {
        "current_process": current,
        "interesting": interesting,
        "top_rss": top_rss_sorted,
    }


def _collect_disk_summary(project_root: Path) -> dict[str, Any]:
    root_usage = shutil.disk_usage(project_root)
    tmp_usage = shutil.disk_usage(Path("/tmp"))
    return {
        "project_root": {
            "path": str(project_root),
            "total_bytes": root_usage.total,
            "used_bytes": root_usage.used,
            "free_bytes": root_usage.free,
        },
        "tmp": {
            "path": "/tmp",
            "total_bytes": tmp_usage.total,
            "used_bytes": tmp_usage.used,
            "free_bytes": tmp_usage.free,
        },
    }


def _parse_vcgencmd_throttled(output: str) -> dict[str, Any]:
    token = output.strip().split("=", 1)
    raw_value = token[1] if len(token) == 2 else output.strip()
    try:
        value = int(raw_value, 16)
    except ValueError:
        return {"raw": output.strip(), "parsed": None, "flags": []}
    flags = [name for bit, name in _THROTTLED_FLAG_BITS.items() if value & (1 << bit)]
    return {"raw": output.strip(), "parsed": value, "flags": flags}


def _collect_raspberry_pi_signals() -> dict[str, Any]:
    thermal_raw = _read_text(Path("/sys/class/thermal/thermal_zone0/temp"))
    thermal_c = None
    if thermal_raw:
        try:
            thermal_c = round(int(thermal_raw) / 1000.0, 2)
        except ValueError:
            thermal_c = None
    vcgencmd_temp = _run_command(["vcgencmd", "measure_temp"])
    vcgencmd_throttled = _run_command(["vcgencmd", "get_throttled"])
    result: dict[str, Any] = {
        "thermal_zone0_c": thermal_c,
        "vcgencmd_available": vcgencmd_temp.get("ok") or vcgencmd_throttled.get("ok"),
    }
    if vcgencmd_temp.get("ok"):
        result["vcgencmd_temp"] = vcgencmd_temp["stdout"]
    elif vcgencmd_temp.get("error") or vcgencmd_temp.get("stderr"):
        result["vcgencmd_temp_error"] = vcgencmd_temp.get("error") or vcgencmd_temp.get("stderr")
    if vcgencmd_throttled.get("ok"):
        result["throttled"] = _parse_vcgencmd_throttled(str(vcgencmd_throttled["stdout"]))
    elif vcgencmd_throttled.get("error") or vcgencmd_throttled.get("stderr"):
        result["throttled_error"] = vcgencmd_throttled.get("error") or vcgencmd_throttled.get("stderr")
    return result


def _tail_lines(path: Path, limit: int = 20) -> list[str]:
    try:
        with path.open("rb") as fh:
            fh.seek(0, os.SEEK_END)
            size = fh.tell()
            chunk = min(size, 16384)
            fh.seek(max(0, size - chunk))
            data = fh.read().decode("utf-8", "replace")
    except Exception:
        return []
    lines = [line.rstrip() for line in data.splitlines() if line.strip()]
    return lines[-limit:]


def _collect_latest_startup_logs(project_root: Path) -> dict[str, Any]:
    latest_path = project_root / "logs" / "startups" / "latest"
    resolved = latest_path.resolve() if latest_path.exists() else None
    payload: dict[str, Any] = {
        "latest_path": str(resolved) if resolved else None,
        "logs": {},
    }
    if not resolved or not resolved.exists():
        payload["error"] = "No latest startup log directory found."
        return payload
    for name in ("launcher.log", "api.log", "web.log", "controller.log"):
        log_path = resolved / name
        if log_path.exists():
            payload["logs"][name] = _tail_lines(log_path)
    return payload


def _collect_kernel_hints() -> dict[str, Any]:
    result = _run_command(["journalctl", "-k", "-n", "200", "--no-pager", "--output=short-iso"], timeout_s=3.0)
    if not result.get("ok"):
        return {
            "source": "journalctl -k",
            "error": result.get("error") or result.get("stderr") or "journalctl failed",
            "matches": [],
        }
    stdout = str(result.get("stdout") or "")
    matches = [
        line
        for line in stdout.splitlines()
        if any(pattern.search(line) for pattern in _KERNEL_HINT_PATTERNS)
    ]
    return {
        "source": "journalctl -k",
        "matches": matches[-25:],
    }


def _probe_http(url: str, timeout_s: float = 1.5) -> dict[str, Any]:
    started = time.perf_counter()
    try:
        with urllib.request.urlopen(url, timeout=timeout_s) as response:
            body_preview = response.read(160).decode("utf-8", "replace")
            return {
                "ok": 200 <= response.status < 400,
                "status": response.status,
                "elapsed_ms": round((time.perf_counter() - started) * 1000.0, 2),
                "body_preview": body_preview,
            }
    except urllib.error.HTTPError as exc:
        return {
            "ok": False,
            "status": exc.code,
            "elapsed_ms": round((time.perf_counter() - started) * 1000.0, 2),
            "error": str(exc),
        }
    except Exception as exc:
        return {
            "ok": False,
            "status": None,
            "elapsed_ms": round((time.perf_counter() - started) * 1000.0, 2),
            "error": str(exc),
        }


def _probe_controller_udp(timeout_s: float = 0.5) -> dict[str, Any]:
    started = time.perf_counter()
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.settimeout(timeout_s)
            sock.sendto(b"GET_STATUS", ("127.0.0.1", 3000))
            data, _ = sock.recvfrom(4096)
        return {
            "ok": True,
            "elapsed_ms": round((time.perf_counter() - started) * 1000.0, 2),
            "reply_preview": data.decode("utf-8", "replace")[:160],
        }
    except Exception as exc:
        return {
            "ok": False,
            "elapsed_ms": round((time.perf_counter() - started) * 1000.0, 2),
            "error": str(exc),
        }


def _collect_local_probes() -> dict[str, Any]:
    return {
        "web_root": _probe_http("http://127.0.0.1:8000/"),
        "web_entry": _probe_http("http://127.0.0.1:8000/src/main.tsx"),
        "controller_udp_status": _probe_controller_udp(),
    }


def get_runtime_diagnostics_snapshot(
    project_root: str | os.PathLike[str] | None = None,
    *,
    include_local_probes: bool = False,
) -> dict[str, Any]:
    resolved_root = Path(project_root).resolve() if project_root else _REPO_ROOT
    snapshot = {
        "collected_at": dt.datetime.now(dt.timezone.utc).isoformat(),
        "host": _collect_host_summary(resolved_root),
        "resources": _collect_memory_summary(),
        "disk": _collect_disk_summary(resolved_root),
        "processes": _collect_process_summary(),
        "raspberry_pi": _collect_raspberry_pi_signals(),
        "kernel_hints": _collect_kernel_hints(),
        "latest_startup_logs": _collect_latest_startup_logs(resolved_root),
    }
    if include_local_probes:
        snapshot["probes"] = _collect_local_probes()
    return snapshot


def _default_output_path(project_root: Path) -> Path:
    timestamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%d-%H%M%S")
    return project_root / "logs" / "diagnostics" / f"{timestamp}-runtime.json"


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Capture a local GradientOS runtime diagnostics snapshot")
    parser.add_argument(
        "--project-root",
        default=str(_REPO_ROOT),
        help="Repository root used for log discovery and default output location.",
    )
    parser.add_argument(
        "--output",
        default="",
        help="Optional JSON output path. Defaults to logs/diagnostics/<timestamp>-runtime.json.",
    )
    parser.add_argument(
        "--stdout",
        action="store_true",
        help="Also print the collected JSON to stdout.",
    )
    parser.add_argument(
        "--skip-probes",
        action="store_true",
        help="Skip local web/controller probe requests.",
    )
    args = parser.parse_args(argv)

    project_root = Path(args.project_root).resolve()
    payload = get_runtime_diagnostics_snapshot(
        project_root,
        include_local_probes=not args.skip_probes,
    )
    output_path = Path(args.output).resolve() if args.output else _default_output_path(project_root)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    serialized = json.dumps(payload, indent=2, sort_keys=True)
    output_path.write_text(serialized + "\n", encoding="utf-8")
    if args.stdout:
        print(serialized)
    else:
        print(output_path)


if __name__ == "__main__":
    main()

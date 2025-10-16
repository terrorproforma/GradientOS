from __future__ import annotations

import asyncio
import json
import logging
import os
import socket
from contextlib import closing, asynccontextmanager
from typing import Any, Dict, Tuple

import argparse

from fastapi import FastAPI, HTTPException
from fastapi.concurrency import run_in_threadpool
from fastapi.middleware.cors import CORSMiddleware
from sse_starlette.sse import EventSourceResponse
import numpy as np

try:
    from ..arm_controller import utils as controller_utils
except ImportError:
    controller_utils = None


def _default_controller_port() -> int:
    if controller_utils is not None:
        return int(getattr(controller_utils, "UDP_PORT", 3000))
    return 3000


def _resolve_controller_endpoint() -> Tuple[str, int]:
    host = os.environ.get("GRADIENT_CONTROLLER_HOST", "127.0.0.1")
    port = int(os.environ.get("GRADIENT_CONTROLLER_PORT", _default_controller_port()))
    return host, port


def _probe_controller(timeout: float = 0.5) -> Tuple[bool, str]:
    host, port = _resolve_controller_endpoint()
    payload = b"GET_STATUS"
    with closing(socket.socket(socket.AF_INET, socket.SOCK_DGRAM)) as sock:
        sock.settimeout(max(0.05, timeout))
        try:
            sock.sendto(payload, (host, port))
            data, _addr = sock.recvfrom(1024)
        except socket.timeout:
            return False, f"Timed out waiting for controller response at {host}:{port}"
        except OSError as exc:
            return (
                False,
                f"Socket error connecting to controller at {host}:{port}: {exc}",
            )

    text = data.decode("utf-8", errors="ignore")
    if text.startswith("STATUS"):
        return True, f"Controller reachable at {host}:{port}"
    return False, f"Unexpected controller response '{text}' from {host}:{port}"


def _send_controller_command(
    message: str, timeout: float = 0.5, expect_response: bool = True
) -> Tuple[bool, str]:
    host, port = _resolve_controller_endpoint()
    with closing(socket.socket(socket.AF_INET, socket.SOCK_DGRAM)) as sock:
        sock.settimeout(max(0.05, timeout))
        try:
            sock.sendto(message.encode("utf-8"), (host, port))
        except OSError as exc:
            return False, f"Socket error sending '{message}': {exc}"
        else:
            if not expect_response:
                return True, ""
        try:
            data, _addr = sock.recvfrom(1024)
        except socket.timeout:
            return False, f"No response for command '{message}'"
    text = data.decode("utf-8", errors="ignore")
    if text.startswith("ERROR"):
        return False, text
    return True, text


def _resolve_cors_origins() -> list[str]:
    raw = os.environ.get("GRADIENT_API_CORS", "")
    if not raw:
        return ["*"]
    origins = [item.strip() for item in raw.split(",")]
    return [origin for origin in origins if origin]


class _TelemetryProtocol(asyncio.DatagramProtocol):
    def __init__(self, hub: "TelemetryHub") -> None:
        self.hub = hub

    def datagram_received(self, data: bytes, addr) -> None:  # type: ignore[override]
        self.hub.handle_datagram(data, addr)


class TelemetryHub:
    def __init__(self) -> None:
        self._subscribers: Dict[int, asyncio.Queue[str]] = {}
        self._counter = 0
        self._lock = asyncio.Lock()
        self._transport: asyncio.DatagramTransport | None = None
        self._advertise_host = os.environ.get("GRADIENT_MONITOR_HOST")
        self._bind_host = os.environ.get("GRADIENT_MONITOR_BIND", "127.0.0.1")
        self._listen_port: int | None = None

    async def register(self) -> Tuple[int, asyncio.Queue[str]]:
        queue: asyncio.Queue[str] = asyncio.Queue(maxsize=50)
        async with self._lock:
            first_client = not self._subscribers
            if first_client:
                await self._start()
            self._counter += 1
            token = self._counter
            self._subscribers[token] = queue
        return token, queue

    async def unregister(self, token: int) -> None:
        async with self._lock:
            self._subscribers.pop(token, None)
            if not self._subscribers:
                await self._stop()

    async def _start(self) -> None:
        loop = asyncio.get_running_loop()
        transport, _protocol = await loop.create_datagram_endpoint(
            lambda: _TelemetryProtocol(self),
            local_addr=(self._bind_host, 0),
        )
        self._transport = transport
        sockname = transport.get_extra_info("sockname")
        assert sockname is not None
        listen_host = self._advertise_host or _resolve_controller_endpoint()[0]
        self._listen_port = sockname[1]
        start_cmd = f"START_TELEMETRY,{listen_host}:{self._listen_port},10"
        ok, detail = await run_in_threadpool(_send_controller_command, start_cmd)
        if not ok:
            await self._cleanup_transport()
            raise HTTPException(status_code=503, detail=detail)

    async def _stop(self) -> None:
        if self._listen_port is None:
            return
        stop_cmd = "STOP_TELEMETRY"
        await run_in_threadpool(_send_controller_command, stop_cmd)
        await self._cleanup_transport()

    async def _cleanup_transport(self) -> None:
        if self._transport is not None:
            self._transport.close()
        self._transport = None
        self._listen_port = None

    def handle_datagram(self, data: bytes, addr) -> None:
        try:
            text = data.decode("utf-8")
        except UnicodeDecodeError:
            return
        event_payload = self._format_event(text)
        for queue in list(self._subscribers.values()):
            try:
                queue.put_nowait(event_payload)
            except asyncio.QueueFull:
                try:
                    _ = queue.get_nowait()
                except asyncio.QueueEmpty:
                    pass
                try:
                    queue.put_nowait(event_payload)
                except asyncio.QueueFull:
                    # If still full, skip this subscriber to avoid blocking.
                    continue

    def _format_event(self, text: str) -> str:
        try:
            parsed = json.loads(text)
        except json.JSONDecodeError:
            return text
        return json.dumps(parsed)


telemetry_hub = TelemetryHub()
logger = logging.getLogger("uvicorn.error")
_latest_plan_lock = asyncio.Lock()
_latest_plan: dict[str, Any] | None = None


def create_app() -> FastAPI:
    @asynccontextmanager
    async def lifespan(app: FastAPI):
        ok, detail = await run_in_threadpool(_probe_controller)
        host, port = _resolve_controller_endpoint()
        if ok:
            logger.info("Controller: %s:%s", host, port)
        else:
            logger.warning("Controller: %s:%s (%s)", host, port, detail)
        yield

    api = FastAPI(title="GradientOS API", version="0.1.0", lifespan=lifespan)
    origins = _resolve_cors_origins()
    allow_credentials = "*" not in origins
    api.add_middleware(
        CORSMiddleware,
        allow_origins=origins,
        allow_methods=["*"],
        allow_headers=["*"],
        allow_credentials=allow_credentials,
    )

    @api.get("/health", summary="Controller health probe")
    async def health():
        ok, detail = await run_in_threadpool(_probe_controller)
        if not ok:
            raise HTTPException(status_code=503, detail=detail)
        host, port = _resolve_controller_endpoint()
        return {
            "status": "ok",
            "detail": detail,
            "controller": {"host": host, "port": port},
        }

    def _controller_call_or_503(
        command: str, *, timeout: float = 0.5, expect_response: bool = True
    ) -> str:
        ok, detail = _send_controller_command(
            command, timeout=timeout, expect_response=expect_response
        )
        if not ok:
            raise HTTPException(status_code=503, detail=detail)
        return detail

    def _parse_bool_token(token: str) -> bool:
        return token.strip().lower() in {"1", "true", "yes", "on"}

    @api.post("/control/stop", summary="Emergency stop")
    async def control_stop():
        detail = await run_in_threadpool(
            _controller_call_or_503, "STOP", timeout=1.0, expect_response=True
        )
        return {"status": "ok", "detail": detail}

    @api.post("/control/wait-for-idle", summary="Block until motion completes")
    async def control_wait_for_idle():
        detail = await run_in_threadpool(
            _controller_call_or_503, "WAIT_FOR_IDLE", timeout=60.0, expect_response=True
        )
        return {"status": "ok", "detail": detail}

    @api.post("/control/home", summary="Move all joints to zero position")
    async def control_home():
        await run_in_threadpool(
            _controller_call_or_503, "0,0,0,0,0,0", timeout=2.0, expect_response=False
        )
        return {"status": "ok"}

    @api.get("/info/status", summary="Controller status snapshot")
    async def info_status():
        detail = await run_in_threadpool(_controller_call_or_503, "GET_STATUS")
        parts = detail.split(",")
        if len(parts) != 3 or parts[0] != "STATUS":
            raise HTTPException(status_code=502, detail=f"Malformed status reply: {detail}")
        return {"gripper_present": _parse_bool_token(parts[2])}

    @api.get("/info/pose", summary="Current tool pose and joint angles")
    async def info_pose():
        detail = await run_in_threadpool(_controller_call_or_503, "GET_POSITION", timeout=1.0)
        parts = detail.split(",")
        if len(parts) < 1 or parts[0] != "CURRENT_POSE" or len(parts) < 1 + 3 + 3:
            raise HTTPException(status_code=502, detail=f"Malformed pose reply: {detail}")
        try:
            pos = list(map(float, parts[1:4]))
            orient = list(map(float, parts[4:7]))
            joint_vals = list(map(float, parts[7:]))
        except ValueError as exc:
            raise HTTPException(status_code=502, detail=f"Invalid pose data: {exc}") from exc
        return {
            "position_m": {"x": pos[0], "y": pos[1], "z": pos[2]},
            "orientation_euler_deg": {"roll": orient[0], "pitch": orient[1], "yaw": orient[2]},
            "joints_deg": joint_vals,
        }

    @api.get("/info/orientation", summary="Current end-effector orientation matrix")
    async def info_orientation():
        detail = await run_in_threadpool(_controller_call_or_503, "GET_ORIENTATION", timeout=1.0)
        parts = detail.split(",")
        if not parts or parts[0] != "CURRENT_ORIENTATION" or len(parts) != 10:
            raise HTTPException(status_code=502, detail=f"Malformed orientation reply: {detail}")
        try:
            matrix_values = list(map(float, parts[1:]))
        except ValueError as exc:
            raise HTTPException(status_code=502, detail=f"Invalid orientation data: {exc}") from exc
        return {
            "matrix": [
                matrix_values[0:3],
                matrix_values[3:6],
                matrix_values[6:9],
            ]
        }

    @api.get("/info/joints", summary="Current joint angles")
    async def info_joints():
        detail = await run_in_threadpool(_controller_call_or_503, "GET_JOINT_ANGLES")
        parts = detail.split(",")
        if not parts or parts[0] != "JOINT_ANGLES":
            raise HTTPException(status_code=502, detail=f"Malformed joint reply: {detail}")
        try:
            angles = list(map(float, parts[1:]))
        except ValueError as exc:
            raise HTTPException(status_code=502, detail=f"Invalid joint data: {exc}") from exc
        arm = angles[:6]
        gripper = angles[6] if len(angles) > 6 else None
        payload = {"arm_deg": arm}
        if gripper is not None:
            payload["gripper_deg"] = gripper
        return payload

    @api.get("/info/gripper", summary="Gripper angle snapshot")
    async def info_gripper():
        detail = await run_in_threadpool(_controller_call_or_503, "GET_GRIPPER_STATE")
        parts = detail.split(",")
        if len(parts) != 3 or parts[0] != "GRIPPER_STATE":
            raise HTTPException(status_code=502, detail=f"Malformed gripper reply: {detail}")
        try:
            angle_deg = float(parts[1])
            raw = int(float(parts[2]))
        except ValueError as exc:
            raise HTTPException(status_code=502, detail=f"Invalid gripper data: {exc}") from exc
        return {"angle_deg": angle_deg, "raw_position": raw}

    @api.get("/info/all-positions", summary="Raw servo positions")
    async def info_all_positions():
        detail = await run_in_threadpool(
            _controller_call_or_503, "GET_ALL_POSITIONS", timeout=1.0
        )
        parts = detail.split(",")
        if not parts or parts[0] != "ALL_POS_DATA" or len(parts) < 3:
            raise HTTPException(status_code=502, detail=f"Malformed positions reply: {detail}")
        if (len(parts) - 1) % 2 != 0:
            raise HTTPException(status_code=502, detail=f"Unexpected positions payload: {detail}")
        payload = []
        for i in range(1, len(parts), 2):
            servo_id = parts[i]
            position = parts[i + 1]
            try:
                servo_id_int = int(servo_id)
            except ValueError:
                servo_id_int = servo_id  # fall back to raw string if malformed
            try:
                position_int: int | None = None if position == "FAIL" else int(position)
            except ValueError:
                position_int = None
            payload.append({"servo_id": servo_id_int, "raw_position": position_int})
        return {"servos": payload}

    @api.get("/monitor", summary="Subscribe to controller telemetry stream")
    async def monitor():
        token, queue = await telemetry_hub.register()

        async def event_generator():
            try:
                while True:
                    message = await queue.get()
                    yield message
            except asyncio.CancelledError:
                raise
            finally:
                await telemetry_hub.unregister(token)

        return EventSourceResponse(event_generator(), ping=15)

    @api.post("/trajectory/plan", summary="Begin recording a new trajectory")
    async def trajectory_plan():
        await run_in_threadpool(
            _controller_call_or_503,
            "PLAN_TRAJECTORY",
            timeout=1.0,
            expect_response=False,
        )
        return {"status": "ok"}

    @api.post("/trajectory/record", summary="Record current pose into active trajectory")
    async def trajectory_record():
        await run_in_threadpool(
            _controller_call_or_503, "REC_POS", timeout=1.0, expect_response=False
        )
        return {"status": "ok"}

    @api.post("/trajectory/end", summary="Finish trajectory recording and save by name")
    async def trajectory_end(payload: dict):
        name = (payload or {}).get("name")
        if not isinstance(name, str) or not name.strip():
            raise HTTPException(status_code=400, detail="Field 'name' is required.")
        command = f"END_TRAJECTORY,{name.strip()}"
        await run_in_threadpool(
            _controller_call_or_503, command, timeout=2.0, expect_response=False
        )
        return {"status": "ok", "name": name.strip()}

    @api.get("/trajectory/list", summary="List available recorded trajectories")
    async def trajectory_list():
        detail = await run_in_threadpool(_controller_call_or_503, "GET_TRAJECTORIES")
        parts = detail.split(",")
        if not parts or parts[0] != "TRAJECTORIES":
            raise HTTPException(status_code=502, detail=f"Malformed trajectory list: {detail}")
        names = [name for name in parts[1:] if name]
        return {"trajectories": names}

    @api.post("/trajectory/run", summary="Execute a recorded trajectory")
    async def trajectory_run(payload: dict):
        if not isinstance(payload, dict):
            raise HTTPException(status_code=400, detail="JSON body required.")
        name = payload.get("name")
        if not isinstance(name, str) or not name.strip():
            raise HTTPException(status_code=400, detail="Field 'name' is required.")
        use_cache = payload.get("use_cache", False)
        loop_override = payload.get("loop_override")
        parts = [name.strip()]
        if isinstance(use_cache, bool):
            parts.append("true" if use_cache else "false")
        else:
            parts.append("" if use_cache is None else str(use_cache))
        if isinstance(loop_override, bool):
            parts.append("true" if loop_override else "false")
        elif loop_override is not None:
            parts.append(str(loop_override))
        command = "RUN_TRAJECTORY," + ",".join(parts)
        await run_in_threadpool(
            _controller_call_or_503, command, timeout=2.0, expect_response=False
        )
        return {"status": "ok"}

    @api.post("/trajectory/preview", summary="Plan a trajectory to a target point")
    async def trajectory_preview(payload: dict):
        plan = await _plan_point(payload)
        async with _latest_plan_lock:
            global _latest_plan
            _latest_plan = plan
        return plan

    @api.post("/trajectory/execute-preview", summary="Execute the last planned preview trajectory")
    async def trajectory_execute_preview():
        global _latest_plan
        async with _latest_plan_lock:
            plan = _latest_plan
        if plan is None:
            raise HTTPException(status_code=404, detail="No planned trajectory is available.")

        target = plan.get("target", {})
        try:
            x = float(target["x"])
            y = float(target["y"])
            z = float(target["z"])
        except (KeyError, TypeError, ValueError):
            raise HTTPException(status_code=500, detail="Stored plan is invalid.")

        velocity = float(plan.get("velocity", 0.1))
        acceleration = float(plan.get("acceleration", 0.05))
        closed_loop = bool(plan.get("closed_loop", True))
        closed_loop_token = "true" if closed_loop else "false"

        command = f"MOVE_LINE,{x},{y},{z},{velocity},{acceleration},{closed_loop_token}"
        await run_in_threadpool(
            _controller_call_or_503, command, timeout=5.0, expect_response=False
        )
        await run_in_threadpool(
            _controller_call_or_503, "WAIT_FOR_IDLE", timeout=60.0, expect_response=True
        )

        async with _latest_plan_lock:
            _latest_plan = None
        return {"status": "ok"}

    @api.post("/trajectory/clear-preview", summary="Discard the stored preview trajectory")
    async def trajectory_clear_preview():
        global _latest_plan
        async with _latest_plan_lock:
            _latest_plan = None
        return {"status": "ok"}

    return api


def _parse_pose_response(detail: str) -> list[float]:
    parts = detail.split(",")
    if len(parts) < 7 or parts[0] != "CURRENT_POSE":
        raise ValueError(f"Malformed pose reply: {detail}")
    try:
        joints = [float(value) for value in parts[7:]]
    except ValueError as exc:
        raise ValueError("Invalid joint data from controller") from exc
    return joints


async def _plan_point(payload: dict) -> dict[str, Any]:
    try:
        x = float(payload.get("x"))
        y = float(payload.get("y"))
        z = float(payload.get("z"))
    except (TypeError, ValueError):
        raise HTTPException(status_code=400, detail="Fields 'x', 'y', 'z' are required floats")

    velocity = float(payload.get("velocity", 0.1))
    acceleration = float(payload.get("acceleration", 0.05))
    closed_loop = bool(payload.get("closed_loop", True))

    def _compute_plan() -> dict[str, Any]:
        ok, detail = _send_controller_command("GET_POSITION", timeout=1.0)
        if not ok:
            raise HTTPException(status_code=503, detail=detail)
        try:
            start_joints = _parse_pose_response(detail)
        except ValueError as exc:
            raise HTTPException(status_code=502, detail=str(exc)) from exc

        from ..arm_controller import trajectory_execution
        from .. import ik_solver

        target = np.array([x, y, z], dtype=float)
        path = trajectory_execution._plan_smooth_move(
            start_q=start_joints,
            target_pos=target,
            velocity=velocity,
            acceleration=acceleration,
            frequency=100,
            use_smoothing=True,
        )
        if not path:
            raise HTTPException(status_code=502, detail="Planner failed to produce a path")

        cartesian_points: list[list[float]] = []
        for joints in path:
            try:
                fk_point = ik_solver.get_fk(joints)
            except Exception:
                fk_point = None
            if fk_point is None:
                continue
            arr = np.asarray(fk_point, dtype=float)
            if arr.shape[0] >= 3:
                cartesian_points.append(arr[:3].tolist())

        return {
            "target": {"x": x, "y": y, "z": z},
            "velocity": velocity,
            "acceleration": acceleration,
            "closed_loop": closed_loop,
            "joints_rad": path,
            "cartesian_m": cartesian_points,
        }

    return await run_in_threadpool(_compute_plan)


app = create_app()


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="GradientOS HTTP API server")
    parser.add_argument(
        "--host",
        default=os.environ.get("GRADIENT_API_HOST", "0.0.0.0"),
        help="Interface to bind the HTTP API (env: GRADIENT_API_HOST)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=int(os.environ.get("GRADIENT_API_PORT", "8000")),
        help="Port for the HTTP API (env: GRADIENT_API_PORT)",
    )
    parser.add_argument(
        "--dev",
        action="store_true",
        help="Enable developer mode (uvicorn reload)",
    )
    args = parser.parse_args(argv)

    reload_env = os.environ.get("GRADIENT_API_RELOAD", "").lower()
    reload_enabled = args.dev or (reload_env in {"1", "true", "yes", "on"})

    import uvicorn

    uvicorn.run(
        "gradient_os.api.main:app",
        host=args.host,
        port=args.port,
        reload=reload_enabled,
    )


if __name__ == "__main__":
    main()


def _resolve_cors_origins() -> list[str]:
    raw = os.environ.get("GRADIENT_API_CORS", "")
    if not raw:
        return ["*"]
    origins = [item.strip() for item in raw.split(",")]
    return [origin for origin in origins if origin]

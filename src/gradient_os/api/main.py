from __future__ import annotations

import asyncio
import json
import logging
import os
import socket
from contextlib import closing
from typing import Dict, Tuple

import argparse

from fastapi import FastAPI, HTTPException
from fastapi.concurrency import run_in_threadpool
from fastapi.middleware.cors import CORSMiddleware
from sse_starlette.sse import EventSourceResponse

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


def _send_controller_command(message: str, timeout: float = 0.5) -> Tuple[bool, str]:
    host, port = _resolve_controller_endpoint()
    with closing(socket.socket(socket.AF_INET, socket.SOCK_DGRAM)) as sock:
        sock.settimeout(max(0.05, timeout))
        try:
            sock.sendto(message.encode("utf-8"), (host, port))
            data, _addr = sock.recvfrom(1024)
        except socket.timeout:
            return False, f"No response for command '{message}'"
        except OSError as exc:
            return False, f"Socket error sending '{message}': {exc}"
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


def create_app() -> FastAPI:
    api = FastAPI(title="GradientOS API", version="0.1.0")
    origins = _resolve_cors_origins()
    allow_credentials = "*" not in origins
    api.add_middleware(
        CORSMiddleware,
        allow_origins=origins,
        allow_methods=["*"],
        allow_headers=["*"],
        allow_credentials=allow_credentials,
    )

    @api.on_event("startup")
    async def _log_controller_status() -> None:
        ok, detail = await run_in_threadpool(_probe_controller)
        host, port = _resolve_controller_endpoint()
        if ok:
            logger.info("Controller: %s:%s", host, port)
        else:
            logger.warning("Controller: %s:%s (%s)", host, port, detail)

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

    return api


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

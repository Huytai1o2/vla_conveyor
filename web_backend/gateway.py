from __future__ import annotations

import asyncio
import contextlib
import json
import uuid
from collections import deque
from contextlib import asynccontextmanager
from fractions import Fraction
from typing import Any, Literal

import cv2
import numpy as np
import zmq
import zmq.asyncio
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from av import VideoFrame
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from web_backend.settings import (
    COMMAND_ENDPOINT,
    EVENT_ENDPOINT,
    FRAME_ENDPOINT,
    PREVIEW_HEIGHT,
    PREVIEW_WIDTH,
    WEB_ALLOWED_ORIGINS,
    WEB_DIST_DIR,
)


class WebRtcOffer(BaseModel):
    sdp: str
    type: Literal["offer"]


class ZmqBridge:
    def __init__(self) -> None:
        self.context = zmq.asyncio.Context()
        self.command_socket = self.context.socket(zmq.DEALER)
        self.event_socket = self.context.socket(zmq.SUB)
        self.frame_socket = self.context.socket(zmq.SUB)
        for socket in (
            self.command_socket,
            self.event_socket,
            self.frame_socket,
        ):
            socket.setsockopt(zmq.LINGER, 0)
        self.command_socket.setsockopt(zmq.IDENTITY, uuid.uuid4().hex.encode())
        self.event_socket.setsockopt(zmq.SUBSCRIBE, b"")
        self.event_socket.setsockopt(zmq.RCVHWM, 1000)
        self.frame_socket.setsockopt(zmq.SUBSCRIBE, b"frame")
        self.frame_socket.setsockopt(zmq.RCVHWM, 2)
        self.command_socket.connect(COMMAND_ENDPOINT)
        self.event_socket.connect(EVENT_ENDPOINT)
        self.frame_socket.connect(FRAME_ENDPOINT)

        self.pending: dict[str, asyncio.Future[dict[str, Any]]] = {}
        self.tasks: list[asyncio.Task[Any]] = []
        self.websockets: set[WebSocket] = set()
        self.history: deque[dict[str, Any]] = deque(maxlen=250)
        self.state = "CONNECTING"
        self.prompt_ready = False
        self.telemetry: dict[str, Any] = {
            "kind": "telemetry",
            "step": 0,
            "direction": "STOP",
            "waypointIndex": None,
            "waypointCount": None,
            "destinationX": None,
            "camera": "Waiting",
            "mcu": "Waiting",
            "model": "Waiting",
            "calibrationPoints": 0,
            "calibrationValid": False,
            "frameMode": "calibration",
        }
        self.latest_frame = np.zeros(
            (PREVIEW_HEIGHT, PREVIEW_WIDTH, 3),
            dtype=np.uint8,
        )
        self.frame_version = 0
        self.frame_condition = asyncio.Condition()

    async def start(self) -> None:
        self.tasks = [
            asyncio.create_task(self._receive_replies(), name="zmq-command-replies"),
            asyncio.create_task(self._receive_events(), name="zmq-events"),
            asyncio.create_task(self._receive_frames(), name="zmq-frames"),
        ]
        try:
            snapshot = await self.request({"kind": "snapshot"}, timeout=12.0)
            self._apply_snapshot(snapshot)
        except Exception as error:
            self.state = "ERROR"
            self.history.append(
                {
                    "kind": "message",
                    "severity": "ERROR",
                    "message": f"Controller service did not answer: {error}",
                }
            )

    async def close(self) -> None:
        for task in self.tasks:
            task.cancel()
        for task in self.tasks:
            with contextlib.suppress(asyncio.CancelledError):
                await task
        for future in self.pending.values():
            if not future.done():
                future.cancel()
        self.pending.clear()
        self.command_socket.close(0)
        self.event_socket.close(0)
        self.frame_socket.close(0)
        self.context.term()

    async def request(
        self,
        command: dict[str, Any],
        *,
        timeout: float = 8.0,
    ) -> dict[str, Any]:
        request_id = uuid.uuid4().hex
        loop = asyncio.get_running_loop()
        future: asyncio.Future[dict[str, Any]] = loop.create_future()
        self.pending[request_id] = future
        await self.command_socket.send_json(
            {**command, "requestId": request_id}
        )
        try:
            response = await asyncio.wait_for(future, timeout=timeout)
        finally:
            self.pending.pop(request_id, None)
        if not response.get("ok"):
            raise RuntimeError(str(response.get("error", "Controller command failed.")))
        return response

    async def _receive_replies(self) -> None:
        while True:
            response = await self.command_socket.recv_json()
            request_id = response.get("requestId")
            future = self.pending.get(str(request_id))
            if future is not None and not future.done():
                future.set_result(response)

    def _apply_event(self, event: dict[str, Any]) -> None:
        kind = event.get("kind")
        if kind == "message":
            self.history.append(dict(event))
        elif kind == "state":
            self.state = str(event.get("state", "UNKNOWN"))
            self.prompt_ready = self.state == "WAITING_FOR_PROMPT"
        elif kind == "prompt_ready":
            self.prompt_ready = True
        elif kind == "fatal":
            self.prompt_ready = False
        elif kind == "telemetry":
            self.telemetry = {**self.telemetry, **event}

    def _apply_snapshot(self, snapshot: dict[str, Any]) -> None:
        self.state = str(snapshot.get("state", self.state))
        self.prompt_ready = bool(snapshot.get("promptReady", False))
        telemetry = snapshot.get("telemetry")
        if isinstance(telemetry, dict):
            self.telemetry = {**self.telemetry, **telemetry}
        events = snapshot.get("events")
        if isinstance(events, list):
            self.history.clear()
            for event in events:
                if isinstance(event, dict):
                    self.history.append(event)

    async def _receive_events(self) -> None:
        while True:
            event = await self.event_socket.recv_json()
            if not isinstance(event, dict):
                continue
            self._apply_event(event)
            await self.broadcast(event)

    async def _receive_frames(self) -> None:
        while True:
            parts = await self.frame_socket.recv_multipart()
            if len(parts) != 2:
                continue
            encoded = np.frombuffer(parts[1], dtype=np.uint8)
            frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
            if frame is None:
                continue
            async with self.frame_condition:
                self.latest_frame = frame
                self.frame_version += 1
                self.frame_condition.notify_all()

    async def next_frame(
        self,
        previous_version: int,
        *,
        timeout: float = 1.0,
    ) -> tuple[np.ndarray, int]:
        async with self.frame_condition:
            if self.frame_version <= previous_version:
                with contextlib.suppress(asyncio.TimeoutError):
                    await asyncio.wait_for(
                        self.frame_condition.wait(),
                        timeout=timeout,
                    )
            return self.latest_frame.copy(), self.frame_version

    async def broadcast(self, event: dict[str, Any]) -> None:
        stale: list[WebSocket] = []
        for websocket in tuple(self.websockets):
            try:
                await websocket.send_json(event)
            except Exception:
                stale.append(websocket)
        for websocket in stale:
            self.websockets.discard(websocket)

    async def send_snapshot(self, websocket: WebSocket) -> None:
        try:
            snapshot = await self.request({"kind": "snapshot"}, timeout=4.0)
            self._apply_snapshot(snapshot)
        except Exception:
            pass
        for event in self.history:
            await websocket.send_json(event)
        await websocket.send_json({"kind": "state", "state": self.state})
        await websocket.send_json(self.telemetry)
        if self.prompt_ready:
            await websocket.send_json({"kind": "prompt_ready"})


class ConveyorVideoTrack(VideoStreamTrack):
    def __init__(self, bridge: ZmqBridge):
        super().__init__()
        self.bridge = bridge
        self.version = 0

    async def recv(self) -> VideoFrame:
        pts, time_base = await self.next_timestamp()
        image, self.version = await self.bridge.next_frame(self.version)
        frame = VideoFrame.from_ndarray(image, format="bgr24")
        frame.pts = pts
        frame.time_base = time_base or Fraction(1, 90_000)
        return frame


@asynccontextmanager
async def lifespan(app: FastAPI):
    bridge = ZmqBridge()
    peers: set[RTCPeerConnection] = set()
    app.state.bridge = bridge
    app.state.peers = peers
    await bridge.start()
    try:
        yield
    finally:
        await asyncio.gather(
            *(peer.close() for peer in tuple(peers)),
            return_exceptions=True,
        )
        await bridge.close()


app = FastAPI(title="Conveyor VLA Gateway", lifespan=lifespan)
app.add_middleware(
    CORSMiddleware,
    allow_origins=WEB_ALLOWED_ORIGINS,
    allow_credentials=True,
    allow_methods=["GET", "POST"],
    allow_headers=["*"],
)


def set_server_shutdown_callback(callback: Any) -> None:
    app.state.server_shutdown_callback = callback


@app.get("/api/health")
async def health() -> dict[str, Any]:
    bridge: ZmqBridge = app.state.bridge
    return {
        "ok": bridge.state != "ERROR",
        "state": bridge.state,
        "promptReady": bridge.prompt_ready,
        "telemetry": bridge.telemetry,
    }


@app.post("/api/webrtc/offer")
async def webrtc_offer(offer: WebRtcOffer) -> dict[str, str]:
    bridge: ZmqBridge = app.state.bridge
    peer = RTCPeerConnection()
    app.state.peers.add(peer)

    @peer.on("connectionstatechange")
    async def connectionstatechange() -> None:
        if peer.connectionState in {"failed", "closed"}:
            await peer.close()
            app.state.peers.discard(peer)

    try:
        await peer.setRemoteDescription(
            RTCSessionDescription(sdp=offer.sdp, type=offer.type)
        )
        peer.addTrack(ConveyorVideoTrack(bridge))
        answer = await peer.createAnswer()
        await peer.setLocalDescription(answer)
        if peer.localDescription is None:
            raise RuntimeError("WebRTC answer was not created.")
        return {
            "sdp": peer.localDescription.sdp,
            "type": peer.localDescription.type,
        }
    except Exception as error:
        await peer.close()
        app.state.peers.discard(peer)
        raise HTTPException(status_code=500, detail=str(error)) from error


@app.websocket("/ws/events")
async def operator_events(websocket: WebSocket) -> None:
    bridge: ZmqBridge = app.state.bridge
    await websocket.accept()
    bridge.websockets.add(websocket)
    await bridge.send_snapshot(websocket)
    try:
        while True:
            command = await websocket.receive_json()
            if not isinstance(command, dict):
                continue
            kind = command.get("kind")
            if kind == "subscribe":
                await bridge.send_snapshot(websocket)
                continue
            try:
                response = await bridge.request(command)
                await websocket.send_json(
                    {
                        "kind": "command_result",
                        "command": kind,
                        "ok": True,
                        "result": {
                            key: value
                            for key, value in response.items()
                            if key not in {"requestId", "ok"}
                        },
                    }
                )
                if kind == "stop":
                    callback = getattr(
                        app.state,
                        "server_shutdown_callback",
                        None,
                    )
                    if callable(callback):
                        asyncio.get_running_loop().call_later(0.2, callback)
            except Exception as error:
                await websocket.send_json(
                    {
                        "kind": "command_result",
                        "command": kind,
                        "ok": False,
                        "error": str(error),
                    }
                )
    except WebSocketDisconnect:
        pass
    finally:
        bridge.websockets.discard(websocket)


if WEB_DIST_DIR.is_dir():
    app.mount(
        "/",
        StaticFiles(directory=WEB_DIST_DIR, html=True),
        name="dashboard",
    )
else:

    @app.get("/", response_class=HTMLResponse)
    async def missing_frontend() -> str:
        return """
        <!doctype html>
        <title>Conveyor VLA</title>
        <main style="font:16px system-ui;max-width:720px;margin:80px auto">
          <h1>Frontend build not found</h1>
          <p>Run <code>cd web &amp;&amp; npm install &amp;&amp; npm run build</code>,
          then restart <code>python web_server.py</code>.</p>
        </main>
        """

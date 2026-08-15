from __future__ import annotations

import asyncio
import threading
import time
import unittest
import uuid
from types import MethodType
from unittest.mock import patch

import numpy as np
from aiortc import RTCPeerConnection, RTCSessionDescription
from httpx import ASGITransport, AsyncClient

import controller
import web_backend.controller_service as controller_service_module
import web_backend.gateway as gateway_module


def free_endpoint(name: str) -> str:
    return f"ipc:///tmp/conveyor-{name}-{uuid.uuid4().hex}.sock"


class FakeCameraStream:
    def __init__(self) -> None:
        self.frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        self.stopped = False

    def read(self) -> np.ndarray:
        return self.frame.copy()

    def assert_healthy(self) -> None:
        return None

    def stop(self) -> None:
        self.stopped = True


class FakeBoard:
    def __init__(self) -> None:
        self.writes: list[bytes] = []
        self.closed = False

    def write(self, value: bytes) -> None:
        self.writes.append(value)

    def flush(self) -> None:
        return None

    def close(self) -> None:
        self.closed = True


class WebBackendIntegrationTest(unittest.TestCase):
    def test_zmq_snapshot_frames_and_calibration_commands(self) -> None:
        command_endpoint = free_endpoint("command")
        event_endpoint = free_endpoint("event")
        frame_endpoint = free_endpoint("frame")
        controller_service_module.COMMAND_ENDPOINT = command_endpoint
        controller_service_module.EVENT_ENDPOINT = event_endpoint
        controller_service_module.FRAME_ENDPOINT = frame_endpoint
        gateway_module.COMMAND_ENDPOINT = command_endpoint
        gateway_module.EVENT_ENDPOINT = event_endpoint
        gateway_module.FRAME_ENDPOINT = frame_endpoint

        shutdown = threading.Event()
        service = controller_service_module.ConveyorControllerService(shutdown)
        fake_camera = FakeCameraStream()
        fake_board = FakeBoard()

        def fake_open_hardware(instance: object) -> None:
            typed = instance
            typed.camera_stream = fake_camera  # type: ignore[attr-defined]
            typed.board = fake_board  # type: ignore[attr-defined]
            typed._set_state(typed.CALIBRATION_LIVE)  # type: ignore[attr-defined]
            typed._publish_telemetry()  # type: ignore[attr-defined]

        def fake_control_loop(
            board: object,
            camera_stream: object,
            matrix: np.ndarray,
            display_state: object,
            prompt_queue: object,
            ui_events: object,
            stop_event: threading.Event,
            control_done: threading.Event,
        ) -> None:
            del board, camera_stream, matrix, display_state
            ui_events.put({"kind": "state", "state": "WAITING_FOR_PROMPT"})
            ui_events.put({"kind": "prompt_ready"})
            first_instruction = prompt_queue.get(timeout=5.0)
            if first_instruction is not None:
                ui_events.put({"kind": "state", "state": "ANALYZING"})
                ui_events.put(
                    {
                        "kind": "message",
                        "severity": "INFO",
                        "message": "Locked waypoint sequence: [800]",
                    }
                )
                ui_events.put(
                    {
                        "kind": "message",
                        "severity": "INFO",
                        "message": "Sending firmware MOVE,RIGHT,250",
                    }
                )
                ui_events.put({"kind": "state", "state": "AT_TARGET"})
                ui_events.put(
                    {
                        "kind": "message",
                        "severity": "SUCCESS",
                        "message": "All 1 waypoint(s) reached. Awaiting next prompt.",
                    }
                )
                ui_events.put({"kind": "state", "state": "WAITING_FOR_PROMPT"})
                ui_events.put({"kind": "prompt_ready"})
            second_instruction = prompt_queue.get(timeout=5.0)
            if second_instruction is not None:
                ui_events.put({"kind": "state", "state": "ANALYZING"})
                ui_events.put(
                    {
                        "kind": "message",
                        "severity": "WARNING",
                        "message": "The requested destination is ambiguous.",
                    }
                )
                ui_events.put(
                    {"kind": "state", "state": "INVALID_INSTRUCTION"}
                )
                ui_events.put({"kind": "state", "state": "WAITING_FOR_PROMPT"})
                ui_events.put({"kind": "prompt_ready"})
            stop_event.wait(5.0)
            control_done.set()

        service._open_hardware = MethodType(fake_open_hardware, service)
        service_thread = threading.Thread(target=service.run, daemon=True)
        service_thread.start()

        async def exercise() -> None:
            bridge = gateway_module.ZmqBridge()
            try:
                await bridge.start()
                self.assertEqual(bridge.state, "CALIBRATION_LIVE")
                await bridge.request({"kind": "calibration_capture"})

                raw_points = [
                    (100, 100),
                    (100, 620),
                    (1180, 100),
                    (1180, 620),
                ]
                scale, _, _, origin_x, origin_y = controller.fit_image_to_view(
                    1280,
                    720,
                    controller_service_module.PREVIEW_WIDTH,
                    controller_service_module.PREVIEW_HEIGHT,
                )
                for x, y in raw_points:
                    preview_x = (origin_x + x * scale) / (
                        controller_service_module.PREVIEW_WIDTH - 1
                    )
                    preview_y = (origin_y + y * scale) / (
                        controller_service_module.PREVIEW_HEIGHT - 1
                    )
                    await bridge.request(
                        {
                            "kind": "calibration_point",
                            "x": preview_x,
                            "y": preview_y,
                        }
                    )

                snapshot = await bridge.request({"kind": "snapshot"})
                telemetry = snapshot["telemetry"]
                self.assertEqual(telemetry["calibrationPoints"], 4)
                self.assertTrue(telemetry["calibrationValid"])

                await bridge.request({"kind": "calibration_confirm"})
                deadline = time.monotonic() + 2.0
                while bridge.state != "WAITING_FOR_PROMPT" and time.monotonic() < deadline:
                    await asyncio.sleep(0.02)
                self.assertEqual(bridge.state, "WAITING_FOR_PROMPT")
                await bridge.request(
                    {"kind": "prompt", "instruction": "move the red tape right"}
                )

                deadline = time.monotonic() + 2.0
                final_events: list[dict[str, object]] = []
                while time.monotonic() < deadline:
                    final_events = [
                        event
                        for event in bridge.history
                        if event.get("presentation") == "final"
                    ]
                    if final_events:
                        break
                    await asyncio.sleep(0.02)
                self.assertEqual(len(final_events), 1)
                final_event = final_events[0]
                run_id = final_event.get("runId")
                self.assertIsInstance(run_id, str)
                self.assertEqual(final_event.get("severity"), "SUCCESS")

                run_events = [
                    event for event in bridge.history if event.get("runId") == run_id
                ]
                self.assertTrue(
                    any(event.get("presentation") == "user" for event in run_events)
                )
                thinking_events = [
                    event
                    for event in run_events
                    if event.get("presentation") == "thinking"
                ]
                self.assertGreaterEqual(len(thinking_events), 3)
                self.assertTrue(
                    all(event.get("presentation") == "thinking" for event in thinking_events)
                )

                await bridge.request(
                    {"kind": "prompt", "instruction": "move it somewhere"}
                )
                deadline = time.monotonic() + 2.0
                warning_finals: list[dict[str, object]] = []
                while time.monotonic() < deadline:
                    warning_finals = [
                        event
                        for event in bridge.history
                        if event.get("presentation") == "final"
                        and event.get("severity") == "WARNING"
                    ]
                    if warning_finals:
                        break
                    await asyncio.sleep(0.02)
                self.assertEqual(len(warning_finals), 1)
                warning_run_id = warning_finals[0].get("runId")
                self.assertTrue(
                    any(
                        event.get("runId") == warning_run_id
                        and event.get("presentation") == "thinking"
                        and event.get("severity") == "WARNING"
                        for event in bridge.history
                    )
                )

                deadline = time.monotonic() + 2.0
                while bridge.frame_version == 0 and time.monotonic() < deadline:
                    await asyncio.sleep(0.02)
                self.assertGreater(bridge.frame_version, 0)
                self.assertEqual(
                    bridge.latest_frame.shape,
                    (
                        controller_service_module.PREVIEW_HEIGHT,
                        controller_service_module.PREVIEW_WIDTH,
                        3,
                    ),
                )
                track = gateway_module.ConveyorVideoTrack(bridge)
                video_frame = await track.recv()
                self.assertEqual(video_frame.width, controller_service_module.PREVIEW_WIDTH)
                self.assertEqual(video_frame.height, controller_service_module.PREVIEW_HEIGHT)
                track.stop()

                async with gateway_module.lifespan(gateway_module.app):
                    transport = ASGITransport(app=gateway_module.app)
                    async with AsyncClient(
                        transport=transport,
                        base_url="http://testserver",
                    ) as client:
                        health = await client.get("/api/health")
                        self.assertEqual(health.status_code, 200)
                        self.assertIn("state", health.json())
                        preflight = await client.options(
                            "/api/webrtc/offer",
                            headers={
                                "Origin": "http://127.0.0.1:8000",
                                "Access-Control-Request-Method": "POST",
                                "Access-Control-Request-Headers": "content-type",
                            },
                        )
                        self.assertEqual(preflight.status_code, 200, preflight.text)
                        self.assertEqual(
                            preflight.headers.get("access-control-allow-origin"),
                            "http://127.0.0.1:8000",
                        )
                        frontend = await client.get("/")
                        self.assertEqual(frontend.status_code, 200)
                        self.assertIn('<div id="root"></div>', frontend.text)

                        browser_peer = RTCPeerConnection()
                        received_track: asyncio.Future[object] = (
                            asyncio.get_running_loop().create_future()
                        )

                        @browser_peer.on("track")
                        def on_track(track: object) -> None:
                            if not received_track.done():
                                received_track.set_result(track)

                        browser_peer.addTransceiver("video", direction="recvonly")
                        offer = await browser_peer.createOffer()
                        await browser_peer.setLocalDescription(offer)
                        response = await client.post(
                            "/api/webrtc/offer",
                            json={
                                "sdp": browser_peer.localDescription.sdp,
                                "type": browser_peer.localDescription.type,
                            },
                        )
                        self.assertEqual(response.status_code, 200, response.text)
                        answer = response.json()
                        await browser_peer.setRemoteDescription(
                            RTCSessionDescription(
                                sdp=answer["sdp"],
                                type=answer["type"],
                            )
                        )
                        self.assertIsNotNone(browser_peer.remoteDescription)
                        remote_track = await asyncio.wait_for(received_track, 8.0)
                        remote_frame = await asyncio.wait_for(
                            remote_track.recv(),  # type: ignore[attr-defined]
                            8.0,
                        )
                        self.assertEqual(
                            remote_frame.width,
                            controller_service_module.PREVIEW_WIDTH,
                        )
                        self.assertEqual(
                            remote_frame.height,
                            controller_service_module.PREVIEW_HEIGHT,
                        )
                        await browser_peer.close()
            finally:
                await bridge.close()

        try:
            with patch.object(controller, "control_loop", fake_control_loop):
                asyncio.run(exercise())
        finally:
            shutdown.set()
            service_thread.join(timeout=5.0)

        self.assertFalse(service_thread.is_alive())
        self.assertTrue(fake_camera.stopped)
        self.assertTrue(fake_board.closed)
        self.assertIn(b"STOP\n", fake_board.writes)


if __name__ == "__main__":
    unittest.main()

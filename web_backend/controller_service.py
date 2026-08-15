from __future__ import annotations

import ast
import json
import queue
import signal
import threading
import time
import traceback
import uuid
from collections import deque
from multiprocessing.synchronize import Event as ProcessEvent
from typing import Any

import cv2
import numpy as np
import zmq

import controller
from web_backend.settings import (
    COMMAND_ENDPOINT,
    EVENT_ENDPOINT,
    FRAME_ENDPOINT,
    PREVIEW_FPS,
    PREVIEW_HEIGHT,
    PREVIEW_JPEG_QUALITY,
    PREVIEW_WIDTH,
)


class ConveyorControllerService:
    """ZMQ adapter around the existing controller implementation.

    This class owns hardware and UI transport state only. VLA inference,
    validation, waypoint progression, retries, and firmware execution remain in
    controller.control_loop without modification.
    """

    CALIBRATION_LIVE = "CALIBRATION_LIVE"
    CALIBRATION_SELECTING = "CALIBRATION_SELECTING"
    RUNTIME = "RUNTIME"
    ERROR = "ERROR"
    STOPPING = "STOPPING"

    def __init__(self, shutdown_event: ProcessEvent):
        self.shutdown_event = shutdown_event
        self.context = zmq.Context()
        self.command_socket = self.context.socket(zmq.ROUTER)
        self.event_socket = self.context.socket(zmq.PUB)
        self.frame_socket = self.context.socket(zmq.PUB)
        self.command_socket.setsockopt(zmq.LINGER, 0)
        self.event_socket.setsockopt(zmq.LINGER, 0)
        self.frame_socket.setsockopt(zmq.LINGER, 0)
        self.event_socket.setsockopt(zmq.SNDHWM, 1000)
        self.frame_socket.setsockopt(zmq.SNDHWM, 2)
        self.command_socket.bind(COMMAND_ENDPOINT)
        self.event_socket.bind(EVENT_ENDPOINT)
        self.frame_socket.bind(FRAME_ENDPOINT)

        self.board: Any | None = None
        self.camera: cv2.VideoCapture | None = None
        self.camera_stream: controller.CameraStream | None = None
        self.display_state = controller.DisplayState()
        self.prompt_queue: queue.Queue[str | None] = queue.Queue()
        self.ui_events: queue.Queue[dict[str, Any]] = queue.Queue()
        self.control_stop_event = threading.Event()
        self.control_done = threading.Event()
        self.worker: threading.Thread | None = None
        self.worker_started = False

        self.mode = self.CALIBRATION_LIVE
        self.current_state = self.CALIBRATION_LIVE
        self.prompt_ready = False
        self.calibration_frame: np.ndarray | None = None
        self.calibration_points: list[tuple[int, int]] = []
        self.matrix: np.ndarray | None = None
        self.history: deque[dict[str, Any]] = deque(maxlen=250)
        self.last_error: str | None = None
        self.waypoint_index: int | None = None
        self.waypoint_count: int | None = None
        self.active_run_id: str | None = None
        self.last_run_state: str | None = None
        self.last_run_error: str | None = None
        self._camera_error_reported = False

    def _send_event(self, event: dict[str, Any], *, remember: bool = True) -> None:
        if event.get("kind") == "message" and "timestamp" not in event:
            event = {**event, "timestamp": time.time()}
        if remember and event.get("kind") == "message":
            self.history.append(dict(event))
        try:
            self.event_socket.send_json(event, flags=zmq.NOBLOCK)
        except zmq.Again:
            pass

    def _send_message(self, severity: str, message: str, **data: Any) -> None:
        print(f"[{severity}] {message}", flush=True)
        self._send_event(
            {
                "kind": "message",
                "severity": severity,
                "message": message,
                **data,
            }
        )

    def _finalize_run(self, *, force_state: str | None = None) -> None:
        """Emit one user-facing result after the controller finishes a prompt.

        The controller's original messages remain untouched and are presented as
        execution progress. This transport-only summary gives the web client a
        stable boundary between expandable progress and the final response.
        """
        run_id = self.active_run_id
        if run_id is None:
            return

        state = force_state or self.last_run_state
        if state == "AT_TARGET":
            count = self.waypoint_count
            suffix = (
                f" All {count} waypoint{'s' if count != 1 else ''} "
                f"{'were' if count != 1 else 'was'} reached."
                if count
                else ""
            )
            severity = "SUCCESS"
            message = f"Task completed.{suffix}"
        elif state == "INVALID_INSTRUCTION":
            severity = "WARNING"
            message = (
                "I could not build an unambiguous waypoint plan. "
                "Please describe one object and its ordered destinations."
            )
        elif state == "TARGET_MISSING":
            severity = "WARNING"
            message = (
                "Task stopped because the requested object could not be "
                "recovered. Place it back on the conveyor and try again."
            )
        elif state == "ERROR":
            severity = "ERROR"
            detail = self.last_run_error or "The controller reported a fatal error."
            message = f"The task could not be completed. {detail}"
        else:
            severity = "INFO"
            message = "Task finished. The controller is ready for another instruction."

        self._send_message(
            severity,
            message,
            runId=run_id,
            presentation="final",
        )
        self.active_run_id = None
        self.last_run_state = None
        self.last_run_error = None

    def _set_state(self, state: str) -> None:
        self.current_state = state
        self.prompt_ready = state == "WAITING_FOR_PROMPT"
        self._send_event({"kind": "state", "state": state}, remember=False)

    def _telemetry(self) -> dict[str, Any]:
        display = self.display_state.snapshot()
        return {
            "kind": "telemetry",
            "step": display["step"],
            "direction": display["direction"],
            "destinationX": display["destination_x"],
            "point": display["point"],
            "waypointIndex": self.waypoint_index,
            "waypointCount": self.waypoint_count,
            "camera": "Connected" if self.camera_stream is not None else "Offline",
            "mcu": "YoloUNO" if self.board is not None else "Offline",
            "model": controller.MODEL_NAME,
            "calibrationPoints": len(self.calibration_points),
            "calibrationValid": controller.calibration_is_valid(
                self.calibration_points
            ),
            "frameMode": (
                "calibrated" if self.mode == self.RUNTIME else "calibration"
            ),
            "previewWidth": PREVIEW_WIDTH,
            "previewHeight": PREVIEW_HEIGHT,
        }

    def _publish_telemetry(self) -> None:
        self._send_event(self._telemetry(), remember=False)

    def _snapshot(self) -> dict[str, Any]:
        return {
            "ok": True,
            "state": self.current_state,
            "promptReady": self.prompt_ready,
            "telemetry": self._telemetry(),
            "events": list(self.history),
            "error": self.last_error,
        }

    def _open_hardware(self) -> None:
        self._send_message("INFO", "Connecting to YoloUNO and camera.")
        self.board = controller.open_yolouno()
        self.camera = controller.open_camera()
        self.camera_stream = controller.CameraStream(self.camera).start()
        self._send_message(
            "INFO",
            "Camera is live. Capture a frame, then click physical TL, BL, TR, BR.",
        )
        self._set_state(self.CALIBRATION_LIVE)
        self._publish_telemetry()

    def _start_worker(self, matrix: np.ndarray) -> None:
        if self.worker_started:
            raise RuntimeError("Control worker is already running.")
        if self.board is None or self.camera_stream is None:
            raise RuntimeError("Hardware is not ready for runtime control.")
        self.worker = threading.Thread(
            target=controller.control_loop,
            args=(
                self.board,
                self.camera_stream,
                matrix,
                self.display_state,
                self.prompt_queue,
                self.ui_events,
                self.control_stop_event,
                self.control_done,
            ),
            name="vla-control",
            daemon=True,
        )
        self.worker.start()
        self.worker_started = True

    @staticmethod
    def _letterbox(image: np.ndarray) -> np.ndarray:
        height, width = image.shape[:2]
        _, display_width, display_height, origin_x, origin_y = (
            controller.fit_image_to_view(
                width,
                height,
                PREVIEW_WIDTH,
                PREVIEW_HEIGHT,
            )
        )
        resized = cv2.resize(
            image,
            (display_width, display_height),
            interpolation=cv2.INTER_AREA,
        )
        output = np.zeros((PREVIEW_HEIGHT, PREVIEW_WIDTH, 3), dtype=np.uint8)
        output[
            origin_y : origin_y + display_height,
            origin_x : origin_x + display_width,
        ] = resized
        return output

    def _calibration_overlay(self, image: np.ndarray) -> np.ndarray:
        output = image.copy()
        marker_scale = max(0.7, min(1.5, image.shape[1] / 1280.0))
        radius = max(8, round(10 * marker_scale))
        for index, point in enumerate(self.calibration_points):
            cv2.circle(output, point, radius, (255, 255, 255), -1)
            cv2.circle(output, point, radius + 2, (0, 0, 0), 2)
            cv2.putText(
                output,
                f"{index + 1} {controller.CALIBRATION_CLICK_LABELS[index]}",
                (point[0] + radius, point[1] - radius),
                cv2.FONT_HERSHEY_SIMPLEX,
                marker_scale,
                (255, 255, 255),
                max(2, round(2 * marker_scale)),
            )
        if len(self.calibration_points) == 4:
            polygon = (
                controller.calibration_source_corners(self.calibration_points)
                .astype(np.int32)
                .reshape((-1, 1, 2))
            )
            cv2.polylines(output, [polygon], True, (255, 255, 255), 3)
        return output

    def _visible_frame(self) -> np.ndarray | None:
        if self.mode == self.CALIBRATION_SELECTING:
            if self.calibration_frame is None:
                return None
            return self._letterbox(
                self._calibration_overlay(self.calibration_frame)
            )
        if self.camera_stream is None:
            return None
        frame = self.camera_stream.read()
        if frame is None:
            return None
        if self.mode == self.RUNTIME and self.matrix is not None:
            calibrated = controller.prepare_vla_image(frame, self.matrix)
            annotated = controller.annotate_vla_frame(
                calibrated,
                self.display_state.snapshot(),
            )
            return self._letterbox(annotated)
        return self._letterbox(frame)

    def _publish_frame(self) -> None:
        if self.camera_stream is not None:
            self.camera_stream.assert_healthy()
        frame = self._visible_frame()
        if frame is None:
            return
        ok, encoded = cv2.imencode(
            ".jpg",
            frame,
            [cv2.IMWRITE_JPEG_QUALITY, PREVIEW_JPEG_QUALITY],
        )
        if not ok:
            raise RuntimeError("Could not encode the WebRTC preview frame.")
        try:
            self.frame_socket.send_multipart(
                [b"frame", encoded.tobytes()],
                flags=zmq.NOBLOCK,
            )
        except zmq.Again:
            pass

    def _raw_point_from_preview(
        self,
        normalized_x: float,
        normalized_y: float,
    ) -> tuple[int, int] | None:
        if self.calibration_frame is None:
            return None
        height, width = self.calibration_frame.shape[:2]
        scale, _, _, origin_x, origin_y = controller.fit_image_to_view(
            width,
            height,
            PREVIEW_WIDTH,
            PREVIEW_HEIGHT,
        )
        view_x = round(normalized_x * (PREVIEW_WIDTH - 1))
        view_y = round(normalized_y * (PREVIEW_HEIGHT - 1))
        return controller.view_to_image_point(
            view_x,
            view_y,
            scale=scale,
            origin_x=origin_x,
            origin_y=origin_y,
            image_width=width,
            image_height=height,
        )

    def _capture_calibration(self) -> dict[str, Any]:
        if self.mode == self.RUNTIME:
            raise RuntimeError("Runtime calibration is already locked.")
        if self.camera_stream is None:
            raise RuntimeError("Camera is offline.")
        frame = self.camera_stream.read()
        if frame is None:
            raise RuntimeError("Camera has not produced a frame yet.")
        self.calibration_frame = frame
        self.calibration_points.clear()
        self.mode = self.CALIBRATION_SELECTING
        self._set_state(self.CALIBRATION_SELECTING)
        self._send_message(
            "INFO",
            "Calibration frame captured. Click TL, BL, TR, BR on the video.",
        )
        self._publish_telemetry()
        return {"ok": True}

    def _handle_command(self, request: dict[str, Any]) -> dict[str, Any]:
        kind = request.get("kind")
        if kind == "snapshot" or kind == "subscribe":
            return self._snapshot()
        if kind == "calibration_capture":
            return self._capture_calibration()
        if kind == "calibration_live":
            if self.mode == self.RUNTIME:
                raise RuntimeError("Runtime calibration is already locked.")
            self.mode = self.CALIBRATION_LIVE
            self.calibration_frame = None
            self.calibration_points.clear()
            self._set_state(self.CALIBRATION_LIVE)
            self._publish_telemetry()
            return {"ok": True}
        if kind == "calibration_point":
            if self.mode != self.CALIBRATION_SELECTING:
                raise RuntimeError("Capture a calibration frame before clicking.")
            if len(self.calibration_points) >= 4:
                raise RuntimeError("Four calibration points are already selected.")
            normalized_x = request.get("x")
            normalized_y = request.get("y")
            if (
                type(normalized_x) not in (int, float)
                or type(normalized_y) not in (int, float)
                or not 0.0 <= float(normalized_x) <= 1.0
                or not 0.0 <= float(normalized_y) <= 1.0
            ):
                raise ValueError("Calibration point must be normalized to 0..1.")
            point = self._raw_point_from_preview(
                float(normalized_x),
                float(normalized_y),
            )
            if point is None:
                raise ValueError("Click inside the visible camera image.")
            self.calibration_points.append(point)
            self._publish_telemetry()
            return {"ok": True, "point": list(point)}
        if kind == "calibration_undo":
            if self.mode != self.CALIBRATION_SELECTING:
                raise RuntimeError("No frozen calibration frame is active.")
            if self.calibration_points:
                self.calibration_points.pop()
            self._publish_telemetry()
            return {"ok": True}
        if kind == "calibration_reset":
            if self.mode != self.CALIBRATION_SELECTING:
                raise RuntimeError("No frozen calibration frame is active.")
            self.calibration_points.clear()
            self._publish_telemetry()
            return {"ok": True}
        if kind == "calibration_confirm":
            if self.mode != self.CALIBRATION_SELECTING:
                raise RuntimeError("No frozen calibration frame is active.")
            matrix = controller.calibration_matrix(self.calibration_points)
            self._start_worker(matrix)
            self.matrix = matrix
            self.mode = self.RUNTIME
            self.calibration_frame = None
            self._send_message(
                "SUCCESS",
                "Calibration accepted. Web runtime is active.",
            )
            self._publish_telemetry()
            return {"ok": True}
        if kind == "prompt":
            instruction = str(request.get("instruction", "")).strip()
            if not instruction:
                raise ValueError("Instruction cannot be blank.")
            if not self.worker_started or not self.prompt_ready:
                raise RuntimeError("Controller is not ready for a new instruction.")
            self.prompt_ready = False
            self.waypoint_index = None
            self.waypoint_count = None
            self.active_run_id = uuid.uuid4().hex
            self.last_run_state = None
            self.last_run_error = None
            self._send_message(
                "USER",
                instruction,
                runId=self.active_run_id,
                presentation="user",
            )
            self.prompt_queue.put(instruction)
            return {"ok": True}
        if kind == "stop":
            self._set_state(self.STOPPING)
            controller.request_stop(self.control_stop_event)
            self.prompt_queue.put(None)
            self.shutdown_event.set()
            return {"ok": True}
        raise ValueError(f"Unsupported command: {kind!r}")

    def _drain_controller_events(self) -> None:
        while True:
            try:
                event = self.ui_events.get_nowait()
            except queue.Empty:
                return

            kind = event.get("kind")
            if kind == "state":
                state = str(event["state"])
                if state == "WAYPOINT_REACHED" and self.waypoint_index is not None:
                    self.waypoint_index += 1
                if self.active_run_id is not None and state != "WAITING_FOR_PROMPT":
                    self.last_run_state = state
                self._set_state(state)
            elif kind == "prompt_ready":
                self._finalize_run()
                self.prompt_ready = True
                self.waypoint_index = None
                self.waypoint_count = None
                self._send_event(event, remember=False)
            elif kind == "fatal":
                self._finalize_run(force_state="ERROR")
                self.prompt_ready = False
                self._send_event(event, remember=False)
            elif kind == "message":
                message = str(event.get("message", ""))
                prefix = "Locked waypoint sequence:"
                if message.startswith(prefix):
                    try:
                        values = ast.literal_eval(message[len(prefix) :].strip())
                        if isinstance(values, list):
                            self.waypoint_index = 0
                            self.waypoint_count = len(values)
                    except (SyntaxError, ValueError):
                        pass
                if self.active_run_id is not None:
                    event = {
                        **event,
                        "runId": self.active_run_id,
                        "presentation": "thinking",
                    }
                    if event.get("severity") == "ERROR":
                        self.last_run_error = message
                self._send_event(event)
            else:
                self._send_event(event, remember=False)
            self._publish_telemetry()

    def _serve_commands(self, poller: zmq.Poller) -> None:
        events = dict(poller.poll(timeout=5))
        if self.command_socket not in events:
            return
        parts = self.command_socket.recv_multipart()
        if len(parts) != 2:
            return
        identity, raw = parts
        request_id: str | None = None
        try:
            request = json.loads(raw.decode("utf-8"))
            request_id = request.get("requestId")
            response = self._handle_command(request)
        except Exception as error:
            response = {"ok": False, "error": str(error)}
        response["requestId"] = request_id
        self.command_socket.send_multipart(
            [identity, json.dumps(response).encode("utf-8")]
        )

    def run(self) -> None:
        poller = zmq.Poller()
        poller.register(self.command_socket, zmq.POLLIN)
        try:
            try:
                self._open_hardware()
            except Exception as error:
                self.last_error = str(error)
                self.mode = self.ERROR
                self._set_state(self.ERROR)
                self._send_message("ERROR", f"Hardware startup failed: {error}")
                traceback.print_exception(type(error), error, error.__traceback__)

            frame_interval = 1.0 / PREVIEW_FPS
            next_frame_at = time.monotonic()
            next_telemetry_at = time.monotonic()
            while not self.shutdown_event.is_set():
                self._serve_commands(poller)
                self._drain_controller_events()
                now = time.monotonic()
                if now >= next_frame_at:
                    try:
                        self._publish_frame()
                    except Exception as error:
                        if not self._camera_error_reported:
                            self._camera_error_reported = True
                            self.last_error = str(error)
                            self.mode = self.ERROR
                            self._set_state(self.ERROR)
                            message = f"Camera stream failed: {error}"
                            if self.active_run_id is not None:
                                self.last_run_error = message
                                self._send_message(
                                    "ERROR",
                                    message,
                                    runId=self.active_run_id,
                                    presentation="thinking",
                                )
                                self._finalize_run(force_state="ERROR")
                            else:
                                self._send_message("ERROR", message)
                            controller.request_stop(self.control_stop_event)
                            self.prompt_queue.put(None)
                            self._send_event({"kind": "fatal"}, remember=False)
                    next_frame_at = now + frame_interval
                if now >= next_telemetry_at:
                    self._publish_telemetry()
                    next_telemetry_at = now + 0.5
        finally:
            self._cleanup()

    def _cleanup(self) -> None:
        controller.request_stop(self.control_stop_event)
        try:
            self.prompt_queue.put_nowait(None)
        except queue.Full:
            pass
        if self.worker is not None and self.worker_started:
            self.worker.join(timeout=(controller.GEMINI_TIMEOUT_MS / 1000.0) + 5.0)
        worker_alive = self.worker is not None and self.worker.is_alive()
        if self.board is not None and not worker_alive:
            controller.stop_motor(self.board, best_effort=True)
            try:
                self.board.close()
            except controller._serial_error_types():
                pass
        if self.camera_stream is not None:
            self.camera_stream.stop()
        elif self.camera is not None:
            self.camera.release()
        self.command_socket.close(0)
        self.event_socket.close(0)
        self.frame_socket.close(0)
        self.context.term()


def run_controller_service(shutdown_event: ProcessEvent) -> None:
    # Uvicorn/launcher in the parent owns Ctrl+C. The child observes the shared
    # shutdown event so hardware cleanup runs through the normal finally path.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    ConveyorControllerService(shutdown_event).run()

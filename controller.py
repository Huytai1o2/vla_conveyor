"""Calibrated Gemini VLA controller for the YoloUNO conveyor.

OpenCV owns camera calibration/warping and display only.  Gemini owns object
selection and the complete action; this module only validates that action and
adapts it to the small serial protocol implemented by conveyor_firmware.
"""

from __future__ import annotations

import os
import queue
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal

import cv2
import numpy as np
import serial
from dotenv import load_dotenv
from pydantic import BaseModel, ConfigDict, StrictBool, StrictFloat, StrictInt, StrictStr

try:  # Keep pure validation/serial tests importable without Gemini installed.
    from google import genai
    from google.genai import types
except ImportError:  # pragma: no cover - exercised only in incomplete installs.
    genai = None
    types = None


load_dotenv(Path(__file__).with_name(".env"))


# ---------- CONFIG ----------
CAMERA_INDEX = 0
SERIAL_PORT = "COM18"
SERIAL_BAUD = 115200
MODEL_NAME = "gemini-robotics-er-1.6-preview"
GEMINI_TIMEOUT_MS = 15_000

BELT_LENGTH_CM = 77.0
SPEED_TEST_DISTANCE_CM = 77.0
SPEED_TEST_TIME_S = 5.5925
BELT_SPEED_CM_S = SPEED_TEST_DISTANCE_CM / SPEED_TEST_TIME_S
CENTER_TOLERANCE_CM = 4.0
MOVE_GAIN = 0.8
MIN_MOVE_MS = 80
MAX_MOVE_MS = 1500
NEAR_CENTER_CM = 8.0
NEAR_CENTER_MAX_MS = 180
MIN_CONFIDENCE = 0.60
SETTLE_S = 1.5
NO_MATCH_LIMIT = 20
TECHNICAL_RETRY_LIMIT = 5
CAMERA_STALE_S = 3.0
STOP_REPLY_TIMEOUT_S = 2.0
WARP_WIDTH = 1000
WARP_HEIGHT = 300
SIDEBAR_WIDTH = 420
WRITE_DEBUG_IMAGES = False


class VLAResult(BaseModel):
    """The exact structured output requested from Gemini."""

    model_config = ConfigDict(extra="forbid", strict=True)

    target_found: StrictBool
    target_matches_prompt: StrictBool
    label: StrictStr | None = None
    point: list[StrictInt] | None = None  # normalized [y, x]
    direction: Literal["LEFT", "RIGHT", "STOP"]
    duration_ms: StrictInt
    task_status: Literal["MOVE", "CENTERED", "TARGET_NOT_FOUND"]
    confidence: StrictFloat


class ActionValidationError(ValueError):
    """Gemini produced JSON that is syntactically valid but unsafe to execute."""


@dataclass(frozen=True)
class ValidatedAction:
    result: VLAResult
    direction: Literal["LEFT", "RIGHT", "STOP"]
    duration_ms: int
    status: Literal["MOVE", "CENTERED", "TARGET_NOT_FOUND"]
    is_no_match: bool


def action_tokens(direction: str, duration_ms: int, status: str) -> str:
    """Visible, auditable representation of Gemini's validated action."""
    return f"[ACT_{direction}] [DURATION_{duration_ms:04d}_MS] [STATUS_{status}]"


def _valid_point(point: list[int] | None) -> bool:
    return (
        isinstance(point, list)
        and len(point) == 2
        and all(type(value) is int and 0 <= value <= 1000 for value in point)
    )


def validate_model_action(raw: VLAResult | dict[str, Any]) -> ValidatedAction:
    """Purely validate Gemini output; never derive an alternative motor action."""
    try:
        result = (
            raw
            if isinstance(raw, VLAResult)
            else VLAResult.model_validate(raw, strict=True)
        )
    except Exception as error:
        raise ActionValidationError(f"invalid Gemini schema: {error}") from error

    if not 0.0 <= result.confidence <= 1.0:
        raise ActionValidationError("confidence must be in 0..1")
    if result.point is not None and not _valid_point(result.point):
        raise ActionValidationError("point must be [y, x] integer values in 0..1000")

    no_match = (
        not result.target_found
        or not result.target_matches_prompt
        or result.confidence < MIN_CONFIDENCE
    )
    if no_match:
        # A model cannot claim an action for a missing/nonmatching/weak target.
        if (
            result.task_status != "TARGET_NOT_FOUND"
            or result.direction != "STOP"
            or result.duration_ms != 0
        ):
            raise ActionValidationError(
                "missing, nonmatching, or low-confidence target must be TARGET_NOT_FOUND/STOP/0"
            )
        if not result.target_found and result.target_matches_prompt:
            raise ActionValidationError("a target cannot match the prompt when target_found is false")
        if not result.target_found and (result.label is not None or result.point is not None):
            raise ActionValidationError("a missing target cannot have a label or point")
        return ValidatedAction(result, "STOP", 0, "TARGET_NOT_FOUND", True)

    # From here on the target is a confident prompt match and needs an audit point.
    if not _valid_point(result.point):
        raise ActionValidationError("a matching target requires a valid point")
    if result.task_status == "MOVE":
        if result.direction not in ("LEFT", "RIGHT"):
            raise ActionValidationError("MOVE requires LEFT or RIGHT")
        if not MIN_MOVE_MS <= result.duration_ms <= MAX_MOVE_MS:
            raise ActionValidationError(
                f"MOVE duration must be {MIN_MOVE_MS}..{MAX_MOVE_MS} ms"
            )
        return ValidatedAction(result, result.direction, result.duration_ms, "MOVE", False)
    if result.task_status == "CENTERED":
        if result.direction != "STOP" or result.duration_ms != 0:
            raise ActionValidationError("CENTERED requires STOP and duration 0")
        return ValidatedAction(result, "STOP", 0, "CENTERED", False)
    raise ActionValidationError("confident matching target cannot be TARGET_NOT_FOUND")


def build_prompt(instruction: str) -> str:
    """Build the per-inference policy, including the active GUI instruction."""
    return f"""
You are the high-level Vision-Language-Action controller of a reversible conveyor.

ACTIVE OPERATOR INSTRUCTION: {instruction!r}

Inspect the supplied calibrated overhead conveyor image. Identify only one loose,
movable object resting on the exposed conveyor surface whose appearance matches the
active operator instruction. Do not substitute another loose object. Ignore belt
slats, target-zone lines, rails, markers, structures, motors, wires, electronics,
people, tables, and background.

Calibration and controller policy (you must calculate the action yourself):
- normalized image coordinates are [y, x], each in 0..1000; target center x = 500;
- belt length = {BELT_LENGTH_CM:g} cm for 1000 normalized x units;
- center tolerance = {CENTER_TOLERANCE_CM:g} cm;
- measured belt speed = {BELT_SPEED_CM_S:.4f} cm/s ({SPEED_TEST_DISTANCE_CM:g} / {SPEED_TEST_TIME_S:g});
- move gain = {MOVE_GAIN:g}; valid MOVE duration = {MIN_MOVE_MS}..{MAX_MOVE_MS} ms;
- near-center threshold = {NEAR_CENTER_CM:g} cm; near-center pulse maximum = {NEAR_CENTER_MAX_MS} ms.

For a matching target calculate:
offset_cm = abs(object_x - 500) * {BELT_LENGTH_CM:g} / 1000
move_distance_cm = max(0, offset_cm - {CENTER_TOLERANCE_CM:g})
duration_ms = clamp(move_distance_cm / {BELT_SPEED_CM_S:.4f} * {MOVE_GAIN:g} * 1000,
                    {MIN_MOVE_MS}, {MAX_MOVE_MS})
if offset_cm <= {NEAR_CENTER_CM:g}: duration_ms = min(duration_ms, {NEAR_CENTER_MAX_MS})
Image-relative direction: an object left of center requires RIGHT; an object right
of center requires LEFT. If it is within tolerance, return CENTERED/STOP/0.

Return JSON only with target_found, target_matches_prompt, label, point, direction,
duration_ms, task_status, confidence. point is [y, x] integer coordinates. Use:
- MOVE only for a matching target with confidence >= {MIN_CONFIDENCE:.2f}, LEFT or RIGHT,
  and a duration in range.
- CENTERED only for a matching target with STOP and duration_ms 0.
- TARGET_NOT_FOUND with STOP and duration_ms 0 whenever the target is absent,
  does not match the instruction, or confidence is below {MIN_CONFIDENCE:.2f}.
""".strip()


# ---------- CAMERA / CALIBRATION ----------
def read_latest_frame(camera: cv2.VideoCapture) -> np.ndarray:
    for _ in range(4):
        camera.grab()
    ok, frame = camera.read()
    if not ok or frame is None:
        raise RuntimeError("Không đọc được camera.")
    return frame


def order_points(points: list[tuple[int, int]]) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float32)
    center = pts.mean(axis=0)
    angles = np.arctan2(pts[:, 1] - center[1], pts[:, 0] - center[0])
    ordered = pts[np.argsort(angles)]
    ordered = np.roll(ordered, -np.argmin(ordered[:, 0] + ordered[:, 1]), axis=0)
    if np.cross(ordered[1] - ordered[0], ordered[2] - ordered[1]) < 0:
        ordered = ordered[[0, 3, 2, 1]]
    return ordered.astype(np.float32)


def calibrate(camera: cv2.VideoCapture) -> np.ndarray | None:
    print("\nCamera live: S = chốt ảnh, Q = thoát.")
    while True:
        frame = read_latest_frame(camera)
        preview = frame.copy()
        cv2.putText(preview, "S: CALIBRATE | Q: QUIT", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow("Live Camera", preview)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            cv2.destroyWindow("Live Camera")
            return None
        if key == ord("s"):
            cv2.destroyWindow("Live Camera")
            break

    points: list[tuple[int, int]] = []
    window = "Select 4 belt corners"

    def on_mouse(event: int, x: int, y: int, _flags: int, _parameter: Any) -> None:
        if event == cv2.EVENT_LBUTTONDOWN and len(points) < 4:
            points.append((x, y))
        elif event == cv2.EVENT_RBUTTONDOWN and points:
            points.pop()

    cv2.namedWindow(window)
    cv2.setMouseCallback(window, on_mouse)
    print("Click 4 góc theo thứ tự bất kỳ. Chuột phải = undo, R = reset, Enter = xác nhận.")
    while True:
        display = frame.copy()
        ordered: np.ndarray | None = None
        valid = False
        for index, point in enumerate(points):
            cv2.circle(display, point, 7, (0, 0, 255), -1)
            cv2.putText(display, str(index + 1), (point[0] + 8, point[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        if len(points) == 4:
            ordered = order_points(points)
            polygon = ordered.astype(np.int32).reshape((-1, 1, 2))
            valid = bool(cv2.isContourConvex(polygon) and cv2.contourArea(polygon) > 2000)
            color = (0, 255, 0) if valid else (0, 0, 255)
            cv2.polylines(display, [polygon], True, color, 3)
            for label, point in zip(("TL", "TR", "BR", "BL"), ordered.astype(int)):
                cv2.putText(display, label, (point[0] + 8, point[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        cv2.imshow(window, display)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("r"):
            points.clear()
        elif key == ord("q"):
            cv2.destroyWindow(window)
            return None
        elif key in (10, 13, 32) and ordered is not None and valid:
            cv2.destroyWindow(window)
            destination = np.float32([[0, 0], [WARP_WIDTH - 1, 0], [WARP_WIDTH - 1, WARP_HEIGHT - 1], [0, WARP_HEIGHT - 1]])
            return cv2.getPerspectiveTransform(ordered, destination)


def prepare_vla_image(frame: np.ndarray, matrix: np.ndarray) -> np.ndarray:
    """Produce the full-size calibrated inference image; never resize it for control."""
    image = cv2.warpPerspective(frame, matrix, (WARP_WIDTH, WARP_HEIGHT))
    tolerance_px = round(CENTER_TOLERANCE_CM / BELT_LENGTH_CM * WARP_WIDTH)
    center_x = WARP_WIDTH // 2
    for x in (center_x - tolerance_px, center_x + tolerance_px):
        cv2.line(image, (x, 0), (x, WARP_HEIGHT), (0, 255, 0), 3)
    return image


class CameraStream:
    """Continuous capture with explicit stale-camera detection."""

    def __init__(self, camera: cv2.VideoCapture):
        self.camera = camera
        self.frame: np.ndarray | None = None
        self._last_frame_at = 0.0
        self._failure_started_at: float | None = None
        self._lock = threading.Lock()
        self.stop_event = threading.Event()
        self._released = False
        self.thread = threading.Thread(target=self._capture_loop, name="camera-stream", daemon=True)

    def start(self) -> "CameraStream":
        self.thread.start()
        deadline = time.monotonic() + CAMERA_STALE_S
        while self.read() is None:
            if time.monotonic() >= deadline:
                self.stop()
                raise RuntimeError("Camera stream không tạo được frame.")
            time.sleep(0.02)
        return self

    def _capture_loop(self) -> None:
        while not self.stop_event.is_set():
            ok, frame = self.camera.read()
            now = time.monotonic()
            with self._lock:
                if ok and frame is not None:
                    self.frame = frame
                    self._last_frame_at = now
                    self._failure_started_at = None
                elif self._failure_started_at is None:
                    self._failure_started_at = now
            if not ok or frame is None:
                time.sleep(0.02)

    def read(self) -> np.ndarray | None:
        with self._lock:
            return None if self.frame is None else self.frame.copy()

    def assert_healthy(self) -> None:
        with self._lock:
            last_frame_at = self._last_frame_at
            failed_at = self._failure_started_at
        now = time.monotonic()
        if (failed_at is not None and now - failed_at > CAMERA_STALE_S) or (last_frame_at and now - last_frame_at > CAMERA_STALE_S):
            raise RuntimeError("Camera stream bị mất hoặc frame đã cũ quá lâu.")

    def stop(self) -> None:
        self.stop_event.set()
        if self.thread.is_alive():
            self.thread.join(timeout=2.0)
        if not self._released:
            self.camera.release()
            self._released = True


# ---------- OBSERVABILITY STATE ----------
class DisplayState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.step = 0
        self.status = "STARTING"
        self.direction = "STOP"
        self.point: list[int] | None = None

    def update(self, *, step: int | None = None, status: str | None = None, direction: str | None = None, point: list[int] | None = None, clear_point: bool = False) -> None:
        with self._lock:
            if step is not None:
                self.step = step
            if status is not None:
                self.status = status
            if direction is not None:
                self.direction = direction
            if clear_point:
                self.point = None
            elif point is not None:
                self.point = list(point)

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {"step": self.step, "status": self.status, "direction": self.direction, "point": None if self.point is None else list(self.point)}


def annotate_vla_frame(image: np.ndarray, state: dict[str, Any]) -> np.ndarray:
    """Display-only annotation; it is never passed back to Gemini or control logic."""
    output = image.copy()
    cv2.rectangle(output, (0, 0), (WARP_WIDTH, 38), (0, 0, 0), -1)
    cv2.putText(output, f"Step {state['step']} | {state['status']} | Action: {state['direction']}", (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
    point = state["point"]
    if _valid_point(point):
        point_x = round(point[1] / 1000.0 * WARP_WIDTH)
        point_y = round(point[0] / 1000.0 * WARP_HEIGHT)
        cv2.circle(output, (point_x, point_y), 9, (0, 0, 255), 3)
        cv2.putText(output, "Last Gemini point", (min(point_x + 12, WARP_WIDTH - 190), max(point_y - 10, 55)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
    return output


# ---------- GEMINI ----------
_client: Any | None = None
_client_lock = threading.Lock()


def get_gemini_client() -> Any:
    """Create the SDK client lazily so importing/testing needs no API key."""
    global _client
    with _client_lock:
        if _client is not None:
            return _client
        if genai is None or types is None:
            raise RuntimeError("Thiếu package google-genai. Hãy cài requirements.txt.")
        api_key = os.getenv("GEMINI_API_KEY")
        if not api_key:
            raise RuntimeError("Thiếu GEMINI_API_KEY. Hãy điền API key vào file .env.")
        try:
            http_options = types.HttpOptions(timeout=GEMINI_TIMEOUT_MS)
            _client = genai.Client(api_key=api_key, http_options=http_options)
        except (AttributeError, TypeError) as error:
            # An unbounded inference can outlive shutdown and must never be accepted
            # for a motor controller.  Require an SDK with client HTTP timeouts.
            raise RuntimeError(
                "Installed google-genai does not support HttpOptions(timeout). "
                "Upgrade it with: pip install -U 'google-genai>=1.15.0'."
            ) from error
        return _client


def detect_object(image: np.ndarray, instruction: str, gemini_client: Any | None = None) -> VLAResult:
    """Ask Gemini for an action. No CV perception/action fallback exists here."""
    if types is None:
        raise RuntimeError("Thiếu package google-genai. Hãy cài requirements.txt.")
    ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 90])
    if not ok:
        raise RuntimeError("Không encode được ảnh.")
    response = (gemini_client or get_gemini_client()).models.generate_content(
        model=MODEL_NAME,
        contents=[types.Part.from_bytes(data=encoded.tobytes(), mime_type="image/jpeg"), build_prompt(instruction)],
        config=types.GenerateContentConfig(
            temperature=0.2,
            thinking_config=types.ThinkingConfig(thinking_budget=0),
            response_mime_type="application/json",
            response_schema=VLAResult,
        ),
    )
    try:
        parsed = response.parsed
        if isinstance(parsed, VLAResult):
            return parsed
        if parsed is not None:
            return VLAResult.model_validate(parsed, strict=True)
        return VLAResult.model_validate_json(response.text, strict=True)
    except Exception as error:
        raise ActionValidationError(f"invalid Gemini JSON/schema: {error}") from error


# ---------- YOLOUNO SERIAL ----------
KNOWN_TELEMETRY = {"READY", "PONG", "STOPPED"}
# Stop requests and the tiny serial write/flush commits share this lock.  It
# makes the final stop check plus MOVE write atomic without holding a GUI
# callback during preparation, serial reads, or a motor pulse.
SERIAL_COMMAND_LOCK = threading.RLock()


def _serial_error_types() -> tuple[type[BaseException], ...]:
    return (serial.SerialException, OSError)


def _read_line(board: Any) -> str:
    raw = board.readline()
    if isinstance(raw, bytes):
        return raw.decode(errors="ignore").strip()
    return str(raw).strip()


def _raise_pending_firmware_error(board: Any) -> None:
    """Inspect buffered board output before clearing it; never erase a pending ERR."""
    for _ in range(32):
        waiting = getattr(board, "in_waiting", 0)
        if not waiting:
            return
        line = _read_line(board)
        if line.startswith("ERR,"):
            raise RuntimeError(f"YoloUNO báo lỗi trước MOVE: {line}")


def request_stop(stop_event: threading.Event) -> None:
    """Atomically publish a shutdown request against the final MOVE write."""
    with SERIAL_COMMAND_LOCK:
        stop_event.set()


def open_yolouno() -> Any:
    try:
        board = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.25, write_timeout=1.0)
    except _serial_error_types() as error:
        raise RuntimeError(f"Không mở được {SERIAL_PORT}. Hãy đóng PlatformIO Monitor. Chi tiết: {error}") from error
    time.sleep(2.5)
    try:
        board.reset_input_buffer()
        for _ in range(5):
            board.write(b"PING\n")
            board.flush()
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                line = _read_line(board)
                if line:
                    print("YoloUNO ->", line)
                if line == "PONG":
                    print("Kết nối YoloUNO thành công.\n")
                    return board
            time.sleep(0.4)
    except _serial_error_types() as error:
        board.close()
        raise RuntimeError(f"Lỗi serial khi bắt tay YoloUNO: {error}") from error
    board.close()
    raise RuntimeError("Mở được COM nhưng YoloUNO không trả PONG. Kiểm tra firmware và USB CDC.")


def stop_motor(board: Any, *, best_effort: bool = False) -> None:
    """Runtime STOP must be acknowledged; teardown STOP intentionally is best effort."""
    try:
        with SERIAL_COMMAND_LOCK:
            board.write(b"STOP\n")
            board.flush()
        if best_effort:
            return
        deadline = time.monotonic() + STOP_REPLY_TIMEOUT_S
        while time.monotonic() < deadline:
            line = _read_line(board)
            if not line:
                continue
            print("YoloUNO ->", line)
            if line == "STOPPED":
                return
            if line.startswith("ERR,"):
                raise RuntimeError(f"YoloUNO báo lỗi STOP: {line}")
        raise TimeoutError("YoloUNO không xác nhận STOPPED.")
    except _serial_error_types() as error:
        if best_effort:
            return
        raise RuntimeError(f"Lỗi serial khi STOP motor: {error}") from error


def run_motor(board: Any, direction: Literal["LEFT", "RIGHT"], duration_ms: int, stop_event: threading.Event) -> bool:
    """Run one firmware pulse and require its exact ACK followed by DONE."""
    if direction not in ("LEFT", "RIGHT") or type(duration_ms) is not int or not MIN_MOVE_MS <= duration_ms <= MAX_MOVE_MS:
        raise ValueError("unsafe MOVE command")
    if stop_event.is_set():
        return False
    expected_ack = f"ACK,{direction},{duration_ms}"
    command = f"MOVE,{direction},{duration_ms}\n"
    try:
        # Preparation intentionally happens outside SERIAL_COMMAND_LOCK.  A Stop
        # request can therefore complete while pending data is inspected; its
        # event wins the final protected check below and prevents the MOVE write.
        _raise_pending_firmware_error(board)
        # Do not reset the input buffer here: after the ERR scan it could erase a
        # newly-arrived fault.  Stale ACK/DONE remains unable to satisfy the exact
        # protocol checks below.
        with SERIAL_COMMAND_LOCK:
            # This second check shares its lock with request_stop(), closing the
            # Stop/Exit -> MOVE write race without blocking Stop/Exit for prep.
            if stop_event.is_set():
                return False
            board.write(command.encode("ascii"))
            board.flush()
        print("TX ->", command.strip())

        got_ack = False
        deadline = time.monotonic() + duration_ms / 1000.0 + 3.0
        while time.monotonic() < deadline:
            if stop_event.is_set():
                stop_motor(board)
                return False
            line = _read_line(board)
            if not line:
                continue
            print("YoloUNO ->", line)
            if line.startswith("ERR,"):
                raise RuntimeError(f"YoloUNO báo lỗi: {line}")
            if line.startswith("ACK,"):
                if line != expected_ack:
                    raise RuntimeError(f"ACK không khớp lệnh MOVE: nhận {line!r}, cần {expected_ack!r}")
                if got_ack:
                    raise RuntimeError("YoloUNO gửi ACK trùng lặp.")
                got_ack = True
                continue
            if line == "DONE":
                if not got_ack:
                    raise RuntimeError("YoloUNO gửi DONE cũ trước ACK.")
                return True
            if line in KNOWN_TELEMETRY:
                continue
            # Known board builds may emit unrelated diagnostic telemetry; it
            # cannot satisfy ACK/DONE and is therefore ignored.
        stop_motor(board, best_effort=True)
    except _serial_error_types() as error:
        raise RuntimeError(f"Lỗi serial khi gửi/chờ MOVE: {error}") from error
    if not got_ack:
        raise TimeoutError("YoloUNO không ACK lệnh MOVE.")
    raise TimeoutError("YoloUNO đã ACK nhưng không trả DONE.")


# ---------- CONTROL WORKER ----------
def publish(ui_events: queue.Queue[dict[str, Any]], severity: str, message: str, **data: Any) -> None:
    ui_events.put({"kind": "message", "severity": severity, "message": message, **data})


def publish_state(ui_events: queue.Queue[dict[str, Any]], state: str) -> None:
    ui_events.put({"kind": "state", "state": state})


def _wait_or_stop(stop_event: threading.Event, seconds: float) -> bool:
    return stop_event.wait(seconds)


def control_loop(board: Any, camera_stream: CameraStream, matrix: np.ndarray, display_state: DisplayState, prompt_queue: queue.Queue[str | None], ui_events: queue.Queue[dict[str, Any]], stop_event: threading.Event, control_done: threading.Event) -> None:
    """One worker: prompt queue -> Gemini -> validated serial action indefinitely."""
    step = 0
    try:
        publish_state(ui_events, "WAITING_FOR_PROMPT")
        publish(ui_events, "INFO", "Calibration complete. Enter a target instruction to begin.")
        while not stop_event.is_set():
            try:
                instruction = prompt_queue.get(timeout=0.20)
            except queue.Empty:
                camera_stream.assert_healthy()
                continue
            if instruction is None or stop_event.is_set():
                break

            no_match_attempts = 0
            technical_attempts = 0
            active = True
            while active and not stop_event.is_set():
                camera_stream.assert_healthy()
                frame = camera_stream.read()
                if frame is None:
                    raise RuntimeError("Camera stream chưa có frame.")
                vla_image = prepare_vla_image(frame, matrix)
                step += 1
                display_state.update(step=step, status="ANALYZING", direction="STOP", clear_point=True)
                publish_state(ui_events, "ANALYZING")
                publish(ui_events, "INFO", f"Analyzing instruction (step {step}): {instruction}")
                if WRITE_DEBUG_IMAGES:
                    try:
                        cv2.imwrite(f"debug_step_{step}.jpg", vla_image)
                    except Exception as error:
                        publish(ui_events, "WARNING", f"Optional debug image was not saved: {error}")
                try:
                    action = validate_model_action(detect_object(vla_image, instruction))
                except Exception as error:
                    technical_attempts += 1
                    # This is a runtime STOP. If serial STOP fails it deliberately
                    # escapes as a fatal hardware failure instead of becoming a retry.
                    stop_motor(board)
                    if technical_attempts >= TECHNICAL_RETRY_LIMIT:
                        raise RuntimeError(f"Gemini technical retry {technical_attempts}/{TECHNICAL_RETRY_LIMIT} exhausted: {error}") from error
                    display_state.update(status="MODEL_RETRY", direction="STOP")
                    publish_state(ui_events, "MODEL_RETRY")
                    severity = "RECONNECTING" if not isinstance(error, ActionValidationError) else "ERROR"
                    publish(ui_events, severity, f"Gemini {'invalid data' if isinstance(error, ActionValidationError) else 'reconnecting'} - retry {technical_attempts}/{TECHNICAL_RETRY_LIMIT}: {error}")
                    if _wait_or_stop(stop_event, 2.0):
                        break
                    continue

                # A model call may return just as Stop/Exit is requested.  Do not
                # publish or execute that late action; run_motor repeats this check
                # under the serial lock immediately before its MOVE write.
                if stop_event.is_set():
                    break
                technical_attempts = 0
                result = action.result
                display_state.update(status=action.status, direction=action.direction, point=result.point)
                publish(ui_events, "INFO", f"Gemini: label={result.label!r}, point_yx={result.point}, confidence={result.confidence:.2f}")
                publish(ui_events, "INFO", action_tokens(action.direction, action.duration_ms, action.status))

                if action.is_no_match:
                    stop_motor(board)
                    no_match_attempts += 1
                    display_state.update(status="TARGET_MISSING", direction="STOP")
                    publish_state(ui_events, "TARGET_MISSING")
                    publish(ui_events, "WARNING", f"target not recognized - retry {no_match_attempts}/{NO_MATCH_LIMIT}")
                    if no_match_attempts >= NO_MATCH_LIMIT:
                        publish(ui_events, "WARNING", "Target could not be recovered after 20 attempts. You can submit a new instruction.")
                        active = False
                        break
                    if _wait_or_stop(stop_event, 1.0):
                        break
                    continue

                no_match_attempts = 0
                if action.status == "CENTERED":
                    stop_motor(board)
                    display_state.update(status="CENTERED", direction="STOP")
                    publish_state(ui_events, "CENTERED")
                    publish(ui_events, "SUCCESS", "Target is centered. Ready for the next instruction.")
                    active = False
                    break

                display_state.update(status="MOVING", direction=action.direction)
                publish_state(ui_events, "MOVING")
                publish(ui_events, "INFO", f"Sending firmware MOVE,{action.direction},{action.duration_ms}")
                if not run_motor(board, action.direction, action.duration_ms, stop_event):
                    break
                display_state.update(status="SETTLING", direction="STOP")
                publish_state(ui_events, "SETTLING")
                publish(ui_events, "INFO", f"Motor pulse complete; settling for {SETTLE_S:.1f}s.")
                if _wait_or_stop(stop_event, SETTLE_S):
                    break

            if not stop_event.is_set() and not active:
                display_state.update(status="WAITING_FOR_PROMPT", direction="STOP")
                publish_state(ui_events, "WAITING_FOR_PROMPT")
                ui_events.put({"kind": "prompt_ready"})
    except Exception as error:
        display_state.update(status="ERROR", direction="STOP")
        publish_state(ui_events, "ERROR")
        publish(ui_events, "ERROR", f"Fatal controller error: {error}")
        request_stop(stop_event)
        ui_events.put({"kind": "fatal"})
    finally:
        stop_motor(board, best_effort=True)
        control_done.set()


# ---------- TKINTER DASHBOARD (main thread only) ----------
class RuntimeDashboard:
    def __init__(self, camera_stream: CameraStream, matrix: np.ndarray, display_state: DisplayState, prompt_queue: queue.Queue[str | None], ui_events: queue.Queue[dict[str, Any]], on_shutdown: Any):
        import tkinter as tk
        from tkinter import scrolledtext, ttk
        from PIL import Image, ImageTk

        self.tk, self.ttk, self.Image, self.ImageTk = tk, ttk, Image, ImageTk
        self.camera_stream = camera_stream
        self.matrix = matrix
        self.display_state = display_state
        self.prompt_queue = prompt_queue
        self.ui_events = ui_events
        self.on_shutdown = on_shutdown
        self.closing = False
        self.photo: Any | None = None  # Retain PhotoImage to prevent Tk GC.

        self.root = tk.Tk()
        self.root.title("Calibrated VLA Conveyor Control")
        self.root.minsize(980, 520)
        self.root.protocol("WM_DELETE_WINDOW", self.request_shutdown)
        self.root.columnconfigure(0, weight=1)
        self.root.columnconfigure(1, weight=0)
        self.root.rowconfigure(0, weight=1)

        left = ttk.Frame(self.root, padding=10)
        left.grid(row=0, column=0, sticky="nsew")
        left.columnconfigure(0, weight=1)
        left.rowconfigure(0, weight=1)
        self.image_label = ttk.Label(left, anchor="center")
        self.image_label.grid(row=0, column=0, sticky="nsew")

        sidebar = ttk.Frame(self.root, padding=10, width=SIDEBAR_WIDTH)
        sidebar.grid(row=0, column=1, sticky="ns")
        sidebar.grid_propagate(False)
        sidebar.columnconfigure(0, weight=1)
        sidebar.rowconfigure(1, weight=1)
        self.state_var = tk.StringVar(value="STATE: STARTING")
        ttk.Label(sidebar, textvariable=self.state_var, font=("TkDefaultFont", 10, "bold")).grid(row=0, column=0, sticky="ew", pady=(0, 8))
        self.transcript = scrolledtext.ScrolledText(sidebar, wrap="word", state="disabled", height=24, font=("TkFixedFont", 10))
        self.transcript.grid(row=1, column=0, sticky="nsew")
        for tag, color in {"USER": "#dbeafe", "INFO": "#1d4ed8", "SUCCESS": "#15803d", "WARNING": "#b45309", "RECONNECTING": "#0369a1", "ERROR": "#b91c1c"}.items():
            self.transcript.tag_configure(tag, foreground=color)
        ttk.Label(sidebar, text="Target instruction").grid(row=2, column=0, sticky="w", pady=(8, 2))
        self.composer = tk.Text(sidebar, height=4, wrap="word")
        self.composer.grid(row=3, column=0, sticky="ew")
        buttons = ttk.Frame(sidebar)
        buttons.grid(row=4, column=0, sticky="ew", pady=(6, 0))
        buttons.columnconfigure(0, weight=1)
        self.send_button = ttk.Button(buttons, text="Send", command=self.send_prompt)
        self.send_button.grid(row=0, column=0, sticky="ew", padx=(0, 6))
        ttk.Button(buttons, text="Stop / Exit", command=self.request_shutdown).grid(row=0, column=1, sticky="e")
        self.composer.bind("<Control-Return>", lambda _event: self.send_prompt())

    def _append(self, severity: str, message: str) -> None:
        self.transcript.configure(state="normal")
        self.transcript.insert("end", f"{severity}: {message}\n", severity)
        self.transcript.configure(state="disabled")
        self.transcript.see("end")

    def _set_composer_enabled(self, enabled: bool) -> None:
        state = "normal" if enabled and not self.closing else "disabled"
        self.composer.configure(state=state)
        self.send_button.configure(state=state)
        if enabled and not self.closing:
            self.composer.focus_set()

    def send_prompt(self) -> None:
        if self.closing:
            return
        instruction = self.composer.get("1.0", "end-1c").strip()
        if not instruction:
            self._append("WARNING", "Enter a non-blank target instruction first.")
            return
        self.composer.delete("1.0", "end")
        self._append("USER", instruction)
        self._set_composer_enabled(False)
        self.prompt_queue.put(instruction)

    def request_shutdown(self) -> None:
        if self.closing:
            return
        self.closing = True
        self._set_composer_enabled(False)
        self.state_var.set("STATE: STOPPING")
        self.on_shutdown()
        self.root.after(100, self.root.destroy)

    def _drain_events(self) -> None:
        try:
            while True:
                event = self.ui_events.get_nowait()
                if event["kind"] == "message":
                    self._append(event["severity"], event["message"])
                elif event["kind"] == "state":
                    self.state_var.set(f"STATE: {event['state']}")
                elif event["kind"] == "prompt_ready":
                    self._set_composer_enabled(True)
                elif event["kind"] == "fatal":
                    self.root.after(200, self.request_shutdown)
        except queue.Empty:
            pass
        if not self.closing:
            self.root.after(50, self._drain_events)

    def _refresh_image(self) -> None:
        if not self.closing:
            frame = self.camera_stream.read()
            if frame is not None:
                calibrated = prepare_vla_image(frame, self.matrix)
                visible = annotate_vla_frame(calibrated, self.display_state.snapshot())
                rgb = cv2.cvtColor(visible, cv2.COLOR_BGR2RGB)
                image = self.Image.fromarray(rgb)
                available_w = max(self.image_label.winfo_width(), 1)
                available_h = max(self.image_label.winfo_height(), 1)
                scale = min(available_w / WARP_WIDTH, available_h / WARP_HEIGHT)
                if scale > 0:
                    image = image.resize((max(1, round(WARP_WIDTH * scale)), max(1, round(WARP_HEIGHT * scale))), self.Image.Resampling.LANCZOS)
                self.photo = self.ImageTk.PhotoImage(image=image)
                self.image_label.configure(image=self.photo)
            self.root.after(33, self._refresh_image)

    def run(self) -> None:
        self._drain_events()
        self._refresh_image()
        self.root.mainloop()


def main() -> None:
    board: Any | None = None
    camera: cv2.VideoCapture | None = None
    camera_stream: CameraStream | None = None
    worker: threading.Thread | None = None
    stop_event = threading.Event()
    control_done = threading.Event()
    prompt_queue: queue.Queue[str | None] = queue.Queue()
    ui_events: queue.Queue[dict[str, Any]] = queue.Queue()

    def request_shutdown() -> None:
        request_stop(stop_event)
        try:
            prompt_queue.put_nowait(None)
        except queue.Full:  # unbounded queue, retained for future replacement.
            pass

    try:
        board = open_yolouno()
        camera = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
        if not camera.isOpened():
            raise RuntimeError("Không mở được camera.")
        camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        matrix = calibrate(camera)
        if matrix is None:
            return
        camera_stream = CameraStream(camera).start()
        display_state = DisplayState()
        worker = threading.Thread(target=control_loop, args=(board, camera_stream, matrix, display_state, prompt_queue, ui_events, stop_event, control_done), name="vla-control", daemon=True)
        dashboard = RuntimeDashboard(camera_stream, matrix, display_state, prompt_queue, ui_events, request_shutdown)
        worker.start()
        dashboard.run()
    finally:
        request_shutdown()
        if worker is not None:
            worker.join(timeout=(GEMINI_TIMEOUT_MS / 1000.0) + 5.0)
        # Do not close the serial port while a still-running worker can use it.
        if board is not None and (worker is None or not worker.is_alive()):
            stop_motor(board, best_effort=True)
            try:
                board.close()
            except _serial_error_types():
                pass
        elif worker is not None and worker.is_alive():
            print("Worker did not exit before shutdown; serial is left open to avoid a concurrent close.")
        if camera_stream is not None:
            camera_stream.stop()
        elif camera is not None:
            camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

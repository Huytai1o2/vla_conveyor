"""Calibrated Gemini VLA controller for the YoloUNO conveyor.

OpenCV owns camera I/O, calibration/warping, and display overlays only. Gemini
owns object selection and the complete action; this module only validates that
action and adapts it to the serial protocol implemented by conveyor_firmware.
"""

from __future__ import annotations

import os
import queue
import threading
import time
import traceback
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
CAMERA_DEVICE = os.getenv("CAMERA_DEVICE", "auto").strip() or "auto"
CAMERA_MATCH = os.getenv("CAMERA_MATCH", "UGREEN").strip()
SERIAL_PORT = os.getenv("SERIAL_PORT", "auto").strip() or "auto"
SERIAL_MATCH = os.getenv("SERIAL_MATCH", "Espressif").strip()
SERIAL_BAUD = int(os.getenv("SERIAL_BAUD", "115200"))
MODEL_NAME = "gemini-robotics-er-1.6-preview"
GEMINI_TIMEOUT_MS = 15_000

BELT_LENGTH_CM = 77.0
SPEED_TEST_DISTANCE_CM = 77.0
SPEED_TEST_TIME_S = 5.5925
BELT_SPEED_CM_S = SPEED_TEST_DISTANCE_CM / SPEED_TEST_TIME_S
TARGET_TOLERANCE_CM = 4.0
MOVE_GAIN = 0.8
MIN_MOVE_MS = 80
MAX_MOVE_MS = 1500
NEAR_TARGET_CM = 8.0
NEAR_TARGET_MAX_MS = 180
DEFAULT_LEFT_X = 200
DEFAULT_CENTER_X = 500
DEFAULT_RIGHT_X = 800
MAX_WAYPOINTS = 12
MIN_CONFIDENCE = 0.60
SETTLE_S = 1.5
NO_MATCH_LIMIT = 20
TECHNICAL_RETRY_LIMIT = 5
CAMERA_STALE_S = 3.0
STOP_REPLY_TIMEOUT_S = 2.0
WARP_WIDTH = 1000
WARP_HEIGHT = 300
SIDEBAR_WIDTH = 580
MIN_DASHBOARD_WIDTH = 1100
MIN_DASHBOARD_HEIGHT = 760
WINDOW_SCREEN_MARGIN = 80
GUI_FONT_SIZE = 17
GUI_MONOSPACE_FONT_SIZE = 16
GUI_HEADING_FONT_SIZE = 20
GUI_MIN_SCALE = 0.90
GUI_MAX_SCALE = 1.50
GUI_RESIZE_DEBOUNCE_MS = 100
WRITE_DEBUG_IMAGES = False


class VLAResult(BaseModel):
    """The exact structured output requested from Gemini."""

    model_config = ConfigDict(extra="forbid", strict=True)

    target_found: StrictBool
    target_matches_prompt: StrictBool
    label: StrictStr | None = None
    point: list[StrictInt] | None = None  # normalized [y, x]
    instruction_valid: StrictBool
    waypoints_x: list[StrictInt]
    active_waypoint_index: StrictInt | None = None
    direction: Literal["LEFT", "RIGHT", "STOP"]
    duration_ms: StrictInt
    task_status: Literal["MOVE", "AT_TARGET", "TARGET_NOT_FOUND", "INVALID_INSTRUCTION"]
    confidence: StrictFloat


class ActionValidationError(ValueError):
    """Gemini produced JSON that is syntactically valid but unsafe to execute."""


@dataclass(frozen=True)
class ValidatedAction:
    result: VLAResult
    direction: Literal["LEFT", "RIGHT", "STOP"]
    duration_ms: int
    status: Literal["MOVE", "AT_TARGET", "TARGET_NOT_FOUND", "INVALID_INSTRUCTION"]
    destination_x: int | None
    is_no_match: bool
    is_invalid_instruction: bool
    guarded_stop: bool = False
    position_confirmed: bool = False
    alignment_normalized: bool = False


def gemini_response_schema() -> Any:
    """Return the API-facing OpenAPI subset without Pydantic-only keywords.

    ``VLAResult`` deliberately uses ``extra="forbid"`` for local safety. Passing
    that model directly as ``response_schema`` makes Pydantic emit
    ``additionalProperties: false``; the Robotics endpoint rejects the converted
    ``additional_properties`` field. Keep the wire schema minimal and perform
    the stricter validation locally after the response arrives.
    """
    if types is None:
        raise RuntimeError("Thiếu package google-genai. Hãy cài requirements.txt.")
    return types.Schema(
        type=types.Type.OBJECT,
        properties={
            "target_found": types.Schema(type=types.Type.BOOLEAN),
            "target_matches_prompt": types.Schema(type=types.Type.BOOLEAN),
            "label": types.Schema(type=types.Type.STRING, nullable=True),
            "point": types.Schema(
                type=types.Type.ARRAY,
                items=types.Schema(type=types.Type.INTEGER),
                min_items=2,
                max_items=2,
                nullable=True,
            ),
            "instruction_valid": types.Schema(type=types.Type.BOOLEAN),
            "waypoints_x": types.Schema(
                type=types.Type.ARRAY,
                items=types.Schema(type=types.Type.INTEGER),
                min_items=0,
                max_items=MAX_WAYPOINTS,
            ),
            "active_waypoint_index": types.Schema(
                type=types.Type.INTEGER,
                nullable=True,
            ),
            "direction": types.Schema(
                type=types.Type.STRING,
                enum=["LEFT", "RIGHT", "STOP"],
            ),
            "duration_ms": types.Schema(type=types.Type.INTEGER),
            "task_status": types.Schema(
                type=types.Type.STRING,
                enum=["MOVE", "AT_TARGET", "TARGET_NOT_FOUND", "INVALID_INSTRUCTION"],
            ),
            "confidence": types.Schema(type=types.Type.NUMBER),
        },
        required=[
            "target_found",
            "target_matches_prompt",
            "instruction_valid",
            "waypoints_x",
            "active_waypoint_index",
            "direction",
            "duration_ms",
            "task_status",
            "confidence",
        ],
    )


def action_tokens(direction: str, duration_ms: int, status: str) -> str:
    """Visible, auditable representation of Gemini's validated action."""
    return f"[ACT_{direction}] [DURATION_{duration_ms:04d}_MS] [STATUS_{status}]"


def _valid_point(point: list[int] | None) -> bool:
    return (
        isinstance(point, list)
        and len(point) == 2
        and all(type(value) is int and 0 <= value <= 1000 for value in point)
    )


def _valid_waypoints(waypoints_x: list[int]) -> bool:
    return (
        isinstance(waypoints_x, list)
        and 1 <= len(waypoints_x) <= MAX_WAYPOINTS
        and all(type(value) is int and 0 <= value <= 1000 for value in waypoints_x)
    )


def validate_model_action(
    raw: VLAResult | dict[str, Any],
    expected_waypoints: list[int] | None = None,
    expected_waypoint_index: int = 0,
) -> ValidatedAction:
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

    if not result.instruction_valid:
        if (
            result.waypoints_x
            or result.active_waypoint_index is not None
            or result.task_status != "INVALID_INSTRUCTION"
            or result.direction != "STOP"
            or result.duration_ms != 0
            or result.target_found
            or result.target_matches_prompt
            or result.label is not None
            or result.point is not None
        ):
            raise ActionValidationError(
                "invalid instruction must use empty waypoints, null active index, "
                "INVALID_INSTRUCTION/STOP/0, and no detected target"
            )
        return ValidatedAction(result, "STOP", 0, "INVALID_INSTRUCTION", None, False, True)

    if not _valid_waypoints(result.waypoints_x):
        raise ActionValidationError(
            f"a valid instruction requires 1..{MAX_WAYPOINTS} waypoint x values in 0..1000"
        )
    if type(result.active_waypoint_index) is not int or not 0 <= result.active_waypoint_index < len(result.waypoints_x):
        raise ActionValidationError("active_waypoint_index is outside waypoints_x")
    if expected_waypoints is not None and result.waypoints_x != expected_waypoints:
        raise ActionValidationError("Gemini changed the locked waypoint sequence")
    if result.active_waypoint_index != expected_waypoint_index:
        raise ActionValidationError(
            f"Gemini returned waypoint index {result.active_waypoint_index}; expected {expected_waypoint_index}"
        )
    destination_x = result.waypoints_x[result.active_waypoint_index]

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
        return ValidatedAction(result, "STOP", 0, "TARGET_NOT_FOUND", destination_x, True, False)

    # From here on the target is a confident prompt match and needs an audit point.
    if not _valid_point(result.point):
        raise ActionValidationError("a matching target requires a valid point")
    object_x = result.point[1]
    offset_cm = abs(object_x - destination_x) * BELT_LENGTH_CM / 1000.0
    within_target_tolerance = offset_cm <= TARGET_TOLERANCE_CM
    if result.task_status not in ("MOVE", "AT_TARGET"):
        raise ActionValidationError(
            "confident matching target requires MOVE or AT_TARGET"
        )

    # Waypoint validation comes before interpreting the model's action tuple.
    # Gemini sometimes reports AT_TARGET while leaving a stale direction or
    # duration in the same response. Once its reported object point is inside
    # tolerance, STOP is the only safe command: advance an intermediate waypoint
    # or finish under the final guard.
    if within_target_tolerance:
        is_final_waypoint = result.active_waypoint_index == len(result.waypoints_x) - 1
        response_was_contradictory = (
            result.task_status != "AT_TARGET"
            or result.direction != "STOP"
            or result.duration_ms != 0
        )
        return ValidatedAction(
            result,
            "STOP",
            0,
            "AT_TARGET",
            destination_x,
            False,
            False,
            is_final_waypoint and response_was_contradictory,
            response_was_contradictory,
        )

    # Validation=false: align toward the current waypoint. Accept a stale
    # AT_TARGET label only when Gemini also supplied a complete, safe alignment
    # action; the normalized executable status remains MOVE.
    if result.direction not in ("LEFT", "RIGHT"):
        raise ActionValidationError(
            "waypoint is not reached; alignment requires LEFT or RIGHT"
        )
    if not MIN_MOVE_MS <= result.duration_ms <= MAX_MOVE_MS:
        raise ActionValidationError(
            f"alignment duration must be {MIN_MOVE_MS}..{MAX_MOVE_MS} ms"
        )
    expected_direction = "RIGHT" if object_x < destination_x else "LEFT"
    if result.direction != expected_direction:
        raise ActionValidationError(
            f"alignment direction {result.direction} does not approach destination_x={destination_x}"
        )
    return ValidatedAction(
        result,
        result.direction,
        result.duration_ms,
        "MOVE",
        destination_x,
        False,
        False,
        False,
        False,
        result.task_status == "AT_TARGET",
    )


def build_prompt(
    instruction: str,
    locked_waypoints: list[int] | None = None,
    active_waypoint_index: int = 0,
) -> str:
    """Build the per-inference policy, including the active GUI instruction."""
    if locked_waypoints is None:
        sequence_context = (
            "No waypoint sequence is locked yet. Parse the full ordered destination "
            "sequence from the operator instruction and set active_waypoint_index=0."
        )
    else:
        sequence_context = (
            f"LOCKED WAYPOINT SEQUENCE: {locked_waypoints}; "
            f"ACTIVE WAYPOINT INDEX: {active_waypoint_index}. Return this exact sequence "
            "and index; do not restart, reorder, add, remove, or reinterpret waypoints."
        )
    return f"""
You are the high-level Vision-Language-Action controller of a reversible conveyor.

ACTIVE OPERATOR INSTRUCTION: {instruction!r}
CONTROLLER SEQUENCE STATE: {sequence_context}

Inspect the supplied calibrated overhead conveyor image. Identify only one loose,
movable object resting on the exposed conveyor surface whose appearance matches the
active operator instruction. Do not substitute another loose object. Ignore belt
slats, rails, markers, structures, motors, wires, electronics, people, tables, and
background.

Interpret the instruction as one object plus one or more ordered destination
positions along the conveyor x-axis. The operator writes unrestricted natural
language, not a command template. Understand paraphrases, references, and temporal
connectors such as "first", "after that", "then", "finally", "return", and their
equivalents in the language used by the operator. Do not require literal words such
as "left", "right", or "center" and do not treat the example below as required
syntax.

Build the complete ordered plan in waypoints_x using normalized x coordinates
0..1000. Preserve repetitions and order. A single destination is a one-element
list. A destination may be an absolute/relative belt position, percentage,
fraction, distance from the physical left edge, the object's initial position, or
an unambiguous visible landmark referenced by the operator. Use the supplied image
when resolving visual or initial-position references. Exact positions, percentages,
fractions, or centimeters override qualitative defaults.
Use these defaults only when the operator gives a qualitative position:
- left = {DEFAULT_LEFT_X}; center/middle = {DEFAULT_CENTER_X}; right = {DEFAULT_RIGHT_X};
- far left/left edge = 0; far right/right edge = 1000.
Non-template example: "move the white charger left, then right, then back to the middle"
means waypoints_x=[{DEFAULT_LEFT_X}, {DEFAULT_RIGHT_X}, {DEFAULT_CENTER_X}].
If the object or ordered destinations cannot be identified unambiguously, never
invent them. For a missing/ambiguous destination return instruction_valid=false,
empty waypoints_x, null active_waypoint_index, INVALID_INSTRUCTION/STOP/0, with no
target data. A valid sequence contains at most {MAX_WAYPOINTS} waypoints.

Calibration and controller policy (you must calculate the action yourself):
- normalized image coordinates are [y, x], each in 0..1000;
- active destination_x = waypoints_x[active_waypoint_index];
- belt length = {BELT_LENGTH_CM:g} cm for 1000 normalized x units;
- destination tolerance = {TARGET_TOLERANCE_CM:g} cm;
- measured belt speed = {BELT_SPEED_CM_S:.4f} cm/s ({SPEED_TEST_DISTANCE_CM:g} / {SPEED_TEST_TIME_S:g});
- move gain = {MOVE_GAIN:g}; valid MOVE duration = {MIN_MOVE_MS}..{MAX_MOVE_MS} ms;
- near-target threshold = {NEAR_TARGET_CM:g} cm; near-target pulse maximum = {NEAR_TARGET_MAX_MS} ms.

For a matching target calculate:
offset_cm = abs(object_x - destination_x) * {BELT_LENGTH_CM:g} / 1000
move_distance_cm = max(0, offset_cm - {TARGET_TOLERANCE_CM:g})
duration_ms = clamp(move_distance_cm / {BELT_SPEED_CM_S:.4f} * {MOVE_GAIN:g} * 1000,
                    {MIN_MOVE_MS}, {MAX_MOVE_MS})
if offset_cm <= {NEAR_TARGET_CM:g}: duration_ms = min(duration_ms, {NEAR_TARGET_MAX_MS})
Image-relative direction: object_x < destination_x requires RIGHT; object_x >
destination_x requires LEFT. If it is within tolerance, return AT_TARGET/STOP/0.
For an intermediate waypoint, AT_TARGET means validation passed and the controller
will advance to the next locked waypoint. For the final waypoint, the controller's
final guard blocks further movement inside tolerance; AT_TARGET then ends the task.

Return JSON only with target_found, target_matches_prompt, label, point,
instruction_valid, waypoints_x, active_waypoint_index, direction, duration_ms,
task_status, confidence. point is the current object [y, x] integer coordinate. Use:
- MOVE only for a matching target with confidence >= {MIN_CONFIDENCE:.2f}, LEFT or RIGHT,
  and a duration in range.
- AT_TARGET only for a matching target within destination tolerance, with STOP and duration_ms 0.
- TARGET_NOT_FOUND with STOP and duration_ms 0 whenever the target is absent,
  does not match the instruction, or confidence is below {MIN_CONFIDENCE:.2f}.
- INVALID_INSTRUCTION only for an ambiguous/missing destination sequence.
""".strip()


# ---------- CAMERA / CALIBRATION ----------
def _unique_devices(devices: list[int | str]) -> list[int | str]:
    """Deduplicate device nodes while preserving discovery priority."""
    unique: list[int | str] = []
    identities: set[str] = set()
    for device in devices:
        identity = f"index:{device}" if isinstance(device, int) else os.path.realpath(device)
        if identity not in identities:
            identities.add(identity)
            unique.append(device)
    return unique


def _video_node_name(device: Path) -> str:
    """Read the Linux V4L2 product name associated with a /dev/video node."""
    resolved = Path(os.path.realpath(device))
    name_file = Path("/sys/class/video4linux") / resolved.name / "name"
    try:
        return name_file.read_text(encoding="utf-8").strip()
    except OSError:
        return ""


def camera_candidates() -> list[int | str]:
    """Return explicit or auto-discovered camera devices in preferred order."""
    requested = CAMERA_DEVICE
    if requested.casefold() != "auto":
        return [int(requested)] if requested.isdecimal() else [requested]

    if os.name == "nt":
        return list(range(5))

    match = CAMERA_MATCH.casefold()
    matched: list[int | str] = []
    fallback: list[int | str] = []

    by_id = Path("/dev/v4l/by-id")
    if by_id.is_dir():
        for path in sorted(by_id.glob("*video-index0")):
            description = f"{path.name} {_video_node_name(path)}".casefold()
            (matched if match and match in description else fallback).append(str(path))

    for path in sorted(Path("/dev").glob("video*")):
        description = _video_node_name(path).casefold()
        (matched if match and match in description else fallback).append(str(path))

    candidates = _unique_devices(matched if matched else fallback)
    if not candidates:
        raise RuntimeError(
            "Không tìm thấy camera. Kiểm tra CAMERA_DEVICE/CAMERA_MATCH và quyền đọc /dev/video*."
        )
    return candidates


def open_camera() -> cv2.VideoCapture:
    """Open the configured camera without forcing a Windows backend on Linux."""
    attempts: list[str] = []
    for device in camera_candidates():
        if os.name == "nt":
            backend = cv2.CAP_DSHOW if isinstance(device, int) else cv2.CAP_ANY
        else:
            backend = cv2.CAP_V4L2

        camera = cv2.VideoCapture(device, backend)
        if camera.isOpened():
            camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            deadline = time.monotonic() + 1.5
            while time.monotonic() < deadline:
                ok, frame = camera.read()
                if ok and frame is not None:
                    print(f"Kết nối camera thành công: {device}")
                    return camera
                time.sleep(0.05)
        attempts.append(str(device))
        camera.release()

    raise RuntimeError(
        "Không mở được camera từ các candidate: " + ", ".join(attempts)
    )


CALIBRATION_CLICK_LABELS = ("TL", "BL", "TR", "BR")


def calibration_source_corners(points: list[tuple[int, int]]) -> np.ndarray:
    """Convert semantic click order TL, BL, TR, BR to TL, TR, BR, BL.

    The operator, rather than screen geometry, defines the physical conveyor
    orientation. This preserves left/right semantics when the camera is mounted
    at an arbitrary angle or the raw view appears inverted.
    """
    if len(points) != 4:
        raise ValueError("Calibration requires exactly four points.")
    clicked = np.asarray(points, dtype=np.float32)
    return clicked[[0, 2, 3, 1]]


def fit_image_to_view(
    image_width: int,
    image_height: int,
    view_width: int,
    view_height: int,
) -> tuple[float, int, int, int, int]:
    """Return scale, displayed size, and centered origin for a Tk canvas."""
    if min(image_width, image_height, view_width, view_height) <= 0:
        raise ValueError("image and view dimensions must be positive")
    scale = min(view_width / image_width, view_height / image_height)
    display_width = max(1, round(image_width * scale))
    display_height = max(1, round(image_height * scale))
    origin_x = (view_width - display_width) // 2
    origin_y = (view_height - display_height) // 2
    return scale, display_width, display_height, origin_x, origin_y


def view_to_image_point(
    view_x: int,
    view_y: int,
    *,
    scale: float,
    origin_x: int,
    origin_y: int,
    image_width: int,
    image_height: int,
) -> tuple[int, int] | None:
    """Map a Tk canvas click into source-image coordinates after fit/centering."""
    if scale <= 0:
        return None
    image_x = round((view_x - origin_x) / scale)
    image_y = round((view_y - origin_y) / scale)
    if not 0 <= image_x < image_width or not 0 <= image_y < image_height:
        return None
    return image_x, image_y


def calibration_is_valid(points: list[tuple[int, int]]) -> bool:
    if len(points) != 4:
        return False
    polygon = calibration_source_corners(points).astype(np.int32).reshape((-1, 1, 2))
    return bool(cv2.isContourConvex(polygon) and cv2.contourArea(polygon) > 2000)


def calibration_matrix(points: list[tuple[int, int]]) -> np.ndarray:
    if not calibration_is_valid(points):
        raise ValueError("Calibration points are incomplete, crossed, or too close together.")
    destination = np.float32(
        [
            [0, 0],
            [WARP_WIDTH - 1, 0],
            [WARP_WIDTH - 1, WARP_HEIGHT - 1],
            [0, WARP_HEIGHT - 1],
        ]
    )
    return cv2.getPerspectiveTransform(calibration_source_corners(points), destination)


def prepare_vla_image(frame: np.ndarray, matrix: np.ndarray) -> np.ndarray:
    """Produce the unannotated calibrated inference image at control resolution."""
    return cv2.warpPerspective(frame, matrix, (WARP_WIDTH, WARP_HEIGHT))


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
        self.destination_x: int | None = None

    def update(self, *, step: int | None = None, status: str | None = None, direction: str | None = None, point: list[int] | None = None, destination_x: int | None = None, clear_point: bool = False, clear_destination: bool = False) -> None:
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
            if clear_destination:
                self.destination_x = None
            elif destination_x is not None:
                self.destination_x = destination_x

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "step": self.step,
                "status": self.status,
                "direction": self.direction,
                "point": None if self.point is None else list(self.point),
                "destination_x": self.destination_x,
            }


def annotate_vla_frame(image: np.ndarray, state: dict[str, Any]) -> np.ndarray:
    """Display-only annotation; it is never passed back to Gemini or control logic."""
    output = image.copy()
    destination_x = state["destination_x"]
    if type(destination_x) is int and 0 <= destination_x <= 1000:
        target_px = round(destination_x / 1000.0 * (WARP_WIDTH - 1))
        tolerance_px = round(TARGET_TOLERANCE_CM / BELT_LENGTH_CM * WARP_WIDTH)
        for x in (
            max(0, target_px - tolerance_px),
            min(WARP_WIDTH - 1, target_px + tolerance_px),
        ):
            cv2.line(output, (x, 38), (x, WARP_HEIGHT - 1), (0, 255, 255), 2)
        cv2.putText(
            output,
            f"Requested target x={destination_x}",
            (max(8, min(target_px + 8, WARP_WIDTH - 245)), 62),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            2,
        )
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


def detect_object(
    image: np.ndarray,
    instruction: str,
    gemini_client: Any | None = None,
    locked_waypoints: list[int] | None = None,
    active_waypoint_index: int = 0,
) -> VLAResult:
    """Ask Gemini for an action. No CV perception/action fallback exists here."""
    if types is None:
        raise RuntimeError("Thiếu package google-genai. Hãy cài requirements.txt.")
    ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 90])
    if not ok:
        raise RuntimeError("Không encode được ảnh.")
    response = (gemini_client or get_gemini_client()).models.generate_content(
        model=MODEL_NAME,
        contents=[
            types.Part.from_bytes(data=encoded.tobytes(), mime_type="image/jpeg"),
            build_prompt(instruction, locked_waypoints, active_waypoint_index),
        ],
        config=types.GenerateContentConfig(
            temperature=0.2,
            thinking_config=types.ThinkingConfig(thinking_budget=0),
            response_mime_type="application/json",
            response_schema=gemini_response_schema(),
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


def serial_candidates() -> list[str]:
    """Return an explicit port or stable Linux MCU paths, then safe fallbacks."""
    if SERIAL_PORT.casefold() != "auto":
        return [SERIAL_PORT]

    match = SERIAL_MATCH.casefold()
    matched: list[str] = []
    fallback: list[str] = []
    by_id = Path("/dev/serial/by-id")
    if by_id.is_dir():
        for path in sorted(by_id.iterdir()):
            (matched if match and match in path.name.casefold() else fallback).append(str(path))

    if os.name == "nt":
        try:
            from serial.tools import list_ports

            for port in list_ports.comports():
                description = f"{port.device} {port.description} {port.manufacturer}".casefold()
                (matched if match and match in description else fallback).append(port.device)
        except ImportError:
            pass
    else:
        for pattern in ("ttyACM*", "ttyUSB*"):
            fallback.extend(str(path) for path in sorted(Path("/dev").glob(pattern)))

    candidates = [str(device) for device in _unique_devices(matched if matched else fallback)]
    if not candidates:
        raise RuntimeError(
            "Không tìm thấy serial MCU. Kiểm tra SERIAL_PORT/SERIAL_MATCH và quyền truy cập thiết bị."
        )
    return candidates


def open_yolouno() -> Any:
    errors: list[str] = []
    for port in serial_candidates():
        try:
            board = serial.Serial(port, SERIAL_BAUD, timeout=0.25, write_timeout=1.0)
        except _serial_error_types() as error:
            errors.append(f"{port}: {error}")
            continue

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
                        print(f"Kết nối YoloUNO thành công: {port}\n")
                        return board
                time.sleep(0.4)
            errors.append(f"{port}: không nhận được PONG")
        except _serial_error_types() as error:
            errors.append(f"{port}: {error}")
        if board.is_open:
            board.close()

    raise RuntimeError(
        "Không kết nối được YoloUNO. Đóng PlatformIO Monitor và kiểm tra firmware. "
        + " | ".join(errors)
    )


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
    # Keep the terminal useful during a GUI run. In particular, Gemini API and
    # schema errors must remain visible if the dashboard closes unexpectedly.
    print(f"[{severity}] {message}", flush=True)
    ui_events.put({"kind": "message", "severity": severity, "message": message, **data})


def publish_state(ui_events: queue.Queue[dict[str, Any]], state: str) -> None:
    ui_events.put({"kind": "state", "state": state})


def _wait_or_stop(stop_event: threading.Event, seconds: float) -> bool:
    return stop_event.wait(seconds)


def _is_non_retryable_gemini_error(error: BaseException) -> bool:
    """Treat deterministic client/request errors as fatal, except 408/429."""
    code = getattr(error, "code", None)
    return type(code) is int and 400 <= code < 500 and code not in (408, 429)


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
            locked_waypoints: list[int] | None = None
            waypoint_index = 0
            active = True
            display_state.update(
                status="ANALYZING",
                direction="STOP",
                clear_point=True,
                clear_destination=True,
            )
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
                    result = detect_object(
                        vla_image,
                        instruction,
                        locked_waypoints=locked_waypoints,
                        active_waypoint_index=waypoint_index,
                    )
                    action = validate_model_action(
                        result,
                        expected_waypoints=locked_waypoints,
                        expected_waypoint_index=waypoint_index,
                    )
                except Exception as error:
                    technical_attempts += 1
                    print(
                        f"[GEMINI ERROR] attempt "
                        f"{technical_attempts}/{TECHNICAL_RETRY_LIMIT}: "
                        f"{type(error).__name__}: {error}",
                        flush=True,
                    )
                    traceback.print_exception(type(error), error, error.__traceback__)
                    # This is a runtime STOP. If serial STOP fails it deliberately
                    # escapes as a fatal hardware failure instead of becoming a retry.
                    stop_motor(board)
                    if _is_non_retryable_gemini_error(error):
                        display_state.update(status="ERROR", direction="STOP")
                        publish_state(ui_events, "ERROR")
                        publish(
                            ui_events,
                            "ERROR",
                            f"Gemini rejected the request (HTTP {error.code}); "
                            f"retry disabled because the request must be fixed: {error}",
                        )
                        raise RuntimeError(
                            f"Gemini request rejected with non-retryable HTTP {error.code}: {error}"
                        ) from error
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
                if action.is_invalid_instruction:
                    stop_motor(board)
                    display_state.update(
                        status="INVALID_INSTRUCTION",
                        direction="STOP",
                        clear_point=True,
                        clear_destination=True,
                    )
                    publish_state(ui_events, "INVALID_INSTRUCTION")
                    publish(
                        ui_events,
                        "WARNING",
                        "Describe the object and its ordered destination(s) in natural "
                        "language; Gemini could not build an unambiguous waypoint plan.",
                    )
                    active = False
                    break

                if locked_waypoints is None:
                    locked_waypoints = list(result.waypoints_x)
                    publish(ui_events, "INFO", f"Locked waypoint sequence: {locked_waypoints}")
                    if len(locked_waypoints) == 1:
                        publish(
                            ui_events,
                            "INFO",
                            f"Final waypoint guard armed at x={locked_waypoints[0]}.",
                        )

                display_state.update(
                    status=action.status,
                    direction=action.direction,
                    point=result.point,
                    destination_x=action.destination_x,
                )
                if action.position_confirmed and not action.guarded_stop:
                    publish(
                        ui_events,
                        "INFO",
                        f"Waypoint validation passed from Gemini point: object is within "
                        f"{TARGET_TOLERANCE_CM:g} cm of waypoint "
                        f"{waypoint_index + 1}/{len(locked_waypoints)}.",
                    )
                elif action.guarded_stop:
                    publish(
                        ui_events,
                        "WARNING",
                        f"Final waypoint guard replaced contradictory Gemini motion with STOP: object is "
                        f"within {TARGET_TOLERANCE_CM:g} cm of final waypoint "
                        f"x={action.destination_x}.",
                    )
                elif action.alignment_normalized:
                    publish(
                        ui_events,
                        "WARNING",
                        "Gemini labeled the waypoint AT_TARGET while its point was "
                        "outside tolerance; normalized the validated direction/duration "
                        "to an alignment MOVE.",
                    )
                publish(
                    ui_events,
                    "INFO",
                    f"Gemini: label={result.label!r}, point_yx={result.point}, "
                    f"waypoint={waypoint_index + 1}/{len(locked_waypoints)} "
                    f"x={action.destination_x}, confidence={result.confidence:.2f}, "
                    f"raw_action={result.direction}/{result.duration_ms}/{result.task_status}",
                )
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
                if action.status == "AT_TARGET":
                    stop_motor(board)
                    completed_waypoint = waypoint_index + 1
                    if completed_waypoint < len(locked_waypoints):
                        waypoint_index = completed_waypoint
                        next_destination_x = locked_waypoints[waypoint_index]
                        display_state.update(
                            status="WAYPOINT_REACHED",
                            direction="STOP",
                            destination_x=next_destination_x,
                        )
                        publish_state(ui_events, "WAYPOINT_REACHED")
                        publish(
                            ui_events,
                            "SUCCESS",
                            f"Waypoint {completed_waypoint}/{len(locked_waypoints)} reached; "
                            f"continuing to x={next_destination_x}.",
                        )
                        if waypoint_index == len(locked_waypoints) - 1:
                            publish(
                                ui_events,
                                "INFO",
                                f"Final waypoint guard armed at x={next_destination_x}.",
                            )
                        continue
                    display_state.update(status="AT_TARGET", direction="STOP")
                    publish_state(ui_events, "AT_TARGET")
                    publish(
                        ui_events,
                        "SUCCESS",
                        f"All {len(locked_waypoints)} waypoint(s) reached. "
                        "Ready for the next instruction.",
                    )
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
def ensure_dashboard_dependencies() -> None:
    """Fail before opening hardware when the desktop GUI dependencies are absent."""
    try:
        import tkinter  # noqa: F401
    except ImportError as error:
        raise RuntimeError(
            "Thiếu Tkinter. Trên Ubuntu/Debian hãy cài package python3-tk."
        ) from error

    try:
        from PIL import Image, ImageTk  # noqa: F401
    except ImportError as error:
        raise RuntimeError(
            "Thiếu Pillow cho dashboard. Chạy: pip install -r requirements.txt"
        ) from error


class RuntimeDashboard:
    """Single-window calibration and runtime dashboard, owned by the main thread."""

    CALIBRATION_LIVE = "CALIBRATION_LIVE"
    CALIBRATION_SELECTING = "CALIBRATION_SELECTING"
    RUNTIME = "RUNTIME"
    ERROR = "ERROR"

    def __init__(
        self,
        camera_stream: CameraStream,
        display_state: DisplayState,
        prompt_queue: queue.Queue[str | None],
        ui_events: queue.Queue[dict[str, Any]],
        on_calibrated: Any,
        on_shutdown: Any,
    ):
        import tkinter as tk
        from tkinter import font as tkfont
        from tkinter import scrolledtext, ttk
        from PIL import Image, ImageTk

        self.tk, self.ttk, self.Image, self.ImageTk = tk, ttk, Image, ImageTk
        self.camera_stream = camera_stream
        self.display_state = display_state
        self.prompt_queue = prompt_queue
        self.ui_events = ui_events
        self.on_calibrated = on_calibrated
        self.on_shutdown = on_shutdown
        self.matrix: np.ndarray | None = None
        self.mode = self.CALIBRATION_LIVE
        self.calibration_frame: np.ndarray | None = None
        self.calibration_points: list[tuple[int, int]] = []
        self.closing = False
        self.fullscreen = False
        self.photo: Any | None = None
        self._canvas_image_item: Any | None = None
        self._resize_job: Any | None = None
        self._font_scale = 1.0
        self._camera_error_reported = False
        self._render_scale = 1.0
        self._render_origin_x = 0
        self._render_origin_y = 0
        self._render_image_width = 0
        self._render_image_height = 0

        self.root = tk.Tk()
        self.root.title("Calibrated VLA Conveyor Control")
        self.root.minsize(MIN_DASHBOARD_WIDTH, 620)
        self.default_font = tkfont.nametofont("TkDefaultFont", root=self.root)
        self.text_font = tkfont.nametofont("TkTextFont", root=self.root)
        self.fixed_font = tkfont.nametofont("TkFixedFont", root=self.root)
        self.heading_font = tkfont.nametofont("TkHeadingFont", root=self.root)
        self.default_font.configure(size=GUI_FONT_SIZE)
        self.text_font.configure(size=GUI_FONT_SIZE)
        self.fixed_font.configure(size=GUI_MONOSPACE_FONT_SIZE)
        self.heading_font.configure(size=GUI_HEADING_FONT_SIZE, weight="bold")
        self.style = ttk.Style(self.root)
        self.style.configure("TButton", padding=(12, 10))

        window_width = min(
            WARP_WIDTH + SIDEBAR_WIDTH + 40,
            max(MIN_DASHBOARD_WIDTH, self.root.winfo_screenwidth() - WINDOW_SCREEN_MARGIN),
        )
        window_height = min(
            MIN_DASHBOARD_HEIGHT,
            max(620, self.root.winfo_screenheight() - WINDOW_SCREEN_MARGIN),
        )
        self.root.geometry(f"{window_width}x{window_height}+40+40")
        self.root.protocol("WM_DELETE_WINDOW", self.request_shutdown)
        self.root.bind("<Configure>", self._schedule_responsive_scale)
        self.root.bind("<F11>", self.toggle_fullscreen)
        self.root.bind("<Escape>", self.exit_fullscreen)
        self.root.bind("<Key-s>", self._capture_shortcut)
        self.root.bind("<Key-r>", self._reset_shortcut)
        self.root.bind("<Return>", self._confirm_shortcut)
        self.root.columnconfigure(0, weight=1)
        self.root.columnconfigure(1, weight=0)
        self.root.rowconfigure(0, weight=1)

        left = ttk.Frame(self.root, padding=10)
        left.grid(row=0, column=0, sticky="nsew")
        left.columnconfigure(0, weight=1)
        left.rowconfigure(0, weight=1)
        self.image_canvas = tk.Canvas(
            left,
            background="#111827",
            highlightthickness=0,
            cursor="arrow",
        )
        self.image_canvas.grid(row=0, column=0, sticky="nsew")
        self.image_canvas.bind("<Button-1>", self._on_canvas_click)
        self.image_canvas.bind("<Button-3>", lambda _event: self.undo_calibration_point())

        self.sidebar = ttk.Frame(self.root, padding=10, width=SIDEBAR_WIDTH)
        self.sidebar.grid(row=0, column=1, sticky="ns")
        self.sidebar.grid_propagate(False)
        self.sidebar.columnconfigure(0, weight=1)
        self.sidebar.rowconfigure(1, weight=1)
        self.state_var = tk.StringVar(value=f"STATE: {self.CALIBRATION_LIVE}")
        ttk.Label(self.sidebar, textvariable=self.state_var, font=self.heading_font).grid(row=0, column=0, sticky="ew", pady=(0, 12))
        self.transcript = scrolledtext.ScrolledText(
            self.sidebar,
            wrap="word",
            state="disabled",
            height=20,
            font=self.fixed_font,
            spacing1=2,
            spacing3=4,
        )
        self.transcript.grid(row=1, column=0, sticky="nsew")
        for tag, color in {"USER": "#dbeafe", "INFO": "#1d4ed8", "SUCCESS": "#15803d", "WARNING": "#b45309", "RECONNECTING": "#0369a1", "ERROR": "#b91c1c"}.items():
            self.transcript.tag_configure(tag, foreground=color)

        self.calibration_panel = ttk.Frame(self.sidebar)
        self.calibration_panel.grid(row=2, column=0, sticky="ew", pady=(12, 0))
        for column in range(3):
            self.calibration_panel.columnconfigure(column, weight=1)
        self.calibration_instruction = tk.StringVar()
        ttk.Label(
            self.calibration_panel,
            textvariable=self.calibration_instruction,
            wraplength=SIDEBAR_WIDTH - 30,
        ).grid(row=0, column=0, columnspan=3, sticky="ew", pady=(0, 8))
        self.capture_button = ttk.Button(self.calibration_panel, text="Capture (S)", command=self.capture_calibration_frame)
        self.capture_button.grid(row=1, column=0, sticky="ew", padx=(0, 4))
        self.live_button = ttk.Button(self.calibration_panel, text="Live", command=self.resume_calibration_live)
        self.live_button.grid(row=1, column=1, sticky="ew", padx=4)
        self.confirm_button = ttk.Button(self.calibration_panel, text="Confirm", command=self.confirm_calibration)
        self.confirm_button.grid(row=1, column=2, sticky="ew", padx=(4, 0))
        self.undo_button = ttk.Button(self.calibration_panel, text="Undo", command=self.undo_calibration_point)
        self.undo_button.grid(row=2, column=0, sticky="ew", padx=(0, 4), pady=(6, 0))
        self.reset_button = ttk.Button(self.calibration_panel, text="Reset (R)", command=self.reset_calibration_points)
        self.reset_button.grid(row=2, column=1, sticky="ew", padx=4, pady=(6, 0))
        ttk.Button(self.calibration_panel, text="Exit", command=self.request_shutdown).grid(row=2, column=2, sticky="ew", padx=(4, 0), pady=(6, 0))

        self.task_label = ttk.Label(self.sidebar, text="Natural-language task")
        self.task_label.grid(row=3, column=0, sticky="w", pady=(12, 4))
        self.composer = tk.Text(
            self.sidebar,
            height=4,
            wrap="word",
            font=self.text_font,
            padx=6,
            pady=6,
        )
        self.composer.grid(row=4, column=0, sticky="ew")
        self.runtime_buttons = ttk.Frame(self.sidebar)
        self.runtime_buttons.grid(row=5, column=0, sticky="ew", pady=(10, 0))
        self.runtime_buttons.columnconfigure(0, weight=1)
        self.send_button = ttk.Button(self.runtime_buttons, text="Send", command=self.send_prompt)
        self.send_button.grid(row=0, column=0, sticky="ew", padx=(0, 6))
        ttk.Button(self.runtime_buttons, text="Stop / Exit", command=self.request_shutdown).grid(row=0, column=1, sticky="e")
        self.composer.bind("<Control-Return>", lambda _event: self.send_prompt())

        self._set_composer_enabled(False)
        self._update_calibration_controls()
        self._append("INFO", "Camera is live in this window. Press Capture or S to freeze a calibration frame.")

    def _schedule_responsive_scale(self, event: Any) -> None:
        if event.widget is not self.root or self.closing:
            return
        if self._resize_job is not None:
            self.root.after_cancel(self._resize_job)
        self._resize_job = self.root.after(GUI_RESIZE_DEBOUNCE_MS, self._apply_responsive_scale)

    def _apply_responsive_scale(self) -> None:
        self._resize_job = None
        width = max(self.root.winfo_width(), 1)
        height = max(self.root.winfo_height(), 1)
        base_width = WARP_WIDTH + SIDEBAR_WIDTH + 40
        scale = min(width / base_width, height / MIN_DASHBOARD_HEIGHT)
        scale = max(GUI_MIN_SCALE, min(GUI_MAX_SCALE, scale))
        if abs(scale - self._font_scale) < 0.02:
            return
        self._font_scale = scale
        self.default_font.configure(size=max(1, round(GUI_FONT_SIZE * scale)))
        self.text_font.configure(size=max(1, round(GUI_FONT_SIZE * scale)))
        self.fixed_font.configure(size=max(1, round(GUI_MONOSPACE_FONT_SIZE * scale)))
        self.heading_font.configure(size=max(1, round(GUI_HEADING_FONT_SIZE * scale)), weight="bold")
        self.style.configure("TButton", padding=(max(8, round(12 * scale)), max(6, round(10 * scale))))
        self.sidebar.configure(width=max(460, round(SIDEBAR_WIDTH * scale)))

    def toggle_fullscreen(self, _event: Any = None) -> str:
        self.fullscreen = not self.fullscreen
        self.root.attributes("-fullscreen", self.fullscreen)
        return "break"

    def exit_fullscreen(self, _event: Any = None) -> str:
        if self.fullscreen:
            self.fullscreen = False
            self.root.attributes("-fullscreen", False)
        return "break"

    def _capture_shortcut(self, _event: Any) -> str | None:
        if self.mode != self.RUNTIME:
            self.capture_calibration_frame()
            return "break"
        return None

    def _reset_shortcut(self, _event: Any) -> str | None:
        if self.mode == self.CALIBRATION_SELECTING:
            self.reset_calibration_points()
            return "break"
        return None

    def _confirm_shortcut(self, _event: Any) -> str | None:
        if self.mode == self.CALIBRATION_SELECTING:
            self.confirm_calibration()
            return "break"
        return None

    def _append(self, severity: str, message: str) -> None:
        self.transcript.configure(state="normal")
        self.transcript.insert("end", f"{severity}: {message}\n", severity)
        self.transcript.configure(state="disabled")
        self.transcript.see("end")

    def _set_composer_enabled(self, enabled: bool) -> None:
        state = "normal" if enabled and not self.closing and self.mode == self.RUNTIME else "disabled"
        self.composer.configure(state=state)
        self.send_button.configure(state=state)
        if state == "normal":
            self.composer.focus_set()

    def _update_calibration_controls(self) -> None:
        selecting = self.mode == self.CALIBRATION_SELECTING
        enabled = "normal" if selecting else "disabled"
        self.live_button.configure(state=enabled)
        self.undo_button.configure(state="normal" if selecting and self.calibration_points else "disabled")
        self.reset_button.configure(state=enabled)
        valid = selecting and calibration_is_valid(self.calibration_points)
        self.confirm_button.configure(state="normal" if valid else "disabled")
        self.image_canvas.configure(cursor="crosshair" if selecting else "arrow")
        if self.mode == self.CALIBRATION_LIVE:
            self.calibration_instruction.set("Live camera. Press Capture (or S) when the full conveyor is visible.")
        elif selecting:
            next_index = len(self.calibration_points)
            if next_index < 4:
                self.calibration_instruction.set(
                    f"Click physical belt corners in order: 1=TL, 2=BL, 3=TR, 4=BR. Next: {next_index + 1}={CALIBRATION_CLICK_LABELS[next_index]}. Right-click=Undo."
                )
            elif valid:
                self.calibration_instruction.set("Four corners are valid. Press Confirm or Enter.")
            else:
                self.calibration_instruction.set("Invalid/crossed corner order. Undo or Reset and click TL, BL, TR, BR again.")

    def capture_calibration_frame(self) -> None:
        if self.closing or self.mode == self.RUNTIME:
            return
        frame = self.camera_stream.read()
        if frame is None:
            self._append("WARNING", "Camera has not produced a frame yet.")
            return
        self.calibration_frame = frame
        self.calibration_points.clear()
        self.mode = self.CALIBRATION_SELECTING
        self.state_var.set(f"STATE: {self.CALIBRATION_SELECTING}")
        self.capture_button.configure(text="Retake (S)")
        self._update_calibration_controls()
        self._append("INFO", "Calibration frame captured. Click TL, BL, TR, BR on the same image panel.")

    def resume_calibration_live(self) -> None:
        if self.closing or self.mode == self.RUNTIME:
            return
        self.mode = self.CALIBRATION_LIVE
        self.calibration_frame = None
        self.calibration_points.clear()
        self.state_var.set(f"STATE: {self.CALIBRATION_LIVE}")
        self.capture_button.configure(text="Capture (S)")
        self._update_calibration_controls()

    def undo_calibration_point(self) -> None:
        if self.mode == self.CALIBRATION_SELECTING and self.calibration_points:
            self.calibration_points.pop()
            self._update_calibration_controls()

    def reset_calibration_points(self) -> None:
        if self.mode == self.CALIBRATION_SELECTING:
            self.calibration_points.clear()
            self._update_calibration_controls()

    def _on_canvas_click(self, event: Any) -> None:
        if self.mode != self.CALIBRATION_SELECTING or len(self.calibration_points) >= 4:
            return
        point = view_to_image_point(
            event.x,
            event.y,
            scale=self._render_scale,
            origin_x=self._render_origin_x,
            origin_y=self._render_origin_y,
            image_width=self._render_image_width,
            image_height=self._render_image_height,
        )
        if point is None:
            self._append("WARNING", "Click inside the displayed camera image.")
            return
        self.calibration_points.append(point)
        self._update_calibration_controls()

    def confirm_calibration(self) -> None:
        if self.mode != self.CALIBRATION_SELECTING:
            return
        try:
            matrix = calibration_matrix(self.calibration_points)
            self.on_calibrated(matrix)
        except Exception as error:
            self._append("ERROR", f"Calibration could not start runtime: {error}")
            return
        self.matrix = matrix
        self.mode = self.RUNTIME
        self.calibration_frame = None
        self.state_var.set("STATE: WAITING_FOR_PROMPT")
        self.calibration_panel.grid_remove()
        self._append("SUCCESS", "Calibration accepted. Runtime is active in the same window.")
        self._set_composer_enabled(True)

    def send_prompt(self) -> None:
        if self.closing or self.mode != self.RUNTIME:
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
                elif event["kind"] == "state" and self.mode == self.RUNTIME:
                    self.state_var.set(f"STATE: {event['state']}")
                elif event["kind"] == "prompt_ready" and self.mode == self.RUNTIME:
                    self._set_composer_enabled(True)
                elif event["kind"] == "fatal":
                    self._set_composer_enabled(False)
        except queue.Empty:
            pass
        if not self.closing:
            self.root.after(50, self._drain_events)

    def _calibration_overlay(self, image: np.ndarray) -> np.ndarray:
        output = image.copy()
        marker_scale = max(0.7, min(1.5, image.shape[1] / 1280.0))
        radius = max(8, round(10 * marker_scale))
        for index, point in enumerate(self.calibration_points):
            cv2.circle(output, point, radius, (0, 0, 255), -1)
            cv2.putText(output, f"{index + 1} {CALIBRATION_CLICK_LABELS[index]}", (point[0] + radius, point[1] - radius), cv2.FONT_HERSHEY_SIMPLEX, marker_scale, (0, 0, 255), max(2, round(2 * marker_scale)))
        if len(self.calibration_points) == 4:
            source = calibration_source_corners(self.calibration_points)
            polygon = source.astype(np.int32).reshape((-1, 1, 2))
            color = (0, 255, 0) if calibration_is_valid(self.calibration_points) else (0, 0, 255)
            cv2.polylines(output, [polygon], True, color, max(3, round(3 * marker_scale)))
        return output

    def _visible_frame(self) -> np.ndarray | None:
        if self.mode == self.CALIBRATION_SELECTING:
            return None if self.calibration_frame is None else self._calibration_overlay(self.calibration_frame)
        frame = self.camera_stream.read()
        if frame is None:
            return None
        if self.mode == self.RUNTIME and self.matrix is not None:
            return annotate_vla_frame(prepare_vla_image(frame, self.matrix), self.display_state.snapshot())
        return frame

    def _render_frame(self, frame: np.ndarray) -> None:
        view_width = max(self.image_canvas.winfo_width(), 1)
        view_height = max(self.image_canvas.winfo_height(), 1)
        image_height, image_width = frame.shape[:2]
        scale, display_width, display_height, origin_x, origin_y = fit_image_to_view(
            image_width,
            image_height,
            view_width,
            view_height,
        )
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = self.Image.fromarray(rgb)
        if display_width != image_width or display_height != image_height:
            image = image.resize((display_width, display_height), self.Image.Resampling.LANCZOS)
        self.photo = self.ImageTk.PhotoImage(image=image)
        if self._canvas_image_item is None:
            self._canvas_image_item = self.image_canvas.create_image(origin_x, origin_y, anchor="nw", image=self.photo)
        else:
            self.image_canvas.coords(self._canvas_image_item, origin_x, origin_y)
            self.image_canvas.itemconfigure(self._canvas_image_item, image=self.photo)
        self._render_scale = scale
        self._render_origin_x = origin_x
        self._render_origin_y = origin_y
        self._render_image_width = image_width
        self._render_image_height = image_height

    def _refresh_image(self) -> None:
        if self.closing:
            return
        try:
            self.camera_stream.assert_healthy()
        except Exception as error:
            if not self._camera_error_reported:
                self._camera_error_reported = True
                self.mode = self.ERROR
                self.state_var.set("STATE: ERROR")
                self._set_composer_enabled(False)
                self._append("ERROR", f"Camera stream failed: {error}")
                self.on_shutdown()
        frame = self._visible_frame()
        if frame is not None:
            self._render_frame(frame)
        self.root.after(33, self._refresh_image)

    def run(self) -> None:
        self._drain_events()
        self.root.update_idletasks()
        self._apply_responsive_scale()
        self.root.after_idle(self._refresh_image)
        self.root.mainloop()


def main() -> None:
    board: Any | None = None
    camera: cv2.VideoCapture | None = None
    camera_stream: CameraStream | None = None
    display_state: DisplayState | None = None
    worker: threading.Thread | None = None
    worker_started = False
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

    def start_control_after_calibration(matrix: np.ndarray) -> None:
        nonlocal worker, worker_started
        if worker_started:
            raise RuntimeError("Control worker is already running.")
        if board is None or camera_stream is None or display_state is None:
            raise RuntimeError("Hardware is not ready for runtime control.")
        worker = threading.Thread(
            target=control_loop,
            args=(
                board,
                camera_stream,
                matrix,
                display_state,
                prompt_queue,
                ui_events,
                stop_event,
                control_done,
            ),
            name="vla-control",
            daemon=True,
        )
        worker.start()
        worker_started = True

    try:
        ensure_dashboard_dependencies()
        board = open_yolouno()
        camera = open_camera()
        camera_stream = CameraStream(camera).start()
        display_state = DisplayState()
        dashboard = RuntimeDashboard(
            camera_stream,
            display_state,
            prompt_queue,
            ui_events,
            start_control_after_calibration,
            request_shutdown,
        )
        dashboard.run()
    finally:
        request_shutdown()
        if worker is not None and worker_started:
            worker.join(timeout=(GEMINI_TIMEOUT_MS / 1000.0) + 5.0)
        # Do not close the serial port while a still-running worker can use it.
        if board is not None and (not worker_started or worker is None or not worker.is_alive()):
            stop_motor(board, best_effort=True)
            try:
                board.close()
            except _serial_error_types():
                pass
        elif worker is not None and worker_started and worker.is_alive():
            print("Worker did not exit before shutdown; serial is left open to avoid a concurrent close.")
        if camera_stream is not None:
            camera_stream.stop()
        elif camera is not None:
            camera.release()


if __name__ == "__main__":
    main()

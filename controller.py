import threading
import time
import os
from pathlib import Path
from typing import Literal

import cv2
import numpy as np
import serial
from google import genai
from google.genai import types
from pydantic import BaseModel, Field
from dotenv import load_dotenv


load_dotenv(Path(__file__).with_name(".env"))


# ---------- CONFIG ----------
CAMERA_INDEX = 0
SERIAL_PORT = "COM18"
SERIAL_BAUD = 115200
MODEL_NAME = "gemini-robotics-er-1.6-preview"

# This must be the real distance represented by the calibrated image.
BELT_LENGTH_CM = 77.0

# The belt travelled this distance in this measured time at motor speed 100.
SPEED_TEST_DISTANCE_CM = 77.0
SPEED_TEST_TIME_S = 5.5925
BELT_SPEED_CM_S = (
    SPEED_TEST_DISTANCE_CM
    / SPEED_TEST_TIME_S
)

CENTER_TOLERANCE_CM = 4.0 #2.0
MOVE_GAIN = 0.8
MIN_MOVE_MS = 80
MAX_MOVE_MS = 1500
SETTLE_S = 1.5
MAX_STEPS = 6
MIN_CONFIDENCE = 0.60

NEAR_CENTER_CM = 8.0
NEAR_CENTER_MAX_MS = 180

WARP_WIDTH = 1000
WARP_HEIGHT = 300


class VLAResult(BaseModel):
    object_found: bool
    label: str | None = None
    point: list[int] | None = None  # [y, x], normalized 0..1000
    direction: Literal["LEFT", "RIGHT", "STOP"]
    task_status: Literal["IN_PROGRESS", "CENTERED", "OBJECT_NOT_FOUND"]
    confidence: float = Field(ge=0.0, le=1.0)


api_key = os.getenv("GEMINI_API_KEY")
if not api_key:
    raise RuntimeError(
        "Thiếu GEMINI_API_KEY. Hãy điền API key vào file .env."
    )

client = genai.Client(api_key=api_key)

PROMPT = """
You are the high-level Vision-Language-Action controller of a reversible conveyor.

Find exactly one loose movable object resting on the exposed conveyor surface.
The object may have any color, shape, or material.

Ignore permanent components: belt slats, red markers, rails, yellow structures,
motors, wires, electronics, people, tables, and background objects.
If several loose objects exist, choose the largest movable object.

Return the object's center as [y, x], normalized from 0 to 1000.
The two green vertical lines define the target zone.

Actions:
- object left of target: RIGHT and IN_PROGRESS
- object right of target: LEFT and IN_PROGRESS
- object inside target: STOP and CENTERED
- no object visible: STOP and OBJECT_NOT_FOUND
"""


# ---------- VLA PIPELINE LOG ----------
def _pipeline_header(step, stage, title):
    """Print one easy-to-follow stage of the VLA demo pipeline."""
    print(f"\n[VLA][STEP {step:02d}][{stage}/3] {title}", flush=True)


def log_captured_image(step, image_path, image):
    _pipeline_header(step, 1, "CAPTURE IMAGE")
    print(f"  image : {Path(image_path).resolve()}")
    print(f"  shape : {image.shape[1]}x{image.shape[0]} px")
    print("  state : READY", flush=True)


def log_prompt(step):
    _pipeline_header(step, 2, "PROMPT")
    print("  ----- PROMPT BEGIN -----")
    for line in PROMPT.strip().splitlines():
        print(f"  {line}")
    print("  ----- PROMPT END -------", flush=True)


def action_tokens(direction, duration_ms, status):
    """Serialize the executable controller action as demo-friendly tokens."""
    return (
        f"[ACT_{direction}] "
        f"[DURATION_{duration_ms:04d}_MS] "
        f"[STATUS_{status}]"
    )


def log_action(step, result, direction, duration_ms, status, info):
    _pipeline_header(step, 3, "ACTION TOKENS")
    print(
        "  perception : "
        f"label={result.label!r}, point_yx={result.point}, "
        f"confidence={result.confidence:.2f}"
    )

    if info is not None:
        object_x_cm, error_cm = info
        print(
            "  geometry   : "
            f"x={object_x_cm:.2f} cm, error={error_cm:+.2f} cm"
        )

    print(f"  tokens     : {action_tokens(direction, duration_ms, status)}")
    print("[VLA] IMAGE -> PROMPT -> ACTION COMPLETE\n", flush=True)


# ---------- CAMERA / CALIBRATION ----------
def read_latest_frame(camera):
    for _ in range(4):
        camera.grab()
    ok, frame = camera.read()
    if not ok or frame is None:
        raise RuntimeError("Không đọc được camera.")
    return frame


def wait_camera_stable(camera, seconds):
    """
    Đợi vật ổn định và liên tục bỏ các frame cũ.
    Nhờ vậy lần chụp tiếp theo không lấy ảnh lúc motor còn chạy.
    """
    deadline = time.monotonic() + seconds

    while time.monotonic() < deadline:
        camera.grab()
        time.sleep(0.02)

    # Bỏ thêm một số frame cuối trong buffer.
    for _ in range(10):
        camera.grab()



class CameraStream:
    """Đọc camera liên tục trong thread riêng để cửa sổ không bị đứng."""

    def __init__(self, camera):
        self.camera = camera
        self.frame = None
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = threading.Thread(
            target=self._capture_loop,
            name="camera-stream",
            daemon=True,
        )

    def start(self):
        self.thread.start()
        deadline = time.monotonic() + 3.0

        while self.read() is None:
            if time.monotonic() >= deadline:
                raise RuntimeError("Camera stream không tạo được frame.")
            time.sleep(0.02)

        return self

    def _capture_loop(self):
        while not self.stop_event.is_set():
            ok, frame = self.camera.read()

            if not ok or frame is None:
                time.sleep(0.02)
                continue

            with self.lock:
                self.frame = frame

    def read(self):
        with self.lock:
            if self.frame is None:
                return None
            return self.frame.copy()

    def stop(self):
        self.stop_event.set()

        if self.thread.is_alive():
            self.thread.join(timeout=2.0)

        self.camera.release()

def order_points(points):
    pts = np.asarray(points, dtype=np.float32)
    center = pts.mean(axis=0)
    angles = np.arctan2(pts[:, 1] - center[1], pts[:, 0] - center[0])
    ordered = pts[np.argsort(angles)]

    tl_index = np.argmin(ordered[:, 0] + ordered[:, 1])
    ordered = np.roll(ordered, -tl_index, axis=0)

    v1 = ordered[1] - ordered[0]
    v2 = ordered[2] - ordered[1]
    if v1[0] * v2[1] - v1[1] * v2[0] < 0:
        ordered = ordered[[0, 3, 2, 1]]

    return ordered.astype(np.float32)


def calibrate(camera):
    print("\nCamera live: S = chốt ảnh, Q = thoát.")

    while True:
        frame = read_latest_frame(camera)
        preview = frame.copy()
        cv2.putText(
            preview, "S: CALIBRATE | Q: QUIT", (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2
        )
        cv2.imshow("Live Camera", preview)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            cv2.destroyWindow("Live Camera")
            return None
        if key == ord("s"):
            cv2.destroyWindow("Live Camera")
            break

    points = []
    window = "Select 4 belt corners"

    def on_mouse(event, x, y, flags, parameter):
        if event == cv2.EVENT_LBUTTONDOWN and len(points) < 4:
            points.append((x, y))
        elif event == cv2.EVENT_RBUTTONDOWN and points:
            points.pop()

    cv2.namedWindow(window)
    cv2.setMouseCallback(window, on_mouse)
    print("Click 4 góc theo thứ tự bất kỳ. Chuột phải = undo, R = reset, Enter = xác nhận.")

    while True:
        display = frame.copy()
        ordered = None
        valid = False

        for index, point in enumerate(points):
            cv2.circle(display, point, 7, (0, 0, 255), -1)
            cv2.putText(
                display, str(index + 1), (point[0] + 8, point[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2
            )

        if len(points) == 4:
            ordered = order_points(points)
            polygon = ordered.astype(np.int32).reshape((-1, 1, 2))
            valid = cv2.isContourConvex(polygon) and cv2.contourArea(polygon) > 2000
            color = (0, 255, 0) if valid else (0, 0, 255)
            cv2.polylines(display, [polygon], True, color, 3)

            for label, point in zip(("TL", "TR", "BR", "BL"), ordered.astype(int)):
                cv2.putText(
                    display, label, (point[0] + 8, point[1] + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2
                )

        cv2.imshow(window, display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("r"):
            points.clear()
        elif key == ord("q"):
            cv2.destroyWindow(window)
            return None
        elif key in (10, 13, 32) and ordered is not None and valid:
            cv2.destroyWindow(window)
            destination = np.float32([
                [0, 0],
                [WARP_WIDTH - 1, 0],
                [WARP_WIDTH - 1, WARP_HEIGHT - 1],
                [0, WARP_HEIGHT - 1],
            ])
            return cv2.getPerspectiveTransform(ordered, destination)


def prepare_vla_image(frame, matrix):
    image = cv2.warpPerspective(frame, matrix, (WARP_WIDTH, WARP_HEIGHT))
    center_x = WARP_WIDTH // 2
    tolerance_px = round(CENTER_TOLERANCE_CM / BELT_LENGTH_CM * WARP_WIDTH)

    cv2.line(
        image, (center_x - tolerance_px, 0),
        (center_x - tolerance_px, WARP_HEIGHT), (0, 255, 0), 3
    )
    cv2.line(
        image, (center_x + tolerance_px, 0),
        (center_x + tolerance_px, WARP_HEIGHT), (0, 255, 0), 3
    )
    return image



# ---------- DISPLAY STATE ----------
class DisplayState:
    def __init__(self):
        self.lock = threading.Lock()
        self.step = 0
        self.status = "STARTING"
        self.direction = "STOP"
        self.point = None

    def update(self, step=None, status=None, direction=None, point=None):
        with self.lock:
            if step is not None:
                self.step = step
            if status is not None:
                self.status = status
            if direction is not None:
                self.direction = direction
            if point is not None:
                self.point = list(point)

    def clear_point(self):
        with self.lock:
            self.point = None

    def snapshot(self):
        with self.lock:
            return {
                "step": self.step,
                "status": self.status,
                "direction": self.direction,
                "point": None if self.point is None else list(self.point),
            }


def annotate_live_frame(frame):
    output = frame.copy()
    cv2.putText(
        output,
        "REAL-TIME CAMERA | Q: QUIT",
        (20, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 0),
        2,
    )
    return output


def annotate_vla_frame(image, state):
    output = image.copy()
    status_text = (
        f"Step {state['step']} | {state['status']} | "
        f"Action: {state['direction']}"
    )

    cv2.rectangle(output, (0, 0), (WARP_WIDTH, 38), (0, 0, 0), -1)
    cv2.putText(
        output,
        status_text,
        (12, 26),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (255, 255, 255),
        2,
    )

    point = state["point"]
    if point is not None and len(point) == 2:
        y_normalized = int(np.clip(point[0], 0, 1000))
        x_normalized = int(np.clip(point[1], 0, 1000))
        point_x = round(x_normalized / 1000.0 * WARP_WIDTH)
        point_y = round(y_normalized / 1000.0 * WARP_HEIGHT)

        cv2.circle(output, (point_x, point_y), 9, (0, 0, 255), 3)
        cv2.putText(
            output,
            "Last Gemini point",
            (min(point_x + 12, WARP_WIDTH - 190), max(point_y - 10, 55)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 0, 255),
            2,
        )

    return output

# ---------- GEMINI ----------
def detect_object(image):
    ok, encoded = cv2.imencode(
        ".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 90]
    )
    if not ok:
        raise RuntimeError("Không encode được ảnh.")

    response = client.models.generate_content(
        model=MODEL_NAME,
        contents=[
            types.Part.from_bytes(
                data=encoded.tobytes(),
                mime_type="image/jpeg",
            ),
            PROMPT,
        ],
        config=types.GenerateContentConfig(
            temperature=0.2,
            thinking_config=types.ThinkingConfig(thinking_budget=0),
            response_mime_type="application/json",
            response_schema=VLAResult,
        ),
    )

    if isinstance(response.parsed, VLAResult):
        return response.parsed
    if response.parsed is not None:
        return VLAResult.model_validate(response.parsed)
    return VLAResult.model_validate_json(response.text)


def plan_action(result):
    if (
        not result.object_found
        or result.point is None
        or len(result.point) != 2
        or result.confidence < MIN_CONFIDENCE
    ):
        return "STOP", 0, None, "OBJECT_NOT_FOUND"

    x_normalized = int(
        np.clip(result.point[1], 0, 1000)
    )

    object_x_cm = (
        x_normalized
        / 1000.0
        * BELT_LENGTH_CM
    )

    center_cm = BELT_LENGTH_CM / 2.0
    error_cm = object_x_cm - center_cm
    distance_cm = abs(error_cm)

    # Dừng nếu Gemini nói CENTERED hoặc
    # tọa độ nằm trong vùng giữa.
    if (
        result.task_status == "CENTERED"
        or distance_cm <= CENTER_TOLERANCE_CM
    ):
        return (
            "STOP",
            0,
            (object_x_cm, error_cm),
            "CENTERED",
        )

    direction = (
        "LEFT"
        if error_cm < 0
        else "RIGHT"
    )

    move_distance_cm = max(
        0.0,
        distance_cm - CENTER_TOLERANCE_CM,
    )

    theoretical_s = (
        move_distance_cm
        / BELT_SPEED_CM_S
    )

    duration_ms = int(
        np.clip(
            theoretical_s
            * MOVE_GAIN
            * 1000.0,
            MIN_MOVE_MS,
            MAX_MOVE_MS,
        )
    )

    # Khi gần tâm, không cho chạy một xung dài.
    if distance_cm <= NEAR_CENTER_CM:
        duration_ms = min(
            duration_ms,
            NEAR_CENTER_MAX_MS,
        )

    return (
        direction,
        duration_ms,
        (object_x_cm, error_cm),
        "MOVE",
    )


# ---------- YOLOUNO SERIAL ----------
def open_yolouno():
    try:
        board = serial.Serial(
            SERIAL_PORT,
            SERIAL_BAUD,
            timeout=0.25,
            write_timeout=1.0,
        )
    except serial.SerialException as error:
        raise RuntimeError(
            f"Không mở được {SERIAL_PORT}. Hãy đóng PlatformIO Monitor. Chi tiết: {error}"
        ) from error

    time.sleep(2.5)
    board.reset_input_buffer()

    for _ in range(5):
        board.write(b"PING\n")
        board.flush()
        deadline = time.monotonic() + 1.0

        while time.monotonic() < deadline:
            line = board.readline().decode(errors="ignore").strip()
            if line:
                print("YoloUNO ->", line)
            if line == "PONG":
                print("Kết nối YoloUNO thành công.\n")
                return board

        time.sleep(0.4)

    board.close()
    raise RuntimeError(
        "Mở được COM nhưng YoloUNO không trả PONG. Kiểm tra firmware và USB CDC."
    )


def stop_motor(board):
    try:
        board.write(b"STOP\n")
        board.flush()
    except serial.SerialException:
        pass


def run_motor(board, direction, duration_ms, stop_event):
    board.reset_input_buffer()
    command = f"MOVE,{direction},{duration_ms}\n"
    board.write(command.encode("ascii"))
    board.flush()
    print("TX ->", command.strip())

    got_ack = False
    deadline = time.monotonic() + duration_ms / 1000.0 + 3.0

    while time.monotonic() < deadline:
        if stop_event.is_set():
            stop_motor(board)
            return False

        line = board.readline().decode(errors="ignore").strip()
        if not line:
            continue

        print("YoloUNO ->", line)

        if line.startswith("ACK,"):
            got_ack = True
        elif line == "DONE":
            return True
        elif line.startswith("ERR,"):
            raise RuntimeError(f"YoloUNO báo lỗi: {line}")

    stop_motor(board)

    if not got_ack:
        raise TimeoutError("YoloUNO không ACK lệnh MOVE.")

    raise TimeoutError("YoloUNO đã ACK nhưng không trả DONE.")


# ---------- CLOSED LOOP ----------
def control_loop(
    board,
    camera_stream,
    matrix,
    display_state,
    stop_event,
    control_done,
):
    try:
        for step in range(1, MAX_STEPS + 1):
            if stop_event.is_set():
                break

            frame = camera_stream.read()
            if frame is None:
                raise RuntimeError("Camera stream chưa có frame.")

            vla_image = prepare_vla_image(frame, matrix)
            debug_path = f"debug_step_{step}.jpg"
            if not cv2.imwrite(debug_path, vla_image):
                raise RuntimeError(f"Không lưu được ảnh pipeline: {debug_path}")

            log_captured_image(step, debug_path, vla_image)
            log_prompt(step)
            print(f"\n[VLA][STEP {step:02d}] Model is analyzing...", flush=True)

            display_state.clear_point()
            display_state.update(
                step=step,
                status="ANALYZING",
                direction="STOP",
            )

            result = detect_object(vla_image)

            if stop_event.is_set():
                break

            direction, duration_ms, info, status = plan_action(result)

            display_state.update(
                step=step,
                status=status,
                direction=direction,
                point=result.point,
            )

            log_action(
                step,
                result,
                direction,
                duration_ms,
                status,
                info,
            )

            if status == "CENTERED":
                stop_motor(board)
                display_state.update(status="CENTERED", direction="STOP")
                print("HOÀN THÀNH: vật đã nằm trong vùng giữa.")
                break

            if direction == "STOP":
                stop_motor(board)
                if stop_event.wait(1.0):
                    break
                continue

            display_state.update(status="MOVING", direction=direction)

            completed = run_motor(
                board,
                direction,
                duration_ms,
                stop_event,
            )

            if not completed:
                break

            display_state.update(status="SETTLING", direction="STOP")
            print(
                f"Motor đã dừng, chờ {SETTLE_S:.1f}s "
                "để vật ổn định..."
            )

            # Camera thread vẫn đọc liên tục, nên sau khi đợi xong
            # vòng tiếp theo sẽ lấy đúng frame mới nhất.
            if stop_event.wait(SETTLE_S):
                break

    except Exception as error:
        stop_motor(board)
        error_text = str(error)

        if "429" in error_text or "RESOURCE_EXHAUSTED" in error_text:
            print()
            print("Gemini đã hết quota hoặc đang bị rate limit.")
            print("Motor đã STOP. Dừng closed-loop để tránh gọi API liên tục.")
            display_state.update(status="API_LIMIT", direction="STOP")
        else:
            print(f"Lỗi closed-loop: {error}")
            display_state.update(status="ERROR", direction="STOP")

    finally:
        stop_motor(board)
        control_done.set()


def main():
    board = open_yolouno()

    camera = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
    if not camera.isOpened():
        board.close()
        raise RuntimeError("Không mở được camera.")

    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    matrix = calibrate(camera)
    if matrix is None:
        stop_motor(board)
        board.close()
        camera.release()
        cv2.destroyAllWindows()
        return

    camera_stream = CameraStream(camera).start()
    stop_event = threading.Event()
    control_done = threading.Event()
    display_state = DisplayState()

    worker = threading.Thread(
        target=control_loop,
        args=(
            board,
            camera_stream,
            matrix,
            display_state,
            stop_event,
            control_done,
        ),
        name="vla-control",
        daemon=True,
    )

    print(f"\nTốc độ ước tính: {BELT_SPEED_CM_S:.2f} cm/s")
    print("Bắt đầu closed-loop.")
    print("Camera Real-time: ảnh camera gốc.")
    print("Conveyor VLA: ảnh warp và vùng giữa.")
    print("Nhấn Q ở một trong hai cửa sổ để dừng.\n")

    cv2.namedWindow("Camera Real-time", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Conveyor VLA", cv2.WINDOW_NORMAL)

    worker.start()

    try:
        while not control_done.is_set():
            frame = camera_stream.read()

            if frame is None:
                time.sleep(0.01)
                continue

            live_display = annotate_live_frame(frame)
            vla_display = prepare_vla_image(frame, matrix)
            vla_display = annotate_vla_frame(
                vla_display,
                display_state.snapshot(),
            )

            cv2.imshow("Camera Real-time", live_display)
            cv2.imshow("Conveyor VLA", vla_display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                print("Đã nhận Q. Đang dừng hệ thống...")
                stop_event.set()
                break

        worker.join(timeout=5.0)

    finally:
        stop_event.set()
        stop_motor(board)
        board.close()
        camera_stream.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

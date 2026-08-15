# Calibrated VLA Conveyor Controller — Implementation and Validation Plan

## 1. Objective

`controller.py` is a continuously running desktop controller for a reversible
YoloUNO conveyor. The operator calibrates an arbitrarily mounted camera, submits
a natural-language task, and Gemini converts the task and the current calibrated
image into an ordered waypoint plan plus executable movement actions.

The implemented flow is:

```text
Connect MCU and camera
        → open one dashboard
        → capture calibration frame
        → click TL, BL, TR, BR and confirm
        → submit a natural-language instruction
        → Gemini parses and returns ordered waypoints
        → validate/execute the active waypoint
        → capture a fresh frame after each pulse
        → advance through all locked waypoints
        → enable the composer for the next instruction
```

The process has no fixed step limit. It remains available for new instructions
until Stop/Exit is selected or a fatal controller condition sets the shared stop
event.

## 2. VLA and OpenCV Boundary

OpenCV is permitted for camera and geometric calibration only:

- camera acquisition;
- four-point perspective calibration;
- perspective warping to `1000 x 300`;
- JPEG encoding for Gemini;
- display-only calibration markers, target lines, status text, and Gemini point;
- optional debug-frame output when `WRITE_DEBUG_IMAGES=True`.

OpenCV must not detect, classify, segment, or track the target object. It must
not calculate the object's center or generate a MOVE duration.

Gemini is responsible for:

- matching the loose movable object described by the operator;
- parsing unrestricted natural language into `1..12` ordered waypoint x values;
- reporting the object's normalized `[y, x]` point;
- generating `LEFT`, `RIGHT`, or `STOP`;
- calculating `duration_ms` from the supplied physical calibration policy;
- reporting `MOVE`, `AT_TARGET`, `TARGET_NOT_FOUND`, or
  `INVALID_INSTRUCTION`.

Python performs strict safety validation. It uses Gemini's reported point to
check waypoint tolerance and movement direction, but it never creates a fallback
MOVE or recomputes Gemini's duration. It may normalize an unsafe or stale result
to `STOP/0/AT_TARGET` when the reported point is already inside tolerance.

## 3. Hardware Discovery and Startup

1. Load `.env` next to `controller.py`.
2. Verify Tkinter and Pillow before opening hardware.
3. Open the YoloUNO serial port and require a `PING`/`PONG` handshake.
4. Open the configured camera and require at least one valid frame.
5. Start one continuous `CameraStream` capture thread.
6. Open the single Tkinter dashboard in `CALIBRATION_LIVE`.
7. Start the Gemini/serial control worker only after calibration is confirmed.

Device settings:

- `CAMERA_DEVICE`: explicit index/path or `auto`;
- `CAMERA_MATCH`: preferred Linux V4L2/by-id description;
- `SERIAL_PORT`: explicit device/COM port or `auto`;
- `SERIAL_MATCH`: preferred serial by-id/port description;
- `SERIAL_BAUD`: defaults to `115200`;
- `GEMINI_API_KEY`: required when the first instruction is analyzed.

On Linux, camera auto-discovery prefers matching `*-video-index0` paths under
`/dev/v4l/by-id`, followed by `/dev/video*`. Serial auto-discovery prefers a
matching `/dev/serial/by-id` entry and otherwise probes `ttyACM*`/`ttyUSB*` using
the firmware handshake.

## 4. Single-Window Calibration

The dashboard must not open a separate OpenCV HighGUI window.

1. Show the raw real-time camera in the left canvas.
2. Capture/freeze one frame with **Capture** or `S`.
3. Require semantic click order:
   `1=physical top-left`, `2=physical bottom-left`,
   `3=physical top-right`, `4=physical bottom-right`.
4. Interpret physical orientation rather than apparent screen orientation so a
   rotated or inverted camera remains valid.
5. Map resized/full-screen canvas clicks back to original-frame coordinates.
6. Allow right-click/**Undo**, `R`/**Reset**, **Live**, and retake.
7. Accept only a convex four-corner polygon with sufficient source-frame area.
8. Build one homography with `cv2.getPerspectiveTransform()` and map the physical
   belt to `1000 x 300` using `cv2.warpPerspective()`.
9. Switch the same canvas to the annotated warped live stream and enable chat.

The camera must remain fixed after confirmation. Calibration is not persisted;
moving the camera or restarting the program requires calibration again.

## 5. Physical Calibration Policy

The current controller constants are:

```text
belt length                  77 cm
speed calibration distance  77 cm
speed calibration time      5.5925 s
measured speed               approximately 13.77 cm/s
normalized belt x            0..1000
target tolerance             4 cm
move gain                    0.8
host MOVE duration           80..1500 ms
near-target threshold        8 cm
near-target maximum pulse    180 ms
qualitative left/center/right 200/500/800
```

These values describe the deployed physical belt and must be recalibrated in
code if the conveyor speed, length, load, voltage, gearing, or motor changes.

## 6. Natural-Language Waypoint Planning

Each sidebar message describes one object and one or more ordered destinations.
The language is unrestricted; it is not a command template. Gemini resolves
qualitative positions, percentages, fractions, centimeters from the physical
left edge, visible landmarks, and temporal phrases such as “then” or “return”.

Examples:

```text
move the red tape roll to the right, then return it to the middle
    → approximately [800, 500]

move the white charger to 25% of the belt, then to the far right
    → [250, 1000]
```

The first valid model response supplies the complete waypoint array. The host
copies and locks it. Every later inference must return the exact locked array and
the exact active index. Reordering, restarting, adding, deleting, or silently
reinterpreting waypoints is invalid model data.

## 7. Gemini Structured Contract

The API wire schema and strict local `VLAResult` contain:

```text
target_found: bool
target_matches_prompt: bool
label: string | null
point: [y, x] | null              # integer values in 0..1000
instruction_valid: bool
waypoints_x: list[int]            # 1..12 values in 0..1000 when valid
active_waypoint_index: int | null
direction: LEFT | RIGHT | STOP
duration_ms: int
task_status: MOVE | AT_TARGET | TARGET_NOT_FOUND | INVALID_INSTRUCTION
confidence: float                 # 0..1
```

The controller uses a minimal Gemini API schema because the Robotics endpoint
rejects Pydantic's `additionalProperties` conversion. Full strictness, including
unknown-field rejection, is applied locally with Pydantic.

Gemini receives an unannotated calibrated image, never the dashboard overlay.
The configured model is `gemini-robotics-er-1.6-preview`, temperature is `0.2`,
thinking budget is `0`, and the client timeout is `15 s`.

## 8. Validation and Waypoint State Machine

Validation occurs before any motor command:

- Invalid/ambiguous instruction: empty waypoint plan,
  `INVALID_INSTRUCTION/STOP/0`, no target data; end that instruction.
- Missing, nonmatching, or confidence below `0.60`:
  `TARGET_NOT_FOUND/STOP/0`.
- Matching target: require a valid normalized point.
- Inside `4 cm` of the active waypoint: normalize to `AT_TARGET/STOP/0` before
  interpreting stale action fields.
- Outside tolerance: require `LEFT` or `RIGHT`, `80..1500 ms`, and a direction
  that approaches the destination. No Python-generated correction MOVE exists.

For each locked waypoint:

```text
validate current waypoint
    reached and intermediate → STOP → increment index → continue
    reached and final        → final guard/STOP → complete instruction
    not reached              → execute validated MOVE → settle → reanalyze
```

The final guard applies only to task completion at the last waypoint. Intermediate
waypoints are position validations that advance the locked sequence; they do not
finish the instruction.

## 9. Action Tokens and Firmware Adapter

The dashboard and terminal log the validated executable action as:

```text
[ACT_RIGHT] [DURATION_0780_MS] [STATUS_MOVE]
```

This token format is for VLA observability only. The YoloUNO firmware does not
parse brackets. The host adapts actions to:

```text
PING\n
STOP\n
MOVE,<LEFT|RIGHT>,<duration_ms>\n
```

For MOVE, the host requires the exact `ACK,<direction>,<duration_ms>` before a
later `DONE`. A mismatched/duplicate ACK, DONE before ACK, firmware `ERR,*`, serial
exception, or timeout is fatal. The Stop/Exit event and final MOVE write share a
lock so a late model response cannot race a shutdown request into a new MOVE.

Firmware accepts `50..3000 ms`; the host deliberately restricts Gemini to
`80..1500 ms`. `IMAGE_RIGHT_IS_FORWARD` maps image-relative directions to motor
forward/backward and must be verified on the physical conveyor.

## 10. Continuous Operation, Missing Objects, and Retries

After every MOVE, wait for firmware DONE, wait `1.5 s` for settling, then analyze
the newest frame. A motor pulse already in progress is allowed to finish unless
Stop/Exit is requested.

If the instructed object is grabbed or disappears:

- issue/maintain STOP;
- log `target not recognized - retry n/20`;
- wait `1 s` and analyze a fresh frame;
- reset the counter after a valid matching response;
- after attempt 20, abandon only the current instruction and accept a new one.

Gemini/API/schema failures have a separate counter:

- print the exception and traceback to the terminal;
- show `RECONNECTING` for API/runtime failures or `ERROR` for invalid model data;
- require firmware `STOPPED` before retrying;
- wait `2 s` between attempts;
- reset the counter after any valid model result;
- fail on the fifth consecutive technical error.

Deterministic HTTP `4xx` errors are immediately fatal except `408` and `429`,
which use the technical retry policy.

Camera stale/disconnect (`>3 s`), serial failures, STOP acknowledgement failure,
and motor protocol timeouts are fatal. Camera auto-reconnect is not implemented.
A fatal runtime condition sets ERROR, disables new prompts, and requests stop;
the dashboard remains available to show the error until the operator closes it.

## 11. GUI and Concurrency

- Tkinter and all widget updates remain on the main thread.
- `CameraStream` owns continuous camera reads.
- One control worker owns Gemini inference, validation, and serial actions.
- A prompt queue transfers sidebar input to the worker.
- A UI event queue transfers state/log events back to Tkinter.
- The left canvas fits and centers frames without cumulative zoom animation.
- Fonts, button padding, sidebar width, and click mapping respond to resize and
  full-screen (`F11`); `Esc` exits full-screen.
- The composer is disabled during calibration and while an instruction is active.
- Gemini errors are printed both in the GUI and terminal.

## 12. Validation Plan

### Software checks

- Compile `controller.py` and run `git diff --check`.
- Test canvas-to-source coordinate mapping with letterboxing and full-screen sizes.
- Test normal, rotated, and visually inverted calibration corner arrangements.
- Mock strict schema validation, waypoint locking, intermediate advancement, and
  final completion.
- Mock no-match recovery, invalid instruction, retry exhaustion, late Stop/Exit,
  exact ACK/DONE ordering, wrong ACK, stale DONE, ERR, and timeouts.
- Confirm no `cv2.imshow`, `waitKey`, or separate HighGUI calibration window exists.

### Required hardware bench tests

- Verify `PING/PONG`, exact ACK, DONE, STOP/STOPPED, and physical motor stop.
- Verify `MOVE,RIGHT` moves right in the calibrated image and `MOVE,LEFT` moves
  left; change and reflash `IMAGE_RIGHT_IS_FORWARD` if required.
- Validate physical scale and duration at several belt locations and camera angles.
- Run a single arbitrary destination and a multi-waypoint return sequence.
- Remove and restore the target during closed-loop operation.
- Disconnect the camera and MCU separately and verify no further MOVE is sent.
- Test Stop/Exit during Gemini analysis, settling, and an active pulse.
- Test manual firmware buttons because they share `motor_state` with host control.

The system is not production-ready or safe for unattended operation until all
hardware bench tests pass with the deployed camera, board, motor, power supply,
load, and emergency-stop procedure.

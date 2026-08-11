# Calibrated VLA Conveyor Control

## Summary

Refactor `controller.py` into a continuously running, prompt-driven VLA pipeline:

`CALIBRATE → ASK PROMPT → CAPTURE/WARP → GEMINI ACTION → EXECUTE → REPEAT`

OpenCV remains responsible only for camera I/O, camera-position calibration, perspective normalization, visualization, and image encoding. Gemini is responsible for identifying the prompted object and generating the complete VLA action: `direction`, `duration_ms`, and `status`. Python validates that action and adapts it to the existing firmware serial protocol; it does not detect the object or calculate movement duration itself.

The application processes prompts indefinitely. It exits only on `Q`, terminal input `quit`, a fatal camera/serial/motor error, or five consecutive Gemini technical failures.

## Calibration and VLA Boundary

- Keep the existing four-corner calibration so the camera may be mounted at an arbitrary fixed position.
- Use `cv2.getPerspectiveTransform()` once after corner selection and apply the resulting homography with `cv2.warpPerspective()` to every inference frame.
- Keep the calibrated output at `1000 x 300`. Its full width represents the configured physical belt length of `77 cm`, giving a scale of `77 / 1000 = 0.077 cm` per normalized x unit.
- Draw the two green target-zone lines on the warped image. With a `4 cm` tolerance, the target zone is approximately `x=448..552`, centered at `x=500`.
- Recalibrate whenever the camera position changes; reuse the matrix while the camera remains fixed.
- OpenCV may capture, warp, annotate, display, save, and JPEG-encode frames. It must not detect/classify the object, find its center using classical vision, track it, decide movement direction, or calculate `duration_ms`.

## Gemini Action Contract

- After hardware connection and calibration, ask the operator for a full task instruction. Reject blank input and accept case-insensitive `quit` for safe shutdown.
- Send Gemini the warped image, the operator instruction, the fixed conveyor policy, and these calibration/control values:
  - belt length: `77 cm`;
  - normalized image x range: `0..1000`, with center at `500`;
  - center tolerance: `4 cm`;
  - measured belt speed: `77 / 5.5925`, approximately `13.77 cm/s`;
  - move gain: `0.8`;
  - valid move duration: `80..1500 ms`;
  - near-center threshold: `8 cm`, with a maximum pulse of `180 ms`.
- Instruct Gemini to identify only a loose object whose appearance matches the active user instruction. Other loose objects are not valid substitutes.
- Gemini must return a structured `VLAResult` containing:
  - `target_found: bool`;
  - `target_matches_prompt: bool`;
  - `label: str | None`;
  - `point: list[int] | None` as normalized `[y, x]` for logging/auditing;
  - `direction: LEFT | RIGHT | STOP`;
  - `duration_ms: int`;
  - `task_status: MOVE | CENTERED | TARGET_NOT_FOUND`;
  - `confidence: float` in `0..1`.
- Gemini calculates movement duration using the calibrated scale and current controller policy:

  ```text
  offset_cm = abs(object_x - 500) * 77 / 1000
  move_distance_cm = max(0, offset_cm - 4)
  duration_ms = clamp(move_distance_cm / 13.77 * 0.8 * 1000, 80, 1500)
  if offset_cm <= 8: duration_ms = min(duration_ms, 180)
  ```

- Direction semantics remain image-relative: an object left of the target requires `RIGHT`; an object right of the target requires `LEFT`.
- Enforce output invariants before touching the motor:
  - `MOVE` requires a matching target, confidence at least `0.60`, direction `LEFT` or `RIGHT`, and duration `80..1500 ms`;
  - `CENTERED` requires a matching target, `STOP`, and duration `0`;
  - missing, nonmatching, or low-confidence targets resolve to `TARGET_NOT_FOUND`, `STOP`, and duration `0`;
  - contradictory, malformed, or out-of-range output is invalid model data and follows the technical retry policy.
- Remove the existing `plan_action()` geometry and duration calculation. Python may validate or reject Gemini's values but must not replace them with a separately calculated action.

## Action Tokens and Firmware Integration

- Serialize the validated Gemini action with the existing function:

  ```text
  [ACT_RIGHT] [DURATION_0780_MS] [STATUS_MOVE]
  ```

- Treat this string as the visible/auditable VLA action token output. `action_tokens()` must receive the direction, duration, and status produced by Gemini rather than values calculated by Python.
- Preserve the current firmware protocol without modifying `conveyor_firmware`:
  - a validated move token is adapted to `MOVE,<direction>,<duration_ms>\n`;
  - `CENTERED`, `TARGET_NOT_FOUND`, cancellation, and failures send `STOP\n`;
  - wait for firmware `ACK` and `DONE` exactly as the current `run_motor()` does.
- Keep the controller's `1500 ms` maximum even though the firmware accepts up to `3000 ms`; this is the host-side VLA safety limit.

## Continuous Control Behavior

- Replace the fixed `MAX_STEPS` loop with an outer prompt loop and an unlimited inner control loop for each active instruction.
- For every valid `MOVE` result:
  1. log the Gemini perception and exact action tokens;
  2. send the adapted serial command;
  3. allow the complete timed motor pulse to finish;
  4. wait `SETTLE_S` while the camera stream continues updating;
  5. capture and analyze the newest warped frame.
- When Gemini returns `CENTERED`, send `STOP`, log completion, stop making model calls for that instruction, and immediately ask for the next terminal prompt.
- When the prompted object is absent, no longer matches, or is below the confidence threshold:
  - send/keep `STOP`;
  - log `WARNING: target not recognized - retry n/20`;
  - wait one second and analyze a fresh frame;
  - reset the no-match counter after any valid matching-target result.
- After 20 consecutive valid no-match results, abandon the active instruction and request a new prompt without exiting the application.
- If the object is grabbed during a motor pulse, finish that pulse. The first post-settle inference detects the missing target, stops further movement, and starts the warning/retry sequence.
- Maintain separate technical retries for Gemini connection, timeout, rate-limit, invalid JSON, schema, and invariant failures. Keep the motor stopped, log `reconnect` or `invalid data`, wait two seconds, and retry the same instruction. Valid model output resets this counter; the fifth consecutive technical failure triggers safe application shutdown.
- Camera read failures, serial failures, missing firmware acknowledgements, and motor timeouts remain fatal and trigger safe shutdown.
- Preserve `Q` in either camera window and add terminal `quit`. Both paths stop the motor and release serial, camera, worker, and OpenCV window resources.

## Threading and Display

- Retain `CameraStream` as the continuous capture thread so the real-time view remains active during motor movement, Gemini calls, settling, warnings, and terminal input.
- Retain the control worker for prompt processing, Gemini inference, action validation, and serial execution.
- Keep the main thread responsible for the live and warped OpenCV windows and `Q` handling.
- Add display states for `WAITING_FOR_PROMPT`, `ANALYZING`, `MOVING`, `SETTLING`, `TARGET_MISSING`, `MODEL_RETRY`, `CENTERED`, and `ERROR`.
- Display the last Gemini point and Gemini-generated action only for observability; neither overlay participates in control calculations.

## Test Plan

- Calibrate from several camera angles and verify the selected belt corners consistently map to the full `1000 x 300` inference image and the green target zone remains centered.
- Mock Gemini with valid `LEFT` and `RIGHT` actions and verify the exact model-generated durations appear in `action_tokens()` and in the corresponding firmware `MOVE` commands without Python recomputation.
- Verify `CENTERED` produces `[ACT_STOP] [DURATION_0000_MS] [STATUS_CENTERED]`, sends `STOP`, and immediately returns to the prompt loop.
- Reject MOVE responses with `STOP`, duration outside `80..1500`, missing target data, contradictory status, malformed point, or low confidence; never send a MOVE command for these cases.
- Remove the prompted object during a pulse and verify the pulse finishes, the next inference warns, the motor remains stopped, and the same instruction resumes when the matching object returns.
- Return 20 consecutive valid no-match results and verify warnings `1/20` through `20/20`, followed by a new prompt. Verify a valid match resets this counter.
- Simulate five consecutive Gemini technical failures and verify they do not consume no-match attempts, every retry keeps the motor stopped, and the fifth failure performs safe shutdown. Verify valid output resets the technical counter.
- Verify blank prompts reprompt, terminal `quit` exits, and window `Q` exits even while the control worker is awaiting input.
- Verify serial `ACK`, `DONE`, `ERR`, and timeout handling with mocked board responses.
- Verify the live camera continues updating while the motor runs and while the terminal waits for the next instruction.

## Assumptions

- The camera remains physically fixed after calibration; moving it requires a new calibration.
- OpenCV calibration and perspective normalization are permitted because they establish the camera-to-belt scale and do not perform object perception or action selection.
- Gemini, not `plan_action()`, is the authoritative source of direction, duration, and status.
- The bracketed action-token string is the VLA output representation for logging and review. The unchanged firmware still receives its supported `MOVE,...` or `STOP` serial command.
- Twenty no-match retries means 20 consecutive valid model responses in which the instructed target is absent, nonmatching, or below the confidence threshold.
- Five Gemini technical retries means safe shutdown on the fifth consecutive technical failure.

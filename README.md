# VLA Conveyor Control

A Vision-Language-Action (VLA) system that uses Gemini to identify an operator-specified object, generate conveyor actions, and center the object on a reversible conveyor belt.

> Current status: the software has passed syntax checks and mock tests for the state machine, action validation, and serial protocol. The hardware bench-test checklist must still be completed with the real camera, YoloUNO board, and motor before the system is considered production-ready or safe for unattended operation.

## Architecture and Runtime Flow

```text
Connect the YoloUNO and camera
              |
              v
Calibration: select the four conveyor corners
              |
              v
OpenCV creates a 1000 x 300 perspective transform
              |
              v
Open the dashboard
  - calibrated real-time image on the left
  - chat, status, and logs on the right
              |
              v
The operator submits an object instruction
              |
              v
Calibrated image + instruction + conveyor parameters
              |
              v
Gemini generates a VLA action
  direction + duration_ms + task_status
              |
              v
The controller validates and displays the action tokens
              |
              v
The action is adapted to the firmware serial protocol
              |
              v
YoloUNO ACK -> motor movement -> DONE
              |
              v
Wait for settling, capture a fresh frame, and repeat
```

### Responsibility Boundaries

OpenCV is used only to:

- Capture camera frames.
- Let the operator select the four conveyor corners.
- Calculate and apply the perspective transform.
- Produce the calibrated `1000 x 300` image and draw the center target zone.
- Display and encode images.

Gemini is responsible for:

- Finding the object that matches the instruction submitted in the sidebar.
- Returning the normalized object center as `[y, x]`.
- Selecting `LEFT`, `RIGHT`, or `STOP`.
- Generating `duration_ms` and `task_status` directly.

The Python controller does not recalculate an action from the returned coordinates. It only verifies that Gemini's result is safe and valid before communicating with the firmware.

## VLA Actions and Firmware Protocol

A valid Gemini result is shown in the dashboard as an action-token sequence:

```text
[ACT_RIGHT] [DURATION_0780_MS] [STATUS_MOVE]
```

The bracketed action tokens are used only for display and auditing. The current firmware does not parse the bracketed representation. The controller adapts it to the protocol implemented in `conveyor_firmware/src/main.cpp`:

| Controller sends | Firmware responds | Meaning |
|---|---|---|
| `PING\n` | `PONG` | Verify the board connection |
| `STOP\n` | `STOPPED` | Stop the motor |
| `MOVE,RIGHT,780\n` | `ACK,RIGHT,780`, then `DONE` | Move right for 780 ms |
| `MOVE,LEFT,300\n` | `ACK,LEFT,300`, then `DONE` | Move left for 300 ms |

The host controller allows durations from `80` to `1500 ms`, even though the firmware accepts values from `50` to `3000 ms`.

`IMAGE_RIGHT_IS_FORWARD` in `conveyor_firmware/src/main.cpp` maps image-relative movement to the physical motor direction:

- When set to `false`, `MOVE,RIGHT` uses `MOTOR_BACKWARD`, while `MOVE,LEFT` uses `MOTOR_FORWARD`.
- If `MOVE,RIGHT` makes the object move left in the calibrated image, change this value to `true` and flash the firmware again.

## Instruction Lifecycle

1. After calibration, the dashboard enters `WAITING_FOR_PROMPT`.
2. The operator enters an instruction in the right sidebar and selects **Send**.
3. The composer is disabled while that instruction is active.
4. Gemini analyzes the latest frame and generates an action.
5. The controller validates the action and displays its action tokens before sending a serial command.
6. For `MOVE`, the controller requires the exact `ACK,<direction>,<duration>` before accepting `DONE`.
7. After every pulse, the controller waits for the object to settle and analyzes a fresh frame.
8. For `CENTERED`, the motor receives `STOP`, the sidebar displays `SUCCESS`, and the composer is enabled for the next instruction.

If the object is removed or no longer matches the instruction:

- A motor pulse that has already started is allowed to finish.
- The next inference sends or maintains `STOP`.
- The sidebar displays `WARNING: target not recognized - retry n/20`.
- If the matching object returns, the counter resets and control resumes.
- After 20 consecutive no-match results, the instruction is abandoned and the composer is enabled again.

Gemini connection failures, timeouts, rate limits, invalid JSON/schema, and invalid actions use a separate `n/5` technical retry counter:

- The motor remains at `STOP`.
- Network failures are displayed as `RECONNECTING`.
- Invalid model data or schema is displayed as `ERROR`.
- Any valid Gemini result resets the technical counter.
- The fifth consecutive technical failure triggers a safe shutdown.

## Installation and Startup

Create a virtual environment and install the dependencies:

```bash
python -m venv .venv
source .venv/bin/activate  # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

Create `.env` from the example file and add a Gemini API key:

```bash
cp .env.example .env
```

```dotenv
GEMINI_API_KEY=your_gemini_api_key
```

Verify the following constants in `controller.py` before starting:

```python
CAMERA_INDEX = 0
SERIAL_PORT = "COM18"
SERIAL_BAUD = 115200
BELT_LENGTH_CM = 77.0
SPEED_TEST_DISTANCE_CM = 77.0
SPEED_TEST_TIME_S = 5.5925
```

Close the PlatformIO Serial Monitor so it does not hold the COM port, then run:

```bash
python controller.py
```

### Calibration

1. Press `S` to capture the calibration frame or `Q` to exit.
2. Click the four corners of the conveyor region in any order.
3. Right-click to undo, or press `R` to start the selection again.
4. Press `Enter`, `Return`, or `Space` when the polygon is valid.
5. Do not move the camera after calibration. Recalibrate whenever the camera position changes.

After calibration, the dashboard opens with the calibrated real-time image on the left and the control sidebar on the right.

## Hardware Bench-Test Checklist

A bench test verifies the complete integration with the physical camera, controller board, and motor. Run the checks in the following order and do not skip the initial safety steps.

### 0. Safety Preparation

- Clear the hazardous area around the conveyor.
- Use a lightweight test object that is unlikely to jam the mechanism.
- Ensure that motor power can be disconnected immediately.
- Keep hands away from the drivetrain whenever a MOVE command may be sent.
- Close the PlatformIO Serial Monitor before opening the controller.

### 1. Firmware Handshake

- Flash the firmware from `conveyor_firmware` to the YoloUNO.
- Verify the `115200` baud rate and the selected COM port.
- Run `serial_test.py` only when conveyor movement is safe.
- Pass when the board returns `PONG`, `ACK,RIGHT,300`, and `DONE` in that order.
- Fail if ACK/DONE is missing, ACK contains the wrong direction or duration, or any `ERR,*` line appears.

> `serial_test.py` sends one `MOVE,RIGHT,300` pulse. Run it only when the motor can move safely.

### 2. Motor Direction

- Place an object where its movement is easy to observe.
- Send `MOVE,RIGHT,300`.
- Pass if the object moves right in the calibrated image.
- If it moves left, change `IMAGE_RIGHT_IS_FORWARD` in the firmware and flash the board again.
- Repeat with `MOVE,LEFT,300` and confirm that the object moves left.

### 3. Physical STOP and DONE

- Send a short MOVE command and observe the motor.
- Pass if the motor physically stops when the firmware emits `DONE`.
- Select **Stop / Exit** while the motor is moving.
- Pass if the controller sends STOP, the motor stops, and no delayed MOVE appears after the Stop action.
- Disconnect motor power immediately if the motor continues moving after `DONE` or `STOPPED`.

### 4. Camera and Calibration

- Calibrate from at least three reasonable camera angles or mounting heights.
- Confirm that the complete conveyor length maps into the `1000 x 300` warped image.
- Confirm that the two green lines consistently mark the physical center zone.
- Move the camera after calibration and verify that calibration is performed again before operation continues.

### 5. Real-Time Dashboard

- Confirm that the calibrated image on the left continues updating while idle, during Gemini analysis, while the motor is moving, and during retries.
- Confirm that the right sidebar includes an auto-scrolling transcript, current state, composer, Send, and Stop/Exit controls.
- Confirm that `INFO`, `SUCCESS`, `WARNING`, `RECONNECTING`, and `ERROR` messages are visually distinguishable.
- Confirm that the GUI remains responsive during Gemini API calls and firmware waits.

### 6. Closed-Loop Centering

- Place an object away from the center and submit an instruction that describes it accurately.
- Confirm that the action tokens appear before the firmware command.
- Confirm that the direction and duration in the action tokens exactly match the `MOVE` command.
- Pass when the object reaches the center, the firmware receives STOP, the sidebar displays SUCCESS, and the composer is enabled again.

### 7. Object Removal

- While centering is active, remove the object after a motor pulse has started.
- Pass if the current pulse completes, the next inference keeps the motor stopped, and the warning increases through `1/20`, `2/20`, and so on.
- Return the matching object and confirm that control resumes automatically and the no-match counter resets.
- Leave the object absent for 20 attempts and confirm that the instruction is abandoned while the application remains active.

### 8. Gemini Retries

- Temporarily interrupt network access or use a mock to produce a timeout or rate-limit error.
- Confirm that the sidebar displays `RECONNECTING n/5` and that the motor remains stopped.
- Use an invalid-schema mock response and confirm that the sidebar displays `ERROR` without sending MOVE.
- Pass if the fifth consecutive technical failure shuts the application down safely.

### 9. Camera Disconnection

- Disconnect the camera while the dashboard is idle and while an instruction is active.
- Pass if stale-frame detection enters ERROR, stops the motor, and shuts down safely.
- Fail if the controller continues using an old frame to issue MOVE actions.

### 10. Manual Buttons

- Test the physical forward and reverse buttons while the host is idle and while a host pulse is active.
- Confirm that manual input cannot leave the motor running beyond the duration requested by the host.
- This test is mandatory because the button task can change `motor_state` independently of host commands.

## Production Acceptance Criteria

Do not consider the system production-ready until all of the following conditions pass:

- `PING/PONG`, exact `ACK`, and `DONE` follow the documented protocol.
- `RIGHT/LEFT` matches movement in the calibrated image.
- The motor physically stops after `DONE`, STOP, and Stop/Exit.
- No MOVE command is sent after Stop/Exit begins.
- A disconnected camera cannot produce actions from stale frames.
- Calibration is correct at the deployed camera position.
- Closed-loop control centers the correct object and enables the composer again.
- The 20 no-match attempts and five technical retries behave as documented.
- Manual buttons do not violate the host controller's safety limits.

## Known Limitations and Risks

- The firmware emits `DONE` immediately after changing the shared motor state; the motor task may need a short additional interval to perform the physical I2C STOP.
- The manual button task can change `motor_state` while the firmware is still tracking a host pulse.
- `IMAGE_RIGHT_IS_FORWARD` depends on motor wiring and must be verified on the real system.
- The camera must remain fixed after calibration.

## Security

Never commit `.env` or an API key. `.env` is excluded from Git; share only `.env.example`.

## Firmware

The `conveyor_firmware` directory is the PlatformIO project for the YoloUNO/ESP32. The controller preserves the firmware's existing serial protocol and never sends bracketed action tokens directly to the board.

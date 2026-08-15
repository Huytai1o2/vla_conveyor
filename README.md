# Calibrated VLA Conveyor Control

A continuously running Vision-Language-Action controller for a reversible
YoloUNO conveyor. An operator calibrates the camera and enters unrestricted
natural-language instructions in one desktop GUI. Gemini identifies the requested
object, plans one or more ordered belt waypoints, and generates each movement
direction and duration.

> The software has passed syntax and pure/mock checks, but the complete system is
> not production-ready until the hardware bench-test checklist passes with the
> deployed camera, YoloUNO, motor, load, power supply, and safety procedure.

## Runtime Flow

```text
YoloUNO PING/PONG + camera frame check
                    |
                    v
One Tkinter dashboard opens
  left: raw camera / calibration / warped live image
  right: state, logs, calibration controls, chat
                    |
                    v
Capture one frame and click physical TL, BL, TR, BR
                    |
                    v
OpenCV creates a 1000 x 300 perspective transform
                    |
                    v
Enter one natural-language object + destination sequence
                    |
                    v
Gemini returns waypoints + perception + executable action
                    |
                    v
Python validates the current waypoint and action
                    |
                    v
Visible action tokens -> firmware MOVE/STOP command
                    |
                    v
Exact ACK -> timed motor pulse -> DONE -> 1.5 s settle
                    |
                    v
Fresh image -> next correction or next locked waypoint
                    |
                    v
Final waypoint reached -> next operator instruction
```

There is no fixed inference-step limit. The application continues accepting new
instructions until Stop/Exit is selected or a fatal error stops the controller.

## Responsibility Boundary

OpenCV is used for:

- camera capture;
- four-point calibration and perspective warping;
- conversion to a normalized `1000 x 300` belt image;
- JPEG encoding;
- display-only calibration markers, state text, target lines, and model point.

OpenCV does **not** detect, classify, segment, center, or track the object. It
does not generate motor duration.

Gemini is responsible for:

- matching the loose movable object described by the operator;
- parsing natural language into `1..12` ordered waypoint x values;
- returning the object's normalized `[y, x]` point;
- generating `LEFT`, `RIGHT`, or `STOP`;
- calculating `duration_ms` from the supplied belt calibration;
- returning `MOVE`, `AT_TARGET`, `TARGET_NOT_FOUND`, or
  `INVALID_INSTRUCTION`.

Python strictly validates Gemini's structured result. It checks the reported
point against the current waypoint, verifies that direction approaches the
destination, enforces duration limits, and blocks movement inside tolerance.
Python never creates a fallback MOVE or recalculates Gemini's duration.

The Gemini model configured in `controller.py` is
`gemini-robotics-er-1.6-preview`.

## Natural-Language Waypoints

The sidebar accepts natural language, not a command template. One message must
identify one object and one or more ordered destinations, for example:

```text
move the red tape roll to the right, then return it to the middle
move the white charger to 25% of the belt, then to the far right
đưa cuộn băng keo đỏ sang trái rồi về giữa băng chuyền
```

Gemini initially returns the complete ordered `waypoints_x` array in normalized
coordinates `0..1000`. Qualitative defaults are:

| Position | Normalized x |
|---|---:|
| far left / left edge | `0` |
| left | `200` |
| center / middle | `500` |
| right | `800` |
| far right / right edge | `1000` |

Exact percentages, fractions, centimeters, initial positions, or visible
landmarks override these defaults. The controller locks the first valid array.
Every subsequent model result must preserve that array and the active index.

For every waypoint:

- Outside the `4 cm` tolerance: validate and execute Gemini's alignment MOVE.
- At an intermediate waypoint: send STOP and advance to the next locked index.
- At the final waypoint: apply the final guard, send STOP, finish the instruction,
  and enable the composer.

## Action Tokens and Firmware Protocol

The GUI and terminal show the validated executable action as:

```text
[ACT_RIGHT] [DURATION_0780_MS] [STATUS_MOVE]
```

These bracketed tokens are for VLA observability and auditing. The firmware does
not parse them. `controller.py` adapts them to the protocol implemented by
`conveyor_firmware/src/main.cpp`:

| Host sends | Firmware response | Meaning |
|---|---|---|
| `PING\n` | `PONG` | Verify the YoloUNO connection |
| `STOP\n` | `STOPPED` | Request motor stop |
| `MOVE,RIGHT,780\n` | `ACK,RIGHT,780`, then `DONE` | Timed image-right pulse |
| `MOVE,LEFT,300\n` | `ACK,LEFT,300`, then `DONE` | Timed image-left pulse |

The host accepts Gemini MOVE durations from `80` through `1500 ms`. Firmware
constrains received durations to `50..3000 ms`. The host requires the exact ACK
before accepting DONE; mismatched/duplicate ACK, DONE before ACK, `ERR,*`, and
timeouts are fatal.

`IMAGE_RIGHT_IS_FORWARD` in `conveyor_firmware/src/main.cpp` maps image-relative
direction to motor wiring:

- Current value: `false`.
- `MOVE,RIGHT` therefore selects `MOTOR_BACKWARD`.
- `MOVE,LEFT` selects `MOTOR_FORWARD`.
- If RIGHT moves the object left in the calibrated image, change the constant to
  `true`, rebuild, and flash the firmware.

## Requirements

- Python 3.10 or newer is recommended.
- A desktop environment with Tkinter.
- Gemini API key.
- YoloUNO/ESP32 running the firmware in `conveyor_firmware`.
- UVC camera and reversible conveyor.

On Ubuntu/Debian, install Tkinter if it is missing:

```bash
sudo apt install python3-tk
```

Create the environment and install Python dependencies:

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Windows activation:

```powershell
.venv\Scripts\activate
```

## Configuration

Copy the example and add your own API key:

```bash
cp .env.example .env
```

Current example configuration:

```dotenv
GEMINI_API_KEY=your_gemini_api_key

CAMERA_DEVICE=auto
CAMERA_MATCH=UGREEN

SERIAL_PORT=/dev/ttyACM0
SERIAL_MATCH=Espressif
SERIAL_BAUD=115200
```

### Camera selection

- `CAMERA_DEVICE=auto` prefers a matching stable Linux
  `/dev/v4l/by-id/*-video-index0`, then `/dev/video*`.
- `CAMERA_MATCH=UGREEN` selects the external camera instead of the laptop webcam.
- Use an explicit path when needed, for example
  `CAMERA_DEVICE=/dev/v4l/by-id/usb-...-video-index0` or `/dev/video4`.
- On Windows, use a numeric index such as `CAMERA_DEVICE=0`.

### MCU selection

- The deployed YoloUNO currently appears as `/dev/ttyACM0`.
- `SERIAL_PORT=auto` prefers a matching `/dev/serial/by-id` entry and otherwise
  probes `ttyACM*`/`ttyUSB*` using `PING/PONG`.
- On Windows, use a value such as `SERIAL_PORT=COM18`.
- Close PlatformIO Serial Monitor before starting the controller because only one
  process can own the port.

The physical calibration constants are currently hard-coded in `controller.py`:

```python
BELT_LENGTH_CM = 77.0
SPEED_TEST_DISTANCE_CM = 77.0
SPEED_TEST_TIME_S = 5.5925
TARGET_TOLERANCE_CM = 4.0
```

Re-measure these values when belt length, motor speed, voltage, load, gearing, or
mechanics change.

## Running the Controller

```bash
python controller.py
```

Startup order:

1. Verify Tkinter/Pillow.
2. Connect to the YoloUNO and receive `PONG`.
3. Open the camera and receive a valid frame.
4. Open the single calibration/runtime dashboard.

The Gemini client is created lazily when the first instruction is analyzed.

## Calibration in the Dashboard

1. Confirm the complete conveyor is visible in the raw live view.
2. Select **Capture** or press `S`.
3. Click the physical belt corners in this exact semantic order:
   `1=top-left`, `2=bottom-left`, `3=top-right`, `4=bottom-right`.
4. Follow the belt's physical orientation even if the camera view looks rotated
   or inverted.
5. Right-click/**Undo** removes the last point.
6. `R`/**Reset** clears all points.
7. **Live** discards the frozen frame; **Retake** captures another one.
8. Select **Confirm** or press `Enter` after the polygon becomes valid.

The same left canvas then changes to the calibrated `1000 x 300` live image. No
OpenCV calibration window or second GUI opens. The composer becomes available
only after confirmation.

GUI shortcuts:

| Key | Action |
|---|---|
| `S` | Capture/retake during calibration |
| `R` | Reset calibration points while selecting |
| `Enter` | Confirm valid calibration |
| `Ctrl+Enter` | Submit a runtime instruction |
| `F11` | Toggle full-screen |
| `Esc` | Exit full-screen |

Resize and full-screen operations fit the image without cumulative zoom and map
calibration clicks back to the original camera frame.

## Runtime States and Logs

The right sidebar displays controller states including:

```text
CALIBRATION_LIVE, CALIBRATION_SELECTING, WAITING_FOR_PROMPT,
ANALYZING, MOVING, SETTLING, WAYPOINT_REACHED, TARGET_MISSING,
MODEL_RETRY, INVALID_INSTRUCTION, AT_TARGET, ERROR, STOPPING
```

Messages appear in both the GUI and terminal. Gemini exceptions additionally
print their Python traceback in the terminal.

The composer is disabled during calibration and while one instruction is active.
Blank messages are rejected; instructions are not queued behind an active task.

## Missing Object and Retry Behavior

If the requested object is absent, no longer matches the prompt, or confidence is
below `0.60`:

- the current pulse may finish;
- the next inference requests/maintains STOP;
- the sidebar logs `target not recognized - retry n/20`;
- the controller waits one second and uses a fresh frame;
- returning the matching object resets the no-match counter;
- attempt 20 abandons only that instruction and re-enables the composer.

Gemini connection, timeout, rate-limit, JSON/schema, and validation errors use a
separate technical counter:

- the motor is stopped before retry;
- API/runtime failures appear as `RECONNECTING`;
- invalid model data appears as `ERROR`;
- retry delay is two seconds;
- a valid result resets the counter;
- the fifth consecutive error is fatal.

HTTP `4xx` errors other than `408` and `429` are treated as non-retryable and
become fatal immediately.

A fatal runtime error sets the stop event, disables the composer, and leaves the
dashboard available to display the error. Select **Stop / Exit** or close the
window to finish resource cleanup.

## Camera and Serial Troubleshooting

### `VIDIOC_REQBUFS: errno=19 (No such device)`

The camera was opened but disappeared from Linux while OpenCV requested buffers.
This is normally a USB disconnect/reset, power, cable, hub, or UVC transport
problem—not a calibration-GUI error. Camera auto-reconnect is not implemented.

Check:

```bash
lsusb
ls -l /dev/v4l/by-id/
v4l2-ctl --list-devices
journalctl -k -n 100 --no-pager
```

Reconnect the camera directly to the laptop, try another port/cable, wait for the
stable by-id path to return, and restart `controller.py`.

### Camera cannot be opened

- Verify `CAMERA_DEVICE` and `CAMERA_MATCH`.
- Close browsers, video-call applications, and other camera viewers.
- Check ownership with `fuser /dev/videoX`.
- Confirm the user has permission to read the video device.

### YoloUNO cannot connect

- Check `ls -l /dev/ttyACM* /dev/serial/by-id/`.
- Close PlatformIO Serial Monitor.
- Verify `115200` baud and that the firmware prints `READY`/responds `PONG`.
- Confirm serial-device permissions (commonly membership in `dialout` on Linux).
- Use `SERIAL_PORT=auto` if the ACM device number changes.

### Gemini fails

- Confirm `GEMINI_API_KEY` is present in `.env` without quotes or trailing text.
- Read the complete `[GEMINI ERROR]` traceback in the terminal.
- A deterministic HTTP 4xx usually requires fixing the request, API access, model
  availability, billing, or credentials rather than retrying.

## Firmware and Serial Test

Build/flash the PlatformIO project in `conveyor_firmware`. The firmware uses:

- `115200` baud;
- motor output M4 over I2C;
- motor speed magnitude `100`;
- host pulse range constrained to `50..3000 ms`;
- physical forward/reverse buttons with a `20 s` hold timeout.

`serial_test.py` currently defaults to `PORT = "COM18"` and sends one real
`MOVE,RIGHT,300` pulse. Edit `PORT` before running it on Linux. Run it only when
physical movement is safe:

```bash
python serial_test.py
```

Expected order:

```text
PONG
ACK,RIGHT,300
DONE
```

## Hardware Bench-Test Checklist

### 0. Safety

- Clear the conveyor and use a lightweight non-jamming test object.
- Keep hands, hair, and clothing away from the drivetrain.
- Provide a way to disconnect motor power immediately.
- Do not treat the software Stop button as a certified emergency stop.

### 1. Firmware handshake and protocol

- Verify `READY`, `PING/PONG`, exact ACK, DONE, and `STOP/STOPPED`.
- Inject wrong ACK, stale DONE, missing DONE, and `ERR,*` using a mock before
  relying on the real motor.
- Confirm the motor physically stops after DONE and STOPPED.

### 2. Direction

- After calibration, send a short image-relative RIGHT pulse.
- Pass only if the object moves right in the calibrated image.
- Repeat for LEFT.
- If reversed, change `IMAGE_RIGHT_IS_FORWARD`, rebuild, and flash.

### 3. Calibration and scale

- Test multiple reasonable camera angles/heights, including a visually inverted
  view.
- Verify physical `TL, BL, TR, BR` fills the complete `1000 x 300` output.
- Resize/full-screen before clicking and verify the selected physical points stay
  correct.
- Verify requested waypoint/tolerance lines move to the active destination.
- Re-measure full-belt travel time under the deployed load.

### 4. Single and multi-waypoint control

- Test an arbitrary single destination rather than only center.
- Test natural language such as right → middle and left → right → middle.
- Verify `Locked waypoint sequence` appears once and remains unchanged.
- Verify each intermediate waypoint advances exactly once.
- Verify only the last waypoint completes the instruction.
- Confirm displayed validated tokens match each firmware MOVE.

### 5. Object removal

- Remove the requested object after a pulse begins.
- Verify the current pulse finishes, the next inference stops, and warnings count
  upward.
- Return the correct object and verify control resumes with the same waypoint.
- Leave it absent for 20 valid no-match responses and verify a new prompt becomes
  available without restarting the application.

### 6. Retry and invalid instruction

- Submit an ambiguous instruction and verify no MOVE is sent.
- Simulate timeout, rate-limit, invalid JSON, schema mismatch, and invalid action.
- Verify the motor remains stopped and counters reset after a valid result.
- Verify failure number five enters ERROR without sending another MOVE.

### 7. Disconnect and shutdown

- Disconnect the camera while calibrating, idle, analyzing, and moving.
- Disconnect the MCU during STOP and MOVE waits.
- Select Stop/Exit during Gemini analysis, settling, and a motor pulse.
- Verify no late model response produces a MOVE after shutdown begins.

### 8. Manual firmware buttons

- Test both physical buttons while host control is idle and active.
- Verify release stops the motor and the `20 s` timeout prevents indefinite hold.
- Confirm button writes to shared `motor_state` cannot violate the host safety
  expectations for the deployed operating procedure.

## Known Limitations

- Camera auto-reconnect is not implemented; a USB reset requires restart and
  recalibration.
- Calibration is not saved between runs.
- Physical belt constants and Gemini model name are code constants, not `.env`
  settings.
- Control depends on Gemini API latency, availability, and model quality.
- The firmware's manual buttons and host commands share the same global
  `motor_state`.
- Firmware emits DONE when it updates the shared stop state; the motor task applies
  the I2C stop on its next polling cycle.
- The GUI remains open after a fatal worker error so the operator can read the log;
  close it to release all resources.
- A physical emergency stop and guarded mechanical design are still required for
  safe deployment.

## Production Acceptance Criteria

Do not approve unattended operation until all of these pass on the real system:

- correct camera calibration and physical scale at the installed position;
- correct image-relative LEFT/RIGHT mapping;
- exact firmware handshake, ACK/DONE ordering, and reliable STOP;
- correct arbitrary and multi-waypoint completion;
- no movement for missing/wrong objects, invalid instructions, or invalid model
  output;
- safe behavior for object removal, camera loss, MCU loss, Gemini failure, and
  Stop/Exit races;
- manual buttons cannot defeat the operating safety procedure;
- independent physical emergency-stop capability is available.

## Security

Never commit `.env` or an API key. `.env` and debug images are ignored by Git;
share only `.env.example`.

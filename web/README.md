# Conveyor VLA Web Dashboard

Minimal black-and-white operator UI built with React, Vite, Tailwind CSS, and
shadcn-style local components.

## Status

The frontend, FastAPI/WebSocket gateway, aiortc video path, ZMQ bridge, web
calibration flow, and one-command launcher are implemented. The controller
service imports and reuses `controller.py`; it does not replace the existing VLA,
waypoint, retry, serial, or motor-safety logic.

## Layout

- Desktop: `2/3` live calibrated video and `1/3` VLA conversation.
- Mobile/small window: stacked video and conversation panels.
- Monochrome shadcn-style components and Claude Code-inspired transcript.
- Persistent Stop control, controller state, current waypoint, MCU, camera, and
  action telemetry.
- Full-screen and contain/cover video controls.

## Production-style local run

From the repository root:

```bash
source .venv/bin/activate
pip install -r requirements.txt
cd web && npm install && npm run build && cd ..
python web_server.py
```

Open <http://127.0.0.1:8000>.

`web_server.py` starts the hardware-owning controller service in a child process,
starts the FastAPI/aiortc gateway, and serves `web/dist` from the same origin.
Stop it with the web **Stop** button followed by `Ctrl+C`, or directly with
`Ctrl+C` when the controller is idle.

## Frontend development

```bash
cd web
npm install
cp .env.example .env
npm run dev
```

Production verification:

```bash
npm run typecheck
npm run build
```

`VITE_GATEWAY_URL` is read only by `npm run dev`. Production builds always use
the page origin so `localhost` and `127.0.0.1` cannot accidentally create a CORS
split. During local split development it normally points to the Python gateway:

```dotenv
VITE_GATEWAY_URL=http://127.0.0.1:8000
```

## Browser-to-Gateway Contract

### WebRTC offer

The browser sends a non-trickle, receive-only video offer:

```http
POST /api/webrtc/offer
Content-Type: application/json

{"sdp":"...","type":"offer"}
```

The Python `aiortc` gateway returns:

```json
{"sdp":"...","type":"answer"}
```

The gateway must use frames published by the controller's existing
`CameraStream`; it must not open the UVC camera a second time.

### Operator WebSocket

The browser connects to:

```text
GET /ws/events
```

Browser commands:

```json
{"kind":"subscribe","channel":"operator"}
{"kind":"calibration_capture"}
{"kind":"calibration_point","x":0.15,"y":0.20}
{"kind":"calibration_undo"}
{"kind":"calibration_reset"}
{"kind":"calibration_live"}
{"kind":"calibration_confirm"}
{"kind":"prompt","instruction":"move the red tape roll right, then middle"}
{"kind":"stop"}
```

Gateway events preserve the current controller event names:

```json
{"kind":"message","severity":"INFO","message":"Locked waypoint sequence: [800, 500]"}
{"kind":"state","state":"ANALYZING"}
{"kind":"prompt_ready"}
{"kind":"fatal"}
```

Optional telemetry snapshot/update:

```json
{
  "kind": "telemetry",
  "step": 4,
  "direction": "RIGHT",
  "waypointIndex": 0,
  "waypointCount": 2,
  "destinationX": 800,
  "camera": "UGREEN 2K",
  "mcu": "YoloUNO",
  "model": "Gemini Robotics"
}
```

## ZMQ Boundary

ZMQ belongs behind the Python gateway, never in the browser:

```text
React browser <-- WebRTC + WebSocket --> Python gateway <-- ZMQ --> controller
```

Implemented channels:

- `ROUTER/DEALER` on `CONVEYOR_COMMAND_ENDPOINT`: prompt, calibration, STOP,
  snapshots, and correlated acknowledgements.
- `PUB/SUB` on `CONVEYOR_EVENT_ENDPOINT`: state, transcript, waypoint, and health
  telemetry.
- Latest-frame-only bounded `PUB/SUB` on `CONVEYOR_FRAME_ENDPOINT`: fixed-size
  JPEG preview frames consumed by `aiortc`.

Motor commands must remain inside `controller.py`; the browser submits operator
intent and Stop requests, not raw firmware MOVE commands.

# VLA Conveyor Control

Vision-Language-Action controller for centering an object on a reversible conveyor.

## Python controller

Create and activate a virtual environment, then install the dependencies:

```bash
python -m venv .venv
source .venv/bin/activate  # Windows: .venv\\Scripts\\activate
pip install -r requirements.txt
```

Set the credentials required by the Google GenAI SDK, configure the camera and serial-port constants in `controller.py`, then run:

```bash
python controller.py
```

## Firmware

The `conveyor_firmware` directory is a PlatformIO project for the conveyor controller board.

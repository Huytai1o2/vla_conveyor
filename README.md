# VLA Conveyor Control

Vision-Language-Action controller for centering an object on a reversible conveyor.

## Python controller

Create and activate a virtual environment, then install the dependencies:

```bash
python -m venv .venv
source .venv/bin/activate  # Windows: .venv\\Scripts\\activate
pip install -r requirements.txt
```

Create a local `.env` file from the example and add a Gemini API key created in [Google AI Studio](https://aistudio.google.com/app/apikey):

```bash
cp .env.example .env
```

Open `.env` and set the value (keep the key private):

```dotenv
GEMINI_API_KEY=your_gemini_api_key
```

The controller automatically loads this file. Configure the camera and serial-port constants in `controller.py`, then run:

```bash
python controller.py
```

Never commit `.env`; it is excluded by `.gitignore`. Use `.env.example` as the shareable template.

## Firmware

The `conveyor_firmware` directory is a PlatformIO project for the conveyor controller board.

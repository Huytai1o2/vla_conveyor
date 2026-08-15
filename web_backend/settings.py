from __future__ import annotations

import os
from pathlib import Path

from dotenv import load_dotenv


ROOT_DIR = Path(__file__).resolve().parents[1]
WEB_DIST_DIR = ROOT_DIR / "web" / "dist"
load_dotenv(ROOT_DIR / ".env")

COMMAND_ENDPOINT = os.getenv(
    "CONVEYOR_COMMAND_ENDPOINT",
    "tcp://127.0.0.1:5555",
).strip()
EVENT_ENDPOINT = os.getenv(
    "CONVEYOR_EVENT_ENDPOINT",
    "tcp://127.0.0.1:5556",
).strip()
FRAME_ENDPOINT = os.getenv(
    "CONVEYOR_FRAME_ENDPOINT",
    "tcp://127.0.0.1:5557",
).strip()

WEB_HOST = os.getenv("WEB_HOST", "127.0.0.1").strip() or "127.0.0.1"
WEB_PORT = int(os.getenv("WEB_PORT", "8000"))
WEB_LOG_LEVEL = os.getenv("WEB_LOG_LEVEL", "info").strip() or "info"
_configured_origins = [
    value.strip()
    for value in os.getenv(
        "WEB_ALLOWED_ORIGINS",
        "",
    ).split(",")
    if value.strip()
]
WEB_ALLOWED_ORIGINS = list(
    dict.fromkeys(
        [
            *_configured_origins,
            "http://localhost:5173",
            "http://127.0.0.1:5173",
            "http://localhost:8000",
            "http://127.0.0.1:8000",
        ]
    )
)

PREVIEW_WIDTH = 1000
PREVIEW_HEIGHT = 600
PREVIEW_FPS = 15.0
PREVIEW_JPEG_QUALITY = 82

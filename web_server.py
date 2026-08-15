from __future__ import annotations

import multiprocessing

import uvicorn

from web_backend.controller_service import run_controller_service
from web_backend.gateway import app, set_server_shutdown_callback
from web_backend.settings import WEB_HOST, WEB_LOG_LEVEL, WEB_PORT


def main() -> None:
    context = multiprocessing.get_context("spawn")
    shutdown_event = context.Event()
    controller_process = context.Process(
        target=run_controller_service,
        args=(shutdown_event,),
        name="conveyor-controller-service",
    )
    controller_process.start()
    try:
        server = uvicorn.Server(
            uvicorn.Config(
                app,
                host=WEB_HOST,
                port=WEB_PORT,
                log_level=WEB_LOG_LEVEL,
                reload=False,
            )
        )
        set_server_shutdown_callback(
            lambda: setattr(server, "should_exit", True)
        )
        server.run()
    finally:
        shutdown_event.set()
        controller_process.join(timeout=25.0)
        if controller_process.is_alive():
            controller_process.terminate()
            controller_process.join(timeout=5.0)


if __name__ == "__main__":
    multiprocessing.freeze_support()
    main()

#!/usr/bin/env python3
"""One-command launcher for the FIELDLINE AMR simulation dashboard.

Starts the FastAPI backend (server.py), which launches the three
fleet_controller.py processes and drives the simulation clock, then opens
the dashboard in the browser. Ctrl+C stops everything.
"""

import socket
import threading
import time
import webbrowser

import uvicorn

HOST = "127.0.0.1"
PORT = 8000
URL = f"http://{HOST}:{PORT}"


def open_when_ready() -> None:
    # uvicorn binds the port only after startup (controllers + clock) finishes.
    for _ in range(300):
        try:
            socket.create_connection((HOST, PORT), timeout=0.2).close()
        except OSError:
            time.sleep(0.1)
            continue
        print(f"[DEMO] Dashboard ready at {URL}", flush=True)
        webbrowser.open(URL)
        return
    print("[DEMO] Server did not come up; check the log above.", flush=True)


def main() -> int:
    print("=" * 72)
    print("FIELDLINE — EDGE-AI DISTRIBUTED AMR FLEET DEMO")
    print("=" * 72)
    print(f"Dashboard  : {URL}")
    print("Controllers: AMR-1 / AMR-2 / AMR-3 (auction run by AMR-1)")
    print("Fleet      : starts idle; the clock runs live")
    print("=" * 72)

    threading.Thread(target=open_when_ready, daemon=True).start()
    uvicorn.run("server:app", host=HOST, port=PORT, log_level="warning")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

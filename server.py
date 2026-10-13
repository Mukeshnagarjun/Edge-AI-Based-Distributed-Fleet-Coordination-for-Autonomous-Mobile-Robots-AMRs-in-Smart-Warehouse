"""FIELDLINE simulation backend.

Start, then open http://localhost:8000
    .venv/bin/uvicorn server:app --port 8000

The fleet controllers stay the source of truth. This server launches them,
runs the clock that drives them and exposes their combined state.
"""

import asyncio
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from pydantic import BaseModel

from scenario import AMR_CONFIG, HEIGHT, OBSTACLES, WIDTH
from simulation import SimulationManager

manager = None


@asynccontextmanager
async def lifespan(_app):
    global manager
    manager = SimulationManager()
    manager.start()
    yield
    manager.shutdown()


app = FastAPI(title="FIELDLINE AMR Simulation", lifespan=lifespan)


@app.get("/", include_in_schema=False)
def dashboard():
    return FileResponse(Path(__file__).with_name("static") / "index.html")


@app.get("/api/scenario")
def scenario():
    return {
        "width": WIDTH,
        "height": HEIGHT,
        "obstacles": sorted(OBSTACLES),
        "amrs": [
            {"amr_id": amr_id, "start": robot["start"], "priority": robot["priority"]}
            for amr_id, robot in AMR_CONFIG.items()
        ],
    }


@app.get("/api/state")
def state():
    return manager.snapshot()


class TaskRequest(BaseModel):
    pickup: tuple[int, int]
    dropoff: tuple[int, int]


@app.post("/api/tasks", status_code=201)
def add_task(request: TaskRequest):
    try:
        return manager.add_task(request.pickup, request.dropoff)
    except ValueError as error:
        raise HTTPException(status_code=422, detail=str(error))
    except RuntimeError as error:
        raise HTTPException(status_code=409, detail=str(error))


class AmrRequest(BaseModel):
    start: tuple[int, int]


@app.post("/api/amrs", status_code=201)
def add_amr(request: AmrRequest):
    try:
        return manager.add_amr(request.start)
    except ValueError as error:
        raise HTTPException(status_code=422, detail=str(error))
    except RuntimeError as error:
        raise HTTPException(status_code=409, detail=str(error))


@app.websocket("/ws")
async def updates(websocket: WebSocket):
    """Push the fleet state whenever it changes."""
    await websocket.accept()
    sent = None
    try:
        while True:
            if manager.version != sent:
                snapshot = manager.snapshot()
                sent = snapshot["version"]
                await websocket.send_json(snapshot)
            await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        pass

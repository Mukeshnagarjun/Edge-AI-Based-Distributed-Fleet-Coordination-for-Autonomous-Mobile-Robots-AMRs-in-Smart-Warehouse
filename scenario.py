# ============================================================
# SHARED WAREHOUSE SCENARIO
# Single source of truth for the grid, AMR fleet and demo tasks.
# Imported by fleet_controller.py, simulation.py, server.py and main.py.
# ============================================================

WIDTH = 15
HEIGHT = 10


# Static warehouse obstacles / racks
OBSTACLES = {
    (4, 2), (4, 3), (4, 4), (4, 5),
    (8, 4), (8, 5), (8, 6),
    (11, 2), (11, 3), (11, 4), (11, 5),
}


# Lower priority number = higher priority
AMR_CONFIG = {
    "AMR-1": {
        "port": 7001,
        "start": (1, 1),
        "priority": 1,
    },

    "AMR-2": {
        "port": 7002,
        "start": (7, 1),
        "priority": 2,
    },

    "AMR-3": {
        "port": 7003,
        "start": (13, 8),
        "priority": 3,
    },
}


# Sample tasks for tests and the A* self-check. The live fleet starts idle
# and is not given these.
TASKS = {
    "TASK-01": {
        "task_id": "TASK-01",
        "pickup": (10, 2),
        "dropoff": (3, 7),
    },

    "TASK-02": {
        "task_id": "TASK-02",
        "pickup": (8, 2),
        "dropoff": (13, 7),
    },
}


# Observation-only telemetry channel to the visualizer
TELEMETRY_HOST = "127.0.0.1"
TELEMETRY_PORT = 7100


# Simulation clock: sends TICK phases to every AMR and receives their acks
CLOCK_PORT = 7200

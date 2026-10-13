"""Simulation clock for the AMR fleet controllers.

One tick is one logical simulation step with two phases:
    INTENT: every AMR announces its next cell to its peers
    MOVE:   every AMR resolves conflicts, then moves or waits
The clock waits for every AMR to acknowledge each phase before going on,
so stepping once moves each AMR by at most one cell. Each acknowledgement
carries the AMR's position, which lets the clock check every tick for two
AMRs in one cell or two AMRs swapping cells.
"""

import json
import os
import socket
import subprocess
import sys
import threading
import time
from collections import deque
from pathlib import Path

from scenario import AMR_CONFIG, CLOCK_PORT, HEIGHT, OBSTACLES, TELEMETRY_HOST, TELEMETRY_PORT, WIDTH

BASE_DIR = Path(__file__).resolve().parent

# Simulated seconds per tick: one tick is one cell of travel.
SIM_SECONDS_PER_TICK = 1.0

# The live fleet ticks at a fixed rate, in ticks per wall-clock second.
TICK_RATE = 1.0

# Telemetry statuses worth an event, and what to call them.
STATUS_EVENTS = {
    "WAITING": "AMR_WAITING",
    "PICKUP_REACHED": "PICKUP_REACHED",
    "TASK_COMPLETE": "TASK_COMPLETED",
}

ALLOCATION_NOTE = (
    "Auction run by AMR-1: every idle AMR computes its own bid (route length "
    "to pickup plus pickup to drop-off); AMR-1 picks the lowest bid."
)

# Ports 7001-7099 are free for controllers; 7100 is telemetry.
MAX_AMRS = 10

# Task status from the AMR telemetry statuses that move it along.
TASK_PROGRESS = {
    "PICKUP_REACHED": "TO_DROPOFF",
    "TASK_COMPLETE": "COMPLETED",
}


class ClockError(RuntimeError):
    """An AMR did not acknowledge a phase in time."""


def detect_collisions(before, after):
    """Return collisions in one tick, given {amr_id: (x, y)} before and after."""
    collisions = []
    ids = sorted(after)
    for i, a in enumerate(ids):
        for b in ids[i + 1:]:
            if after[a] == after[b]:
                collisions.append({"type": "SAME_CELL", "amrs": [a, b], "cell": list(after[a])})
            elif before.get(a) == after[b] and before.get(b) == after[a]:
                collisions.append({"type": "SWAP", "amrs": [a, b], "cells": [list(after[b]), list(after[a])]})
    return collisions


class TickClock:

    def __init__(self, amr_ports=None, port=CLOCK_PORT, ack_timeout=5.0):
        self.amr_ports = amr_ports or {
            amr_id: robot["port"] for amr_id, robot in AMR_CONFIG.items()
        }
        self.ack_timeout = ack_timeout
        self.tick = 0
        self.positions = {}
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", port))

    def _broadcast(self, tick, phase):
        data = json.dumps({"type": "TICK", "tick": tick, "phase": phase}).encode()
        for amr_port in self.amr_ports.values():
            self.sock.sendto(data, ("127.0.0.1", amr_port))

    def _collect(self, tick, phase, timeout):
        acks = {}
        deadline = time.monotonic() + timeout
        while len(acks) < len(self.amr_ports):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                missing = ", ".join(sorted(set(self.amr_ports) - set(acks)))
                raise ClockError(f"No {phase} acknowledgement for tick {tick} from {missing}")
            self.sock.settimeout(remaining)
            try:
                data, _ = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            try:
                message = json.loads(data)
            except ValueError:
                continue
            if (
                message.get("type") == "TICK_ACK"
                and message.get("tick") == tick
                and message.get("phase") == phase
                and message.get("amr_id") in self.amr_ports
            ):
                acks[message["amr_id"]] = message
        return acks

    @staticmethod
    def _positions(acks):
        return {amr_id: tuple(ack["position"]) for amr_id, ack in acks.items()}

    def sync(self, timeout=60.0):
        """Wait until every controller answers. SYNC is idempotent, so retry."""
        deadline = time.monotonic() + timeout
        while True:
            self._broadcast(0, "SYNC")
            try:
                self.positions = self._positions(self._collect(0, "SYNC", 0.5))
                return self.positions
            except ClockError:
                if time.monotonic() >= deadline:
                    raise

    def add_amr(self, amr_id, port, timeout=10.0):
        """Admit a running controller. Call between ticks, never during step()."""
        deadline = time.monotonic() + timeout
        data = json.dumps({"type": "TICK", "tick": 0, "phase": "SYNC"}).encode()
        while time.monotonic() < deadline:
            self.sock.sendto(data, ("127.0.0.1", port))
            self.sock.settimeout(0.5)
            try:
                message = json.loads(self.sock.recvfrom(65535)[0])
            except (socket.timeout, ValueError):
                continue
            if message.get("type") == "TICK_ACK" and message.get("amr_id") == amr_id:
                self.amr_ports[amr_id] = port
                self.positions[amr_id] = tuple(message["position"])
                return
        raise ClockError(f"{amr_id} did not answer the clock")

    def step(self):
        """Advance exactly one tick and report where every AMR ended up."""
        self.tick += 1
        self._broadcast(self.tick, "INTENT")
        before = self._positions(self._collect(self.tick, "INTENT", self.ack_timeout))
        self._broadcast(self.tick, "MOVE")
        acks = self._collect(self.tick, "MOVE", self.ack_timeout)
        after = self._positions(acks)
        self.positions = after
        return {
            "tick": self.tick,
            "positions": after,
            "statuses": {amr_id: ack.get("status") for amr_id, ack in acks.items()},
            "collisions": detect_collisions(before, after),
        }

    def close(self):
        self.sock.close()


class SimulationManager:
    """Runs the controller processes and the clock; serves one fleet state.

    Once every controller joins, the clock ticks on its own at TICK_RATE,
    like a live fleet. Movement still happens only on ticks.
    """

    def __init__(self, telemetry_port=TELEMETRY_PORT, log_dir=BASE_DIR / "logs"):
        self.lock = threading.RLock()
        self.tick_lock = threading.Lock()
        self.log_dir = Path(log_dir)
        self.mode = "stopped"
        self.error = None
        self.version = 0
        self.tick = 0
        self.clock = None
        self.processes = []
        self.amrs = {}
        self.events = deque(maxlen=500)
        self.collisions = []
        self.tasks = {}
        self.fleet = {}
        self._closing = threading.Event()
        self._wake = threading.Event()

        self.telemetry_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.telemetry_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.telemetry_sock.bind((TELEMETRY_HOST, telemetry_port))
        self.telemetry_sock.settimeout(0.5)

        threading.Thread(target=self._receive_telemetry, daemon=True).start()
        threading.Thread(target=self._run_loop, daemon=True).start()

    # ---------------------------------------------------------- lifecycle

    def start(self):
        """Launch every controller; the clock starts once all of them join."""
        with self.lock:
            self._stop_processes()
            self.clock = TickClock()
            self.mode = "starting"
            self.error = None
            self.tick = 0
            self.amrs = {}
            self.events.clear()
            self.collisions = []
            self.tasks = {}
            self.fleet = {amr_id: dict(robot) for amr_id, robot in AMR_CONFIG.items()}
            self._changed()

            self.log_dir.mkdir(exist_ok=True)
            env = dict(os.environ, PYTHONUNBUFFERED="1")
            self.processes = []
            for amr_id in self.fleet:
                self._launch(amr_id, env)
            clock = self.clock

        threading.Thread(target=self._sync, args=(clock,), daemon=True).start()

    def _sync(self, clock):
        try:
            clock.sync()
        except (ClockError, OSError) as error:
            if clock is self.clock:  # otherwise a restart replaced it
                self._fail(f"Controllers did not start: {error}")
            return
        with self.lock:
            if clock is self.clock and self.mode == "starting":
                self.mode = "running"
                self._changed()
        self._wake.set()

    def _launch(self, amr_id, env):
        with open(self.log_dir / f"{amr_id}.log", "w") as log:
            self.processes.append(subprocess.Popen(
                [sys.executable, str(BASE_DIR / "fleet_controller.py"), amr_id],
                cwd=BASE_DIR, env=env, stdout=log, stderr=subprocess.STDOUT,
            ))

    def shutdown(self):
        self._closing.set()
        self._wake.set()
        with self.lock:
            self._stop_processes()
        self.telemetry_sock.close()

    def _stop_processes(self):
        for process in self.processes:
            process.terminate()
        for process in self.processes:
            try:
                process.wait(3)
            except subprocess.TimeoutExpired:
                process.kill()
        self.processes = []
        if self.clock is not None:
            self.clock.close()
            self.clock = None

    # --------------------------------------------------------------- tasks

    def add_task(self, pickup, dropoff):
        """Queue an operator task; AMR-1 auctions it to the fleet."""
        for name, (x, y) in (("pickup", pickup), ("dropoff", dropoff)):
            if not (0 <= x < WIDTH and 0 <= y < HEIGHT):
                raise ValueError(f"{name} ({x}, {y}) is outside the {WIDTH}x{HEIGHT} warehouse")
            if (x, y) in OBSTACLES:
                raise ValueError(f"{name} ({x}, {y}) is a rack")
        if tuple(pickup) == tuple(dropoff):
            raise ValueError("pickup and drop-off must differ")
        with self.lock:
            if self.mode != "running":
                raise RuntimeError(f"Fleet is {self.mode}; tasks can be added once it is live")
            task = {
                "task_id": f"T-{len(self.tasks) + 1:02d}",
                "pickup": list(pickup),
                "dropoff": list(dropoff),
                "status": "AUCTION",
                "winner": None,
                "cost": None,
                "bids": [],
                "created_tick": self.tick,
            }
            self.tasks[task["task_id"]] = task
            self._event(None, "TASK_CREATED", {k: task[k] for k in ("task_id", "pickup", "dropoff")})
            self._changed()
        message = {"type": "OPERATOR_TASK", "task": {k: task[k] for k in ("task_id", "pickup", "dropoff")}}
        self.telemetry_sock.sendto(json.dumps(message).encode(), ("127.0.0.1", AMR_CONFIG["AMR-1"]["port"]))
        return dict(task)

    # ---------------------------------------------------------------- fleet

    def add_amr(self, start):
        """Start one more controller at `start` and bring it into the live fleet."""
        x, y = start
        if not (0 <= x < WIDTH and 0 <= y < HEIGHT):
            raise ValueError(f"({x}, {y}) is outside the {WIDTH}x{HEIGHT} warehouse")
        if (x, y) in OBSTACLES:
            raise ValueError(f"({x}, {y}) is a rack")
        with self.lock:
            if self.mode != "running":
                raise RuntimeError(f"Fleet is {self.mode}; AMRs can be added once it is live")
            if len(self.fleet) >= MAX_AMRS:
                raise ValueError(f"The fleet is full ({MAX_AMRS} AMRs)")
            occupied = {tuple(a["position"]) for a in self.amrs.values() if a["position"]}
            occupied |= {tuple(r["start"]) for a, r in self.fleet.items() if a not in self.amrs}
            if (x, y) in occupied:
                raise ValueError(f"({x}, {y}) is occupied by an AMR")
            number = len(self.fleet) + 1
            amr_id = f"AMR-{number}"
            robot = {"port": 7000 + number, "start": [x, y], "priority": number}

            # Peers learn the newcomer's priority before it first broadcasts.
            joined = json.dumps({"type": "AMR_JOINED", "amr_id": amr_id, **robot}).encode()
            for peer in self.fleet.values():
                self.telemetry_sock.sendto(joined, ("127.0.0.1", peer["port"]))

            self.fleet[amr_id] = robot
            env = dict(os.environ, PYTHONUNBUFFERED="1", AMR_FLEET=json.dumps(self.fleet))
            self._launch(amr_id, env)
            self._event(amr_id, "AMR_ADDED", {"start": [x, y], "priority": number})
            self._changed()
            clock = self.clock
        threading.Thread(target=self._admit, args=(clock, amr_id, robot["port"]), daemon=True).start()
        return {"amr_id": amr_id, **robot}

    def _admit(self, clock, amr_id, port):
        # Wait for its first telemetry outside the tick lock, so the fleet
        # keeps moving while the new controller starts up.
        deadline = time.monotonic() + 60
        while amr_id not in self.amrs:
            if time.monotonic() > deadline or clock is not self.clock:
                if clock is self.clock:
                    self._fail(f"{amr_id} did not start")
                return
            time.sleep(0.1)
        with self.tick_lock:
            if clock is not self.clock:
                return
            try:
                clock.add_amr(amr_id, port)
            except (ClockError, OSError) as error:
                self._fail(str(error))
                return
        with self.lock:
            self._event(amr_id, "AMR_JOINED_CLOCK", {"position": self.amrs[amr_id]["position"]})
            self._changed()

    # --------------------------------------------------------------- ticks

    def _run_loop(self):
        while not self._closing.is_set():
            if self.mode == "running":
                started = time.monotonic()
                self._advance()
                delay = 1.0 / TICK_RATE - (time.monotonic() - started)
                self._wake.wait(max(0.0, delay))
            else:
                self._wake.wait(0.2)
            self._wake.clear()

    def _advance(self):
        with self.tick_lock:
            clock = self.clock
            if clock is None:
                return
            try:
                result = clock.step()
            except (ClockError, OSError) as error:
                if clock is self.clock:  # otherwise a restart replaced it
                    self._fail(str(error))
                return
        with self.lock:
            if clock is not self.clock:
                return  # restart happened mid-tick
            self.tick = result["tick"]
            for collision in result["collisions"]:
                collision = dict(collision, tick=self.tick)
                self.collisions.append(collision)
                self._event(None, "COLLISION", collision, severity="critical")
            self._changed()

    def _fail(self, message):
        with self.lock:
            self.mode = "error"
            self.error = message
            self._event(None, "SIMULATION_ERROR", {"message": message}, severity="critical")
            self._changed()

    # ----------------------------------------------------------- telemetry

    def _receive_telemetry(self):
        while not self._closing.is_set():
            try:
                data, _ = self.telemetry_sock.recvfrom(65535)
                message = json.loads(data)
            except socket.timeout:
                continue
            except (OSError, ValueError):
                if self._closing.is_set():
                    return
                continue
            if message.get("type") == "TELEMETRY" and message.get("amr_id") in self.fleet:
                self._apply_telemetry(message)

    def _apply_telemetry(self, message):
        amr_id = message["amr_id"]
        with self.lock:
            previous = self.amrs.get(amr_id, {})
            self.amrs[amr_id] = {
                "amr_id": amr_id,
                "priority": message.get("priority"),
                "position": message.get("position"),
                "next_position": message.get("next_position"),
                "target": message.get("target"),
                "status": message.get("status"),
                "fleet_status": message.get("fleet_status"),
                "task_id": message.get("assigned_task"),
                "path": message.get("path") or [],
                "path_index": message.get("path_index", 0),
                "metrics": message.get("metrics") or {},
                "edge_ai": message.get("edge_ai") or {},
                "dynamic_obstacles": message.get("dynamic_obstacles") or [],
            }
            current = self.amrs[amr_id]

            if current["task_id"] and current["task_id"] != previous.get("task_id"):
                self._event(amr_id, "TASK_ASSIGNED", {"task_id": current["task_id"]})
            if current["status"] != previous.get("status") and current["status"] in STATUS_EVENTS:
                self._event(amr_id, STATUS_EVENTS[current["status"]], {
                    "task_id": current["task_id"], "position": current["position"],
                    "next_position": current["next_position"],
                }, severity="warning" if current["status"] == "WAITING" else "info")

            event = message.get("event")
            data = message.get("event_data") or {}
            task = self.tasks.get(data.get("task_id") or current["task_id"])
            if task and current["status"] in TASK_PROGRESS and current["status"] != previous.get("status"):
                task["status"] = TASK_PROGRESS[current["status"]]
            if event == "AUCTION_RESULT":
                if task:
                    task.update(
                        status="ASSIGNED" if data["winner"] else "UNASSIGNED",
                        winner=data["winner"], cost=data["cost"], bids=data["bids"],
                    )
                self._event(amr_id, event, data, severity="info" if data["winner"] else "warning")
            elif event == "TASK_FAILED":
                if task:
                    task["status"] = "FAILED"
                self._event(amr_id, event, data, severity="warning")
            elif event == "EDGE_AI_RISK":
                if data.get("action") != "MOVE":
                    self._event(amr_id, "EDGE_AI_ADVICE", data, severity="warning")
            elif event:
                self._event(amr_id, event, data, severity="warning")
            self._changed()

    # ------------------------------------------------------------- state

    def _event(self, amr_id, event_type, data, severity="info"):
        # Telemetry sent during tick N arrives before step() returns, so use
        # the clock's in-progress tick rather than the last completed one.
        tick = self.clock.tick if self.clock is not None else self.tick
        self.events.append({
            "tick": tick,
            "sim_time": tick * SIM_SECONDS_PER_TICK,
            "amr_id": amr_id,
            "type": event_type,
            "severity": severity,
            "data": data,
        })

    def _changed(self):
        self.version += 1

    def snapshot(self):
        with self.lock:
            return {
                "version": self.version,
                "mode": self.mode,
                "error": self.error,
                "tick_rate": TICK_RATE,
                "tick": self.tick,
                "sim_time": self.tick * SIM_SECONDS_PER_TICK,
                "sim_seconds_per_tick": SIM_SECONDS_PER_TICK,
                "allocation": ALLOCATION_NOTE,
                "amrs": [self.amrs[a] for a in self.fleet if a in self.amrs],
                "collisions": list(self.collisions),
                "tasks": [dict(t) for t in self.tasks.values()],
                "events": list(self.events),
            }

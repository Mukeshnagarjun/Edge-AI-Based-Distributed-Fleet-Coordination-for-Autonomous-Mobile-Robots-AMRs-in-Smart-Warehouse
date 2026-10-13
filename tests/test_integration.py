"""End-to-end: three real controller processes driven by the tick clock.

Binds the real controller ports (7001-7003) and the clock port (7200), so
stop run_demo.py (or any running server.py) first. Takes about 20 seconds.

Run from the project root:
    python3 -m unittest tests.test_integration -v
"""

import json
import os
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scenario import AMR_CONFIG, TASKS  # noqa: E402
from simulation import TickClock  # noqa: E402

TIMEOUT_SECONDS = 90


def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


class DefaultScenarioTest(unittest.TestCase):

    def setUp(self):
        self.logs = Path(tempfile.mkdtemp())
        self.clock = TickClock()
        env = dict(os.environ, PYTHONUNBUFFERED="1", PYTHONDONTWRITEBYTECODE="1")
        self.processes = []
        for amr_id in AMR_CONFIG:
            with open(self.logs / f"{amr_id}.log", "w") as log:
                self.processes.append(subprocess.Popen(
                    [sys.executable, str(ROOT / "fleet_controller.py"), amr_id],
                    cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT,
                ))

    def tearDown(self):
        for process in self.processes:
            process.terminate()
        for process in self.processes:
            process.wait(5)
        self.clock.close()

    def log(self):
        return "".join((self.logs / f"{amr_id}.log").read_text() for amr_id in AMR_CONFIG)

    def test_fleet_starts_idle(self):
        starts = self.clock.sync()
        for _ in range(10):
            result = self.clock.step()
            self.assertEqual(result["collisions"], [])
        self.assertEqual(result["positions"], starts)
        self.assertNotIn("ANNOUNCING", self.log())
        self.assertNotIn("Traceback", self.log())

    def submit(self, task_id):
        # What the backend sends when the operator adds a task.
        task = {k: list(v) if k != "task_id" else v for k, v in TASKS[task_id].items()}
        self.clock.sock.sendto(
            json.dumps({"type": "OPERATOR_TASK", "task": task}).encode(),
            ("127.0.0.1", AMR_CONFIG["AMR-1"]["port"]),
        )

    def test_operator_tasks_complete_with_one_cell_per_tick_and_no_collisions(self):
        previous = self.clock.sync()
        self.submit("TASK-01")
        self.submit("TASK-02")
        deadline = time.monotonic() + TIMEOUT_SECONDS

        while self.log().count("DELIVERY COMPLETE") < 2:
            self.assertLess(time.monotonic(), deadline, f"tasks not delivered; logs in {self.logs}")
            result = self.clock.step()

            self.assertEqual(result["collisions"], [], f"tick {result['tick']}")
            for amr_id, cell in result["positions"].items():
                self.assertLessEqual(
                    manhattan(previous[amr_id], cell), 1,
                    f"{amr_id} jumped at tick {result['tick']}",
                )
            previous = result["positions"]
            time.sleep(0.02)

        log = self.log()
        self.assertIn("DISTRIBUTED WINNER | TASK-01 -> AMR-2", log)
        self.assertIn("DISTRIBUTED WINNER | TASK-02 -> AMR-1", log)
        self.assertNotIn("Traceback", log)
        print(f"\n  delivered both tasks by tick {self.clock.tick}; logs in {self.logs}")


class EventTickTest(unittest.TestCase):

    def test_live_event_timeline_matches_route(self):
        # The event timeline in run mode matches the movement model.
        from simulation import SimulationManager

        manager = SimulationManager(log_dir=Path(tempfile.mkdtemp()))
        self.addCleanup(manager.shutdown)
        manager.start()
        deadline = time.monotonic() + TIMEOUT_SECONDS
        while manager.snapshot()["mode"] != "running":
            self.assertLess(time.monotonic(), deadline)
            time.sleep(0.1)
        task = manager.add_task(TASKS["TASK-01"]["pickup"], TASKS["TASK-01"]["dropoff"])

        def tick_of(event_type):
            for event in manager.snapshot()["events"]:
                if event["type"] == event_type and event["amr_id"] == "AMR-2":
                    return event["tick"]

        while tick_of("TASK_COMPLETED") is None:
            self.assertLess(time.monotonic(), deadline, "TASK-01 not delivered")
            time.sleep(0.1)

        # TASK-01: 1 tick dwell at the pickup, then a 12-cell A* route.
        self.assertEqual(tick_of("TASK_COMPLETED") - tick_of("PICKUP_REACHED"), 1 + 12)

        # The auction result reached the task table: every idle AMR bid, lowest won.
        row = next(t for t in manager.snapshot()["tasks"] if t["task_id"] == task["task_id"])
        self.assertEqual((row["winner"], row["status"]), ("AMR-2", "COMPLETED"))
        self.assertEqual(sorted(b["amr_id"] for b in row["bids"]), ["AMR-1", "AMR-2", "AMR-3"])
        self.assertEqual(row["cost"], min(b["cost"] for b in row["bids"]))


if __name__ == "__main__":
    unittest.main()

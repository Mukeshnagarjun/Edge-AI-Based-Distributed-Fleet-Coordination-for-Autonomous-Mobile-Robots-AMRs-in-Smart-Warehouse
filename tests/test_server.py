"""Backend API against the real controllers and clock.

Needs FastAPI (installed in .venv), binds ports 7001-7003, 7100 and 7200,
so stop run_demo.py (or any running server.py) first. Takes about 30 seconds.

Run from the project root:
    .venv/bin/python -m unittest tests.test_server -v
"""

import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scenario import AMR_CONFIG  # noqa: E402

try:
    from fastapi.testclient import TestClient
    import server
except ImportError:
    server = None

STARTS = {amr_id: list(robot["start"]) for amr_id, robot in AMR_CONFIG.items()}


@unittest.skipIf(server is None, "FastAPI not installed; use .venv/bin/python")
class ServerTests(unittest.TestCase):

    def setUp(self):
        self.client = TestClient(server.app)
        self.client.__enter__()
        self.addCleanup(self.client.__exit__, None, None, None)

    def state(self):
        return self.client.get("/api/state").json()

    def wait_until(self, condition, timeout=60):
        deadline = time.monotonic() + timeout
        while True:
            state = self.state()
            if condition(state):
                return state
            self.assertNotEqual(state["mode"], "error", state["error"])
            self.assertLess(time.monotonic(), deadline, f"timed out; mode={state['mode']}")
            time.sleep(0.1)

    def positions(self, state):
        return {amr["amr_id"]: amr["position"] for amr in state["amrs"]}

    def test_fleet_runs_live_and_idle(self):
        state = self.wait_until(lambda s: s["mode"] == "running" and len(s["amrs"]) == 3)
        self.assertEqual(self.positions(state), STARTS)

        # The clock ticks on its own; with no tasks nobody moves.
        later = self.wait_until(lambda s: s["tick"] >= state["tick"] + 2, timeout=10)
        self.assertEqual(self.positions(later), STARTS)
        self.assertEqual(later["collisions"], [])

        # The manual simulation controls are gone.
        for path in ("/api/run", "/api/pause", "/api/step", "/api/reset"):
            self.assertEqual(self.client.post(path).status_code, 404, path)

    def test_operator_tasks_are_auctioned_and_delivered(self):
        self.wait_until(lambda s: s["mode"] == "running" and len(s["amrs"]) == 3)

        # Bad cells are refused before reaching the fleet.
        for pickup, dropoff in (((4, 2), (3, 7)), ((15, 0), (3, 7)), ((2, 2), (2, 2))):
            response = self.client.post("/api/tasks", json={"pickup": pickup, "dropoff": dropoff})
            self.assertEqual(response.status_code, 422, (pickup, dropoff))

        # Four short tasks for three AMRs: the fourth waits for a free AMR.
        routes = (((2, 1), (2, 3)), ((6, 1), (6, 3)), ((13, 7), (13, 5)), ((1, 3), (1, 5)))
        ids = []
        for pickup, dropoff in routes:
            response = self.client.post("/api/tasks", json={"pickup": pickup, "dropoff": dropoff})
            self.assertEqual(response.status_code, 201)
            ids.append(response.json()["task_id"])
        self.assertEqual(ids, ["T-01", "T-02", "T-03", "T-04"])

        state = self.wait_until(
            lambda s: all(t["status"] == "COMPLETED" for t in s["tasks"]), timeout=150,
        )
        self.assertEqual(state["collisions"], [])
        winners = [t["winner"] for t in state["tasks"]]
        self.assertEqual(winners[:3], ["AMR-1", "AMR-2", "AMR-3"])
        self.assertEqual(len([e for e in state["events"] if e["type"] == "AUCTION_RESULT"]), 4)

    def test_added_amr_joins_the_live_fleet_and_wins_work(self):
        self.wait_until(lambda s: s["mode"] == "running" and len(s["amrs"]) == 3)

        # Racks and occupied cells are refused.
        self.assertEqual(self.client.post("/api/amrs", json={"start": [4, 2]}).status_code, 422)
        self.assertEqual(self.client.post("/api/amrs", json={"start": [1, 1]}).status_code, 422)

        response = self.client.post("/api/amrs", json={"start": [1, 8]})
        self.assertEqual(response.status_code, 201)
        self.assertEqual(response.json()["amr_id"], "AMR-4")

        # It reports in, then joins the clock between ticks.
        state = self.wait_until(lambda s: any(e["type"] == "AMR_JOINED_CLOCK" for e in s["events"]))
        self.assertEqual(self.positions(state)["AMR-4"], [1, 8])

        # Right next to the task, it should outbid the original three.
        self.client.post("/api/tasks", json={"pickup": [2, 8], "dropoff": [2, 6]})
        state = self.wait_until(lambda s: s["tasks"][0]["status"] == "COMPLETED", timeout=60)
        task = state["tasks"][0]
        self.assertEqual(task["winner"], "AMR-4")
        self.assertEqual(sorted(b["amr_id"] for b in task["bids"]), ["AMR-1", "AMR-2", "AMR-3", "AMR-4"])
        self.assertEqual(state["collisions"], [])
        self.assertNotEqual(state["mode"], "error")

    def test_scenario_and_live_updates(self):
        scenario = self.client.get("/api/scenario").json()
        self.assertEqual((scenario["width"], scenario["height"]), (15, 10))
        self.assertEqual(len(scenario["amrs"]), 3)
        self.assertNotIn("tasks", scenario)

        with self.client.websocket_connect("/ws") as websocket:
            self.assertIn(websocket.receive_json()["mode"], ("starting", "running"))


if __name__ == "__main__":
    unittest.main()

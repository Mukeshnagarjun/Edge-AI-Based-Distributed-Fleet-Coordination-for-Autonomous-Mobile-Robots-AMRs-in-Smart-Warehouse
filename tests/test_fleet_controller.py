"""Tests for fleet_controller.py.

Run from the project root:
    python3 -m unittest discover -s tests -v

Each test loads a fresh controller module for one AMR and swaps its UDP
sockets for an in-memory fake, so no ports are bound and no threads run
unless a test starts them.
"""

import importlib
import json
import os
import sys
import threading
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scenario import AMR_CONFIG, OBSTACLES, TASKS  # noqa: E402


class StopReceiving(BaseException):
    """Ends receive_messages(), which only catches Exception."""


class FakeSocket:
    def __init__(self, incoming=()):
        self.sent = []
        self.incoming = list(incoming)

    def sendto(self, data, address):
        self.sent.append(json.loads(data))

    def recvfrom(self, _size):
        if self.incoming:
            return json.dumps(self.incoming.pop(0)).encode(), None
        raise StopReceiving


def load_controller(amr_id="AMR-1"):
    with mock.patch.object(sys, "argv", ["fleet_controller.py", amr_id]):
        sys.modules.pop("fleet_controller", None)
        fc = importlib.import_module("fleet_controller")
    fc.sock.close()
    fc.telemetry_sock.close()
    fc.sock = FakeSocket()
    fc.telemetry_sock = FakeSocket()
    return fc


def bid(amr_id, task_id, cost):
    return {
        "amr_id": amr_id,
        "task_id": task_id,
        "cost": cost,
        "distance_to_pickup": cost,
        "distance_pickup_to_dropoff": 0,
    }


def sent_types(fc):
    return [message["type"] for message in fc.sock.sent]


# ============================================================
# Characterization: behaviour that must not change
# ============================================================

class PathPlanningTests(unittest.TestCase):

    def setUp(self):
        self.fc = load_controller()

    def test_path_is_connected_and_avoids_obstacles(self):
        path = self.fc.a_star((1, 1), (10, 2))
        self.assertEqual(path[0], (1, 1))
        self.assertEqual(path[-1], (10, 2))
        for a, b in zip(path, path[1:]):
            self.assertEqual(abs(a[0] - b[0]) + abs(a[1] - b[1]), 1)
        self.assertFalse(set(path) & OBSTACLES)

    def test_path_is_shortest_on_open_route(self):
        # Row 1 is free between x=1 and x=10, so Manhattan distance is optimal.
        self.assertEqual(len(self.fc.a_star((1, 1), (10, 1))) - 1, 9)

    def test_blocked_goal_has_no_path(self):
        self.assertIsNone(self.fc.a_star((1, 1), (10, 2), {(10, 2)}))


class DefaultAuctionTests(unittest.TestCase):

    def costs(self, task_id):
        return {
            amr_id: load_controller(amr_id).calculate_bid(TASKS[task_id])["cost"]
            for amr_id in AMR_CONFIG
        }

    def test_default_scenario_bids(self):
        # TASK-01 goes to AMR-2; TASK-02 then goes to AMR-1 (AMR-2 is busy).
        task_01 = self.costs("TASK-01")
        self.assertEqual(min(task_01, key=task_01.get), "AMR-2")
        self.assertEqual(task_01["AMR-2"], 16)

        task_02 = self.costs("TASK-02")
        del task_02["AMR-2"]
        self.assertEqual(min(task_02, key=task_02.get), "AMR-1")
        self.assertEqual(task_02["AMR-1"], 18)


# ============================================================
# Bug regressions
# ============================================================

class HeartbeatTests(unittest.TestCase):

    def test_heartbeat_keeps_pending_move_intent(self):
        # Bug: the 1 s heartbeat sent next_position=None, erasing the
        # INTENT peers use to detect same-cell and head-on conflicts.
        fc = load_controller()
        fc.fleet_status["AMR-1"] = "BUSY"
        fc.task_execution_started = True

        fc.broadcast_state("INTENT", (2, 1))
        fc.send_heartbeat()

        last_state = [m for m in fc.sock.sent if m["type"] == "STATE"][-1]
        self.assertEqual(last_state["next_position"], [2, 1])


class TaskFailureTests(unittest.TestCase):

    def test_failed_task_releases_amr_and_notifies_peers(self):
        # Bug: an early `return` in execute_task left the AMR BUSY forever.
        fc = load_controller()
        fc.fleet_status["AMR-1"] = "BUSY"
        fc.assigned_tasks["AMR-1"] = "TASK-01"
        fc.task_execution_started = True

        with mock.patch.object(fc, "calculate_path", return_value=None):
            fc.execute_task(fc.get_task_by_id("TASK-01"))

        self.assertEqual(fc.fleet_status["AMR-1"], "IDLE")
        self.assertIsNone(fc.assigned_tasks["AMR-1"])
        self.assertFalse(fc.task_execution_started)
        self.assertIn("TASK_FAILED", sent_types(fc))
        self.assertNotIn("TASK_COMPLETE", sent_types(fc))

    def test_peer_task_failure_marks_peer_idle(self):
        fc = load_controller()
        fc.fleet_status["AMR-2"] = "BUSY"
        fc.assigned_tasks["AMR-2"] = "TASK-01"
        fc.sock = FakeSocket([{
            "type": "TASK_FAILED",
            "task_id": "TASK-01",
            "amr_id": "AMR-2",
            "position": [7, 1],
        }])

        with self.assertRaises(StopReceiving):
            fc.receive_messages()

        self.assertEqual(fc.fleet_status["AMR-2"], "IDLE")
        self.assertIsNone(fc.assigned_tasks["AMR-2"])


class AuctionTests(unittest.TestCase):

    def test_auction_finishes_when_peers_never_bid(self):
        # Bug: the auction waited forever for bids from every idle AMR.
        fc = load_controller("AMR-1")
        fc.TASK_SETTLE_DELAY = 0
        fc.BID_TIMEOUT = 0.5

        def auction_both():
            for task_id in ("TASK-01", "TASK-02"):
                fc.auction_task(TASKS[task_id])

        with mock.patch.object(fc, "execute_task"):
            auction = threading.Thread(target=auction_both, daemon=True)
            auction.start()
            auction.join(5)

        self.assertFalse(auction.is_alive(), "auction hung waiting for bids")
        # Only AMR-1 bid, so it wins TASK-01; it is then busy and nobody
        # bids on TASK-02, which stays unassigned instead of blocking.
        self.assertEqual(fc.task_winners["TASK-01"], "AMR-1")
        self.assertIsNone(fc.task_winners["TASK-02"])
        reasons = [m["event_data"]["reason"] for m in fc.telemetry_sock.sent if m.get("event") == "AUCTION_RESULT"]
        self.assertEqual(reasons, [None, "No bids received in time"])

    def test_operator_task_is_queued_by_amr1_only(self):
        task = {"task_id": "T-01", "pickup": [2, 1], "dropoff": [2, 3]}
        for amr_id, queued in (("AMR-1", 1), ("AMR-2", 0)):
            fc = load_controller(amr_id)
            fc.sock = FakeSocket([{"type": "OPERATOR_TASK", "task": task}])
            with self.assertRaises(StopReceiving):
                fc.receive_messages()
            self.assertEqual(fc.operator_tasks.qsize(), queued, amr_id)

    def test_winner_can_run_a_task_it_only_heard_announced(self):
        fc = load_controller("AMR-2")
        task = {"task_id": "T-01", "pickup": [2, 1], "dropoff": [2, 3]}
        fc.sock = FakeSocket([{"type": "TASK", "task": task}])
        with self.assertRaises(StopReceiving):
            fc.receive_messages()
        self.assertEqual(fc.get_task_by_id("T-01"), task)

    def test_unreachable_task_is_not_assigned(self):
        fc = load_controller()
        for amr_id in AMR_CONFIG:
            fc.known_bids["TASK-01"][amr_id] = bid(amr_id, "TASK-01", float("inf"))

        self.assertIsNone(fc.determine_task_winner("TASK-01"))

    def test_committed_task_is_not_reported_with_a_new_winner(self):
        # Bug: after commit, winners were recomputed among the AMRs still
        # idle, so AMR-3 logged phantom winners for tasks it never ran.
        fc = load_controller("AMR-3")
        for amr_id, cost in {"AMR-1": 20, "AMR-2": 16, "AMR-3": 21}.items():
            fc.known_bids["TASK-01"][amr_id] = bid(amr_id, "TASK-01", cost)

        reported = {}
        fc.report_independent_winners(reported)
        self.assertEqual(reported, {"TASK-01": "AMR-2"})

        fc.commit_task_assignment("TASK-01", "AMR-2")
        fc.report_independent_winners(reported)
        self.assertEqual(reported, {"TASK-01": "AMR-2"})


class FleetJoinTests(unittest.TestCase):

    def test_peers_adopt_a_joined_amr(self):
        fc = load_controller("AMR-1")
        joined = {"type": "AMR_JOINED", "amr_id": "AMR-4", "port": 7004, "start": [1, 8], "priority": 4}
        fc.sock = FakeSocket([joined, joined])  # a repeat must not add it twice
        with self.assertRaises(StopReceiving):
            fc.receive_messages()

        self.assertEqual(fc.PEERS["AMR-1"], [7002, 7003, 7004])
        self.assertEqual(fc.AMR_PRIORITY["AMR-4"], 4)
        self.assertEqual(fc.fleet_status["AMR-4"], "IDLE")

    def test_added_amr_starts_from_the_fleet_it_is_given(self):
        fleet = {amr_id: dict(robot, start=list(robot["start"])) for amr_id, robot in AMR_CONFIG.items()}
        fleet["AMR-4"] = {"port": 7004, "start": [1, 8], "priority": 4}
        with mock.patch.dict(os.environ, {"AMR_FLEET": json.dumps(fleet)}):
            fc = load_controller("AMR-4")

        self.assertEqual((fc.position, fc.priority), ((1, 8), 4))
        self.assertEqual(fc.PEERS["AMR-4"], [7001, 7002, 7003])
        self.assertEqual(set(fc.fleet_status), {"AMR-1", "AMR-2", "AMR-3", "AMR-4"})


if __name__ == "__main__":
    unittest.main()

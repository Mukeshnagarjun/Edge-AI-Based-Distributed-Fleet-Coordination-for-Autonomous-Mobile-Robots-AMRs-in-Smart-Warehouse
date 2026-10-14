"""Tick-gated movement, collision detection and Edge-AI deference.

Run from the project root:
    python3 -m unittest discover -s tests -v
"""

import sys
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))

from test_fleet_controller import load_controller  # noqa: E402

from simulation import SimulationManager, detect_collisions  # noqa: E402


def wait_for(condition, timeout=2.0):
    deadline = time.monotonic() + timeout
    while not condition():
        if time.monotonic() > deadline:
            raise AssertionError("condition not met in time")
        time.sleep(0.01)


def ai_advice(action):
    return SimpleNamespace(action=action, confidence=95.0, risk_level="HIGH")


class TickGateTests(unittest.TestCase):

    def setUp(self):
        self.acks = []
        self.fc = load_controller()
        self.gate = self.fc.TickGate(lambda tick, phase: self.acks.append((tick, phase)))

    def test_idle_amr_acknowledges_immediately(self):
        self.gate.deliver(1, "INTENT")
        self.assertEqual(self.acks, [(1, "INTENT")])

    def test_task_joining_mid_tick_skips_rest_of_that_tick(self):
        self.gate.set_active(True)
        self.gate.deliver(5, "MOVE")
        taken = []
        worker = threading.Thread(target=lambda: taken.append(self.gate.take("INTENT")))
        worker.start()
        wait_for(lambda: self.acks == [(5, "MOVE")])
        self.gate.deliver(6, "INTENT")
        worker.join(2)
        self.assertEqual(taken, [6])

    def test_finishing_task_acknowledges_pending_phase(self):
        self.gate.set_active(True)
        self.gate.deliver(3, "INTENT")
        self.gate.set_active(False)
        self.assertEqual(self.acks, [(3, "INTENT")])


class CollisionDetectionTests(unittest.TestCase):

    def test_same_cell(self):
        result = detect_collisions({"A": (1, 1), "B": (3, 1)}, {"A": (2, 1), "B": (2, 1)})
        self.assertEqual([c["type"] for c in result], ["SAME_CELL"])

    def test_swap(self):
        result = detect_collisions({"A": (1, 1), "B": (2, 1)}, {"A": (2, 1), "B": (1, 1)})
        self.assertEqual([c["type"] for c in result], ["SWAP"])

    def test_following_into_vacated_cell_is_not_a_collision(self):
        self.assertEqual(detect_collisions({"A": (1, 1), "B": (2, 1)}, {"A": (2, 1), "B": (3, 1)}), [])

    def test_stationary_fleet(self):
        positions = {"A": (1, 1), "B": (5, 5)}
        self.assertEqual(detect_collisions(positions, positions), [])


class EventStampTests(unittest.TestCase):

    def test_event_during_a_tick_is_stamped_with_that_tick(self):
        # Bug: events used the last completed tick, so telemetry arriving
        # while tick 5 was still collecting acknowledgements said tick 4.
        manager = SimulationManager(telemetry_port=0)
        self.addCleanup(manager.shutdown)
        manager.clock = SimpleNamespace(tick=5, close=lambda: None)
        manager.tick = 4

        manager._apply_telemetry({
            "type": "TELEMETRY", "amr_id": "AMR-2",
            "status": "PICKUP_REACHED", "position": [10, 2],
        })

        self.assertEqual(manager.snapshot()["events"][-1]["tick"], 5)


class GatedMovementTests(unittest.TestCase):

    def setUp(self):
        self.fc = load_controller("AMR-1")
        advice = mock.patch.object(self.fc, "assess_next_move_risk", return_value=ai_advice("MOVE"))
        advice.start()
        self.addCleanup(advice.stop)
        self.fc.tick_gate.set_active(True)

    def drive(self, goal):
        self.fc.current_target = goal
        self.fc.current_path = self.fc.calculate_path(goal)
        self.fc.path_index = 0
        result = {}

        def run():
            result["ok"] = self.fc.drive_to(goal, {"task_id": "T"}, "PICKUP", "MOVING_TO_PICKUP")

        thread = threading.Thread(target=run, daemon=True)
        thread.start()
        return thread, result

    def acked(self, tick, phase):
        return any(
            m["type"] == "TICK_ACK" and m["tick"] == tick and m["phase"] == phase
            for m in self.fc.sock.sent
        )

    def run_tick(self, tick):
        for phase in ("INTENT", "MOVE"):
            self.fc.tick_gate.deliver(tick, phase)
            wait_for(lambda: self.acked(tick, phase))

    def last_status(self):
        return [m for m in self.fc.sock.sent if m["type"] == "STATE"][-1]["status"]

    def test_each_tick_moves_one_cell_and_nothing_moves_between_ticks(self):
        thread, result = self.drive((3, 1))

        time.sleep(0.2)
        self.assertEqual(self.fc.position, (1, 1), "moved without a tick")

        self.run_tick(1)
        self.assertEqual(self.fc.position, (2, 1))
        time.sleep(0.2)
        self.assertEqual(self.fc.position, (2, 1), "moved while paused")

        self.run_tick(2)
        thread.join(2)
        self.assertEqual(self.fc.position, (3, 1))
        self.assertTrue(result["ok"])

    def test_priority_rules_decide_when_a_conflict_is_detected(self):
        # Bug: Edge-AI REPLAN pre-empted the priority rules, so the
        # higher-priority AMR rerouted too.
        self.fc.assess_next_move_risk.return_value = ai_advice("REPLAN")
        self.fc.known_amrs["AMR-2"] = {
            "position": (2, 2), "next_position": (2, 1),
            "target": None, "status": "INTENT", "timestamp": time.time(),
        }

        with mock.patch.object(self.fc, "replan_current_task") as replan:
            self.drive((3, 1))
            self.run_tick(1)

        replan.assert_not_called()
        self.assertEqual(self.fc.position, (1, 1))
        self.assertEqual(self.last_status(), "WAITING")

    def test_edge_ai_does_not_reroute_the_amr_with_right_of_way(self):
        self.fc.assess_next_move_risk.return_value = ai_advice("REPLAN")
        self.fc.known_amrs["AMR-3"] = {
            "position": (1, 3), "next_position": None,
            "target": None, "status": "READY", "timestamp": time.time(),
        }

        with mock.patch.object(self.fc, "replan_current_task") as replan:
            self.drive((3, 1))
            self.run_tick(1)

        replan.assert_not_called()
        self.assertEqual(self.fc.position, (2, 1))

    def block_with_moving_peer(self):
        # A moving peer on the next cell makes the higher-priority AMR WAIT.
        self.fc.known_amrs["AMR-2"] = {
            "position": (2, 1), "next_position": (2, 2),
            "target": None, "status": "MOVING", "timestamp": time.time(),
        }

    def test_blocked_timeout_replans_and_resumes_on_new_path(self):
        self.block_with_moving_peer()
        self.drive((3, 1))

        for tick in range(1, self.fc.BLOCKED_REPLAN_THRESHOLD):
            self.run_tick(tick)
        self.assertEqual(self.fc.replan_count, 0, "replanned before the timeout")

        self.run_tick(self.fc.BLOCKED_REPLAN_THRESHOLD)
        self.assertEqual(self.fc.replan_count, 1)
        self.assertNotIn((2, 1), self.fc.current_path)
        self.assertEqual(self.fc.position, (1, 1), "moved on the replan tick")

        self.fc.known_amrs["AMR-2"]["timestamp"] = time.time()
        self.run_tick(self.fc.BLOCKED_REPLAN_THRESHOLD + 1)
        self.assertEqual(self.fc.position, self.fc.current_path[1])

    def test_failed_recovery_stays_put_and_retries_after_interval(self):
        self.block_with_moving_peer()
        ticks = self.fc.BLOCKED_REPLAN_THRESHOLD + 2 * self.fc.BLOCKED_REPLAN_RETRY_TICKS

        with mock.patch.object(self.fc, "replan_current_task", return_value=False) as replan:
            self.drive((3, 1))
            for tick in range(1, ticks + 1):
                self.fc.known_amrs["AMR-2"]["timestamp"] = time.time()
                self.run_tick(tick)

        self.assertEqual(replan.call_count, 3)  # threshold tick, then each retry
        self.assertEqual(self.fc.position, (1, 1))
        self.assertEqual(self.fc.waiting_steps, ticks)
        self.assertEqual(self.last_status(), "WAITING")

    def test_successful_move_resets_blocked_counter(self):
        self.block_with_moving_peer()
        self.drive((3, 1))

        self.run_tick(1)
        self.assertEqual(self.fc.waiting_steps, 1)

        del self.fc.known_amrs["AMR-2"]
        self.run_tick(2)
        self.assertEqual(self.fc.position, (2, 1))
        self.assertEqual(self.fc.waiting_steps, 0)

    def test_wait_cycle_is_logged_once_and_changes_no_decision(self):
        # Head-on with a lower-priority peer: AMR-1 still WAITs and acks every tick.
        self.fc.known_amrs["AMR-2"] = {
            "position": (2, 1), "next_position": (1, 1),
            "target": None, "status": "WAITING", "timestamp": time.time(),
        }
        with mock.patch("builtins.print") as printed:
            self.drive((3, 1))
            self.run_tick(1)
            self.fc.known_amrs["AMR-2"]["timestamp"] = time.time()
            self.run_tick(2)

        cycle_logs = [c for c in printed.call_args_list if "WAIT CYCLE" in str(c)]
        self.assertEqual(len(cycle_logs), 1)
        self.assertEqual(self.fc.position, (1, 1))
        self.assertEqual(self.last_status(), "WAITING")


class EdgeAiYieldTests(unittest.TestCase):

    def test_edge_ai_reroutes_amr_that_must_yield(self):
        fc = load_controller("AMR-2")
        fc.tick_gate.set_active(True)
        fc.known_amrs["AMR-1"] = {
            "position": (5, 1), "next_position": None,
            "target": None, "status": "READY", "timestamp": time.time(),
        }
        fc.current_target = (7, 3)
        fc.current_path = fc.calculate_path((7, 3))
        fc.path_index = 0

        with mock.patch.object(fc, "assess_next_move_risk", return_value=ai_advice("REPLAN")), \
                mock.patch.object(fc, "replan_current_task", return_value=True) as replan:
            threading.Thread(
                target=fc.drive_to, args=((7, 3), {"task_id": "T"}, "PICKUP", "MOVING_TO_PICKUP"),
                daemon=True,
            ).start()
            for phase in ("INTENT", "MOVE"):
                fc.tick_gate.deliver(1, phase)
                wait_for(lambda: any(
                    m["type"] == "TICK_ACK" and m["tick"] == 1 and m["phase"] == phase
                    for m in fc.sock.sent
                ))

        replan.assert_called_once()


if __name__ == "__main__":
    unittest.main()

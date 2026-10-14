import contextlib
import io
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from deadlock_detector import find_wait_cycles  # noqa: E402
from test_fleet_controller import load_controller  # noqa: E402


class WaitForCycleTests(unittest.TestCase):

    def test_empty_graph(self):
        self.assertEqual(find_wait_cycles({}), [])

    def test_acyclic_chain(self):
        graph = {
            "AMR-1": {"AMR-2"},
            "AMR-2": {"AMR-3"},
            "AMR-3": set(),
        }
        self.assertEqual(find_wait_cycles(graph), [])

    def test_two_amr_cycle(self):
        graph = {
            "AMR-1": {"AMR-2"},
            "AMR-2": {"AMR-1"},
        }
        self.assertEqual(
            find_wait_cycles(graph),
            [("AMR-1", "AMR-2")],
        )

    def test_three_amr_cycle(self):
        graph = {
            "AMR-1": {"AMR-2"},
            "AMR-2": {"AMR-3"},
            "AMR-3": {"AMR-1"},
        }
        self.assertEqual(
            find_wait_cycles(graph),
            [("AMR-1", "AMR-2", "AMR-3")],
        )

    def test_disjoint_cycles(self):
        graph = {
            "AMR-1": {"AMR-2"},
            "AMR-2": {"AMR-1"},
            "AMR-3": {"AMR-4"},
            "AMR-4": {"AMR-3"},
            "AMR-5": {"AMR-1"},
        }
        self.assertEqual(
            find_wait_cycles(graph),
            [
                ("AMR-1", "AMR-2"),
                ("AMR-3", "AMR-4"),
            ],
        )

    def test_cycle_is_not_duplicated_by_rotation(self):
        graph = {
            "AMR-1": {"AMR-2"},
            "AMR-2": {"AMR-3"},
            "AMR-3": {"AMR-1"},
        }
        self.assertEqual(len(find_wait_cycles(graph)), 1)


class ControllerWaitForGraphTests(unittest.TestCase):
    """AMR-1 sits at (1, 1); peers are injected as fresh STATE messages."""

    def setUp(self):
        self.fc = load_controller("AMR-1")

    def peer(self, peer_id, position, next_position, age=0.0):
        self.fc.known_amrs[peer_id] = {
            "position": position, "next_position": next_position,
            "target": None, "status": "WAITING", "timestamp": time.time() - age,
        }

    def report(self, next_position):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            self.fc.report_wait_cycles(next_position)
        return [line for line in out.getvalue().splitlines() if "WAIT CYCLE" in line]

    def test_no_cycle_when_next_cell_is_free(self):
        self.peer("AMR-2", (5, 1), (6, 1))
        self.assertEqual(self.fc.build_wait_for_graph((2, 1)), {})
        self.assertEqual(self.report((2, 1)), [])

    def test_two_amr_head_on_cycle(self):
        self.peer("AMR-2", (2, 1), (1, 1))
        self.assertEqual(
            self.fc.build_wait_for_graph((2, 1)),
            {"AMR-1": {"AMR-2"}, "AMR-2": {"AMR-1"}},
        )
        self.assertEqual(self.report((2, 1)), [
            "[AMR-1] WAIT CYCLE DETECTED (advisory) | AMR-1 -> AMR-2 -> AMR-1",
        ])

    def test_three_amr_cycle(self):
        # Graph logic only: on a 4-connected grid three AMRs cannot each
        # want the next one's cell, so these intents are not adjacent moves.
        self.peer("AMR-2", (2, 1), (2, 2))
        self.peer("AMR-3", (2, 2), (1, 1))
        self.assertEqual(self.report((2, 1)), [
            "[AMR-1] WAIT CYCLE DETECTED (advisory) | AMR-1 -> AMR-2 -> AMR-3 -> AMR-1",
        ])

    def test_stale_peer_state_creates_no_edge(self):
        self.peer("AMR-2", (2, 1), (1, 1), age=5.0)
        self.assertEqual(self.fc.build_wait_for_graph((2, 1)), {})
        self.assertEqual(self.report((2, 1)), [])

    def test_ordinary_wait_chain_is_not_a_cycle(self):
        # AMR-1 waits on AMR-2, which waits on AMR-3, which can move.
        self.peer("AMR-2", (2, 1), (3, 1))
        self.peer("AMR-3", (3, 1), (4, 1))
        self.assertEqual(
            self.fc.build_wait_for_graph((2, 1)),
            {"AMR-1": {"AMR-2"}, "AMR-2": {"AMR-3"}},
        )
        self.assertEqual(self.report((2, 1)), [])

    def test_persistent_cycle_logged_once_and_again_after_it_returns(self):
        self.peer("AMR-2", (2, 1), (1, 1))
        self.assertEqual(len(self.report((2, 1))), 1)
        self.assertEqual(self.report((2, 1)), [], "persistent cycle logged twice")

        self.peer("AMR-2", (2, 1), None)  # AMR-2 no longer wants our cell
        self.assertEqual(self.report((2, 1)), [])

        self.peer("AMR-2", (2, 1), (1, 1))
        self.assertEqual(len(self.report((2, 1))), 1, "returning cycle not logged")


if __name__ == "__main__":
    unittest.main()

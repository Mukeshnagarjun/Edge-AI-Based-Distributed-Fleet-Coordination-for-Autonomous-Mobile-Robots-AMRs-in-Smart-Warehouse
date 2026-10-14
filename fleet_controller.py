import socket
import threading
import json
import os
import time
import sys
import heapq
import queue

from edge_ai_integration import assess_risk, get_edge_ai_model

from deadlock_detector import find_wait_cycles


from scenario import (
    WIDTH,
    HEIGHT,
    OBSTACLES,
    AMR_CONFIG,
    TASKS,
    TELEMETRY_HOST,
    TELEMETRY_PORT,
    CLOCK_PORT,
)


# ============================================================
# FLEET
# The scenario's AMRs, or the live fleet handed over by the backend
# when an AMR is added while the fleet runs. AMR_JOINED messages
# extend it at runtime.
# ============================================================

FLEET = (
    {
        robot_id: dict(robot, start=tuple(robot["start"]))
        for robot_id, robot in json.loads(os.environ["AMR_FLEET"]).items()
    }
    if "AMR_FLEET" in os.environ
    else dict(AMR_CONFIG)
)


# ============================================================
# PRIORITY LOOKUP
# Lower number = higher priority
# ============================================================

AMR_PRIORITY = {
    robot_id: robot["priority"]
    for robot_id, robot in FLEET.items()
}


# ============================================================
# PEER PORTS
# ============================================================

PEERS = {
    robot_id: [
        peer["port"]
        for peer_id, peer in FLEET.items()
        if peer_id != robot_id
    ]
    for robot_id in FLEET
}


# ============================================================
# TELEMETRY CONFIGURATION
# Telemetry is observation-only. It does not make movement
# or task-allocation decisions.
# ============================================================

TELEMETRY_ENABLED = True


# ============================================================
# READ AMR ID
# ============================================================

if len(sys.argv) != 2:

    print("Usage:")
    print("  python3 fleet_controller.py AMR-1")
    print("  python3 fleet_controller.py AMR-2")
    print("  python3 fleet_controller.py AMR-3")

    sys.exit(1)


amr_id = sys.argv[1]


if amr_id not in FLEET:

    print("Invalid AMR ID.")
    sys.exit(1)


config = FLEET[amr_id]


# ============================================================
# LOCAL AMR STATE
# ============================================================

position = config["start"]

priority = config["priority"]

current_task = None

assigned_task = None

current_target = None

current_path = []

path_index = 0


# ============================================================
# SHARED FLEET KNOWLEDGE
# ============================================================

known_amrs = {}

known_bids = {
    task_id: {}
    for task_id in TASKS
}

# Every task this AMR has heard announced, so a winner can run
# operator tasks that are not in the scenario file.
known_tasks = {}

task_winners = {
    task_id: None
    for task_id in TASKS
}


# ============================================================
# LOCAL FLEET AVAILABILITY VIEW
# ============================================================

fleet_status = {
    robot_id: "IDLE"
    for robot_id in FLEET
}


assigned_tasks = {
    robot_id: None
    for robot_id in FLEET
}


# ============================================================
# DYNAMIC STATE
# ============================================================

dynamic_obstacles = set()

# ============================================================
# DYNAMIC OBSTACLE DEMONSTRATION
# ============================================================

# Enable the simulated blockage test. The same architecture can
# later be connected to a LiDAR/depth-camera/vision detector.
DYNAMIC_OBSTACLE_TEST_ENABLED = True

# Cell that becomes blocked during the demo.
DEMO_DYNAMIC_OBSTACLE = (9, 1)

# Each AMR triggers the simulated detection only once.
dynamic_obstacle_triggered = False

task_execution_started = False

replan_count = 0

conflict_count = 0

collision_count = 0

move_count = 0

cycle = 0


# ============================================================
# EDGE-AI RISK STATE
# ============================================================
# Edge-AI is advisory: existing distributed safety checks remain
# responsible for the final movement safety decision.
last_edge_ai_risk = "UNKNOWN"
last_edge_ai_confidence = 0.0
last_edge_ai_action = "MOVE"
last_edge_ai_reason = ""
edge_ai_decision_count = 0
waiting_steps = 0


# ============================================================
# THREAD LOCKS
# ============================================================

state_lock = threading.Lock()

obstacle_lock = threading.Lock()

# Serialises STATE broadcasts so the heartbeat cannot interleave a stale
# intent with a newer one from the task thread. Re-entrant because the
# heartbeat holds it while calling broadcast_state().
broadcast_lock = threading.RLock()

# Last (status, next_position) broadcast, repeated by the heartbeat.
last_broadcast = ("READY", None)


# ============================================================
# SIMULATION TICK GATE
# The simulation clock drives movement. Each tick has two phases:
#   INTENT: every AMR announces its next cell
#   MOVE:   every AMR checks conflicts, then moves or waits
# Every AMR acknowledges every phase, so the clock only starts the
# next phase once all peers have acted on the current one.
# ============================================================

class TickGate:

    def __init__(self, send_ack):
        self.condition = threading.Condition()
        self.pending = None
        self.active = False
        self.send_ack = send_ack

    def deliver(self, tick, phase):
        """Receiver thread: hand a clock phase to the task thread."""
        with self.condition:
            if not self.active:
                # No task running: nothing to do in this phase.
                self.send_ack(tick, phase)
                return
            self.pending = (tick, phase)
            self.condition.notify_all()

    def take(self, phase, tick=None):
        """Task thread: block until the clock opens `phase`; return its tick."""
        with self.condition:
            while True:
                while self.pending is None:
                    self.condition.wait()
                got_tick, got_phase = self.pending
                self.pending = None
                if got_phase == phase and tick in (None, got_tick):
                    return got_tick
                # A task that starts mid-tick skips the rest of that tick.
                self.send_ack(got_tick, got_phase)

    def set_active(self, active):
        with self.condition:
            self.active = active
            if not active and self.pending is not None:
                self.send_ack(*self.pending)
                self.pending = None


def send_tick_ack(tick, phase):
    message = {
        "type": "TICK_ACK",
        "amr_id": amr_id,
        "tick": tick,
        "phase": phase,
        "position": list(position),
        "status": last_broadcast[0],
    }
    try:
        sock.sendto(json.dumps(message).encode(), ("127.0.0.1", CLOCK_PORT))
    except Exception as e:
        print(f"[{amr_id}] TICK ACK ERROR: {e}")


tick_gate = TickGate(send_tick_ack)


def wait_ticks(count):
    """Stay in place for `count` ticks (pickup dwell, blocked route)."""
    for _ in range(count):
        tick = tick_gate.take("INTENT")
        tick_gate.send_ack(tick, "INTENT")
        tick_gate.take("MOVE", tick)
        tick_gate.send_ack(tick, "MOVE")


def yields_to_nearby_peer():
    """True when a higher-priority AMR is within 3 cells (Manhattan)."""
    for peer_id, peer in get_active_peers().items():
        distance = (
            abs(peer["position"][0] - position[0])
            + abs(peer["position"][1] - position[1])
        )
        if AMR_PRIORITY[peer_id] < priority and distance <= 3:
            return True
    return False


# ============================================================
# A* HEURISTIC
# ============================================================

def heuristic(a, b):

    return (
        abs(a[0] - b[0])
        +
        abs(a[1] - b[1])
    )


# ============================================================
# VALID WAREHOUSE NEIGHBORS
# ============================================================

def get_neighbors(cell):

    x, y = cell

    candidates = [

        (x + 1, y),
        (x - 1, y),
        (x, y + 1),
        (x, y - 1),

    ]

    valid = []

    for nx, ny in candidates:

        if nx < 0 or nx >= WIDTH:
            continue

        if ny < 0 or ny >= HEIGHT:
            continue

        if (nx, ny) in OBSTACLES:
            continue

        valid.append(
            (nx, ny)
        )

    return valid


# ============================================================
# A* PATH PLANNER
# ============================================================

def a_star(
    start,
    goal,
    extra_obstacles=None
):

    if extra_obstacles is None:

        extra_obstacles = set()


    blocked = set(
        extra_obstacles
    )


    # Never block our current position.

    blocked.discard(
        start
    )


    open_set = []

    heapq.heappush(
        open_set,
        (0, start)
    )


    came_from = {}

    g_score = {
        start: 0
    }

    visited = set()


    while open_set:

        _, current = heapq.heappop(
            open_set
        )


        if current in visited:
            continue


        visited.add(
            current
        )


        # ----------------------------------------------------
        # GOAL REACHED
        # ----------------------------------------------------

        if current == goal:

            path = []

            while current in came_from:

                path.append(
                    current
                )

                current = came_from[
                    current
                ]


            path.append(
                start
            )

            path.reverse()

            return path


        # ----------------------------------------------------
        # EXPLORE NEIGHBORS
        # ----------------------------------------------------

        for neighbor in get_neighbors(
            current
        ):

            if neighbor in blocked:

                continue


            tentative_g = (
                g_score[current] + 1
            )


            if tentative_g < g_score.get(
                neighbor,
                float("inf")
            ):

                came_from[
                    neighbor
                ] = current


                g_score[
                    neighbor
                ] = tentative_g


                f_score = (
                    tentative_g
                    +
                    heuristic(
                        neighbor,
                        goal
                    )
                )


                heapq.heappush(
                    open_set,
                    (
                        f_score,
                        neighbor
                    )
                )


    return None


# ============================================================
# CALCULATE PATH USING LOCAL MAP
# ============================================================

def calculate_path(goal):

    with obstacle_lock:

        blocked = set(
            dynamic_obstacles
        )


    return a_star(
        position,
        goal,
        blocked
    )


# ============================================================
# CALCULATE TASK BID
# ============================================================

def calculate_bid(task):

    pickup = tuple(
        task["pickup"]
    )

    dropoff = tuple(
        task["dropoff"]
    )


    # --------------------------------------------------------
    # Current position -> pickup
    # --------------------------------------------------------

    path_to_pickup = a_star(
        position,
        pickup
    )


    if path_to_pickup is None:

        return {

            "amr_id":
                amr_id,

            "task_id":
                task["task_id"],

            "cost":
                float("inf"),

            "distance_to_pickup":
                float("inf"),

            "distance_pickup_to_dropoff":
                float("inf"),
        }


    distance_to_pickup = (
        len(path_to_pickup) - 1
    )


    # --------------------------------------------------------
    # Pickup -> drop-off
    # --------------------------------------------------------

    path_to_dropoff = a_star(
        pickup,
        dropoff
    )


    if path_to_dropoff is None:

        return {

            "amr_id":
                amr_id,

            "task_id":
                task["task_id"],

            "cost":
                float("inf"),

            "distance_to_pickup":
                distance_to_pickup,

            "distance_pickup_to_dropoff":
                float("inf"),
        }


    distance_pickup_to_dropoff = (
        len(path_to_dropoff) - 1
    )


    total_cost = (
        distance_to_pickup
        +
        distance_pickup_to_dropoff
    )


    return {

        "amr_id":
            amr_id,

        "task_id":
            task["task_id"],

        "cost":
            total_cost,

        "distance_to_pickup":
            distance_to_pickup,

        "distance_pickup_to_dropoff":
            distance_pickup_to_dropoff,
    }


# ============================================================
# TASK LOOKUP
# ============================================================

def get_task_by_id(task_id):

    task_data = (
        known_tasks.get(task_id)
        or TASKS.get(task_id)
    )


    if task_data is None:

        return None


    return {

        "task_id":
            task_data["task_id"],

        "pickup":
            list(
                task_data["pickup"]
            ),

        "dropoff":
            list(
                task_data["dropoff"]
            ),
    }


# ============================================================
# BROADCAST STATE + NEXT INTENT
# ============================================================

def broadcast_state(
    status,
    next_position=None
):

    global last_broadcast


    with broadcast_lock:

        last_broadcast = (
            status,
            next_position
        )

        send_state(
            status,
            next_position
        )


def send_state(
    status,
    next_position=None
):

    message = {
        "type":
            "STATE",

        "amr_id":
            amr_id,

        "position":
            list(position),

        "next_position":
            (
                list(next_position)
                if next_position is not None
                else None
            ),

        "target":
            (
                list(current_target)
                if current_target is not None
                else None
            ),

        "status":
            status,

        "cycle":
            cycle,
    }


    data = json.dumps(
        message
    ).encode()


    for peer_port in PEERS[
        amr_id
    ]:

        try:

            sock.sendto(
                data,
                (
                    "127.0.0.1",
                    peer_port
                )
            )


        except Exception as e:

            print(
                f"[{amr_id}] "
                f"SEND ERROR: {e}"
            )


    # Telemetry is observation-only and is sent separately
    # from the AMR-to-AMR coordination channel.
    broadcast_telemetry(
        status,
        next_position
    )


# ============================================================
# BROADCAST TELEMETRY
# ============================================================


def broadcast_telemetry(
    status,
    next_position=None,
    event=None,
    event_data=None
):

    if not TELEMETRY_ENABLED:
        return


    with state_lock:

        telemetry_task_id = (
            assigned_tasks.get(
                amr_id
            )
        )

        telemetry_position = tuple(
            position
        )

        telemetry_target = (
            tuple(current_target)
            if current_target is not None
            else None
        )

        # current_path is None when no route was found.
        telemetry_path = [
            list(cell)
            for cell in current_path or []
        ]

        telemetry_path_index = path_index

        telemetry_replans = replan_count
        telemetry_conflicts = conflict_count
        telemetry_collisions = collision_count
        telemetry_moves = move_count
        telemetry_cycle = cycle

        telemetry_edge_ai = {
            "risk_level": last_edge_ai_risk,
            "confidence": last_edge_ai_confidence,
            "action": last_edge_ai_action,
            "reason": last_edge_ai_reason,
            "decisions": edge_ai_decision_count,
        }

        local_status = fleet_status.get(
            amr_id,
            status
        )

        assigned_task_name = assigned_tasks.get(
            amr_id
        )

    with obstacle_lock:

        telemetry_obstacles = [
            list(cell)
            for cell in sorted(
                dynamic_obstacles
            )
        ]

    message = {
        "type": "TELEMETRY",
        "amr_id": amr_id,
        "timestamp": time.time(),
        "position": list(
            telemetry_position
        ),
        "next_position": (
            list(next_position)
            if next_position is not None
            else None
        ),
        "target": (
            list(telemetry_target)
            if telemetry_target is not None
            else None
        ),
        "status": status,
        "fleet_status": local_status,
        "task_id": telemetry_task_id,
        "assigned_task": assigned_task_name,
        "priority": priority,
        "path": telemetry_path,
        "path_index": telemetry_path_index,
        "metrics": {
            "moves": telemetry_moves,
            "conflicts": telemetry_conflicts,
            "replans": telemetry_replans,
            "collisions": telemetry_collisions,
        },
        "edge_ai": telemetry_edge_ai,
        "dynamic_obstacles": telemetry_obstacles,
        "cycle": telemetry_cycle,
        "event": event,
        "event_data": event_data,
    }

    data = json.dumps(
        message
    ).encode()

    try:

        telemetry_sock.sendto(
            data,
            (
                TELEMETRY_HOST,
                TELEMETRY_PORT
            )
        )

    except Exception:

        # The visualizer is optional. Never allow a telemetry
        # failure to interrupt AMR coordination or execution.
        pass


# ============================================================
# BROADCAST GENERIC MESSAGE
# ============================================================

def broadcast_message(
    message
):

    data = json.dumps(
        message
    ).encode()


    for peer_port in PEERS[
        amr_id
    ]:

        try:

            sock.sendto(
                data,
                (
                    "127.0.0.1",
                    peer_port
                )
            )


        except Exception as e:

            print(
                f"[{amr_id}] "
                f"SEND ERROR: {e}"
            )


# ============================================================
# BROADCAST DYNAMIC OBSTACLE
# ============================================================

def broadcast_obstacle(cell):

    message = {
        "type": "OBSTACLE",
        "amr_id": amr_id,
        "cell": list(cell),
        "timestamp": time.time(),
    }

    print()
    print(
        f"[{amr_id}] DYNAMIC OBSTACLE DETECTED: {cell}"
    )

    print(
        f"[{amr_id}] Broadcasting obstacle to peers."
    )

    broadcast_message(message)

    broadcast_telemetry(
        "OBSTACLE",
        event="DYNAMIC_OBSTACLE_DETECTED",
        event_data={
            "cell": list(cell),
        }
    )


# ============================================================
# COMMIT TASK ASSIGNMENT
# ============================================================

def commit_task_assignment(
    task_id,
    winner_id
):

    global assigned_task
    global current_task
    global current_target
    global task_execution_started


    with state_lock:

        fleet_status[
            winner_id
        ] = "BUSY"

        assigned_tasks[
            winner_id
        ] = task_id


    task_winners[
        task_id
    ] = winner_id


    # This AMR is not the winner.

    if winner_id != amr_id:

        return


    # --------------------------------------------------------
    # Get task
    # --------------------------------------------------------

    task = get_task_by_id(
        task_id
    )


    if task is None:

        print(
            f"[{amr_id}] "
            f"ERROR: Task {task_id} "
            "not found."
        )

        return


    assigned_task = task

    current_task = task

    current_target = tuple(
        task["pickup"]
    )


    print()

    print(
        f"[{amr_id}] "
        f"ASSIGNMENT COMMITTED | "
        f"{task_id}"
    )

    print(
        f"[{amr_id}] "
        "STATUS -> BUSY"
    )


    broadcast_state(
        "BUSY"
    )


    # --------------------------------------------------------
    # Start execution only once.
    # --------------------------------------------------------

    if not task_execution_started:

        task_execution_started = True

        tick_gate.set_active(
            True
        )


        execution_thread = threading.Thread(

            target=execute_task,

            args=(task,),

            daemon=True
        )


        execution_thread.start()


# ============================================================
# RECEIVE ALL P2P MESSAGES
# ============================================================

def receive_messages():

    global current_task


    while True:

        try:

            data, _ = (
                sock.recvfrom(4096)
            )


            message = json.loads(
                data.decode()
            )


            message_type = message.get(
                "type",
                "STATE"
            )


            # =================================================
            # SIMULATION CLOCK TICK
            # =================================================

            if message_type == "TICK":

                tick_gate.deliver(
                    message["tick"],
                    message["phase"]
                )

                continue


            # =================================================
            # AMR JOINED (from the backend, before the new AMR starts)
            # =================================================

            if message_type == "AMR_JOINED":

                joined_id = message["amr_id"]

                with state_lock:

                    if joined_id not in FLEET:

                        FLEET[joined_id] = {
                            "port": message["port"],
                            "start": tuple(message["start"]),
                            "priority": message["priority"],
                        }

                        AMR_PRIORITY[joined_id] = message["priority"]

                        PEERS[amr_id].append(message["port"])

                        fleet_status[joined_id] = "IDLE"

                        assigned_tasks[joined_id] = None

                print(
                    f"[{amr_id}] "
                    f"FLEET JOIN | {joined_id} "
                    f"at {message['start']} "
                    f"priority {message['priority']}"
                )

                continue


            # =================================================
            # OPERATOR TASK (from the dashboard, to AMR-1 only)
            # =================================================

            if message_type == "OPERATOR_TASK":

                if amr_id == "AMR-1":

                    operator_tasks.put(
                        message["task"]
                    )

                continue


            # =================================================
            # TASK
            # =================================================

            if message_type == "TASK":

                task = message[
                    "task"
                ]

                task_id = task[
                    "task_id"
                ]

                known_tasks[
                    task_id
                ] = task


                # Busy AMR does not bid.

                if (
                    fleet_status[
                        amr_id
                    ]
                    !=
                    "IDLE"
                ):

                    print()

                    print(
                        f"[{amr_id}] "
                        f"BUSY with "
                        f"{assigned_tasks[amr_id]} "
                        f"-> SKIPPING BID for "
                        f"{task_id}"
                    )

                    continue


                with state_lock:

                    current_task = task

                    known_bids.setdefault(
                        task_id,
                        {}
                    )


                bid = calculate_bid(
                    task
                )


                with state_lock:

                    known_bids[
                        task_id
                    ][
                        amr_id
                    ] = bid


                print()

                print(
                    f"[{amr_id}] "
                    f"TASK RECEIVED: "
                    f"{task_id} "
                    f"| BID = "
                    f"{bid['cost']}"
                )


                broadcast_message({

                    "type":
                        "BID",

                    "task_id":
                        task_id,

                    "amr_id":
                        amr_id,

                    "cost":
                        bid["cost"],

                    "distance_to_pickup":
                        bid[
                            "distance_to_pickup"
                        ],

                    "distance_pickup_to_dropoff":
                        bid[
                            "distance_pickup_to_dropoff"
                        ],
                })


                continue


            # =================================================
            # BID
            # =================================================

            if message_type == "BID":

                task_id = message[
                    "task_id"
                ]


                with state_lock:

                    known_bids.setdefault(
                        task_id,
                        {}
                    )


                    known_bids[
                        task_id
                    ][
                        message[
                            "amr_id"
                        ]
                    ] = {

                        "amr_id":
                            message[
                                "amr_id"
                            ],

                        "task_id":
                            task_id,

                        "cost":
                            message[
                                "cost"
                            ],

                        "distance_to_pickup":
                            message[
                                "distance_to_pickup"
                            ],

                        "distance_pickup_to_dropoff":
                            message[
                                "distance_pickup_to_dropoff"
                            ],
                    }


                continue


            # =================================================
            # TASK ASSIGNED
            # =================================================

            if message_type == "TASK_ASSIGNED":

                task_id = message[
                    "task_id"
                ]

                winner_id = message[
                    "amr_id"
                ]


                print()

                print(
                    f"[{amr_id}] "
                    f"TASK {task_id} "
                    f"ASSIGNED TO "
                    f"{winner_id}"
                )


                commit_task_assignment(
                    task_id,
                    winner_id
                )


                continue


            # =================================================
            # TASK STATUS
            # =================================================

            if message_type == "TASK_STATUS":

                print(

                    f"[{amr_id}] "
                    f"TASK STATUS from "
                    f"{message['amr_id']} "
                    f"-> "
                    f"{message['status']} "
                    f"at "
                    f"{message['position']}"
                )


                continue


            # =================================================
            # TASK FAILED
            # =================================================

            if message_type == "TASK_FAILED":

                failed_by = message[
                    "amr_id"
                ]


                with state_lock:

                    fleet_status[
                        failed_by
                    ] = "IDLE"

                    assigned_tasks[
                        failed_by
                    ] = None


                print()

                print(
                    f"[{amr_id}] "
                    f"TASK FAILED "
                    f"reported by "
                    f"{failed_by} "
                    f"| {message['task_id']}"
                )


                continue


            # =================================================
            # TASK COMPLETE
            # =================================================

            if message_type == "TASK_COMPLETE":

                completed_by = message[
                    "amr_id"
                ]

                task_id = message[
                    "task_id"
                ]


                with state_lock:

                    fleet_status[
                        completed_by
                    ] = "IDLE"

                    assigned_tasks[
                        completed_by
                    ] = None


                print()

                print(
                    f"[{amr_id}] "
                    f"TASK COMPLETE "
                    f"reported by "
                    f"{completed_by} "
                    f"| {task_id}"
                )


                continue


            # =================================================
            # REPLAN EVENT
            # =================================================

            if message_type == "REPLAN_EVENT":

                print()

                print(
                    f"[{amr_id}] "
                    f"REPLAN EVENT from "
                    f"{message['amr_id']} "
                    f"| Task: "
                    f"{message['task_id']} "
                    f"| Count: "
                    f"{message['replan_count']}"
                )


                continue


            # =================================================
            # DYNAMIC OBSTACLE
            # =================================================

            if message_type == "OBSTACLE":

                obstacle_cell = tuple(
                    message["cell"]
                )

                sender = message["amr_id"]

                # Ignore our own obstacle message if it somehow
                # returns through the local receive socket.
                if sender == amr_id:
                    continue

                with obstacle_lock:

                    already_known = (
                        obstacle_cell
                        in dynamic_obstacles
                    )

                    dynamic_obstacles.add(
                        obstacle_cell
                    )

                if not already_known:

                    print()
                    print(
                        f"[{amr_id}] OBSTACLE UPDATE from "
                        f"{sender} | Cell: {obstacle_cell}"
                    )

                    print(
                        f"[{amr_id}] Added dynamic obstacle "
                        f"{obstacle_cell} to local map."
                    )

                continue


            # =================================================
            # STATE MESSAGE
            # =================================================

            if message_type != "STATE":

                continue


            sender = message[
                "amr_id"
            ]


            if sender == amr_id:

                continue


            new_position = tuple(
                message[
                    "position"
                ]
            )


            next_value = (
                message.get(
                    "next_position"
                )
            )


            new_next = (

                tuple(next_value)

                if next_value is not None

                else None
            )


            target_value = (
                message.get(
                    "target"
                )
            )


            new_target = (

                tuple(target_value)

                if target_value is not None

                else None
            )


            new_status = message[
                "status"
            ]


            with state_lock:

                previous = known_amrs.get(
                    sender
                )


                known_amrs[
                    sender
                ] = {

                    "position":
                        new_position,

                    "next_position":
                        new_next,

                    "target":
                        new_target,

                    "status":
                        new_status,

                    "timestamp":
                        time.time(),
                }


            # Print only meaningful changes.

            state_changed = (

                previous is None

                or

                previous[
                    "position"
                ]
                !=
                new_position

                or

                previous[
                    "next_position"
                ]
                !=
                new_next

                or

                previous[
                    "target"
                ]
                !=
                new_target

                or

                previous[
                    "status"
                ]
                !=
                new_status
            )


            if state_changed:

                next_display = (

                    list(new_next)

                    if new_next is not None

                    else None
                )


                print()

                print(

                    f"[{amr_id}] "
                    f"PEER UPDATE: "
                    f"{sender} "
                    f"| Position: "
                    f"{list(new_position)} "
                    f"| Next: "
                    f"{next_display} "
                    f"| Status: "
                    f"{new_status}"
                )


        except socket.timeout:

            continue


        except Exception as e:

            print(
                f"[{amr_id}] "
                f"RECEIVE ERROR: "
                f"{e}"
            )


# ============================================================
# GET ACTIVE PEERS
# ============================================================

def get_active_peers():

    active = {}

    now = time.time()


    with state_lock:

        for peer_id, peer in known_amrs.items():

            age = (
                now
                -
                peer["timestamp"]
            )


            if age <= 2.0:

                active[
                    peer_id
                ] = peer


    return active


# ============================================================
# GET PEER CELLS FOR REPLANNING
# ============================================================

def get_peer_obstacles():

    blocked = set()


    active_peers = (
        get_active_peers()
    )


    for peer in active_peers.values():

        blocked.add(
            peer["position"]
        )


        if (
            peer["next_position"]
            is not None
        ):

            blocked.add(
                peer["next_position"]
            )


    # Never block our own cell.

    blocked.discard(
        position
    )


    return blocked


# ============================================================
# DETECT MOVEMENT CONFLICT
# ============================================================

def detect_conflict(
    next_position
):

    conflicts = []


    active_peers = (
        get_active_peers()
    )


    for peer_id, peer in active_peers.items():

        peer_position = (
            peer["position"]
        )

        peer_next = (
            peer["next_position"]
        )


        # ----------------------------------------------------
        # SAME DESTINATION
        # ----------------------------------------------------

        if (

            peer_next is not None

            and

            next_position
            ==
            peer_next

            and

            next_position
            !=
            peer_position

        ):

            conflicts.append({

                "peer_id":
                    peer_id,

                "type":
                    "SAME_DESTINATION",
            })


            continue


        # ----------------------------------------------------
        # HEAD-ON SWAP
        # ----------------------------------------------------

        if (

            peer_next is not None

            and

            next_position
            ==
            peer_position

            and

            peer_next
            ==
            position

        ):

            conflicts.append({

                "peer_id":
                    peer_id,

                "type":
                    "HEAD_ON",
            })


            continue


        # ----------------------------------------------------
        # PEER CURRENTLY OCCUPIES NEXT CELL
        # ----------------------------------------------------

        if (

            next_position
            ==
            peer_position

        ):

            conflicts.append({

                "peer_id":
                    peer_id,

                "type":
                    "OCCUPIED",
            })


    return conflicts


# ============================================================
# EDGE-AI RISK ASSESSMENT
# ============================================================
def assess_next_move_risk(next_position, conflicts):
    """Run local Edge-AI using current peer state and movement intent."""
    global last_edge_ai_risk, last_edge_ai_confidence
    global last_edge_ai_action, last_edge_ai_reason
    global edge_ai_decision_count

    active_peers = get_active_peers()
    distance_to_peer = 8.0
    nearby_amr_count = 0
    relative_direction = 0.0
    same_destination = 0.0
    head_on = 0.0
    next_cell_occupied = 0.0

    if active_peers:
        distances = []
        for peer in active_peers.values():
            peer_position = tuple(peer["position"])
            peer_next = peer.get("next_position")
            dx = position[0] - peer_position[0]
            dy = position[1] - peer_position[1]
            distance = (dx * dx + dy * dy) ** 0.5
            distances.append(distance)

            manhattan = abs(dx) + abs(dy)
            if manhattan <= 3:
                nearby_amr_count += 1

            if peer_next is not None:
                peer_next = tuple(peer_next)
                if next_position == peer_position and peer_next == position:
                    head_on = 1.0
                    relative_direction = 2.0
                if next_position == peer_next and next_position != peer_position:
                    same_destination = 1.0
                    relative_direction = max(relative_direction, 1.0)

            if next_position == peer_position:
                next_cell_occupied = 1.0
                relative_direction = max(relative_direction, 1.0)
            if distance <= 2.5:
                relative_direction = max(relative_direction, 1.0)

        distance_to_peer = max(0.5, min(8.0, min(distances)))

    assessment = assess_risk(
        distance_to_peer=distance_to_peer,
        relative_direction=relative_direction,
        same_destination=same_destination,
        head_on=head_on,
        next_cell_occupied=next_cell_occupied,
        nearby_amr_count=float(nearby_amr_count),
        waiting_steps=float(waiting_steps),
        recent_conflicts=float(conflict_count),
        conflict_detected=bool(conflicts),
    )

    previous = (last_edge_ai_risk, last_edge_ai_action)
    last_edge_ai_risk = assessment.risk_level
    last_edge_ai_confidence = assessment.confidence
    last_edge_ai_action = assessment.action
    last_edge_ai_reason = assessment.reason
    edge_ai_decision_count += 1

    if (last_edge_ai_risk, last_edge_ai_action) != previous or assessment.action != "MOVE":
        print(
            f"[{amr_id}] EDGE-AI | Risk: {assessment.risk_level} | "
            f"Confidence: {assessment.confidence:.2f}% | Action: {assessment.action}"
        )

    broadcast_telemetry(
        "EDGE_AI",
        next_position,
        event="EDGE_AI_RISK",
        event_data={
            "risk_level": assessment.risk_level,
            "confidence": assessment.confidence,
            "action": assessment.action,
            "probabilities": assessment.probabilities,
            "next_position": list(next_position),
        },
    )
    return assessment


# ============================================================
# RESOLVE MOVEMENT CONFLICT
#
# Returns:
#   MOVE
#   WAIT
#   REPLAN
# ============================================================

def resolve_movement_conflict(
    conflicts
):

    decision = "MOVE"


    for conflict in conflicts:

        peer_id = (
            conflict[
                "peer_id"
            ]
        )

        conflict_type = (
            conflict[
                "type"
            ]
        )


        peer_priority = (
            AMR_PRIORITY[
                peer_id
            ]
        )


        print()

        print(
            f"[{amr_id}] "
            f"CONFLICT with "
            f"{peer_id} "
            f"| Type: "
            f"{conflict_type}"
        )


        # ----------------------------------------------------
        # LOWER PRIORITY -> REPLAN
        # ----------------------------------------------------

        if priority > peer_priority:

            print(
                f"[{amr_id}] "
                f"Priority {priority} > "
                f"{peer_id} priority "
                f"{peer_priority} -> REPLAN"
            )

            return "REPLAN"


        # ----------------------------------------------------
        # HIGHER PRIORITY
        # ----------------------------------------------------

        if conflict_type == "OCCUPIED":

            # A higher-priority AMR should not wait forever for
            # a stationary peer. Instead, it attempts a local
            # route around that peer's cell.

            with state_lock:
                peer_state = dict(
                    known_amrs.get(
                        peer_id,
                        {}
                    )
                )

            peer_next = peer_state.get(
                "next_position"
            )

            peer_status = peer_state.get(
                "status"
            )

            peer_stationary = (
                peer_next is None
                or peer_status in {
                    "WAITING",
                    "READY",
                }
            )

            if peer_stationary:

                print(
                    f"[{amr_id}] "
                    f"Higher priority but {peer_id} "
                    f"is stationary at the target cell "
                    "-> REPLAN"
                )

                return "REPLAN"

            print(
                f"[{amr_id}] "
                "Higher priority and peer is moving "
                "-> WAIT"
            )

            decision = "WAIT"


        elif conflict_type == "HEAD_ON":

            print(
                f"[{amr_id}] "
                "Higher priority head-on conflict "
                "-> WAIT"
            )

            decision = "WAIT"


        else:

            # SAME_DESTINATION: never allow two AMRs to enter
            # the same destination simultaneously.

            print(
                f"[{amr_id}] "
                "Higher priority same-destination "
                "conflict -> WAIT"
            )

            decision = "WAIT"


    return decision


# ============================================================
# LOCAL REPLANNING
# ============================================================

def replan_current_task():

    global current_path
    global path_index
    global replan_count


    if current_target is None:

        return False


    # --------------------------------------------------------
    # Dynamic obstacles
    # --------------------------------------------------------

    with obstacle_lock:

        blocked = set(
            dynamic_obstacles
        )


    # --------------------------------------------------------
    # Peer cells
    # --------------------------------------------------------

    blocked.update(
        get_peer_obstacles()
    )


    blocked.discard(
        position
    )


    # --------------------------------------------------------
    # A*
    # --------------------------------------------------------

    new_path = a_star(

        position,

        current_target,

        blocked
    )


    if new_path is None:

        print()

        print(
            f"[{amr_id}] "
            "NO ALTERNATIVE PATH FOUND"
        )


        return False


    current_path = (
        new_path
    )

    path_index = 0

    replan_count += 1


    print()

    print(
        "=" * 70
    )

    print(
        f"[{amr_id}] "
        "LOCAL REPLANNING"
    )

    print(
        f"Current position : "
        f"{position}"
    )

    print(
        f"Target           : "
        f"{current_target}"
    )

    print(
        f"Replan count     : "
        f"{replan_count}"
    )

    print(
        f"New path length  : "
        f"{len(current_path) - 1}"
    )

    print(
        f"New path         : "
        f"{current_path}"
    )

    print(
        "=" * 70
    )


    task_id = assigned_tasks.get(
        amr_id
    )


    broadcast_message({

        "type":
            "REPLAN_EVENT",

        "amr_id":
            amr_id,

        "task_id":
            task_id,

        "position":
            list(position),

        "replan_count":
            replan_count,
    })

    broadcast_telemetry(
        "REPLANNING",
        event="REPLAN",
        event_data={
            "task_id": task_id,
            "replan_count": replan_count,
            "target": (
                list(current_target)
                if current_target is not None
                else None
            ),
            "path_length": (
                len(current_path) - 1
                if current_path
                else 0
            ),
        }
    )


    return True


# ============================================================
# SIMULATED DYNAMIC OBSTACLE DETECTION
# ============================================================

def detect_dynamic_obstacle(next_position):

    global dynamic_obstacle_triggered

    if not DYNAMIC_OBSTACLE_TEST_ENABLED:
        return False

    if dynamic_obstacle_triggered:
        return False

    if next_position != DEMO_DYNAMIC_OBSTACLE:
        return False

    dynamic_obstacle_triggered = True

    with obstacle_lock:
        dynamic_obstacles.add(
            DEMO_DYNAMIC_OBSTACLE
        )

    broadcast_obstacle(
        DEMO_DYNAMIC_OBSTACLE
    )

    return True



# ============================================================
# DETERMINE TASK WINNER
# ============================================================

def determine_task_winner(
    task_id,
    require_all_bids=True
):

    with state_lock:

        bids_for_task = dict(
            known_bids.get(
                task_id,
                {}
            )
        )


        # Only IDLE AMRs are eligible.

        eligible_amrs = [

            robot_id

            for robot_id, status

            in fleet_status.items()

            if status == "IDLE"

        ]


    if not eligible_amrs:

        return None


    # We wait for every currently eligible
    # AMR to submit a bid.

    if require_all_bids:

        for robot_id in eligible_amrs:

            if (
                robot_id
                not in
                bids_for_task
            ):

                return None


    # An infinite cost means the AMR has no route to the task.

    bids = [

        bids_for_task[
            robot_id
        ]

        for robot_id
        in eligible_amrs

        if robot_id in bids_for_task
        and bids_for_task[robot_id]["cost"] < float("inf")

    ]


    if not bids:

        return None


    # Lowest cost wins.
    # AMR ID is deterministic tie-breaker.

    bids.sort(

        key=lambda bid: (

            bid["cost"],

            bid["amr_id"],
        )

    )


    return bids[0]


# ============================================================
# OPERATOR TASK AUCTIONS (run by AMR-1)
# ============================================================

# Seconds to let BUSY state propagate between auctions.
TASK_SETTLE_DELAY = 2

# Seconds to wait for bids before deciding with those received.
BID_TIMEOUT = 3


# Operator tasks wait here for AMR-1 to auction them, in order.
operator_tasks = queue.Queue()


def run_auctions():

    while True:

        auction_task(
            operator_tasks.get()
        )


def report_auction(task, winner, reason=None):

    with state_lock:

        bids = sorted(
            (
                {
                    "amr_id": bid["amr_id"],
                    "cost": bid["cost"] if bid["cost"] < float("inf") else None,
                    "distance_to_pickup": bid["distance_to_pickup"] if bid["distance_to_pickup"] < float("inf") else None,
                }
                for bid in known_bids.get(task["task_id"], {}).values()
            ),
            key=lambda bid: (bid["cost"] is None, bid["cost"] or 0, bid["amr_id"]),
        )

    broadcast_telemetry(
        last_broadcast[0],
        event="AUCTION_RESULT",
        event_data={
            "task_id": task["task_id"],
            "pickup": task["pickup"],
            "dropoff": task["dropoff"],
            "bids": bids,
            "winner": winner["amr_id"] if winner else None,
            "cost": winner["cost"] if winner else None,
            "reason": reason,
        },
    )


def auction_task(task):

    task = {
        "task_id": task["task_id"],
        "pickup": list(task["pickup"]),
        "dropoff": list(task["dropoff"]),
    }

    task_id = task["task_id"]

    known_tasks[task_id] = task

    with state_lock:

        known_bids.setdefault(task_id, {})


    # Wait until some AMR is free; tasks stay in submission order.

    while True:

        with state_lock:

            anyone_idle = "IDLE" in fleet_status.values()

        if anyone_idle:

            break

        time.sleep(
            0.5
        )


    print()

    print(
        "=" * 70
    )

    print(
        f"[{amr_id}] "
        f"ANNOUNCING {task_id}"
    )

    print(
        f"Pickup : "
        f"{task['pickup']}"
    )

    print(
        f"Drop   : "
        f"{task['dropoff']}"
    )

    print(
        "=" * 70
    )


    # ----------------------------------------------------
    # Announcer's own bid
    # ----------------------------------------------------

    if (

        fleet_status[
            amr_id
        ]
        ==
        "IDLE"

    ):

        bid = calculate_bid(
            task
        )


        with state_lock:

            known_bids[
                task_id
            ][
                amr_id
            ] = bid


        print(

            f"[{amr_id}] "
            f"LOCAL BID | "
            f"{task_id} = "
            f"{bid['cost']}"
        )


        broadcast_message({

            "type":
                "BID",

            "task_id":
                task_id,

            "amr_id":
                amr_id,

            "cost":
                bid["cost"],

            "distance_to_pickup":
                bid[
                    "distance_to_pickup"
                ],

            "distance_pickup_to_dropoff":
                bid[
                    "distance_pickup_to_dropoff"
                ],
        })


    # ----------------------------------------------------
    # Announce task
    # ----------------------------------------------------

    broadcast_message({

        "type":
            "TASK",

        "task":
            task,
    })


    # ----------------------------------------------------
    # Wait for winner
    # ----------------------------------------------------

    deadline = (
        time.time()
        +
        BID_TIMEOUT
    )


    while True:

        winner = (
            determine_task_winner(
                task_id
            )
        )


        if winner is not None:

            break


        # A lost bid or a stopped peer must not stall the queue.

        if time.time() >= deadline:

            winner = determine_task_winner(
                task_id,
                require_all_bids=False
            )

            break


        time.sleep(
            0.2
        )


    if winner is None:

        print(
            f"[{amr_id}] "
            f"NO VALID BIDS | "
            f"{task_id} left unassigned"
        )

        report_auction(
            task,
            None,
            "No AMR has a route to this task"
            if known_bids.get(task_id)
            else "No bids received in time"
        )

        return


    # ----------------------------------------------------
    # Commit winner
    # ----------------------------------------------------

    task_winners[
        task_id
    ] = winner[
        "amr_id"
    ]


    print()

    print(

        f"[{amr_id}] "
        f"DISTRIBUTED WINNER | "
        f"{task_id} -> "
        f"{winner['amr_id']} "
        f"| Cost: "
        f"{winner['cost']}"
    )


    report_auction(
        task,
        winner
    )


    commit_task_assignment(

        task_id,

        winner[
            "amr_id"
        ]
    )


    # ----------------------------------------------------
    # Tell all peers
    # ----------------------------------------------------

    broadcast_message({

        "type":
            "TASK_ASSIGNED",

        "task_id":
            task_id,

        "amr_id":
            winner[
                "amr_id"
            ],
    })


    # Allow BUSY state to propagate
    # before announcing the next task.

    time.sleep(
        TASK_SETTLE_DELAY
    )


# ============================================================
# EXECUTE ASSIGNED TASK
# Always releases the AMR, whether the task completes or fails.
# ============================================================

def execute_task(
    task
):

    completed = False

    try:

        completed = run_task(
            task
        )

    finally:

        release_amr(
            task,
            completed
        )


def release_amr(
    task,
    completed
):

    global assigned_task
    global current_task
    global current_target
    global current_path
    global path_index
    global task_execution_started


    if not completed:

        print()

        print(
            f"[{amr_id}] "
            f"TASK FAILED | "
            f"{task['task_id']}"
        )


        broadcast_message({

            "type":
                "TASK_FAILED",

            "task_id":
                task["task_id"],

            "amr_id":
                amr_id,

            "position":
                list(position),
        })


        broadcast_telemetry(
            "TASK_FAILED",
            event="TASK_FAILED",
            event_data={
                "task_id": task["task_id"],
            }
        )


    # ========================================================
    # RELEASE AMR
    # ========================================================

    with state_lock:

        fleet_status[
            amr_id
        ] = "IDLE"

        assigned_tasks[
            amr_id
        ] = None


    assigned_task = None

    current_task = None

    current_target = None

    current_path = []

    path_index = 0

    task_execution_started = False

    tick_gate.set_active(
        False
    )


    broadcast_state(
        "IDLE"
    )


    print()

    print(

        f"[{amr_id}] "
        "STATUS -> IDLE | "
        "READY FOR NEW TASKS"
    )


# ============================================================
# WAIT-CYCLE (DEADLOCK) DETECTION
# Advisory only: logs cycles, never changes a movement decision.
# Runs only on ticks where this AMR is blocked.
# ============================================================

# Wait cycles including this AMR that are already logged.
reported_wait_cycles = set()


def build_wait_for_graph(next_position):
    """Edge A -> B when A's announced next cell is B's current cell.

    That is the cell the occupied-cell safety check refuses to enter,
    so A cannot progress until B leaves. Only fresh peer state is used.
    """

    states = {amr_id: (position, next_position)}

    for peer_id, peer in get_active_peers().items():
        states[peer_id] = (peer["position"], peer["next_position"])

    occupant = {cell: robot_id for robot_id, (cell, _) in states.items()}

    return {
        robot_id: {occupant[wanted]}
        for robot_id, (_, wanted) in states.items()
        if wanted in occupant and occupant[wanted] != robot_id
    }


def report_wait_cycles(next_position):
    """Log each wait cycle including this AMR once while it persists."""

    cycles = {
        cycle
        for cycle in find_wait_cycles(build_wait_for_graph(next_position))
        if amr_id in cycle
    }

    for cycle in sorted(cycles - reported_wait_cycles):
        print(
            f"[{amr_id}] "
            f"WAIT CYCLE DETECTED (advisory) | "
            f"{' -> '.join(cycle + cycle[:1])}"
        )

    # A cycle that has gone is dropped, so its return is logged again.
    reported_wait_cycles.clear()
    reported_wait_cycles.update(cycles)


# ============================================================
# BLOCKED-MOVE RECOVERY
# A higher-priority AMR that keeps WAITING (moving peer, head-on,
# occupied cell) replans after a timeout instead of waiting forever.
# ============================================================

# Consecutive blocked MOVE ticks before the first recovery replan.
BLOCKED_REPLAN_THRESHOLD = 3

# Blocked ticks between recovery attempts while still stuck.
BLOCKED_REPLAN_RETRY_TICKS = 3


def record_blocked_tick(next_position):
    """Count a blocked tick; replan on the threshold tick and every retry interval after."""

    global waiting_steps

    waiting_steps += 1

    report_wait_cycles(next_position)

    overdue = waiting_steps - BLOCKED_REPLAN_THRESHOLD

    if overdue < 0 or overdue % BLOCKED_REPLAN_RETRY_TICKS:
        return False

    print(
        f"[{amr_id}] "
        f"BLOCKED {waiting_steps} TICKS -> RECOVERY REPLAN"
    )

    # The new path still goes through detect_conflict() and the
    # occupied-cell check on the next tick before any move.
    return replan_current_task()


# ============================================================
# DRIVE TO GOAL
# Shared by the pickup and drop-off legs. Returns False when no
# route to the goal exists.
# ============================================================

def drive_to(
    goal,
    task,
    leg_name,
    moving_status
):

    global position
    global current_path
    global path_index
    global move_count
    global conflict_count
    global waiting_steps


    while position != goal:

        # ----------------------------------------------------
        # Make sure a path exists.
        # ----------------------------------------------------

        if (

            current_path is None

            or

            path_index
            >=
            len(current_path) - 1

        ):

            current_path = (
                calculate_path(
                    goal
                )
            )

            path_index = 0


            if current_path is None:

                print(
                    f"[{amr_id}] "
                    f"NO PATH TO {leg_name}."
                )

                return False


        # ----------------------------------------------------
        # Determine next cell.
        # ----------------------------------------------------

        next_position = (
            current_path[
                path_index + 1
            ]
        )


        # ----------------------------------------------------
        # Detect dynamic obstacle
        # ----------------------------------------------------

        if detect_dynamic_obstacle(
            next_position
        ):

            print(
                f"[{amr_id}] Path cell {next_position} "
                "is now blocked."
            )

            success = replan_current_task()

            if success:
                continue

            print(
                f"[{amr_id}] WARNING: Dynamic obstacle "
                "detected but no alternate path exists."
            )

            wait_ticks(1)
            continue


        # ----------------------------------------------------
        # Announce intent in this tick's INTENT phase.
        # ----------------------------------------------------

        tick = tick_gate.take(
            "INTENT"
        )


        broadcast_state(

            "INTENT",

            next_position
        )


        tick_gate.send_ack(
            tick,
            "INTENT"
        )


        # Every peer has announced its intent before MOVE opens.

        tick_gate.take(
            "MOVE",
            tick
        )


        # A WAIT, REPLAN or MOVE each use up this tick.

        try:

            # ----------------------------------------------------
            # Detect conflicts.
            # ----------------------------------------------------

            conflicts = (
                detect_conflict(
                    next_position
                )
            )


            # ----------------------------------------------------
            # Edge-AI advisory layer
            # ----------------------------------------------------
            ai_assessment = assess_next_move_risk(
                next_position,
                conflicts,
            )

            # Advisory only: when a conflict is detected the priority
            # rules below decide. Without one, a confident HIGH-risk
            # prediction triggers an early replan, but only for an AMR
            # that must yield to a nearby higher-priority peer.
            if (
                not conflicts
                and ai_assessment.action == "REPLAN"
                and ai_assessment.confidence >= 70.0
                and yields_to_nearby_peer()
            ):
                success = replan_current_task()
                if success:
                    continue


            if conflicts:

                conflict_count += 1


                print(

                    f"[{amr_id}] "
                    f"CONFLICT EVENT #"
                    f"{conflict_count}"
                )


                decision = (
                    resolve_movement_conflict(
                        conflicts
                    )
                )


                # -----------------------------------------------
                # REPLAN
                # -----------------------------------------------

                if decision == "REPLAN":

                    success = (
                        replan_current_task()
                    )


                    if success:

                        continue


                    broadcast_state(

                        "WAITING",

                        next_position
                    )

                    waiting_steps += 1

                    report_wait_cycles(next_position)


                    continue


                # -----------------------------------------------
                # WAIT
                # -----------------------------------------------

                if decision == "WAIT":

                    broadcast_state(

                        "WAITING",

                        next_position
                    )

                    record_blocked_tick(next_position)


                    continue


            # ----------------------------------------------------
            # Final safety check.
            # ----------------------------------------------------

            occupied_now = any(

                peer["position"]
                ==
                next_position

                for peer
                in
                get_active_peers().values()

            )


            if occupied_now:

                broadcast_state(

                    "WAITING",

                    next_position
                )

                record_blocked_tick(next_position)


                continue


            # ----------------------------------------------------
            # SAFE MOVE
            # ----------------------------------------------------

            with state_lock:

                old_position = position

                position = next_position

                path_index += 1

                move_count += 1
                waiting_steps = 0

            # Having moved, this AMR is in no wait cycle; a later one is new.
            reported_wait_cycles.clear()


            print(

                f"[{amr_id}] "
                f"MOVE "
                f"{old_position} -> "
                f"{position}"
            )


            broadcast_state(
                "MOVING"
            )


            broadcast_message({

                "type":
                    "TASK_STATUS",

                "task_id":
                    task["task_id"],

                "amr_id":
                    amr_id,

                "status":
                    moving_status,

                "position":
                    list(position),
            })


        finally:

            tick_gate.send_ack(
                tick,
                "MOVE"
            )

    return True


# ============================================================
# RUN ASSIGNED TASK
# Returns True on delivery, False when no route exists.
# ============================================================

def run_task(
    task
):

    global position
    global current_target
    global current_path
    global path_index
    global move_count
    global assigned_task
    global current_task
    global task_execution_started
    global replan_count
    global conflict_count
    global collision_count
    global waiting_steps


    pickup = tuple(
        task["pickup"]
    )

    dropoff = tuple(
        task["dropoff"]
    )


    # Reset task metrics.

    replan_count = 0

    conflict_count = 0

    collision_count = 0

    move_count = 0

    waiting_steps = 0


    # ========================================================
    # LEG 1 — GO TO PICKUP
    # ========================================================

    current_target = pickup

    current_path = (
        calculate_path(
            pickup
        )
    )

    path_index = 0


    if current_path is None:

        print(
            f"[{amr_id}] "
            "ERROR: No path to pickup."
        )

        return False


    print()

    print(
        "=" * 70
    )

    print(
        f"[{amr_id}] "
        "TASK EXECUTION STARTED"
    )

    print(
        f"Task   : "
        f"{task['task_id']}"
    )

    print(
        f"Pickup : "
        f"{pickup}"
    )

    print(
        f"Drop   : "
        f"{dropoff}"
    )

    print(
        f"Pickup path length: "
        f"{len(current_path) - 1}"
    )

    print(
        "=" * 70
    )


    broadcast_state(
        "BUSY"
    )


    broadcast_message({

        "type":
            "TASK_STATUS",

        "task_id":
            task["task_id"],

        "amr_id":
            amr_id,

        "status":
            "TASK_STARTED",

        "position":
            list(position),
    })


    # ========================================================
    # MOVE TO PICKUP
    # ========================================================

    if not drive_to(
        pickup,
        task,
        "PICKUP",
        "MOVING_TO_PICKUP"
    ):

        return False


    # ========================================================
    # PICKUP
    # ========================================================

    print()

    print(

        f"[{amr_id}] "
        f"📦 PICKUP REACHED at "
        f"{position}"
    )


    broadcast_state(
        "PICKUP_REACHED"
    )


    broadcast_message({

        "type":
            "TASK_STATUS",

        "task_id":
            task["task_id"],

        "amr_id":
            amr_id,

        "status":
            "PICKUP_REACHED",

        "position":
            list(position),
    })


    wait_ticks(
        1
    )


    print(

        f"[{amr_id}] "
        "📦 Pickup completed."
    )


    # ========================================================
    # LEG 2 — GO TO DROP-OFF
    # ========================================================

    current_target = dropoff

    current_path = (
        calculate_path(
            dropoff
        )
    )

    path_index = 0


    if current_path is None:

        print(
            f"[{amr_id}] "
            "ERROR: No path to drop-off."
        )

        return False


    print()

    print(

        f"[{amr_id}] "
        "ROUTING TO DROP-OFF "
        f"| Path length: "
        f"{len(current_path) - 1}"
    )


    # ========================================================
    # MOVE TO DROP-OFF
    # ========================================================

    if not drive_to(
        dropoff,
        task,
        "DROP-OFF",
        "MOVING_TO_DROPOFF"
    ):

        return False


    # ========================================================
    # DELIVERY COMPLETE
    # ========================================================

    print()

    print(
        "=" * 70
    )

    print(

        f"[{amr_id}] "
        "✅ DELIVERY COMPLETE"
    )


    print(

        f"Task      : "
        f"{task['task_id']}"
    )


    print(

        f"Final pos : "
        f"{position}"
    )


    print(

        f"Moves     : "
        f"{move_count}"
    )


    print(

        f"Conflicts : "
        f"{conflict_count}"
    )


    print(

        f"Replans   : "
        f"{replan_count}"
    )


    print(

        f"Collisions: "
        f"{collision_count}"
    )


    print(
        "=" * 70
    )


    broadcast_state(
        "TASK_COMPLETE"
    )


    broadcast_message({

        "type":
            "TASK_COMPLETE",

        "task_id":
            task["task_id"],

        "amr_id":
            amr_id,

        "position":
            list(position),
    })


    return True


# ============================================================
# TELEMETRY SOCKET
# ============================================================

# Separate UDP socket for the optional observation/visualization
# layer. It does not participate in AMR coordination decisions.
telemetry_sock = socket.socket(
    socket.AF_INET,
    socket.SOCK_DGRAM,
)


# ============================================================
# UDP SOCKET
# ============================================================

sock = socket.socket(
    socket.AF_INET,
    socket.SOCK_DGRAM,
)


sock.setsockopt(
    socket.SOL_SOCKET,
    socket.SO_REUSEADDR,
    1,
)


# ============================================================
# HEARTBEAT
# ============================================================

def send_heartbeat():

    with broadcast_lock:

        if task_execution_started:

            # Repeat the latest state: a bare BUSY would erase the
            # pending move intent that peers use to detect conflicts.
            broadcast_state(
                *last_broadcast
            )

            return


    if fleet_status[
        amr_id
    ] == "BUSY":

        broadcast_state(
            "BUSY"
        )

    else:

        broadcast_state(
            "READY"
        )


# ============================================================
# INDEPENDENT WINNER REPORT
# Every AMR independently determines winners.
# ============================================================

def report_independent_winners(
    reported_winners
):

    for task_id in list(known_bids):

        # Once committed, the idle set has changed and a recount
        # would name a different, phantom winner.

        if task_winners.get(task_id) is not None:

            continue


        winner = (
            determine_task_winner(
                task_id
            )
        )


        if winner is None:

            continue


        winner_id = (
            winner["amr_id"]
        )


        if (
            reported_winners.get(
                task_id
            )
            ==
            winner_id
        ):

            continue


        reported_winners[
            task_id
        ] = winner_id


        print()

        print(
            "=" * 70
        )

        print(

            f"[{amr_id}] "
            "TASK WINNER DETERMINED"
        )


        print(

            f"Task   : "
            f"{task_id}"
        )


        print(

            f"Winner : "
            f"{winner_id}"
        )


        print(

            f"Cost   : "
            f"{winner['cost']}"
        )


        print(
            "This AMR independently "
            "computed the winner."
        )


        print(
            "=" * 70
        )


# ============================================================
# MAIN
# ============================================================

def main():

    global cycle


    sock.bind(
        (
            "127.0.0.1",
            config["port"]
        )
    )


    sock.settimeout(
        0.2
    )


    # ============================================================
    # STARTUP
    # ============================================================

    print()

    print(
        "=" * 70
    )

    print(
        "UNIFIED DISTRIBUTED FLEET CONTROLLER"
    )

    print(
        "=" * 70
    )

    print(
        f"AMR ID   : "
        f"{amr_id}"
    )

    print(
        f"Position : "
        f"{position}"
    )

    print(
        f"Priority : "
        f"{priority}"
    )

    print(
        f"Port     : "
        f"{config['port']}"
    )

    print(
        "=" * 70
    )

    print(
        "Phase: Integrated Fleet Architecture"
    )

    print(
        "=" * 70
    )


    # Train the Edge-AI model now; training inside a tick would
    # stall the simulation clock.

    get_edge_ai_model()


    # ============================================================
    # START RECEIVER THREAD
    # ============================================================

    receiver_thread = threading.Thread(

        target=receive_messages,

        daemon=True
    )

    receiver_thread.start()


    # ============================================================
    # INITIAL A* TEST
    # ============================================================

    test_goal = TASKS[
        "TASK-01"
    ][
        "pickup"
    ]


    test_path = calculate_path(
        test_goal
    )


    print()

    print(

        f"[{amr_id}] "
        f"A* TEST GOAL: "
        f"{test_goal}"
    )


    if test_path is None:

        print(

            f"[{amr_id}] "
            "NO PATH FOUND"
        )

    else:

        print(

            f"[{amr_id}] "
            "A* PATH FOUND"
        )


        print(

            f"[{amr_id}] "
            f"Path length: "
            f"{len(test_path) - 1}"
        )


        print(

            f"[{amr_id}] "
            f"Path: "
            f"{test_path}"
        )


    # ============================================================
    # P2P READY
    # ============================================================

    print()

    print(

        f"[{amr_id}] "
        "P2P communication active."
    )

    print(
        f"[{amr_id}] "
        f"Telemetry UDP target: "
        f"{TELEMETRY_HOST}:{TELEMETRY_PORT}"
    )

    broadcast_state(
        "READY"
    )


    # The fleet starts idle; AMR-1 auctions tasks as the operator adds them.

    if amr_id == "AMR-1":

        threading.Thread(

            target=run_auctions,

            daemon=True

        ).start()


    # ============================================================
    # MAIN FLEET LOOP
    # ============================================================

    reported_winners = {}


    try:

        while True:

            cycle += 1


            send_heartbeat()


            report_independent_winners(
                reported_winners
            )


            time.sleep(
                1
            )


    except KeyboardInterrupt:

        print()

        print(
            "=" * 70
        )

        print(

            f"[{amr_id}] "
            "SHUTDOWN"
        )

        print(
            "=" * 70
        )


        sock.close()
        telemetry_sock.close()


if __name__ == "__main__":

    main()

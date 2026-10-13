import socket
import threading
import json
import time
import sys
import heapq


# ============================================================
# WAREHOUSE CONFIGURATION
# ============================================================

WIDTH = 15
HEIGHT = 10

OBSTACLES = {
    (4, 2), (4, 3), (4, 4), (4, 5),
    (8, 4), (8, 5), (8, 6),
    (11, 2), (11, 3), (11, 4), (11, 5)
}


# ============================================================
# AMR CONFIGURATION
# ============================================================

AMR_CONFIG = {
    "AMR-1": {
        "port": 6001,
        "position": (1, 1),
    },

    "AMR-2": {
        "port": 6002,
        "position": (7, 1),
    },

    "AMR-3": {
        "port": 6003,
        "position": (13, 8),
    },
}


# ============================================================
# PEER PORTS
# ============================================================

PEERS = {
    "AMR-1": [6002, 6003],
    "AMR-2": [6001, 6003],
    "AMR-3": [6001, 6002],
}


# ============================================================
# DEMONSTRATION TASK
# ============================================================

TASK = {
    "task_id": "TASK-01",
    "pickup": (10, 2),
    "dropoff": (3, 7),
}


# ============================================================
# READ AMR ID
# ============================================================

if len(sys.argv) != 2:

    print()
    print("Usage:")
    print("  python3 task_allocation.py AMR-1")
    print("  python3 task_allocation.py AMR-2")
    print("  python3 task_allocation.py AMR-3")
    print()

    sys.exit(1)


amr_id = sys.argv[1]

if amr_id not in AMR_CONFIG:

    print()
    print("Invalid AMR ID.")
    print()

    sys.exit(1)


# ============================================================
# LOCAL STATE
# ============================================================

position = AMR_CONFIG[amr_id]["position"]

known_task = None

task_bids = {}

task_winner = None

task_execution_started = False

state_lock = threading.Lock()


# ============================================================
# HELPER — MANHATTAN HEURISTIC
# ============================================================

def heuristic(a, b):

    return (
        abs(a[0] - b[0])
        +
        abs(a[1] - b[1])
    )


# ============================================================
# GET VALID WAREHOUSE NEIGHBORS
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

        valid.append((nx, ny))

    return valid


# ============================================================
# A* DISTANCE
# ============================================================

def a_star_distance(start, goal):

    open_set = []

    heapq.heappush(
        open_set,
        (0, start)
    )

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

        visited.add(current)

        if current == goal:

            return g_score[current]

        for neighbor in get_neighbors(current):

            tentative_g = (
                g_score[current] + 1
            )

            if tentative_g < g_score.get(
                neighbor,
                float("inf")
            ):

                g_score[neighbor] = (
                    tentative_g
                )

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

    return float("inf")


# ============================================================
# A* PATH
# ============================================================

def a_star_path(start, goal):

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

        visited.add(current)

        if current == goal:

            path = []

            while current in came_from:

                path.append(current)

                current = came_from[current]

            path.append(start)

            path.reverse()

            return path

        for neighbor in get_neighbors(current):

            tentative_g = (
                g_score[current] + 1
            )

            if tentative_g < g_score.get(
                neighbor,
                float("inf")
            ):

                came_from[neighbor] = current

                g_score[neighbor] = (
                    tentative_g
                )

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
# CALCULATE TASK BID
# ============================================================

def calculate_bid(task):

    pickup = tuple(
        task["pickup"]
    )

    dropoff = tuple(
        task["dropoff"]
    )

    distance_to_pickup = (
        a_star_distance(
            position,
            pickup
        )
    )

    distance_pickup_to_dropoff = (
        a_star_distance(
            pickup,
            dropoff
        )
    )

    total_cost = (
        distance_to_pickup
        +
        distance_pickup_to_dropoff
    )

    return {
        "amr_id": amr_id,
        "task_id": task["task_id"],
        "cost": total_cost,
        "distance_to_pickup":
            distance_to_pickup,
        "distance_pickup_to_dropoff":
            distance_pickup_to_dropoff,
    }


# ============================================================
# BROADCAST P2P MESSAGE
# ============================================================

def broadcast(message):

    data = json.dumps(
        message
    ).encode()

    for peer_port in PEERS[amr_id]:

        try:

            sock.sendto(
                data,
                ("127.0.0.1", peer_port)
            )

        except Exception as e:

            print(
                f"[{amr_id}] "
                f"SEND ERROR: {e}"
            )


# ============================================================
# DETERMINE TASK WINNER
# ============================================================

def determine_winner():

    if known_task is None:
        return None

    with state_lock:

        if len(task_bids) < 3:
            return None

        bids = list(
            task_bids.values()
        )

    # Lowest cost wins.
    # AMR ID is deterministic tie-breaker.

    bids.sort(
        key=lambda bid: (
            bid["cost"],
            bid["amr_id"]
        )
    )

    return bids[0]


# ============================================================
# EXECUTE ASSIGNED TASK
# ============================================================

def execute_task(task):

    global position

    pickup = tuple(
        task["pickup"]
    )

    dropoff = tuple(
        task["dropoff"]
    )

    # ========================================================
    # PATH TO PICKUP
    # ========================================================

    pickup_path = a_star_path(
        position,
        pickup
    )

    if pickup_path is None:

        print()
        print(
            f"[{amr_id}] "
            "ERROR: No path to pickup."
        )

        return

    print()
    print("=" * 65)
    print(
        f"[{amr_id}] "
        "TASK EXECUTION STARTED"
    )
    print("=" * 65)

    print(
        f"Task        : "
        f"{task['task_id']}"
    )

    print(
        f"Pickup      : "
        f"{pickup}"
    )

    print(
        f"Drop-off    : "
        f"{dropoff}"
    )

    print(
        f"Pickup path : "
        f"{len(pickup_path) - 1} steps"
    )

    print("=" * 65)

    # ========================================================
    # BROADCAST ASSIGNMENT
    # ========================================================

    broadcast({

        "type": "TASK_ASSIGNED",

        "task_id":
            task["task_id"],

        "amr_id":
            amr_id
    })

    # ========================================================
    # MOVE TO PICKUP
    # ========================================================

    for next_position in pickup_path[1:]:

        time.sleep(0.5)

        with state_lock:

            old_position = position

            position = next_position

        print(
            f"[{amr_id}] "
            f"MOVE "
            f"{old_position} -> "
            f"{position}"
        )

        broadcast({

            "type": "TASK_STATUS",

            "task_id":
                task["task_id"],

            "amr_id":
                amr_id,

            "status":
                "MOVING_TO_PICKUP",

            "position":
                list(position)
        })

    # ========================================================
    # PICKUP REACHED
    # ========================================================

    print()
    print(
        f"[{amr_id}] "
        f"📦 PICKUP REACHED at "
        f"{position}"
    )

    broadcast({

        "type": "TASK_STATUS",

        "task_id":
            task["task_id"],

        "amr_id":
            amr_id,

        "status":
            "PICKUP_REACHED",

        "position":
            list(position)
    })

    time.sleep(1)

    print(
        f"[{amr_id}] "
        "📦 Pickup completed."
    )

    # ========================================================
    # PATH TO DROP-OFF
    # ========================================================

    dropoff_path = a_star_path(
        position,
        dropoff
    )

    if dropoff_path is None:

        print(
            f"[{amr_id}] "
            "ERROR: No path to drop-off."
        )

        return

    print()
    print(
        f"[{amr_id}] "
        "Routing to drop-off..."
    )

    print(
        f"[{amr_id}] "
        f"Drop-off path : "
        f"{len(dropoff_path) - 1} steps"
    )

    # ========================================================
    # MOVE TO DROP-OFF
    # ========================================================

    for next_position in dropoff_path[1:]:

        time.sleep(0.5)

        with state_lock:

            old_position = position

            position = next_position

        print(
            f"[{amr_id}] "
            f"MOVE "
            f"{old_position} -> "
            f"{position}"
        )

        broadcast({

            "type": "TASK_STATUS",

            "task_id":
                task["task_id"],

            "amr_id":
                amr_id,

            "status":
                "MOVING_TO_DROPOFF",

            "position":
                list(position)
        })

    # ========================================================
    # DELIVERY COMPLETE
    # ========================================================

    print()
    print("=" * 65)

    print(
        f"[{amr_id}] "
        "✅ DELIVERY COMPLETE"
    )

    print(
        f"[{amr_id}] "
        f"Task           : "
        f"{task['task_id']}"
    )

    print(
        f"[{amr_id}] "
        f"Final position : "
        f"{position}"
    )

    print("=" * 65)

    broadcast({

        "type": "TASK_COMPLETE",

        "task_id":
            task["task_id"],

        "amr_id":
            amr_id,

        "position":
            list(position)
    })


# ============================================================
# RECEIVE P2P MESSAGES
# ============================================================

def receive_messages():

    global known_task

    while True:

        try:

            data, address = (
                sock.recvfrom(4096)
            )

            message = json.loads(
                data.decode()
            )

            message_type = message.get(
                "type",
                "UNKNOWN"
            )

            # =================================================
            # TASK ANNOUNCEMENT
            # =================================================

            if message_type == "TASK":

                task = message["task"]

                with state_lock:

                    known_task = task

                print()
                print(
                    f"[{amr_id}] "
                    f"TASK RECEIVED: "
                    f"{task['task_id']}"
                )

                print(
                    f"[{amr_id}] "
                    f"Pickup : "
                    f"{task['pickup']}"
                )

                print(
                    f"[{amr_id}] "
                    f"Drop   : "
                    f"{task['dropoff']}"
                )

                bid = calculate_bid(
                    task
                )

                with state_lock:

                    task_bids[
                        bid["amr_id"]
                    ] = bid

                print(
                    f"[{amr_id}] "
                    f"BID = "
                    f"{bid['cost']}"
                )

                # Send bid to peers

                broadcast({

                    "type": "BID",

                    "task_id":
                        bid["task_id"],

                    "amr_id":
                        bid["amr_id"],

                    "cost":
                        bid["cost"],

                    "distance_to_pickup":
                        bid[
                            "distance_to_pickup"
                        ],

                    "distance_pickup_to_dropoff":
                        bid[
                            "distance_pickup_to_dropoff"
                        ]
                })


            # =================================================
            # BID RECEIVED
            # =================================================

            elif message_type == "BID":

                if (
                    known_task is not None
                    and
                    message["task_id"]
                    ==
                    known_task["task_id"]
                ):

                    with state_lock:

                        task_bids[
                            message["amr_id"]
                        ] = {

                            "amr_id":
                                message["amr_id"],

                            "task_id":
                                message["task_id"],

                            "cost":
                                message["cost"],

                            "distance_to_pickup":
                                message[
                                    "distance_to_pickup"
                                ],

                            "distance_pickup_to_dropoff":
                                message[
                                    "distance_pickup_to_dropoff"
                                ]
                        }

                    print(
                        f"[{amr_id}] "
                        f"BID RECEIVED from "
                        f"{message['amr_id']} "
                        f"= {message['cost']}"
                    )


            # =================================================
            # TASK ASSIGNED
            # =================================================

            elif message_type == "TASK_ASSIGNED":

                print()
                print(
                    f"[{amr_id}] "
                    f"TASK {message['task_id']} "
                    f"ASSIGNED TO "
                    f"{message['amr_id']}"
                )


            # =================================================
            # TASK STATUS
            # =================================================

            elif message_type == "TASK_STATUS":

                print(
                    f"[{amr_id}] "
                    f"TASK STATUS from "
                    f"{message['amr_id']} "
                    f"-> "
                    f"{message['status']} "
                    f"at "
                    f"{message['position']}"
                )


            # =================================================
            # TASK COMPLETE
            # =================================================

            elif message_type == "TASK_COMPLETE":

                print()
                print(
                    f"[{amr_id}] "
                    f"✅ TASK COMPLETE "
                    f"reported by "
                    f"{message['amr_id']}"
                )


        except socket.timeout:

            continue

        except Exception as e:

            print(
                f"[{amr_id}] "
                f"RECEIVE ERROR: {e}"
            )


# ============================================================
# TASK ANNOUNCER
# ============================================================

def announce_task():

    time.sleep(3)

    print()
    print("=" * 65)
    print(
        f"[{amr_id}] "
        "ANNOUNCING NEW TASK"
    )
    print("=" * 65)

    print(
        f"Task ID : "
        f"{TASK['task_id']}"
    )

    print(
        f"Pickup  : "
        f"{TASK['pickup']}"
    )

    print(
        f"Dropoff : "
        f"{TASK['dropoff']}"
    )

    print("=" * 65)

    # --------------------------------------------------------
    # Build task message
    # --------------------------------------------------------

    message = {

        "type": "TASK",

        "task": {

            "task_id":
                TASK["task_id"],

            "pickup":
                list(TASK["pickup"]),

            "dropoff":
                list(TASK["dropoff"])
        }
    }

    # --------------------------------------------------------
    # Store task locally
    # --------------------------------------------------------

    global known_task

    known_task = message["task"]

    # --------------------------------------------------------
    # Calculate own bid
    # --------------------------------------------------------

    bid = calculate_bid(
        known_task
    )

    with state_lock:

        task_bids[
            bid["amr_id"]
        ] = bid

    print(
        f"[{amr_id}] "
        f"LOCAL BID = "
        f"{bid['cost']}"
    )

    # --------------------------------------------------------
    # Broadcast task
    # --------------------------------------------------------

    broadcast(
        message
    )

    # --------------------------------------------------------
    # Broadcast own bid
    # --------------------------------------------------------

    bid_message = {

        "type": "BID",

        "task_id":
            bid["task_id"],

        "amr_id":
            bid["amr_id"],

        "cost":
            bid["cost"],

        "distance_to_pickup":
            bid["distance_to_pickup"],

        "distance_pickup_to_dropoff":
            bid["distance_pickup_to_dropoff"]
    }

    broadcast(
        bid_message
    )

    print(
        f"[{amr_id}] "
        "TASK and BID broadcast to peers."
    )


# ============================================================
# CREATE UDP SOCKET
# ============================================================

sock = socket.socket(
    socket.AF_INET,
    socket.SOCK_DGRAM
)

sock.setsockopt(
    socket.SOL_SOCKET,
    socket.SO_REUSEADDR,
    1
)

sock.bind(
    (
        "127.0.0.1",
        AMR_CONFIG[amr_id]["port"]
    )
)

sock.settimeout(0.2)


# ============================================================
# START RECEIVER THREAD
# ============================================================

receiver_thread = threading.Thread(
    target=receive_messages,
    daemon=True
)

receiver_thread.start()


# ============================================================
# STARTUP
# ============================================================

print()
print("=" * 65)
print(
    f"{amr_id} "
    "DISTRIBUTED TASK AUCTION"
)
print("=" * 65)

print(
    f"Position : "
    f"{position}"
)

print(
    f"UDP Port : "
    f"{AMR_CONFIG[amr_id]['port']}"
)

print("=" * 65)


# ============================================================
# AMR-1 CREATES THE DEMO TASK
# ============================================================

if amr_id == "AMR-1":

    threading.Thread(
        target=announce_task,
        daemon=True
    ).start()


# ============================================================
# AUCTION MONITOR
# ============================================================

try:

    while True:

        time.sleep(1)

        winner = determine_winner()

        if winner is None:
            continue

        # ----------------------------------------------------
        # Handle newly determined winner once
        # ----------------------------------------------------

        if task_winner == winner["amr_id"]:
            continue

        task_winner = winner["amr_id"]

        print()
        print("=" * 65)

        print(
            f"[{amr_id}] "
            "TASK WINNER DETERMINED"
        )

        print(
            f"Task : "
            f"{winner['task_id']}"
        )

        print(
            f"Winner : "
            f"{winner['amr_id']}"
        )

        print(
            f"Winning cost : "
            f"{winner['cost']}"
        )

        print(
            "All AMRs independently "
            "computed the same winner."
        )

        print("=" * 65)

        # ----------------------------------------------------
        # Winner executes the task
        # ----------------------------------------------------

        if (
            winner["amr_id"] == amr_id
            and
            not task_execution_started
        ):

            task_execution_started = True

            execute_task(
                known_task
            )


except KeyboardInterrupt:

    print()
    print(
        f"[{amr_id}] "
        "SHUTDOWN"
    )

    sock.close()
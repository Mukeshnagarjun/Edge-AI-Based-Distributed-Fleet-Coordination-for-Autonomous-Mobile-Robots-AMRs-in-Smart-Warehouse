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

dynamic_obstacles = set()


# ============================================================
# AMR CONFIGURATION
# ============================================================

AMR_CONFIG = {

    "AMR-1": {
        "port": 5001,
        "start": (1, 1),
        "target": (13, 1),
        "priority": 1,
    },

    "AMR-2": {
        "port": 5002,
        "start": (7, 1),
        "target": (2, 8),
        "priority": 2,
    },

    "AMR-3": {
        "port": 5003,
        "start": (13, 8),
        "target": (1, 8),
        "priority": 3,
    },
}


# ============================================================
# PEER PORTS
# ============================================================

PEERS = {
    "AMR-1": [5002, 5003],
    "AMR-2": [5001, 5003],
    "AMR-3": [5001, 5002],
}


# ============================================================
# READ AMR ID
# ============================================================

if len(sys.argv) != 2:

    print("Usage:")
    print("python3 distributed_amr.py AMR-1")
    print("python3 distributed_amr.py AMR-2")
    print("python3 distributed_amr.py AMR-3")

    sys.exit(1)


amr_id = sys.argv[1]

if amr_id not in AMR_CONFIG:

    print("Invalid AMR ID.")

    sys.exit(1)


config = AMR_CONFIG[amr_id]


# ============================================================
# LOCAL STATE
# ============================================================

position = config["start"]

target = config["target"]

priority = config["priority"]

path = []

path_index = 0

cycle = 0

replan_count = 0

move_count = 0

obstacle_announced = False

known_amrs = {}

state_lock = threading.Lock()

obstacle_lock = threading.Lock()

# ============================================================
# TEST DYNAMIC OBSTACLE
# ============================================================

DYNAMIC_TEST_OBSTACLE = (9, 1)

# ============================================================
# A* PATH PLANNER
# ============================================================

def heuristic(a, b):

    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def get_neighbors(position):

    x, y = position

    candidates = [
        (x + 1, y),
        (x - 1, y),
        (x, y + 1),
        (x, y - 1)
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


def a_star(start, goal, dynamic_obstacles=None):

    if dynamic_obstacles is None:
        dynamic_obstacles = set()

    # Never block the robot's own start cell
    dynamic_obstacles = set(dynamic_obstacles)
    dynamic_obstacles.discard(start)

    open_set = []

    heapq.heappush(
        open_set,
        (0, start)
    )

    came_from = {}

    g_score = {
        start: 0
    }

    while open_set:

        _, current = heapq.heappop(open_set)

        if current == goal:

            path_result = []

            while current in came_from:

                path_result.append(current)

                current = came_from[current]

            path_result.append(start)

            path_result.reverse()

            return path_result

        for neighbor in get_neighbors(current):

            if neighbor in dynamic_obstacles:
                continue

            tentative_g = (
                g_score[current] + 1
            )

            if tentative_g < g_score.get(
                neighbor,
                float("inf")
            ):

                came_from[neighbor] = current

                g_score[neighbor] = tentative_g

                f_score = (
                    tentative_g
                    + heuristic(neighbor, goal)
                )

                heapq.heappush(
                    open_set,
                    (f_score, neighbor)
                )

    return None


# ============================================================
# CALCULATE INITIAL PATH
# ============================================================

def calculate_initial_path():

    global path
    global path_index

    new_path = a_star(
        position,
        target
    )

    if new_path is None:

        print(
            f"[{amr_id}] ERROR: "
            "No path found."
        )

        sys.exit(1)

    path = new_path

    path_index = 0


# ============================================================
# CALCULATE NEXT POSITION
# ============================================================

def get_next_position():

    with state_lock:

        if path_index >= len(path) - 1:

            return position

        return path[path_index + 1]


# ============================================================
# RECEIVE PEER STATES
# ============================================================

def receive_messages():

    while True:

        try:

            data, address = sock.recvfrom(4096)

            message = json.loads(
                data.decode()
            )

            sender = message["amr_id"]

            if sender == amr_id:
                continue

            message_type = message.get("type", "STATE")

            # =================================================
            # OBSTACLE MESSAGE
            # =================================================

            if message_type == "OBSTACLE":

                obstacle = tuple(
                    message["obstacle"]
                )

                with obstacle_lock:

                    if obstacle not in dynamic_obstacles:

                        dynamic_obstacles.add(
                            obstacle
                        )

                        print()
                        print(
                            f"[{amr_id}] "
                            f"NEW OBSTACLE RECEIVED "
                            f"from {sender}: "
                            f"{obstacle}"
                        )

                continue

            # =================================================
            # NORMAL AMR STATE MESSAGE
            # =================================================

            with state_lock:

                known_amrs[sender] = {
                    "position": tuple(
                        message["position"]
                    ),

                    "next_position": tuple(
                        message["next_position"]
                    ),

                    "status": message["status"],

                    "priority": message["priority"],

                    "cycle": message["cycle"],

                    "timestamp": time.time()
                }

        except socket.timeout:

            continue

        except Exception as e:

            print(
                f"[{amr_id}] "
                f"Receive error: {e}"
            )


# ============================================================
# BROADCAST LOCAL STATE
# ============================================================

def broadcast_state(
    next_position,
    status
):

    message = {

        "amr_id": amr_id,

        "position": list(position),

        "next_position": list(
            next_position
        ),

        "target": list(target),

        "status": status,

        "priority": priority,

        "cycle": cycle
    }

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
                f"Send error: {e}"
            )


# ============================================================
# BROADCAST DISCOVERED OBSTACLE
# ============================================================

def broadcast_obstacle(obstacle):

    message = {

        "type": "OBSTACLE",

        "amr_id": amr_id,

        "obstacle": list(obstacle),

        "timestamp": time.time()
    }

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
                f"Obstacle send error: {e}"
            )

# ============================================================
# GET FRESH PEER STATES
# ============================================================

def get_active_peers():

    active = {}

    current_time = time.time()

    with state_lock:

        for peer_id, peer in known_amrs.items():

            age = (
                current_time
                - peer["timestamp"]
            )

            if age <= 2.0:

                active[peer_id] = peer

    return active


# ============================================================
# DETECT SAME-DESTINATION CONFLICT
# ============================================================

def same_destination_conflict(
    my_next,
    peer
):

    return (
        my_next == peer["next_position"]
        and
        my_next != peer["position"]
    )


# ============================================================
# DETECT HEAD-ON CONFLICT
# ============================================================

def head_on_conflict(
    my_next,
    peer
):

    return (
        my_next == peer["position"]
        and
        peer["next_position"] == position
    )


# ============================================================
# FIND DYNAMIC OBSTACLES
# ============================================================

def get_peer_obstacles():

    blocked = set()

    active_peers = get_active_peers()

    for peer_id, peer in active_peers.items():

        blocked.add(
            peer["position"]
        )

        blocked.add(
            peer["next_position"]
        )

    return blocked


# ============================================================
# REPLAN AROUND DYNAMIC OBSTACLES + PEER POSITIONS
# ============================================================

def replan():

    global path
    global path_index
    global replan_count

    # --------------------------------------------------------
    # Get obstacles discovered through P2P
    # --------------------------------------------------------

    with obstacle_lock:

        local_obstacles = set(
            dynamic_obstacles
        )

    # --------------------------------------------------------
    # Get currently occupied / intended peer cells
    # --------------------------------------------------------

    peer_obstacles = get_peer_obstacles()

    # --------------------------------------------------------
    # Combine both
    # --------------------------------------------------------

    combined_obstacles = (
        local_obstacles
        | peer_obstacles
    )

    # Never block our current cell
    combined_obstacles.discard(
        position
    )

    # --------------------------------------------------------
    # Run A*
    # --------------------------------------------------------

    new_path = a_star(
        position,
        target,
        combined_obstacles
    )

    if new_path is None:

        print()
        print(
            f"[{amr_id}] "
            "NO ALTERNATIVE PATH FOUND."
        )

        return False

    # --------------------------------------------------------
    # Install new route
    # --------------------------------------------------------

    path = new_path

    path_index = 0

    replan_count += 1

    print()
    print("=" * 65)
    print(
        f"[{amr_id}] "
        "DYNAMIC REPLANNING"
    )
    print("=" * 65)

    print(
        f"[{amr_id}] "
        f"Current position : {position}"
    )

    print(
        f"[{amr_id}] "
        f"Target           : {target}"
    )

    print(
        f"[{amr_id}] "
        f"Blocked cells    : "
        f"{combined_obstacles}"
    )

    print(
        f"[{amr_id}] "
        f"New path length  : "
        f"{len(path) - 1}"
    )

    print(
        f"[{amr_id}] "
        f"Replan count     : "
        f"{replan_count}"
    )

    print(
        f"[{amr_id}] "
        f"New path         : {path}"
    )

    print("=" * 65)

    return True


# ============================================================
# STARTUP
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
        config["port"]
    )
)

sock.settimeout(0.1)


# ============================================================
# MAIN AGENT
# ============================================================

def run_agent():

    global position
    global path_index
    global cycle
    global move_count
    global obstacle_announced

    print()

    print("=" * 70)
    print(
        f"{amr_id} "
        "DISTRIBUTED A* AGENT"
    )
    print("=" * 70)

    print(
        f"Start       : {config['start']}"
    )

    print(
        f"Target      : {target}"
    )

    print(
        f"Priority    : {priority}"
    )

    print(
        f"UDP Port    : {config['port']}"
    )

    print(
        f"A* Path     : {path}"
    )

    print(
        f"Path length : {len(path) - 1}"
    )

    print("=" * 70)

    print(
        f"[{amr_id}] "
        "Waiting 3 seconds for peers..."
    )

    time.sleep(3)

    arrived_reported = False

    while True:

        cycle += 1

        # ====================================================
        # ARRIVAL
        # ====================================================

        if position == target:

            if not arrived_reported:

                print()
                print(
                    f"[{amr_id}] "
                    f"ARRIVED at {position}"
                )

                broadcast_state(
                    position,
                    "ARRIVED"
                )

                arrived_reported = True

            time.sleep(2)

            continue

        arrived_reported = False

        # ====================================================
        # GET NEXT POSITION
        # ====================================================

        next_position = (
            get_next_position()
        )

                # ====================================================
        # DYNAMIC OBSTACLE DISCOVERY
        # ====================================================

        if (
            amr_id == "AMR-1"
            and
            next_position == DYNAMIC_TEST_OBSTACLE
        ):

            print(
    f"[{amr_id}] "
    "Obstacle broadcast to peers."
)

            if not obstacle_announced:

                print()
                print("=" * 65)
                print(
                    f"[{amr_id}] "
                    f"OBSTACLE DETECTED at "
                    f"{DYNAMIC_TEST_OBSTACLE}"
                )
                print("=" * 65)

                with obstacle_lock:

                    dynamic_obstacles.add(
                        DYNAMIC_TEST_OBSTACLE
                    )

                broadcast_obstacle(
                    DYNAMIC_TEST_OBSTACLE
                )

                obstacle_announced = True

                print(
                    f"[{amr_id}] "
                    "Obstacle added to local map."
                )

                print(
                    f"[{amr_id}] "
                    "Obstacle broadcast to peers."
                )

                # ============================================
                # RUN LOCAL A* REPLANNING
                # ============================================

                success = replan()

                if success:

                    # Recalculate the next cell from
                    # the newly generated path.

                    next_position = (
                        get_next_position()
                    )

                    print(
                        f"[{amr_id}] "
                        f"NEW NEXT CELL: "
                        f"{next_position}"
                    )

                else:

                    print(
                        f"[{amr_id}] "
                        "No alternate route. "
                        "Holding."
                    )

                    broadcast_state(
                        position,
                        "WAITING_OBSTACLE"
                    )

                    time.sleep(1)

                    continue

        # ====================================================
        # ANNOUNCE INTENTION
        # ====================================================

        broadcast_state(
            next_position,
            "INTENT"
        )

        print()
        print(
            f"[{amr_id}] "
            f"Cycle {cycle}"
        )

        print(
            f"[{amr_id}] "
            f"Current : {position}"
        )

        print(
            f"[{amr_id}] "
            f"Next    : {next_position}"
        )

        # Allow peer packets to arrive
        time.sleep(0.35)

        active_peers = (
            get_active_peers()
        )

        should_wait = False

        should_replan = False

        active_peers = get_active_peers()

        should_wait = False
        should_replan = False

        # ====================================================
        # CHECK PEERS
        # ====================================================

        for peer_id, peer in active_peers.items():

            peer_position = peer["position"]
            peer_next = peer["next_position"]
            peer_priority = peer["priority"]

            # ------------------------------------------------
            # 1. HEAD-ON CONFLICT
            # ------------------------------------------------
            #
            # Example:
            #
            # AMR-1: A -> B
            # AMR-2: B -> A
            #
            # The lower-priority AMR must replan.
            # The higher-priority AMR holds.
            # ------------------------------------------------

            if (
                next_position == peer_position
                and
                peer_next == position
            ):

                print(
                    f"[{amr_id}] "
                    f"HEAD-ON CONFLICT with {peer_id}"
                )

                if priority > peer_priority:

                    # This AMR has lower priority.
                    # Replan around the peer.

                    print(
                        f"[{amr_id}] "
                        f"LOWER PRIORITY -> REPLAN"
                    )

                    should_replan = True

                else:

                    # Higher-priority AMR holds.

                    print(
                        f"[{amr_id}] "
                        f"HIGHER PRIORITY -> HOLD"
                    )

                    should_wait = True

                continue

            # ------------------------------------------------
            # 2. SAME DESTINATION
            # ------------------------------------------------

            if (
                next_position == peer_next
                and
                next_position != peer_position
            ):

                print(
                    f"[{amr_id}] "
                    f"SAME-CELL CONFLICT with {peer_id}"
                )

                if priority > peer_priority:

                    print(
                        f"[{amr_id}] "
                        f"LOWER PRIORITY -> REPLAN"
                    )

                    should_replan = True

                else:

                    print(
                        f"[{amr_id}] "
                        f"HIGHER PRIORITY -> HOLD"
                    )

                    should_wait = True

                continue

            # ------------------------------------------------
            # 3. PEER CURRENTLY OCCUPIES MY NEXT CELL
            # ------------------------------------------------

            if next_position == peer_position:

                print(
                    f"[{amr_id}] "
                    f"Next cell {next_position} "
                    f"is occupied by {peer_id}"
                )

                # Replan instead of waiting forever.

                if priority > peer_priority:

                    print(
                        f"[{amr_id}] "
                        f"LOWER PRIORITY -> REPLAN"
                    )

                    should_replan = True

                else:

                    print(
                        f"[{amr_id}] "
                        f"HIGHER PRIORITY -> HOLD"
                    )

                    should_wait = True

        # ====================================================
        # REPLAN
        # ====================================================

        if should_replan:

            success = replan()

            if success:

                print(
                    f"[{amr_id}] "
                    "Replanning complete."
                )

            else:

                print(
                    f"[{amr_id}] "
                    "No alternate path found."
                )

            # Always hold during the replan cycle.

            should_wait = True

        # ====================================================
        # WAIT
        # ====================================================

        if should_wait:

            print(
                f"[{amr_id}] "
                f"WAIT at {position}"
            )

            broadcast_state(
                position,
                "WAITING"
            )

            time.sleep(1)

            continue

        # ====================================================
        # MOVE
        # ====================================================

        with state_lock:

            old_position = position

            position = next_position

            path_index += 1

            move_count += 1

        print(
            f"[{amr_id}] "
            f"MOVE "
            f"{old_position} -> "
            f"{position}"
        )

        broadcast_state(
            position,
            "MOVING"
        )

        time.sleep(1)


# ============================================================
# CALCULATE INITIAL A* PATH
# ============================================================

calculate_initial_path()


# ============================================================
# START RECEIVER
# ============================================================

receiver_thread = threading.Thread(
    target=receive_messages,
    daemon=True
)

receiver_thread.start()


# ============================================================
# RUN
# ============================================================

try:

    run_agent()

except KeyboardInterrupt:

    print()
    print("=" * 70)

    print(
        f"{amr_id} SHUTDOWN"
    )

    print(
        f"Moves   : {move_count}"
    )

    print(
        f"Replans : {replan_count}"
    )

    print(
        f"Final   : {position}"
    )

    print("=" * 70)

    sock.close()
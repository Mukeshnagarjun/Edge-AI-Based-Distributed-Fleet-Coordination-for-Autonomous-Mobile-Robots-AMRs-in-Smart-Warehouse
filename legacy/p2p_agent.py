import socket
import threading
import json
import time
import sys


# ============================================================
# AMR CONFIGURATION
# ============================================================

AMR_CONFIG = {
    "AMR-1": {
        "port": 5001,
        "start": [1, 1],
        "target": [13, 1],
        "priority": 1,
    },

    "AMR-2": {
        "port": 5002,
        "start": [7, 1],
        "target": [2, 8],
        "priority": 2,
    },

    "AMR-3": {
        "port": 5003,
        "start": [13, 8],
        "target": [1, 8],
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
# DEMONSTRATION PATHS
#
# AMR-1 and AMR-2 intentionally share the same aisle.
# This creates a controlled conflict so that we can demonstrate
# distributed conflict resolution.
# ============================================================

DEMO_PATHS = {

    "AMR-1": [
        [1, 1],
        [2, 1],
        [3, 1],
        [4, 1],
        [5, 1],
        [6, 1],
        [7, 1],
        [8, 1],
        [9, 1],
        [10, 1],
        [11, 1],
        [12, 1],
        [13, 1],
    ],

    "AMR-2": [
        [7, 1],
        [6, 1],
        [5, 1],
        [4, 1],
        [3, 1],
        [2, 1],
        [2, 2],
        [2, 3],
        [2, 4],
        [2, 5],
        [2, 6],
        [2, 7],
        [2, 8],
    ],

    "AMR-3": [
        [13, 8],
        [12, 8],
        [11, 8],
        [10, 8],
        [9, 8],
        [8, 8],
        [7, 8],
        [6, 8],
        [5, 8],
        [4, 8],
        [3, 8],
        [3, 9],
        [2, 9],
        [1, 9],
        [1, 8],
    ],
}


# ============================================================
# READ AMR ID
# ============================================================

if len(sys.argv) != 2:

    print("Usage:")
    print("python3 p2p_agent.py AMR-1")
    print("python3 p2p_agent.py AMR-2")
    print("python3 p2p_agent.py AMR-3")

    sys.exit(1)


amr_id = sys.argv[1]

if amr_id not in AMR_CONFIG:

    print("Invalid AMR ID.")

    sys.exit(1)


config = AMR_CONFIG[amr_id]


# ============================================================
# LOCAL STATE
# ============================================================

position = config["start"].copy()

target = config["target"].copy()

priority = config["priority"]

path = DEMO_PATHS[amr_id]

path_index = 0

cycle = 0

known_amrs = {}

state_lock = threading.Lock()


# ============================================================
# UDP SOCKET
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
    ("127.0.0.1", config["port"])
)

sock.settimeout(0.1)


# ============================================================
# RECEIVE PEER INFORMATION
# ============================================================

def receive_messages():

    while True:

        try:

            data, address = sock.recvfrom(4096)

            message = json.loads(
                data.decode()
            )

            sender = message["amr_id"]

            if sender != amr_id:

                with state_lock:

                    known_amrs[sender] = {
                        "position": message["position"],
                        "next_position": message["next_position"],
                        "status": message["status"],
                        "priority": message["priority"],
                        "cycle": message["cycle"],
                        "timestamp": time.time(),
                    }

        except socket.timeout:

            continue

        except Exception as e:

            print(
                f"[{amr_id}] RECEIVE ERROR: {e}"
            )


# ============================================================
# CALCULATE NEXT CELL
# ============================================================

def get_next_position():

    with state_lock:

        if path_index >= len(path) - 1:

            return position.copy()

        return path[path_index + 1].copy()


# ============================================================
# BROADCAST CURRENT STATE + INTENT
# ============================================================

def broadcast_state(next_position, status):

    message = {

        "amr_id": amr_id,

        "position": position.copy(),

        "next_position": next_position.copy(),

        "target": target.copy(),

        "status": status,

        "priority": priority,

        "cycle": cycle,

        "timestamp": time.time(),
    }

    data = json.dumps(message).encode()

    for peer_port in PEERS[amr_id]:

        try:

            sock.sendto(
                data,
                ("127.0.0.1", peer_port)
            )

        except Exception as e:

            print(
                f"[{amr_id}] SEND ERROR: {e}"
            )


# ============================================================
# CHECK FOR CONFLICT
# ============================================================

def find_conflict(my_next):

    conflicts = []

    with state_lock:

        for peer_id, peer in known_amrs.items():

            # Ignore very old information
            if time.time() - peer["timestamp"] > 2.0:

                continue

            peer_position = peer["position"]

            peer_next = peer["next_position"]

            # ------------------------------------------------
            # CONFLICT TYPE 1:
            # Both want the same cell
            # ------------------------------------------------

            same_destination = (
                my_next == peer_next
            )

            # ------------------------------------------------
            # CONFLICT TYPE 2:
            # Head-on swap
            # ------------------------------------------------

            head_on_swap = (
                my_next == peer_position
                and
                peer_next == position
            )

            # ------------------------------------------------
            # CONFLICT TYPE 3:
            # I want the cell occupied by the peer
            # ------------------------------------------------

            entering_peer_cell = (
                my_next == peer_position
            )

            if (
                same_destination
                or head_on_swap
                or entering_peer_cell
            ):

                conflicts.append(
                    (peer_id, peer)
                )

    return conflicts


# ============================================================
# WAIT FOR PEER INTENTIONS
# ============================================================

def wait_for_peer_updates():

    time.sleep(0.30)


# ============================================================
# MAIN CONTROL LOOP
# ============================================================

def run_agent():

    global position
    global path_index
    global cycle

    print()

    print("=" * 65)
    print(f"{amr_id} DISTRIBUTED AMR AGENT")
    print("=" * 65)

    print(
        f"Start     : {config['start']}"
    )

    print(
        f"Target    : {target}"
    )

    print(
        f"Priority  : {priority}"
    )

    print(
        f"UDP Port  : {config['port']}"
    )

    print("=" * 65)

    arrived_reported = False

    while True:

        cycle += 1

        # ====================================================
        # CHECK ARRIVAL
        # ====================================================

        if position == target:

            if not arrived_reported:

                print()
                print(
                    f"[{amr_id}] ARRIVED at {position}"
                )

                broadcast_state(
                    position.copy(),
                    "ARRIVED"
                )

                arrived_reported = True

            # Continue sending occasional heartbeat
            time.sleep(2)

            broadcast_state(
                position.copy(),
                "ARRIVED"
            )

            continue

        arrived_reported = False

        # ====================================================
        # DETERMINE INTENDED NEXT CELL
        # ====================================================

        next_position = get_next_position()

        # ====================================================
        # BROADCAST INTENTION
        # ====================================================

        broadcast_state(
            next_position,
            "INTENT"
        )

        print()
        print(
            f"[{amr_id}] CYCLE {cycle}"
        )

        print(
            f"[{amr_id}] Current : {position}"
        )

        print(
            f"[{amr_id}] Intent  : {next_position}"
        )

        # ====================================================
        # GIVE PEERS TIME TO RECEIVE INTENTIONS
        # ====================================================

        wait_for_peer_updates()

        # ====================================================
        # FIND CONFLICTS
        # ====================================================

        conflicts = find_conflict(
            next_position
        )

        # ====================================================
        # LOCAL DECISION
        # ====================================================

        should_wait = False

        for peer_id, peer in conflicts:

            peer_priority = peer["priority"]

            if priority > peer_priority:

                # Peer has higher priority
                should_wait = True

                print(
                    f"[{amr_id}] CONFLICT with "
                    f"{peer_id} -> WAIT"
                )

                print(
                    f"[{amr_id}] "
                    f"Priority {priority} < "
                    f"{peer_id} priority {peer_priority}"
                )

                break

        # ====================================================
        # WAIT
        # ====================================================

        if should_wait:

            print(
                f"[{amr_id}] HOLDING position "
                f"{position}"
            )

            broadcast_state(
                position.copy(),
                "WAITING"
            )

            time.sleep(1)

            continue

        # ====================================================
        # MOVE
        # ====================================================

        with state_lock:

            old_position = position.copy()

            position = next_position.copy()

            path_index += 1

        print(
            f"[{amr_id}] MOVE "
            f"{old_position} -> {position}"
        )

        # ====================================================
        # BROADCAST NEW POSITION
        # ====================================================

        broadcast_state(
            position.copy(),
            "MOVING"
        )

        time.sleep(1)


# ============================================================
# START RECEIVER THREAD
# ============================================================

receiver_thread = threading.Thread(
    target=receive_messages,
    daemon=True
)

receiver_thread.start()


# ============================================================
# START AGENT
# ============================================================

try:

    run_agent()

except KeyboardInterrupt:

    print()
    print(
        f"[{amr_id}] SHUTDOWN"
    )

    sock.close()
# ============================================================
# AMR FLEET COORDINATION
# Edge-AI Based Distributed Fleet Coordination for AMRs
#
# Current Prototype:
# - A* Pathfinding
# - Priority-Based Path Reservation
# - Simultaneous Multi-AMR Movement
# - Dynamic Reservations
# - Dynamic Obstacle Detection
# - Dynamic Replanning
# - Collision Detection
# - Performance Benchmark
# - Warehouse Grid Visualization
# ============================================================

import time

from scenario import (
    WIDTH,
    HEIGHT,
    OBSTACLES,
    AMR_CONFIG,
)


# ============================================================
# 1. WAREHOUSE CONFIGURATION
# ============================================================

warehouse = [
    [0 for _ in range(WIDTH)]
    for _ in range(HEIGHT)
]


# Physical warehouse obstacles / racks
physical_obstacles = OBSTACLES


for x, y in physical_obstacles:
    warehouse[y][x] = 1


# ============================================================
# 2. AMR CONFIGURATION
# ============================================================

robots = {
    robot_id: robot["start"]
    for robot_id, robot in AMR_CONFIG.items()
}


targets = {
    "AMR-1": (13, 1),
    "AMR-2": (2, 8),
    "AMR-3": (1, 8)
}


# Priority order (lower number plans first)
priority_order = sorted(
    AMR_CONFIG,
    key=lambda robot_id: AMR_CONFIG[robot_id]["priority"]
)


# ============================================================
# 3. A* PATHFINDING
# ============================================================

def heuristic(a, b):

    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def get_neighbors(position):

    x, y = position

    moves = [
        (x + 1, y),
        (x - 1, y),
        (x, y + 1),
        (x, y - 1)
    ]

    valid = []

    for nx, ny in moves:

        # Warehouse boundary
        if nx < 0 or nx >= WIDTH:
            continue

        if ny < 0 or ny >= HEIGHT:
            continue

        # Physical / dynamic obstacle
        if warehouse[ny][nx] == 1:
            continue

        valid.append((nx, ny))

    return valid


def a_star(start, goal, reserved_cells):

    open_set = [start]

    came_from = {}

    g_score = {
        start: 0
    }

    f_score = {
        start: heuristic(start, goal)
    }

    while open_set:

        current = min(
            open_set,
            key=lambda p: f_score.get(
                p,
                float("inf")
            )
        )

        # Goal reached
        if current == goal:

            path = []

            while current in came_from:

                path.append(current)

                current = came_from[current]

            path.append(start)

            path.reverse()

            return path

        open_set.remove(current)

        for neighbor in get_neighbors(current):

            # Avoid reserved cells
            if neighbor in reserved_cells:
                continue

            new_cost = (
                g_score[current] + 1
            )

            if new_cost < g_score.get(
                neighbor,
                float("inf")
            ):

                came_from[neighbor] = current

                g_score[neighbor] = new_cost

                f_score[neighbor] = (
                    new_cost
                    +
                    heuristic(
                        neighbor,
                        goal
                    )
                )

                if neighbor not in open_set:

                    open_set.append(neighbor)

    return None


# ============================================================
# 4. PRIORITY-BASED PATH PLANNING
# ============================================================

planned_paths = {}

reserved_cells = set()


for robot in priority_order:

    start = robots[robot]

    goal = targets[robot]

    print()
    print("=" * 50)
    print(f"Planning path for {robot}")
    print(f"Start : {start}")
    print(f"Target: {goal}")
    print("=" * 50)

    path = a_star(
        start,
        goal,
        reserved_cells
    )

    if path is None:

        print("NO PATH AVAILABLE")

        print(
            "This robot is blocked "
            "by existing reservations."
        )

        break

    planned_paths[robot] = path

    # Reserve complete path
    for cell in path:

        reserved_cells.add(cell)

    print(
        f"Path found: "
        f"{len(path) - 1} steps"
    )

    print("Path:")
    print(path)


# ============================================================
# 5. SAVE ORIGINAL PATHS
# ============================================================
#
# These paths are preserved before simulation.
# Dynamic replanning will not modify the baseline.
# ============================================================

original_paths = {}

for robot, path in planned_paths.items():

    original_paths[robot] = path.copy()


# ============================================================
# 6. DISPLAY FINAL RESERVED PATHS
# ============================================================

print()
print("=" * 50)
print("FINAL RESERVED PATHS")
print("=" * 50)

for robot, path in planned_paths.items():

    print(
        f"{robot}: "
        f"{len(path) - 1} steps"
    )


print()

print(
    "Reserved cells:",
    len(reserved_cells)
)


# ============================================================
# 7. DISPLAY INITIAL WAREHOUSE GRID
# ============================================================

def display_paths():

    for y in range(HEIGHT):

        row = ""

        for x in range(WIDTH):

            position = (x, y)

            # Physical obstacle
            if warehouse[y][x] == 1:

                row += "██"
                continue

            # Starting AMR position
            robot_here = None

            for robot, start in robots.items():

                if start == position:

                    robot_here = robot
                    break

            if robot_here:

                row += robot_here[-1] + " "
                continue

            # Target
            target_here = None

            for robot, target in targets.items():

                if target == position:

                    target_here = robot
                    break

            if target_here:

                row += "X "
                continue

            # Reserved path
            path_robot = None

            for robot, path in planned_paths.items():

                if position in path:

                    path_robot = robot
                    break

            if path_robot:

                row += "* "

            else:

                row += "· "

        print(row)


print()
print("=" * 50)
print("INITIAL WAREHOUSE GRID")
print("=" * 50)
print()

display_paths()


# ============================================================
# 8. SIMULATION STATE
# ============================================================

current_positions = {}

path_indices = {}


for robot, path in planned_paths.items():

    current_positions[robot] = path[0]

    path_indices[robot] = 0


# ============================================================
# 9. DISPLAY CURRENT SIMULATION GRID
# ============================================================

def display_simulation():

    for y in range(HEIGHT):

        row = ""

        for x in range(WIDTH):

            position = (x, y)

            # Physical / dynamic obstacle
            if warehouse[y][x] == 1:

                row += "██"
                continue

            # AMR position
            robot_here = None

            for robot, current in current_positions.items():

                if current == position:

                    robot_here = robot
                    break

            if robot_here:

                row += robot_here[-1] + " "
                continue

            # Target
            target_here = None

            for robot, target in targets.items():

                if target == position:

                    target_here = robot
                    break

            if target_here:

                row += "X "
                continue

            # Free cell
            row += "· "

        print(row)


# ============================================================
# 10. DYNAMIC RESERVATION SYSTEM
# ============================================================

def get_dynamic_reservations():

    reservations = set()

    for robot, path in planned_paths.items():

        current_index = path_indices[robot]

        # Current position
        reservations.add(
            path[current_index]
        )

        # Future positions
        for cell in path[
            current_index + 1:
        ]:

            reservations.add(cell)

    return reservations


# ============================================================
# 11. DYNAMIC REPLANNING
# ============================================================

def replan_robot(robot):

    current = current_positions[robot]

    goal = targets[robot]

    # Protect other AMRs
    other_reservations = set()

    for other_robot, path in planned_paths.items():

        if other_robot == robot:
            continue

        index = path_indices[other_robot]

        # Reserve current + future cells
        for cell in path[index:]:

            other_reservations.add(cell)

    new_path = a_star(
        current,
        goal,
        other_reservations
    )

    return new_path


# ============================================================
# 12. START SIMULATION
# ============================================================

print()
print("=" * 50)
print("STARTING SIMULATION")
print("=" * 50)


simulation_step = 0

collision_count = 0

replan_count = 0

simulation_failed = False


# ============================================================
# 13. SIMULATION LOOP
# ============================================================

while True:

    simulation_step += 1

    print()
    print("=" * 50)
    print(
        f"SIMULATION STEP "
        f"{simulation_step}"
    )
    print("=" * 50)


    # ========================================================
    # DYNAMIC AISLE BLOCKAGE
    # ========================================================

    if simulation_step == 5:

        blocked_cell = (9, 1)

        warehouse[
            blocked_cell[1]
        ][
            blocked_cell[0]
        ] = 1

        print()
        print("🚧 DYNAMIC OBSTACLE DETECTED")

        print(
            f"Warehouse blockage at "
            f"{blocked_cell}"
        )


    # ========================================================
    # DYNAMIC RESERVATIONS
    # ========================================================

    dynamic_reservations = (
        get_dynamic_reservations()
    )

    print(
        f"Dynamic reservations: "
        f"{len(dynamic_reservations)} cells"
    )


    # ========================================================
    # CHECK IF ALL ROBOTS FINISHED
    # ========================================================

    all_finished = True

    for robot in planned_paths:

        path = planned_paths[robot]

        index = path_indices[robot]

        if index < len(path) - 1:

            all_finished = False
            break


    if all_finished:

        print()
        print(
            "All AMRs reached "
            "their destinations."
        )

        break


    # ========================================================
    # CALCULATE PROPOSED MOVEMENTS
    # ========================================================

    proposed_positions = {}


    for robot in planned_paths:

        path = planned_paths[robot]

        index = path_indices[robot]

        current = path[index]


        # ----------------------------------------------------
        # Already at destination
        # ----------------------------------------------------

        if index >= len(path) - 1:

            proposed_positions[
                robot
            ] = current

            continue


        next_position = path[index + 1]


        # ====================================================
        # DYNAMIC OBSTACLE CHECK
        # ====================================================

        if (
            warehouse[
                next_position[1]
            ][
                next_position[0]
            ] == 1
        ):

            print()
            print("🔄 REPLANNING REQUIRED")

            print(
                f"{robot} detected "
                f"blockage at "
                f"{next_position}"
            )


            new_path = replan_robot(robot)


            # ------------------------------------------------
            # Replanning failed
            # ------------------------------------------------

            if new_path is None:

                print(
                    f"❌ No alternate path "
                    f"available for {robot}"
                )

                proposed_positions[
                    robot
                ] = current

                continue


            # ------------------------------------------------
            # Replanning successful
            # ------------------------------------------------

            print(
                f"✅ New path found "
                f"for {robot}"
            )

            print(
                f"New path length: "
                f"{len(new_path) - 1} steps"
            )

            print("New path:")

            print(new_path)


            # Count replanning event
            replan_count += 1


            # Replace old path
            planned_paths[
                robot
            ] = new_path


            # Reset index
            path_indices[
                robot
            ] = 0


            # Update current position
            current_positions[
                robot
            ] = new_path[0]


            # Move to next cell
            if len(new_path) > 1:

                proposed_positions[
                    robot
                ] = new_path[1]

            else:

                proposed_positions[
                    robot
                ] = new_path[0]


        else:

            proposed_positions[
                robot
            ] = next_position


    # ========================================================
    # COLLISION DETECTION
    # ========================================================

    collision = False

    robot_names = list(
        proposed_positions.keys()
    )


    for i in range(
        len(robot_names)
    ):

        for j in range(
            i + 1,
            len(robot_names)
        ):

            robot_a = robot_names[i]

            robot_b = robot_names[j]


            current_a = (
                current_positions[
                    robot_a
                ]
            )

            current_b = (
                current_positions[
                    robot_b
                ]
            )


            next_a = (
                proposed_positions[
                    robot_a
                ]
            )

            next_b = (
                proposed_positions[
                    robot_b
                ]
            )


            # ------------------------------------------------
            # Same destination
            # ------------------------------------------------

            if next_a == next_b:

                print()
                print(
                    "❌ COLLISION DETECTED"
                )

                print(
                    f"{robot_a} and "
                    f"{robot_b} both attempted "
                    f"to enter {next_a}"
                )

                collision = True


            # ------------------------------------------------
            # Head-on swap
            # ------------------------------------------------

            if (
                next_a == current_b
                and
                next_b == current_a
            ):

                print()
                print(
                    "❌ HEAD-ON COLLISION DETECTED"
                )

                print(
                    f"{robot_a} and "
                    f"{robot_b} attempted "
                    f"to swap positions"
                )

                collision = True


    # ========================================================
    # STOP IF COLLISION OCCURS
    # ========================================================

    if collision:

        collision_count += 1

        print()
        print(
            "Simulation stopped "
            "because of collision."
        )

        simulation_failed = True

        break


    # ========================================================
    # APPLY MOVEMENTS
    # ========================================================

    for robot in planned_paths:

        path = planned_paths[robot]

        index = path_indices[robot]


        if index < len(path) - 1:

            path_indices[
                robot
            ] += 1


            current_positions[
                robot
            ] = path[
                path_indices[robot]
            ]


    # ========================================================
    # DISPLAY CURRENT POSITIONS
    # ========================================================

    print()

    for robot in current_positions:

        print(
            f"{robot}: "
            f"{current_positions[robot]}"
        )


    # ========================================================
    # DISPLAY WAREHOUSE GRID
    # ========================================================

    print()

    display_simulation()


    # ========================================================
    # SMALL VISUALIZATION DELAY
    # ========================================================

    time.sleep(0.3)


# ============================================================
# 14. SIMULATION COMPLETE
# ============================================================

print()
print("=" * 50)
print("SIMULATION COMPLETE")
print("=" * 50)


# ============================================================
# 15. SEQUENTIAL BASELINE
# ============================================================
#
# IMPORTANT:
# Original paths are used.
# Dynamic replanning does NOT modify this benchmark.
# ============================================================

print()
print("=" * 50)
print("SEQUENTIAL BASELINE")
print("=" * 50)


baseline_time = 0


for robot in priority_order:

    path = original_paths[robot]

    steps = len(path) - 1

    print()

    print(
        f"{robot} executing..."
    )

    print(
        f"Steps required: "
        f"{steps}"
    )

    baseline_time += steps


print()
print("=" * 50)
print("BASELINE RESULT")
print("=" * 50)

print(
    f"Total completion time: "
    f"{baseline_time} steps"
)


# ============================================================
# 16. PERFORMANCE COMPARISON
# ============================================================

coordinated_time = simulation_step


if baseline_time > 0:

    improvement = (
        (
            baseline_time -
            coordinated_time
        )
        /
        baseline_time
    ) * 100

else:

    improvement = 0


print()
print("=" * 50)
print("PERFORMANCE COMPARISON")
print("=" * 50)


print(
    f"Sequential baseline : "
    f"{baseline_time} steps"
)


print(
    f"Coordinated system  : "
    f"{coordinated_time} steps"
)


print(
    f"Makespan reduction  : "
    f"{baseline_time - coordinated_time} steps"
)


print(
    f"Improvement         : "
    f"{improvement:.2f}%"
)


print(
    f"Collisions          : "
    f"{collision_count}"
)


print(
    f"Dynamic replans     : "
    f"{replan_count}"
)


# ============================================================
# 17. COMPLETION CHECK
# ============================================================

completed = 0


for robot in planned_paths:

    if (
        current_positions[robot]
        ==
        targets[robot]
    ):

        completed += 1


print(
    f"AMRs completed      : "
    f"{completed}/{len(robots)}"
)


# ============================================================
# 18. FINAL SYSTEM STATUS
# ============================================================

print()
print("=" * 50)
print("FINAL SYSTEM STATUS")
print("=" * 50)


if (
    collision_count == 0
    and
    completed == len(robots)
):

    print(
        "✅ Fleet coordination "
        "completed successfully."
    )

else:

    print(
        "⚠️ Fleet coordination "
        "completed with issues."
    )
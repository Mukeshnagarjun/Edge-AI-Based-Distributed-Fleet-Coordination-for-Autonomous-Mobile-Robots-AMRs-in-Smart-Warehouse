# FIELDLINE

### Edge-AI Based Distributed Fleet Coordination for Autonomous Mobile Robots (AMRs) in Smart Warehouses

**FIELDLINE** is a simulation-based prototype exploring how multiple Autonomous Mobile Robots (AMRs) coordinate tasks, navigate shared warehouse spaces, and manage movement conflicts through distributed decision-making and Edge AI.

The goal is to reduce dependence on a central controller for individual movement decisions while improving fleet coordination, collision safety, and operational awareness.

<p align="center">
  <img src="docs/dashboard.png" alt="FIELDLINE warehouse fleet coordination dashboard" width="100%">
</p>

<p align="center">
  <strong>Distributed Coordination · Edge-AI Risk Prediction · Collision Avoidance · Deadlock Detection</strong>
</p>

---

## Overview

Smart warehouses use multiple mobile robots to transport materials through shared aisles, pickup points, and delivery locations. As the number of robots increases, coordinating their movements becomes more challenging.

Robots may compete for the same cells, block each other's routes, or wait on one another. A centralized system can also become a bottleneck when every movement decision depends on one controller.

FIELDLINE explores a distributed approach in which robots exchange state information, coordinate task assignments, resolve movement conflicts, and use local risk predictions to support safer decisions.

**Scope:** This is a software simulation prototype, not a deployment-ready physical robot fleet.

## Key Features

- **Distributed fleet coordination:** Peer-state exchange and coordinated movement decisions.
- **Task allocation through bidding:** Robots evaluate task costs, including travel distance, to compete for tasks.
- **Collision avoidance:** Checks for occupied destinations and conflicting movements.
- **Dynamic obstacle awareness:** Considers robot positions and planned movements when identifying route constraints.
- **Blocked-route replanning:** Attempts to replan when a robot remains blocked.
- **Edge-AI risk prediction:** A local machine-learning model estimates movement risk.
- **Advisory deadlock detection:** Detects wait cycles that may indicate a deadlock.
- **Live simulation dashboard:** Displays robot positions, obstacles, tasks, fleet status, routes, and risk assessments.

## System Architecture

```text
                WAREHOUSE SIMULATION
                         |
          +--------------+--------------+
          |              |              |
        AMR 1           AMR 2           AMR 3
          |              |              |
      Local logic    Local logic    Local logic
      Risk model     Risk model     Risk model
          |              |              |
          +------ Peer-to-peer ---------+
                  state exchange
                         |
           +-------------+-------------+
           |             |             |
       Task bidding   Conflict      Wait-cycle
                      resolution    detection
           |             |             |
           +-------------+-------------+
                         |
               Movement decisions
                         |
               Warehouse dashboard
```

The architecture illustrates the prototype's conceptual workflow. Deadlock detection is advisory; automatic deadlock recovery is not yet implemented.

## How It Works

1. **Task generation:** Pickup and delivery tasks become available to the simulated fleet.
2. **Distributed bidding:** Robots evaluate task costs and participate in task allocation.
3. **Peer communication:** Robots exchange information about positions, intended movements, targets, and statuses.
4. **Movement coordination:** The controller checks occupied cells and movement conflicts.
5. **Risk assessment:** The local Edge-AI model estimates movement risk and provides an advisory signal.
6. **Blocked-route handling:** Robots can attempt to replan when movement remains blocked.
7. **Deadlock monitoring:** Wait-cycle detection identifies potential circular waiting conditions.

## Technology Stack

| Component | Technology |
|---|---|
| Programming language | Python |
| Machine learning | Scikit-learn, Random Forest |
| Communication | UDP-based telemetry and peer messaging |
| Simulation | Grid-based warehouse environment |
| Visualization | Python-based warehouse dashboard |
| Testing | Python `unittest` |
| Development environment | VS Code |

Physical AMR hardware integration is outside the demonstrated scope.

## Edge-AI Risk Prediction

FIELDLINE includes a compact Random Forest model that estimates movement risk using prototype-generated data.

The reported test accuracy is **91.50% on synthetic, rule-generated labels**. This is an experimental prototype result, not evidence of equivalent accuracy in a real warehouse. Performance under real operating conditions has not yet been validated.

## Deadlock Detection

A deadlock can occur when robots wait on one another in a circular dependency.

FIELDLINE includes advisory wait-cycle detection to identify potential deadlock situations. Automated deadlock recovery remains future work.

## Project Structure

```text
.
├── main.py
├── run_demo.py
├── fleet_controller.py
├── deadlock_detector.py
├── edge_ai_risk.py
├── edge_ai_integration.py
├── scenario.py
├── simulation.py
├── server.py
├── requirements.txt
├── tests/
├── docs/
└── README.md
```

## Getting Started

### Prerequisites

- Python 3
- Git
- A terminal and a code editor such as VS Code

### 1. Clone the repository

```bash
git clone https://github.com/Mukeshnagarjun/Edge-AI-Based-Distributed-Fleet-Coordination-for-Autonomous-Mobile-Robots-AMRs-in-Smart-Warehouse.git
```

```bash
cd Edge-AI-Based-Distributed-Fleet-Coordination-for-Autonomous-Mobile-Robots-AMRs-in-Smart-Warehouse
```

### 2. Create a virtual environment

```bash
python3 -m venv .venv
source .venv/bin/activate
```

### 3. Install dependencies

```bash
python3 -m pip install -r requirements.txt
```

### 4. Run the prototype

```bash
python3 run_demo.py
```

If the demo requires a particular startup sequence or separate visualization process, follow the instructions in the relevant project scripts.

### 5. Run the tests

```bash
python3 -m unittest discover -s tests -v
```

The recorded development milestone passed **49 tests**. Run the test suite on the version you submit to verify its current status.

## Demonstration Scenarios

The simulation can be used to demonstrate:

- Multiple AMRs operating in a shared warehouse grid.
- Task bidding and allocation.
- Movement conflict resolution and collision avoidance.
- Blocked-route detection and replanning.
- Edge-AI risk assessments.
- Advisory wait-cycle deadlock detection.

## Current Limitations

- The prototype has not been validated on a physical warehouse fleet.
- Risk-prediction results use synthetic data.
- Deadlock detection is implemented, but automatic recovery is incomplete.
- Scalability, network failures, and physical robot constraints require further evaluation.

## Future Work

- Develop and evaluate automatic deadlock recovery.
- Validate the risk model using realistic or measured data.
- Test coordination with larger robot fleets.
- Evaluate communication delays and peer failures.
- Integrate with physical AMRs and warehouse navigation systems.
- Measure task completion time, congestion, safety events, and energy efficiency against suitable baselines.

## Project Details

- **Project:** FIELDLINE
- **Domain:** Robotics and Automation
- **Subdomains:** Edge AI, Distributed Multi-Agent Systems, Autonomous Mobile Robots
- **Application:** Smart Warehousing and Logistics

## License

No license has been specified yet. Add a `LICENSE` file before describing the project as open-source under a particular license.

---

*FIELDLINE is a research and prototype project exploring decentralized coordination and Edge-AI-assisted decision-making for autonomous warehouse fleets.*

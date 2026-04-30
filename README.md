# Autonomous Hospital Sanitizing Robot
### Autonomous Mobile Robot · TurtleBot3 Burger · Webots Simulation
#### Junior Project Design

[![Python](https://img.shields.io/badge/Python-3.14%2B-blue?logo=python&logoColor=white)](https://www.python.org/)
[![Webots](https://img.shields.io/badge/Webots-R2025a%2B-brightgreen)](https://cyberbotics.com/)
[![Platform](https://img.shields.io/badge/Platform-TurtleBot3%20Burger-orange)](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

---

A Webots simulation of a TurtleBot3 Burger that autonomously explores, maps, and sanitizes all reachable floor area in a multi-room hospital environment.

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Architecture](#architecture)
- [State Machine](#state-machine)
- [Sensor Suite](#sensor-suite)
- [Project Structure](#project-structure)
- [Getting Started](#getting-started)
- [Configuration](#configuration)
- [How It Works](#how-it-works)
- [Challenges & Limitations](#challenges--limitations)

---

## Overview

This project simulates a **hospital floor sanitization robot** built on the **TurtleBot3 Burger** platform inside the [Webots](https://cyberbotics.com/) robotics simulator.
<img width="521" height="343" alt="image" src="https://github.com/user-attachments/assets/a7c5faf0-303e-4946-aa4c-73d2de4a083f" />

> Fig 1: 3D model of the TurtleBot3 Burger

Rather than following a pre-programmed path, the robot autonomously:

1. **Explores** the hospital using frontier-based navigation to build a live map
2. **Plans** a full-coverage cleaning route using a boustrophedon (lawnmower) sweep
3. **Executes** the sweep, tracking every sanitized cell on an occupancy grid
4. **Returns** to its starting position when coverage is complete

The environment is a four-room hospital arena containing furniture, obstacles, and narrow corridors — a realistic testbed for autonomous coverage tasks.
<img width="520" height="343" alt="image" src="https://github.com/user-attachments/assets/e211077b-5089-4e6e-b645-ba416a0f2db7" /> 

> Fig 2: General layout of the environment

---

## Features

- **Frontier-based exploration** — autonomously discovers unknown space before sweeping
- **Live occupancy grid mapping** — 500×500 cell map (50m × 50m at 10 cm/cell) built in real-time from LiDAR returns
- **Boustrophedon coverage planning** — systematic lawnmower path computed over all mapped free cells
- **A\* path planning** — navigates between waypoints with robot-footprint-aware clearance checking
- **Sensor fusion odometry** — wheel encoders + gyroscope + compass, with optional supervisor ground-truth sync
- **Camera semantic perception** — detects and avoids people; optionally marks furniture on the map
- **Stuck recovery** — automatic reverse-and-turn recovery when progress stalls
- **Fully configurable** — all parameters centralized in a single `Config` class

---

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                        RobotBrain                       │
│                                                         │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌────────┐   │
│  │ EXPLORE  │→ │  PLAN    │→ │  SWEEP   │→ │ RETURN │   │
│  └──────────┘  └──────────┘  └──────────┘  └────────┘   │
│        ↑            ↑             ↑              ↑      │
│  ┌─────────────────────────────────────────────────┐    │
│  │              Supporting Subsystems              │    │
│  │  OccupancyGrid │ Odometry │ LidarInterface      │    │
│  │  CameraProcessor │ A* Planner │ Recovery FSM    │    │
│  └─────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────┘
```

| Class | Responsibility |
|---|---|
| `Config` | All tunable constants (speeds, thresholds, grid params) |
| `OccupancyGrid` | Live map: ray-casting, frontier detection, A\* pathfinding, coverage planning |
| `Odometry` | Pose estimation via encoder + gyro + compass fusion |
| `LidarInterface` | VLP-16 sector reading and map update via Bresenham ray-casting |
| `CameraProcessor` | Object recognition, person detection, dirty-floor detection |
| `RobotBrain` | Master FSM controller, navigation, recovery, main loop |

---

## State Machine

The robot operates through four sequential phases:

```
            ┌─────────────────────────────────────────────────┐
            │                                                 │
   START ──►│  EXPLORE  ──►  PLAN  ──►  SWEEP  ──►  RETURN    │──► DONE
            │                                                 │
            └─────────────────────────────────────────────────┘
```

### `EXPLORE`
The robot wanders using **frontier-based navigation** — it continuously finds the boundary between known and unknown grid cells, then navigates toward the nearest unvisited frontier. A reactive wander mode kicks in when no frontier is directly reachable. Throughout this phase the robot logs a breadcrumb trace of every visited cell.

Exploration ends when map growth converges (few frontiers remain, idle for long enough) or the step budget is exhausted.

### `PLAN`
A one-shot planning phase that generates the full cleaning route. It computes two candidate plans and picks the better one:
- **Grid-based**: a boustrophedon sweep over all reachable free cells, alternating direction each row
- **Trace-based**: replay of the exploration breadcrumbs (used as a fallback when grid coverage is sparse)

### `SWEEP`
The robot follows the waypoint list from `PLAN` using A\* path planning for each segment. After reaching each waypoint it marks a circular footprint of cells as `SWEPT` on the grid. Unreachable waypoints are skipped with a configurable skip limit.

### `RETURN`
The robot navigates back to its home position and aligns to its original heading, then transitions to `DONE`.

---

## Sensor Suite

### Velodyne VLP-16 LiDAR
- 16 vertical layers, 360° horizontal scan
- Layers 5, 7, 9 used for ground-level map updates
- Layers 2–13 collapsed into 5 directional sectors for real-time obstacle avoidance (`front`, `front_left`, `front_right`, `left`, `right`)
- Bresenham ray-casting updates the occupancy grid every 3 simulation steps
<img width="521" height="343" alt="image" src="https://github.com/user-attachments/assets/65e7b9ff-4484-46b7-b3f7-be44d1a904b9" />

> Fig 3: LiDAR Ray Path

### Camera
- Webots object recognition API classifies objects as `person`, `wall`, or `furniture`
- Persons detected within 0.85 m trigger a brief halt
- Optional map blocking for detected furniture positions (`SEMANTIC_BLOCK_MAP`)
- Optional dirty-floor detection via pixel luminance analysis

### Wheel Encoders + Gyroscope + Compass
- Fused pose estimate: 70% gyro weight, 30% encoder weight, small compass correction
- Supervisor ground-truth sync available to eliminate drift entirely (`USE_SUPERVISOR_POSE_SYNC = True`)

---

## Project Structure

```
Amr-Sim/
│
├── controllers/
│   └── burger_code/
│       └── burger_code.py      # Main robot controller (all logic)
│
├── protos/
│   ├── TurtleBot3Burger.proto  # Robot PROTO definition
│   └── VelodyneVLP-16.proto    # LiDAR sensor PROTO
│
├── worlds/
│   └── hospital.wbt            # Webots world: 4-room hospital arena
│
└── README.md
```

---

## Getting Started

### Prerequisites

- [Webots R2025a](https://cyberbotics.com/#download) or later
- Python 3.14+
- No external Python packages required (uses Webots' built-in `controller` library)

### Running the Simulation

1. **Clone the repository**
   ```bash
   git clone https://github.com/fuadhsn1738/Amr-Sim.git
   cd Amr-Sim
   ```

2. **Open Webots** and load the world file:
   ```
   File → Open World → worlds/hospital.wbt
   ```

3. **Verify the controller** — the TurtleBot3 Burger node should already have `burger_code` set as its controller. If not, select the robot node and set the controller field to `burger_code`.

4. **Run the simulation** using the play button (▶). Watch the console for live state and coverage logs.

### Console Output

The controller prints structured logs every 100 steps:

```
[ 12.5s]  State:EXPLORE   Pos:(+0.43,-0.12)  θ: +92.3deg  F:1.24  L:0.87  R:inf  Fronts: 14  Swept:   0
  [FRONTIER] targeting frontier at (1.20, 0.80) size:18
  [EXPLORE] map gain: 142  known: 830  frontiers:  11  idle: 0  failed_targets: 0
  [PLAN] computing coverage waypoints ...
  [PLAN] using grid-based coverage: 214 waypoints
  [SWEEP]  50/214  23%
  [RETURN] reached origin → DONE
```

---

## Configuration

All parameters live in the `Config` class at the top of `burger_code.py`. Key values:

| Parameter | Default | Description |
|---|---|---|
| `CELL_M` | `0.10` | Grid cell size in metres |
| `GRID_N` | `500` | Grid dimension (500×500 cells = 50m×50m) |
| `SPD_FORWARD` | `0.18 m/s` | Normal cruise speed |
| `DANGER_DIST` | `0.28 m` | Emergency stop/avoidance distance |
| `USE_SUPERVISOR_POSE_SYNC` | `True` | Use ground-truth pose from Webots supervisor |
| `MAP_LAYER_IDS` | `(5, 7, 9)` | VLP-16 layers used for map updates |
| `EXPLORE_MAX_STEPS` | `12000` | Step budget for exploration phase |
| `SWEEP_MARK_RADIUS_CELLS` | `2` | Radius of cleaned footprint per waypoint |
| `ENABLE_CAMERA_SEMANTICS` | `True` | Enable camera-based object recognition |
| `PERSON_AVOID_DIST` | `0.85 m` | Distance at which robot halts for a person |

---

## How It Works

### Occupancy Grid
Each cell stores one of four states:

| State | Value | Meaning |
|---|---|---|
| `UNKNOWN` | 0 | Not yet observed by LiDAR |
| `FREE` | 1 | Observed and passable |
| `OCCUPIED` | 2 | Wall or obstacle |
| `SWEPT` | 3 | Passable and sanitized |

LiDAR returns are projected into the grid using Bresenham line-drawing — every cell along the ray is marked `FREE`, and the endpoint is marked `OCCUPIED` if a hit was detected.

### Frontier Detection
Frontiers are free cells with at least one `UNKNOWN` neighbour. They are clustered by BFS and sorted by proximity to the robot, weighted by cluster size. The robot navigates toward the nearest reachable frontier, systematically peeling back the unknown space.

### A\* Path Planning
Navigation between waypoints uses an A\* planner on the occupancy grid with a Manhattan distance heuristic and an extra cost penalty for unknown cells. The robot footprint is inflated by `ROBOT_RADIUS_M + SAFETY_MARGIN_M` so paths naturally maintain clearance from walls.

### Recovery
If the progress watchdog detects the robot hasn't moved at least 7 cm in 120 steps, it triggers a recovery manoeuvre: reverse 12 cm, then rotate by a random angle between 40° and 140°. A 100-step cooldown prevents back-to-back recoveries.

---

## Challenges & Limitations

### Webots & Simulation Environment

| Challenge | Detail |
|---|---|
| **Supervisor pose dependency** | `USE_SUPERVISOR_POSE_SYNC = True` gives the robot access to ground-truth position from Webots' physics engine. This is invaluable during development but would not exist on real hardware — a physical deployment would be entirely reliant on odometry fusion, which accumulates drift over time. |
| **No sensor noise** | The simulated VLP-16 returns perfect, noiseless range values. Real LiDAR suffers from measurement noise, specular reflections off shiny hospital floors, and complete misses on glass surfaces — none of which are present in simulation. |
| **Static arena** | The hospital world does not change during a run. Dynamic obstacles (staff members walking through, doors opening and closing, other robots) are not replanned around beyond the reactive avoidance layer. |
| **Physics simplification** | Webots models wheel–floor contact with simplified rigid-body physics. Real differential-drive robots experience wheel slip, especially on turns, which is not reflected in the simulated encoder readings. |
| **Device name coupling** | All sensor and motor names must match the `hospital.wbt` world file exactly. A mismatch causes a hard `RuntimeError` at startup. |

### Robot Hardware (TurtleBot3 Burger)

| Challenge | Detail |
|---|---|
| **Narrow track width** | The 160 mm track makes the robot susceptible to heading oscillation during straight-line navigation, especially after sharp turns. The heading gain constant `K_ANG` needs careful tuning to avoid wobble without being so aggressive it overshoots. |
| **Motor stall at low speed** | The DYNAMIXEL XL430 motors can lose torque at very low commanded velocities (during tight corridor approach maneuvers), a condition that the proportional controller does not explicitly guard against. |
| **VLP-16 mass** | The Velodyne VLP-16 is considerably heavier than the standard LDS-01 sensor the Burger was designed around. This shifts the robot's center of mass forward and upward — a factor not modeled in the controller dynamics. |
| **No physical sanitization payload** | Coverage is tracked virtually (grid cells are marked SWEPT). A real deployment would require an integrated UV-C lamp or liquid spray mechanism, adding weight, power draw, and control complexity. |

### Python & Software Design

| Challenge | Detail |
|---|---|
| **Single-threaded simulation loop** | The entire controller runs synchronously inside Webots' `robot.step()` call. Computationally heavy operations — `plan_coverage()` (full grid scan), `find_frontiers()` (grid perimeter BFS) — block the simulation tick. At 500×500 resolution this is tolerable, but the design does not scale to higher-resolution maps without threading or an async architecture. |
| **Pure Python grid operations** | The occupancy grid is a flat `bytearray` (no NumPy) to keep the dependency list at zero. Operations like `known_cell_count()` iterate over all 250,000 cells in interpreted Python. While fast enough in practice, this is orders of magnitude slower than an equivalent NumPy vectorized operation would be. |
| **No loop closure or SLAM** | Mapping is purely forward — LiDAR rays mark cells as the robot moves through the world. There is no pose-graph optimization or loop closure. If odometry drifts even slightly, walls mapped on opposite sides of the same corridor may not fully align on the grid. |
| **Coverage completeness not guaranteed** | The boustrophedon planner covers all reachable navigable cells known at the end of EXPLORE. If a doorway or gap only becomes accessible mid-sweep (because the robot's path itself clears the route), those newly reachable cells are not dynamically added to the plan. |
| **Fixed grid resolution** | 10 cm/cell is a design trade-off. Corridors narrower than ~30 cm (3 cells) may not register as navigable once the robot's clearance footprint is applied, causing those strips to be permanently skipped regardless of the robot's actual physical width. |
| **Frontier clustering cost** | `_cluster_frontiers()` performs a BFS over all frontier candidate cells. The cost grows with the perimeter of the unknown region, which in large open environments can be substantial without additional spatial pruning. |
| **Unreliable Boustrophedon results** | While the robot implements the lawnmower pattern, for unknown reasons (perhaps the limitations of the Webots Simulator or the 3D LiDAR's limited memory or perhaps a Python bug), the robot follows a randomized pattern. A thorough fix in the future could possibly fix it. |
| **No multi-robot support** | The system is designed for a single robot. A multi-robot deployment would require shared map infrastructure, task partitioning across robots, and inter-robot collision avoidance — none of which is implemented. |

---

*Built with [Webots](https://cyberbotics.com/) · [TurtleBot3 Burger](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) · Python 3*

*Reference: [hospital_webots](https://github.com/P4B5/hospital_webots)*

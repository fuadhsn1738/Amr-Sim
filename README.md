# 🤖 AMR-Sim — Hospital Floor Sanitization Robot
### Autonomous Mobile Robot · TurtleBot3 Burger · Webots Simulation
#### Junior Project Design

[![Python](https://img.shields.io/badge/Python-3.10%2B-blue?logo=python&logoColor=white)](https://www.python.org/)
[![Webots](https://img.shields.io/badge/Webots-R2023b%2B-brightgreen)](https://cyberbotics.com/)
[![Platform](https://img.shields.io/badge/Platform-TurtleBot3%20Burger-orange)](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

---

> A fully autonomous floor sanitization robot simulated in Webots. The robot explores an unknown hospital environment, builds a live occupancy map, plans a complete coverage path, executes a structured boustrophedon sweep, and returns to its starting position. All of these are done without human intervention.

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Repository Structure](#2-repository-structure)
3. [System Architecture](#3-system-architecture)
4. [Installation & Setup](#4-installation--setup)
5. [Configuration](#5-configuration)
6. [How the Robot Works](#6-how-the-robot-works)
   - [Phase 1 — EXPLORE](#phase-1--explore)
   - [Phase 2 — PLAN](#phase-2--plan)
   - [Phase 3 — SWEEP](#phase-3--sweep)
   - [Phase 4 — RETURN](#phase-4--return)
7. [Navigation & Safety Systems](#7-navigation--safety-systems)
8. [Sensor Stack](#8-sensor-stack)
9. [Occupancy Grid](#9-occupancy-grid)
10. [Challenges & Limitations](#10-challenges--limitations)

---

## 1. Project Overview

This repository contains the simulation for an **Autonomous Mobile Robot (AMR)** designed for hospital floor sanitization, developed as part of my Junior Project Design course.

The robot runs on a **TurtleBot3 Burger** platform inside a four-room 2×2 grid arena simulated in [Webots](https://cyberbotics.com/). The controller is written entirely in Python and implements a four-phase finite state machine (FSM):

```
  ┌─────────┐     ┌────────┐     ┌───────┐     ┌────────┐     ┌──────┐
  │ EXPLORE │────►│  PLAN  │────►│ SWEEP │────►│ RETURN │────►│ DONE │
  └─────────┘     └────────┘     └───────┘     └────────┘     └──────┘
```

**The robot is capable of:**
- Autonomously mapping an unknown environment using a **Velodyne VLP-16 3D LiDAR**
- Planning a complete floor coverage path via a **boustrophedon (lawnmower) algorithm**
- Navigating to waypoints using **A\* pathfinding** on a live occupancy grid
- Classifying environmental objects (people, furniture, walls) using a **monocular camera**
- Fusing encoder, gyroscope, and compass data for real-time odometry
- Recovering autonomously from stuck or blocked states

---

## 2. Repository Structure

```
Amr-Sim/
│
├── controllers/
│   └── burger_code/
│       └── burger_code.py      ← Main robot controller (all logic lives here)
│
├── protos/                     ← Custom Webots PROTO node definitions
│
├── worlds/
│   └── *.wbt                   ← Webots world file: 4-room hospital arena
│
├── .gitattributes
└── README.md
```

> All robot logic — FSM, navigation, mapping, odometry, sensor processing — is contained in a single well-structured file: `controllers/burger_code/burger_code.py`.

---

## 3. System Architecture

The controller is organized into five classes, each with a clear, isolated responsibility:

```
┌──────────────────────────────────────────────────────────────┐
│                        RobotBrain                            │
│         FSM: EXPLORE → PLAN → SWEEP → RETURN → DONE         │
│                                                              │
│   ┌────────────┐   ┌─────────────────┐   ┌───────────────┐  │
│   │  Odometry  │   │  LidarInterface │   │CameraProcessor│  │
│   │            │   │                 │   │               │  │
│   │ Encoder    │   │  Velodyne VLP-16│   │ Object recog. │  │
│   │ + Gyro     │   │  Sector reading │   │ Dirty-floor   │  │
│   │ + Compass  │   │  Map ray-cast   │   │ detection     │  │
│   └─────┬──────┘   └───────┬─────────┘   └──────┬────────┘  │
│         │                  │                    │            │
│         └──────────────────▼────────────────────┘            │
│                            │                                 │
│                  ┌─────────▼──────────┐                      │
│                  │   OccupancyGrid    │                      │
│                  │   500 × 500 cells  │                      │
│                  │  UNKNOWN / FREE /  │                      │
│                  │  OCCUPIED / SWEPT  │                      │
│                  └─────────┬──────────┘                      │
│                            │                                 │
│              ┌─────────────▼─────────────┐                   │
│              │    A* Path Planner        │                   │
│              │  + Nav Step Engine        │                   │
│              │  + Safety & Recovery      │                   │
│              └───────────────────────────┘                   │
└──────────────────────────────────────────────────────────────┘
```

| Class | Role |
|---|---|
| `Config` | Centralized constants — all tunable parameters in one place |
| `OccupancyGrid` | 500×500 spatial map; ray casting, pathfinding, frontier detection, coverage planning |
| `Odometry` | Pose estimation via encoder + gyro + compass fusion |
| `LidarInterface` | VLP-16 sector reading and LiDAR-to-grid ray updates |
| `CameraProcessor` | Object recognition and dirty-floor luminance detection |
| `RobotBrain` | FSM controller; owns all of the above |

---

## 4. Installation & Setup

### Prerequisites

| Requirement | Version |
|---|---|
| [Webots](https://cyberbotics.com/#download) | R2023b or later |
| Python | 3.10+ |

> **No pip packages required.** The controller uses only Python's standard library (`math`, `heapq`, `collections`, `random`) plus Webots' built-in `controller` module.

---

### Step 1 — Clone the Repository

```bash
git clone https://github.com/fuadhsn1738/Amr-Sim.git
cd Amr-Sim
```

### Step 2 — Open the World in Webots

1. Launch **Webots**
2. `File → Open World`
3. Select the `.wbt` file from the `worlds/` folder

### Step 3 — Verify the Controller Assignment

1. In the Webots scene tree, click on the **TurtleBot3 Burger** robot node
2. Confirm the `controller` field is set to `burger_code`
3. Webots will automatically locate `controllers/burger_code/burger_code.py`

### Step 4 — Verify Device Names

The controller expects the following device names to be set in the `.wbt` world file:

| Device | Expected Name in `.wbt` |
|---|---|
| Left drive motor | `left wheel motor` |
| Right drive motor | `right wheel motor` |
| Left wheel encoder | `left wheel sensor` |
| Right wheel encoder | `right wheel sensor` |
| 3D LiDAR | `Velodyne VLP-16` |
| Camera | `camera` |
| Gyroscope | `gyro` |
| Compass | `compass` |

> If any device name does not match, the controller will raise a `RuntimeError` on startup and print which device was not found.
> Ensure that the 3d LiDAR is the Velodyne VLP-16 instead of the LDS-01 the TurtleBot3 comes with. It can be easily changed from the scene view.

### Step 5 — Run

Press ▶ **Play** in Webots. The robot immediately begins the **EXPLORE** phase. Status logs are printed to the Webots console every 100 simulation steps.

---

## 5. Configuration

All tunable parameters live in the `Config` class at the top of `burger_code.py`. Nothing is magic-numbered elsewhere in the code.

```python
# ── Robot Geometry ──────────────────────────────────────────
WHEEL_RADIUS    = 0.033        # metres
TRACK_WIDTH     = 0.160        # metres
MAX_SPEED       = 6.67         # rad/s (motor velocity cap)

# ── Speeds ──────────────────────────────────────────────────
SPD_FORWARD     = 0.18         # m/s  — normal cruise speed
SPD_SLOW        = 0.07         # m/s  — used during recovery
SPD_TURN        = 3.1          # rad/s — in-place turn speed

# ── Obstacle Thresholds ─────────────────────────────────────
DANGER_DIST     = 0.28         # metres — hard avoidance trigger
WARN_DIST       = 0.45         # metres — speed reduction zone

# ── Occupancy Grid ──────────────────────────────────────────
CELL_M          = 0.10         # metres per cell  (10 cm resolution)
GRID_N          = 500          # grid side length (500×500 = 50×50 m)

# ── LiDAR Mapping ───────────────────────────────────────────
MAP_LAYER_IDS   = (5, 7, 9)    # VLP-16 layers used for wall mapping
MAP_H_STEP      = 8            # sample every N-th horizontal ray
MAP_INTERVAL    = 3            # update map every N simulation steps

# ── Coverage ────────────────────────────────────────────────
COV_STRIDE               = 1   # boustrophedon row spacing (1 = every row)
SWEEP_MARK_RADIUS_CELLS  = 2   # sanitization footprint radius in cells

# ── Exploration Convergence ──────────────────────────────────
EXPLORE_MIN_STEPS    = 1200    # minimum steps before checking convergence
EXPLORE_MAX_STEPS    = 12000   # hard time-out regardless of frontier state
EXPLORE_IDLE_LIMIT   = 8       # consecutive low-gain checks before PLAN

# ── Pose Source ─────────────────────────────────────────────
USE_SUPERVISOR_POSE_SYNC = True  # True = use Webots ground-truth pose
```

---

## 6. How the Robot Works

### Phase 1 — EXPLORE

The robot starts with a completely blank map and roams the environment to discover as much of it as possible before committing to a sweep plan.

**Two behaviors run in parallel:**

**① Frontier-based navigation**
Every `EXPLORE_CHECK_INT` (120) steps, the occupancy grid is scanned for *frontier cells* — FREE cells that share a border with at least one UNKNOWN cell. These are the edges of what has been discovered. Frontiers are found via a BFS pass over the grid, then clustered by spatial connectivity. The nearest/largest cluster centroid becomes a navigation goal and the robot pathfinds toward it using A*.

**② Reactive wandering**
When no frontier target is active, the robot cruises forward with a slowly-evolving angular bias (`_wander_omega`), bouncing reactively off obstacles using the five named LiDAR sectors (`front`, `front_left`, `front_right`, `left`, `right`). A randomized hold-time prevents the robot from locking into circular patterns.

**Convergence check (every 120 steps):**
- Counts how many new cells were mapped (`gain`) since the last check
- If `gain < EXPLORE_MIN_GAIN` persists for `EXPLORE_IDLE_LIMIT` (8) consecutive checks **and** fewer than `EXPLORE_FRONTIER_STOP_MAX` frontiers remain → transition to **PLAN**
- Hard time-out at `EXPLORE_MAX_STEPS` (12,000 steps) regardless

**Explore trace:** every distinct grid cell the robot physically drives through is appended to a running trace log, which becomes an alternative sweep plan during PLAN.

---

### Phase 2 — PLAN

A one-shot planning step that executes in a single simulation tick.

Two coverage plans are computed and compared:

**① Grid-based boustrophedon (primary)**
All FREE/SWEPT cells reachable from the current position (found via BFS flood-fill) are organized into horizontal row slices. A **boustrophedon path** is generated — sweeping left→right on one row, then right→left on the next, alternating. Row discontinuities caused by walls or furniture are split into separate row segments and handled individually. The robot's position biases the starting corner and direction.

**② Trace replay (fallback)**
The explore trace from Phase 1 is reversed, de-duplicated, and used as a waypoint list — making the robot re-visit every cell it physically travelled during exploration.

The plan with more waypoints is selected. The robot then pathfinds to the first reachable waypoint and transitions to **SWEEP**.

---

### Phase 3 — SWEEP

The robot works through the waypoint list sequentially.

**Each step:**
1. `_mark_swept_footprint()` — marks a 2-cell-radius disc around the robot as **SWEPT** in the grid
2. `_nav_step()` — steers toward the current waypoint using the proportional nav engine
3. On arrival (within `GOAL_TOL = 0.18 m`) — loads the next waypoint
4. Up to **5 consecutive unreachable waypoints** are skipped before the sweep is aborted early

Progress is logged to console every 25 waypoints (e.g. `[SWEEP] 75/300  25%`). When all waypoints are exhausted, the robot transitions to **RETURN**.

---

### Phase 4 — RETURN

The robot navigates back to its recorded home pose (`_home_wx`, `_home_wy`, `_home_theta`), trying four pathfinding strategies in order of increasing permissiveness:

| Strategy | Allow Unknown Cells | Mode |
|---|---|---|
| 1 | No | Safe known-cells only |
| 2 | Yes (fallback) | Known-first with unknown fallback |
| 3 | No | Explore mode |
| 4 | Yes | Explore + unknown fallback |

On arrival within `HOME_TOL = 0.12 m`, the robot corrects its heading to match the initial orientation, then transitions to **DONE** and stops all motors.

---

## 7. Navigation & Safety Systems

### A* Path Planner

The grid pathfinder implements A* with a Manhattan distance heuristic. Unknown cells are passable but incur an extra cost (`UNKNOWN_COST = 1.5`) to keep the robot biased toward known free space. Paths are compressed by `_compress_grid_path()` — consecutive collinear waypoints are merged into a single step to reduce nav jitter.

### Nav Step Engine (`_nav_step`)

Runs every simulation tick during SWEEP and RETURN:

- **Heading controller:** `ω = K_ANG × heading_error` (proportional)
- **Speed scaling:** forward speed decreases when heading error is large (turn first, then drive) and again when obstacles are close in the front sectors
- **Lateral repulsion:** if left or right flanks are within 0.35 m of a wall, a repulsion term is added to the wheel velocities to push the robot away

### Obstacle Avoidance (`_check_avoid`)

Triggered when any of the following are true:
- Front sector < `DANGER_DIST` (0.28 m) AND both sides are also blocked
- Either side is < 0.16 m from a wall
- A detected person is within `PERSON_AVOID_DIST` (0.85 m)

**Response:** brief reverse (6–8 ticks), then in-place turn toward the more open side. Turn direction is person-aware — always turns away from a detected person's bearing.

### Recovery System (`_start_recovery`)

Triggered by the progress watchdog when stuck. Sequence:
1. Reverse `RECOVERY_BACKUP_M` (0.12 m) at slow speed
2. Turn a random angle between 40° and 140° (randomized to escape corner traps)
3. A 50-step cooldown prevents immediate re-triggering

### Progress Watchdog (`_progress_watchdog`)

Checks every `NAV_STUCK_STEPS` (120) steps whether the robot has moved at least `NAV_MIN_PROGRESS` (0.07 m). If not:
1. Forces a path replan
2. If replanning fails → triggers recovery
3. If in SWEEP → skips current waypoint and advances
4. If all else fails → aborts to RETURN

### Runtime Replan Check

Every `NAV_REPLAN_INT` (18) steps during SWEEP and RETURN, the next waypoint on the active path is checked against the current grid. If a new LiDAR scan has since marked that cell as OCCUPIED, the path is immediately replanned.

---

## 8. Sensor Stack

### Velodyne VLP-16 (3D LiDAR)

| Parameter | Value |
|---|---|
| Total vertical layers | 16 |
| Layers used for wall mapping | 5, 7, 9 |
| Layers used for obstacle sectors | 2 – 13 |
| Horizontal sample step | Every 8th ray per map update |
| Map update interval | Every 3 simulation steps |
| Forward offset (Velodyne) | +180° (rotated relative to LDS-01) |
| Max mapping range | 5.5 m |

Sectors are named and defined in `Config.SECTOR_DEFS`:

| Sector | Angular Range |
|---|---|
| `front` | 340°–360° and 0°–20° |
| `front_left` | 21°–60° |
| `front_right` | 300°–339° |
| `left` | 61°–119° |
| `right` | 240°–299° |

### Camera

- Webots recognition API classifies objects into `person`, `wall`, `furniture` based on model label strings
- Person proximity (< 0.85 m) triggers an avoidance hold for `PERSON_HOLD_STEPS` (16) ticks
- **Dirty-floor detection** *(optional, off by default)*: samples the lower third of the camera frame; if > 15% of pixels have luminance < 60, the floor is flagged as dirty
- Recognition runs every `CAMERA_RECOG_INTERVAL` (8) steps to reduce per-tick overhead

### Odometry Fusion

Each simulation step, the robot's pose is updated by fusing three sources:

```
Δθ  =  0.70 × (gyro_y × dt)  +  0.30 × (Δright − Δleft) / track_width
```

A low-pass compass correction prevents long-term heading drift:

```
θ  ←  θ  +  0.015 × wrap( compass_heading − θ )
```

When `USE_SUPERVISOR_POSE_SYNC = True`, Webots' ground-truth supervisor pose overrides the fused estimate every tick — eliminating localization error during simulation development.

---

## 9. Occupancy Grid

The grid is a 500×500 flat `bytearray` (one byte per cell) covering a 50×50 m area at **10 cm/cell** resolution. The world origin maps to grid cell **(250, 250)**.

| State | Value | Meaning |
|---|---|---|
| `UNKNOWN` | `0` | Never observed by any sensor ray |
| `FREE` | `1` | A LiDAR ray passed through this cell; navigable |
| `OCCUPIED` | `2` | A LiDAR ray terminated here; wall or obstacle |
| `SWEPT` | `3` | Robot has physically driven over this cell |

**Ray casting** uses **Bresenham's line algorithm** — an integer-only line rasterization method — to efficiently mark all cells along a LiDAR ray as FREE, then marks the endpoint as OCCUPIED (if a hit was detected) or FREE (if it was a max-range miss).

**Clearance model:** a robot-radius disc of cell offsets (computed from `ROBOT_RADIUS_M + SAFETY_MARGIN_M = 0.11 m → ~2 cells`) is checked around every candidate waypoint. A cell is only considered navigable if the entire robot footprint centered there is free of OCCUPIED cells.

**Frontier detection** scans for FREE/SWEPT cells adjacent to UNKNOWN cells, clusters them by BFS connectivity, and returns each cluster's centroid and size. Clusters smaller than `FRONT_MIN` (3) cells are discarded as noise.

---

## 10. Challenges & Limitations

### Webots & Simulation Environment

| Challenge | Detail |
|---|---|
| **Supervisor pose dependency** | `USE_SUPERVISOR_POSE_SYNC = True` gives the robot access to ground-truth position from Webots' physics engine. This is invaluable during development but would not exist on real hardware — a physical deployment would be entirely reliant on odometry fusion, which accumulates drift over time. |
| **No sensor noise** | The simulated VLP-16 returns perfect, noiseless range values. Real LiDAR suffers from measurement noise, specular reflections off shiny hospital floors, and complete misses on glass surfaces — none of which are present in simulation. |
| **Static arena** | The hospital world does not change during a run. Dynamic obstacles (staff members walking through, doors opening and closing, other robots) are not replanned around beyond the reactive avoidance layer. |
| **Physics simplification** | Webots models wheel–floor contact with simplified rigid-body physics. Real differential-drive robots experience wheel slip, especially on turns, which is not reflected in the simulated encoder readings. |
| **Device name coupling** | All sensor and motor names must match the `.wbt` world file exactly. A mismatch causes a hard `RuntimeError` at startup. |

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
| **No multi-robot support** | The system is designed for a single robot. A multi-robot deployment would require shared map infrastructure, task partitioning across robots, and inter-robot collision avoidance — none of which is implemented. |

---

*Built with [Webots](https://cyberbotics.com/) · [TurtleBot3 Burger](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) · Python 3*

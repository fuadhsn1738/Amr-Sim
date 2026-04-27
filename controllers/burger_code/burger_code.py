"""
Intelligent Hospital Floor Sweeper
TurtleBot3 Burger  |  Velodyne VLP-16 + Camera  |  Webots R2025a
═══════════════════════════════════════════════════════════════════════

THREE-PHASE ARCHITECTURE
─────────────────────────────────────────────────────────────────────
Phase 1  EXPLORE
    The robot drives toward frontier cells — boundaries between known-
    free space and unexplored space.  BFS path planning on the occupancy
    grid avoids walls.  When no frontiers remain, all reachable space
    has been mapped.

Phase 2  PLAN
    A boustrophedon (row-by-row) coverage path is computed from the
    completed map.  Waypoints are spaced COV_STRIDE cells (≈ 0.3 m)
    apart so every floor tile falls within the robot's body width.
    One-time calculation; result stored as a waypoint list.

Phase 3  SWEEP
    The robot follows the waypoint list.  Each visited cell is marked
    SWEPT.  When the list is exhausted, the floor is clean.

SENSORS & THEIR ROLES
─────────────────────────────────────────────────────────────────────
  "left/right wheel motor"  — locomotion
  "left/right wheel sensor" — encoder odometry: Δdist = Δrad × r_wheel
  "gyro"                    — yaw rate (Y-axis in Webots Y-up world)
  "compass"                 — absolute heading correction (drift guard)
  "Velodyne VLP-16"         — occupancy grid + obstacle avoidance
  "camera"                  — dirty-floor logging (informational)
  "accelerometer"           — collision alert (informational)

ODOMETRY CONVENTION
─────────────────────────────────────────────────────────────────────
  Origin  (0, 0)  = robot start position
  θ = 0           = initial forward direction
  θ increases     counter-clockwise (standard maths convention)

  Gyro:  yaw rate = getValues()[1]   ← Y-axis in Webots Y-up world
         FIX #1 from review: was [2] (Z = pitch), now [1] (Y = yaw)

  Compass heading:  θ = atan2(cv[0], cv[2])
         FIX #2 from review: was atan2(cv[0], cv[1]).
         In Webots Y-up, the compass vector lies in the XZ plane.
         cv[1] (the vertical component) is near zero on flat ground,
         making atan2(cx, cy) unreliable.  The correct formula uses
         cv[2] (the horizontal Z component of the world frame).

VLP-16 RAY-ZERO OFFSET
─────────────────────────────────────────────────────────────────────
  Webots Lidar scans from -FOV/2 = -π → ray 0 points BACKWARD.
  RAY0_OFFSET = H_RES/2 shifts every sector so that
  h_idx = H_RES/2 maps to robot forward direction.

FIXES APPLIED  (from code review)
─────────────────────────────────────────────────────────────────────
  #1  Gyro axis       : getValues()[2] → [1]  (Y = yaw in Y-up world)
  #2  Compass formula : atan2(cv[0], cv[1]) → atan2(cv[0], cv[2])
  #3  update_ray guard: no longer drops finite readings > MAX_RANGE*1.05
  #4  plan_coverage   : includes FREE and SWEPT cells in waypoint scan
  #5  _precompute_sectors: removed inner 5× duplicate loop
  #6  RESCAN_INT      : 28 → 200  (stops 40k-cell BFS firing every 0.9 s)
  #7  Avoidance clear : clears as soon as front is clear + min 10 steps
  #8  _vel_from_vw    : renamed ri → r_vel to avoid confusion with radius r
  #9  SPD_FORWARD     : stored as m/s directly; removed spurious
                        ×WHEEL_RADIUS / ÷WHEEL_RADIUS round-trip

DEVICE NAMES  (must match your .wbt world)
─────────────────────────────────────────────────────────────────────
  "left wheel motor"    "right wheel motor"
  "left wheel sensor"   "right wheel sensor"
  "gyro"  "compass"  "accelerometer"
  "Velodyne VLP-16"   "camera"
"""

from controller import Robot, Camera
import math
from collections import deque
import random


# ═══════════════════════════════════════════════════════════════════════
#  CONFIGURATION
# ═══════════════════════════════════════════════════════════════════════
class Config:
    # ── Simulation ────────────────────────────────────────────────────
    TIME_STEP    = 32           # ms

    # ── Robot geometry  (TurtleBot3 Burger proto) ─────────────────────
    WHEEL_RADIUS = 0.033        # m
    TRACK_WIDTH  = 0.160        # m
    MAX_SPEED    = 6.67         # rad/s  (PROTO maxVelocity)

    # ── Locomotion speeds ─────────────────────────────────────────────
    # FIX #9: SPD_FORWARD is now stored as LINEAR speed [m/s] directly.
    # Previously it was treated as rad/s and multiplied by WHEEL_RADIUS
    # inside _nav_step, only to be divided by WHEEL_RADIUS again inside
    # _vel_from_vw.  The round-trip was functionally a no-op but masked
    # the intent of the constant.
    SPD_FORWARD  = 0.12         # m/s  cruise  (≈ 3.5 rad/s × 0.033 m)
    SPD_SLOW     = 0.05         # m/s  near waypoints
    SPD_TURN     = 2.5          # rad/s  in-place rotation
    K_ANG        = 4.5          # angular proportional gain

    # ── Obstacle thresholds  [m] ──────────────────────────────────────
    DANGER_DIST  = 0.28
    WARN_DIST    = 0.45

    # ── Occupancy grid ────────────────────────────────────────────────
    CELL_M       = 0.10         # m per cell
    GRID_N       = 200          # 200×200 cells → 20 m × 20 m map
    GRID_OX      = 100          # robot start column (centre)
    GRID_OY      = 100          # robot start row    (centre)

    # ── LiDAR  (Velodyne VLP-16) ──────────────────────────────────────
    H_RES           = 1800      # horizontal points per layer
    N_LAYERS        = 16
    # FIX #5: RAY0_OFFSET is H_RES//2 = 900, same value as before but
    # now computed from H_RES so it stays correct if H_RES is changed.
    RAY0_OFFSET     = H_RES // 2
    MAP_LAYERS      = [5, 7, 9] # vertical layers used for grid building
    MAP_H_STEP      = 12        # sample every 12th h_point → 150 rays/layer
    MAP_INTERVAL    = 4         # grid update every N steps
    MAX_RANGE       = 5.5       # m  (cap free-space ray length)
    SEC_LAYER_MIN   = 2         # layers used for obstacle-avoidance sectors
    SEC_LAYER_MAX   = 13

    # Sector definitions  (degrees, 0 = forward, CCW positive)
    SECTOR_DEFS = {
        'front':        [(340, 359), (0, 20)],
        'front_left':   [(21,  60)],
        'front_right':  [(300, 339)],
        'left':         [(61,  119)],
        'right':        [(240, 299)],
    }

    # ── Navigation tolerances ─────────────────────────────────────────
    GOAL_TOL     = 0.18         # m  — waypoint reached
    HEAD_TOL     = 0.15         # rad — heading aligned enough to drive

    # ── Frontier exploration ──────────────────────────────────────────
    FRONT_MIN    = 4            # minimum cells to form a valid cluster
    # FIX #6: RESCAN_INT raised from 28 to 200.
    # find_frontiers() scans all 40,000 grid cells and clusters results
    # via BFS.  Firing every 28 steps (0.9 s) caused the Python loop to
    # dominate CPU time and stall the simulation.  200 steps ≈ 6.4 s is
    # frequent enough to track new frontiers without pegging the CPU.
    RESCAN_INT   = 200
    STUCK_CHECK  = 60           # steps between stuck checks
    STUCK_THRESH = 0.08         # m  — less than this = stuck

    # ── Coverage sweep ────────────────────────────────────────────────
    COV_STRIDE   = 3            # grid cells between sweep waypoints (≈ 0.30 m)

    # ── Odometry fusion ───────────────────────────────────────────────
    GYRO_WEIGHT    = 0.70       # weight for gyro in heading fusion
    ENC_WEIGHT     = 0.30       # weight for encoder Δθ
    COMPASS_ALPHA  = 0.015      # low-pass blend toward compass reading

    # ── Avoidance ─────────────────────────────────────────────────────
    AVOID_MIN_STEPS = 10        # FIX #7: minimum steps before clearing avoidance

    # ── Grid cell states ──────────────────────────────────────────────
    UNKNOWN  = 0
    FREE     = 1
    OCCUPIED = 2
    SWEPT    = 3


# ═══════════════════════════════════════════════════════════════════════
#  OCCUPANCY GRID
# ═══════════════════════════════════════════════════════════════════════
class OccupancyGrid:
    """
    Flat-list 2D occupancy grid.  Coordinate convention:
        world (0,0) → grid (GRID_OX, GRID_OY)
        grid (gx, gy) ↔ world ((gx-OX)*CELL_M, (gy-OY)*CELL_M)
    """

    def __init__(self):
        N = Config.GRID_N
        self._N = N
        self._g = bytearray(N * N)
        # Pre-mark a 7×7 patch around the start as FREE so the robot has
        # valid ground truth before the first LiDAR scan arrives.
        ox, oy = Config.GRID_OX, Config.GRID_OY
        for dy in range(-3, 4):
            for dx in range(-3, 4):
                self._set_val(ox + dx, oy + dy, Config.FREE)

    # ── Coordinate helpers ────────────────────────────────────────────

    @staticmethod
    def w2g(wx, wy):
        gx = int(round(wx / Config.CELL_M)) + Config.GRID_OX
        gy = int(round(wy / Config.CELL_M)) + Config.GRID_OY
        return gx, gy

    @staticmethod
    def g2w(gx, gy):
        wx = (gx - Config.GRID_OX) * Config.CELL_M
        wy = (gy - Config.GRID_OY) * Config.CELL_M
        return wx, wy

    def in_bounds(self, gx, gy):
        return 0 <= gx < self._N and 0 <= gy < self._N

    # ── Cell access ───────────────────────────────────────────────────

    def get(self, gx, gy):
        if not self.in_bounds(gx, gy):
            return Config.OCCUPIED
        return self._g[gy * self._N + gx]

    def _set_val(self, gx, gy, val):
        if self.in_bounds(gx, gy):
            self._g[gy * self._N + gx] = val

    def set_free(self, gx, gy):
        """UNKNOWN → FREE only.  Never downgrades OCCUPIED or SWEPT."""
        if self.in_bounds(gx, gy):
            idx = gy * self._N + gx
            if self._g[idx] == Config.UNKNOWN:
                self._g[idx] = Config.FREE

    def set_occupied(self, gx, gy):
        if self.in_bounds(gx, gy):
            self._g[gy * self._N + gx] = Config.OCCUPIED

    def set_swept(self, gx, gy):
        """FREE → SWEPT only.  Never overwrites OCCUPIED."""
        if self.in_bounds(gx, gy):
            idx = gy * self._N + gx
            if self._g[idx] == Config.FREE:
                self._g[idx] = Config.SWEPT

    # ── Bresenham ray cast ────────────────────────────────────────────

    @staticmethod
    def _bresenham(x0, y0, x1, y1):
        """Yield integer (x, y) cells from start to end inclusive."""
        dx = abs(x1 - x0);  dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        while True:
            yield x0, y0
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:  err -= dy;  x0 += sx
            if e2 <  dx:  err += dx;  y0 += sy

    def update_ray(self, robot_wx, robot_wy, angle_world, dist):
        """
        Mark one LiDAR ray on the grid.

        robot_wx/wy  : robot world position [m]
        angle_world  : ray angle in world frame [rad]
        dist         : measured range [m]  (may be inf for a miss)

        FIX #3: The original guard was:
            if not (0.05 < dist < MAX_RANGE * 1.05) and not isinf(dist): return
        Due to De Morgan's law this silently dropped ALL finite readings
        beyond MAX_RANGE * 1.05 = 5.775 m (e.g. a wall at 5.8 m would
        never be marked OCCUPIED).

        New guard: only skip readings that are too close to be trusted
        (< 0.05 m = sensor blind spot) and are not inf.  Valid long-range
        readings pass through and get clamped to MAX_RANGE below.
        """
        # Skip readings from the sensor blind-spot (not inf)
        if not math.isinf(dist) and dist <= 0.05:
            return

        # Cap ray length: beyond MAX_RANGE we only mark FREE (no wall)
        is_hit  = not math.isinf(dist) and dist < Config.MAX_RANGE
        ray_len = min(dist if not math.isinf(dist) else Config.MAX_RANGE,
                      Config.MAX_RANGE)

        # Robot and endpoint in grid coordinates
        rx, ry = self.w2g(robot_wx, robot_wy)
        ex_w   = robot_wx + ray_len * math.cos(angle_world)
        ey_w   = robot_wy + ray_len * math.sin(angle_world)
        ex, ey = self.w2g(ex_w, ey_w)

        # Mark all cells along the ray FREE (stop before the endpoint)
        for cx, cy in self._bresenham(rx, ry, ex, ey):
            if cx == ex and cy == ey:
                break
            self.set_free(cx, cy)

        # Endpoint: OCCUPIED for a real hit, FREE for a max-range miss
        if is_hit:
            self.set_occupied(ex, ey)
        else:
            self.set_free(ex, ey)

    # ── Frontier detection ────────────────────────────────────────────

    def find_frontiers(self):
        """
        Scan grid for frontier cells (FREE with ≥1 UNKNOWN 4-neighbour).
        Cluster them via BFS.  Return list of (world_x, world_y, size).
        """
        N = self._N
        g = self._g
        F = Config.FREE
        U = Config.UNKNOWN
        candidates = set()

        for gy in range(1, N - 1):
            row = gy * N
            for gx in range(1, N - 1):
                if g[row + gx] != F:
                    continue
                if (g[(gy-1)*N + gx] == U or
                    g[(gy+1)*N + gx] == U or
                    g[row + gx - 1]  == U or
                    g[row + gx + 1]  == U):
                    candidates.add((gx, gy))

        return self._cluster_frontiers(candidates)

    def _cluster_frontiers(self, cells):
        clusters  = []
        remaining = set(cells)
        while remaining:
            seed  = next(iter(remaining))
            group = []
            q     = deque([seed])
            seen  = {seed}
            while q:
                cx, cy = q.popleft()
                group.append((cx, cy))
                remaining.discard((cx, cy))
                for dx, dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,1),(1,-1),(-1,-1)):
                    nb = (cx+dx, cy+dy)
                    if nb in remaining and nb not in seen:
                        seen.add(nb)
                        q.append(nb)
            if len(group) >= Config.FRONT_MIN:
                cgx = sum(p[0] for p in group) / len(group)
                cgy = sum(p[1] for p in group) / len(group)
                wx, wy = self.g2w(int(cgx), int(cgy))
                clusters.append((wx, wy, len(group)))
        return clusters

    # ── BFS path planning ─────────────────────────────────────────────

    def bfs_path(self, sx, sy, gx, gy, allow_unknown=True):
        """
        BFS shortest path from grid (sx,sy) → (gx,gy).
        Returns list of (grid_x, grid_y) or None if unreachable.
        Traverses FREE, SWEPT; optionally UNKNOWN (during exploration).
        """
        if not self.in_bounds(gx, gy):
            return None

        passable = {Config.FREE, Config.SWEPT}
        if allow_unknown:
            passable.add(Config.UNKNOWN)

        came_from = {(sx, sy): None}
        q = deque([(sx, sy)])

        while q:
            cx, cy = q.popleft()
            if cx == gx and cy == gy:
                break
            for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
                nx, ny = cx+dx, cy+dy
                if ((nx, ny) not in came_from and
                        self.in_bounds(nx, ny) and
                        self.get(nx, ny) in passable):
                    came_from[(nx, ny)] = (cx, cy)
                    q.append((nx, ny))

        if (gx, gy) not in came_from:
            return None

        path = []
        node = (gx, gy)
        while node is not None:
            path.append(node)
            node = came_from[node]
        path.reverse()
        return path

    # ── Coverage planning ─────────────────────────────────────────────

    def plan_coverage(self):
        """
        Boustrophedon sweep over all reachable cells, spaced COV_STRIDE apart.
        Returns list of (world_x, world_y) waypoints in sweep order.

        FIX #4: Original only included FREE cells.  Now includes FREE and
        SWEPT so that cells visited during exploration are not silently
        excluded from the coverage plan should any be marked SWEPT early.
        """
        N  = self._N
        g  = self._g
        st = Config.COV_STRIDE
        reachable = {Config.FREE, Config.SWEPT}

        # Bounding box of reachable space
        min_gx = max_gx = Config.GRID_OX
        min_gy = max_gy = Config.GRID_OY
        found  = False
        for gy in range(N):
            for gx in range(N):
                if g[gy*N + gx] in reachable:
                    if not found:
                        min_gx = max_gx = gx
                        min_gy = max_gy = gy
                        found  = True
                    else:
                        if gx < min_gx: min_gx = gx
                        if gx > max_gx: max_gx = gx
                        if gy < min_gy: min_gy = gy
                        if gy > max_gy: max_gy = gy

        if not found:
            return []

        waypoints = []
        row_idx   = 0
        for scan_gy in range(min_gy, max_gy + 1, st):
            row = [
                (scan_gx, scan_gy)
                for scan_gx in range(min_gx, max_gx + 1, st)
                if g[scan_gy * N + scan_gx] in reachable   # FIX #4
            ]
            if row_idx % 2 == 1:
                row.reverse()
            row_idx += 1
            for cell in row:
                wx, wy = self.g2w(cell[0], cell[1])
                waypoints.append((wx, wy))

        return waypoints


# ═══════════════════════════════════════════════════════════════════════
#  HELPERS
# ═══════════════════════════════════════════════════════════════════════

def _wrap(a):
    """Normalise angle to (-π, π]."""
    return math.atan2(math.sin(a), math.cos(a))


# ═══════════════════════════════════════════════════════════════════════
#  ODOMETRY
# ═══════════════════════════════════════════════════════════════════════
class Odometry:
    """
    Dead-reckoning: (x, y, θ) from wheel encoders fused with gyro.
    Periodic compass correction prevents long-term heading drift.

    First call to update() records initial encoder positions and returns
    without advancing state (so Δencoders is always valid on step 2+).
    """

    def __init__(self, left_enc, right_enc, gyro, compass):
        self._le      = left_enc
        self._re      = right_enc
        self._gy      = gyro
        self._cp      = compass
        self._dt      = Config.TIME_STEP / 1000.0
        self._init    = False

        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0
        self._pl   = 0.0    # previous left encoder  [rad]
        self._pr   = 0.0    # previous right encoder [rad]

    def update(self):
        l_pos = self._le.getValue()
        r_pos = self._re.getValue()

        if not self._init:
            self._pl   = l_pos
            self._pr   = r_pos
            self._init = True
            return

        # ── Encoder delta ────────────────────────────────────────────
        dl = (l_pos - self._pl) * Config.WHEEL_RADIUS   # [m]
        dr = (r_pos - self._pr) * Config.WHEEL_RADIUS   # [m]
        self._pl = l_pos
        self._pr = r_pos

        # Safety clamp (catches NaN / initialisation spikes)
        max_d = Config.MAX_SPEED * Config.WHEEL_RADIUS * self._dt * 1.5
        dl = max(-max_d, min(max_d, dl))
        dr = max(-max_d, min(max_d, dr))

        # ── Heading update  (fuse encoder + gyro) ────────────────────
        d_theta_enc  = (dr - dl) / Config.TRACK_WIDTH

        # FIX #1: Gyro yaw rate is on axis [1] (Y) in Webots Y-up world.
        # The original code used [2] (Z-axis = pitch on flat ground),
        # which contributes near-zero noise rather than real yaw data.
        d_theta_gyro = self._gy.getValues()[1] * self._dt

        d_theta = (Config.GYRO_WEIGHT * d_theta_gyro +
                   Config.ENC_WEIGHT  * d_theta_enc)

        # ── Position update  (midpoint integration) ──────────────────
        d_dist   = (dl + dr) / 2.0
        mid_t    = self.theta + d_theta / 2.0
        self.x  += d_dist * math.cos(mid_t)
        self.y  += d_dist * math.sin(mid_t)
        self.theta = _wrap(self.theta + d_theta)

        # ── Compass correction  (gentle low-pass drift guard) ─────────
        cv = self._cp.getValues()   # [cx, cy, cz] in world frame

        # FIX #2: The original formula was atan2(cv[0], cv[1]).
        # In Webots Y-up the compass vector lies in the XZ plane;
        # cv[1] (Y) is the near-zero vertical component.  Passing a
        # near-zero value as the second argument to atan2 produces a
        # result always near ±π/2 regardless of actual heading.
        # Correct formula: atan2(cv[0], cv[2])  (X and Z components).
        t_c  = math.atan2(cv[0], cv[2])
        diff = _wrap(t_c - self.theta)
        self.theta = _wrap(self.theta + Config.COMPASS_ALPHA * diff)


# ═══════════════════════════════════════════════════════════════════════
#  LIDAR INTERFACE
# ═══════════════════════════════════════════════════════════════════════
class LidarInterface:
    """
    Two modes:
    read_sectors()     — fast per-step obstacle-avoidance sectors
    update_map(...)    — sampled multi-layer scan to build occupancy grid
    """

    def __init__(self, device):
        self._dev     = device
        self._sectors = self._precompute_sectors()

    # ── Sector pre-computation ────────────────────────────────────────

    def _precompute_sectors(self):
        """
        Convert degree-range sector definitions to h_idx lists.

        FIX #5: The original loop was:
            for deg in range(sd, ed + 1):
                h = (deg * 5 + off) % H_RES
                for k in range(5):            # ← inner duplicate loop
                    hidx.append((h + k) % H_RES)

        With H_RES=1800, each degree spans exactly 5 consecutive h-indices
        (1800/360 = 5).  Because the outer loop already steps through every
        degree, the inner k-loop added each h-index 5 times.  This caused
        read_sectors() to do 5× redundant work per sector per step.

        Fix: iterate directly over the h-index range for each degree span,
        producing each index exactly once.
        """
        built = {}
        off   = Config.RAY0_OFFSET
        H     = Config.H_RES
        pts_per_deg = H // 360  # = 5 for H_RES = 1800

        for name, ranges in Config.SECTOR_DEFS.items():
            hidx = []
            for sd, ed in ranges:
                # Convert degree bounds to h-index bounds directly
                h_start = (sd * pts_per_deg + off) % H
                h_end   = (ed * pts_per_deg + pts_per_deg - 1 + off) % H

                # Walk the h-index range, wrapping around H if needed
                h = h_start
                count = (ed - sd + 1) * pts_per_deg
                for _ in range(count):
                    hidx.append(h)
                    h = (h + 1) % H
            built[name] = hidx
        return built

    @staticmethod
    def _valid(r):
        return 0.05 < r < 20.0 and not math.isinf(r) and not math.isnan(r)

    # ── Obstacle-avoidance sectors ────────────────────────────────────

    def read_sectors(self):
        """Return {sector_name: min_valid_range_m} for all sectors."""
        ranges = self._dev.getRangeImage()
        lo = Config.SEC_LAYER_MIN
        hi = Config.SEC_LAYER_MAX
        H  = Config.H_RES
        result = {}
        for name, hidx in self._sectors.items():
            best = float('inf')
            for layer in range(lo, hi + 1):
                base = layer * H
                for h in hidx:
                    r = ranges[base + h]
                    if self._valid(r) and r < best:
                        best = r
            result[name] = best
        return result

    # ── Occupancy grid update ─────────────────────────────────────────

    def update_map(self, grid, odom):
        """
        Sample VLP-16 and update the occupancy grid.
        Uses MAP_LAYERS and MAP_H_STEP for efficient Python execution.
        """
        ranges   = self._dev.getRangeImage()
        robot_wx = odom.x
        robot_wy = odom.y
        theta    = odom.theta
        step     = Config.MAP_H_STEP
        off      = Config.RAY0_OFFSET
        tau      = 2 * math.pi
        H        = Config.H_RES

        for layer in Config.MAP_LAYERS:
            base = layer * H
            for h in range(0, H, step):
                r = ranges[base + h]
                # Skip invalid non-inf readings; pass inf through for free-space
                if not math.isinf(r) and not self._valid(r):
                    continue
                angle_robot = (h - off) * (tau / H)   # 0 = forward, CCW+
                angle_world = angle_robot + theta
                grid.update_ray(robot_wx, robot_wy, angle_world, r)


# ═══════════════════════════════════════════════════════════════════════
#  CAMERA PROCESSOR  (informational)
# ═══════════════════════════════════════════════════════════════════════
class CameraProcessor:
    """Samples the camera image for dirty-floor detection.  Log only."""

    def __init__(self, device):
        self._dev = device
        self._w   = device.getWidth()
        self._h   = device.getHeight()

    def is_floor_dirty(self):
        img = self._dev.getImage()
        if not img:
            return False
        w, h  = self._w, self._h
        bot   = (2 * h) // 3
        step  = 8
        dark = tot = 0
        for y in range(bot, h, step):
            for x in range(0, w, step):
                r   = Camera.imageGetRed(img,   w, x, y)
                g   = Camera.imageGetGreen(img, w, x, y)
                b   = Camera.imageGetBlue(img,  w, x, y)
                lum = 0.299*r + 0.587*g + 0.114*b
                tot += 1
                if lum < 60:
                    dark += 1
        return tot > 0 and (dark / tot) > 0.15


# ═══════════════════════════════════════════════════════════════════════
#  ROBOT BRAIN  (main FSM)
# ═══════════════════════════════════════════════════════════════════════
class RobotBrain:
    """
    Top-level state machine:
        EXPLORE  → navigate toward nearest frontier, building the map
        PLAN     → compute boustrophedon coverage waypoints from map
        SWEEP    → follow waypoints, marking each cell SWEPT
        DONE     → stop; all reachable free cells have been cleaned
    """

    EXPLORE = 'EXPLORE'
    PLAN    = 'PLAN'
    SWEEP   = 'SWEEP'
    DONE    = 'DONE'

    def __init__(self):
        robot = Robot()
        self._robot = robot
        ts = Config.TIME_STEP

        # ── Motors ────────────────────────────────────────────────────
        self._lm = robot.getDevice('left wheel motor')
        self._rm = robot.getDevice('right wheel motor')
        for m in (self._lm, self._rm):
            m.setPosition(float('inf'))
            m.setVelocity(0.0)

        # ── Wheel encoders ────────────────────────────────────────────
        ls = robot.getDevice('left wheel sensor')
        rs = robot.getDevice('right wheel sensor')
        ls.enable(ts);  rs.enable(ts)

        # ── IMU ───────────────────────────────────────────────────────
        gyro  = robot.getDevice('gyro')
        comp  = robot.getDevice('compass')
        accel = robot.getDevice('accelerometer')
        gyro.enable(ts);  comp.enable(ts);  accel.enable(ts)
        self._accel = accel

        # ── LiDAR ─────────────────────────────────────────────────────
        lidar_dev = robot.getDevice('Velodyne VLP-16')
        lidar_dev.enable(ts)

        # ── Camera ────────────────────────────────────────────────────
        cam_dev = robot.getDevice('camera')
        cam_dev.enable(ts)

        # ── Subsystems ────────────────────────────────────────────────
        self._odom  = Odometry(ls, rs, gyro, comp)
        self._lidar = LidarInterface(lidar_dev)
        self._grid  = OccupancyGrid()
        self._cam   = CameraProcessor(cam_dev)

        # ── FSM state ─────────────────────────────────────────────────
        self._state       = self.EXPLORE

        # Navigation
        self._goal_wx  = 0.0
        self._goal_wy  = 0.0
        self._nav_path = []     # BFS path as [(gx, gy), ...]
        self._nav_idx  = 0

        # Explore
        self._frontiers  = []
        self._front_idx  = 0
        self._rescan_t   = 0
        self._stuck_t    = 0
        self._stuck_ref  = (0.0, 0.0)

        # Sweep
        self._sweep_plan = []
        self._sweep_idx  = 0

        # Obstacle avoidance
        # FIX #7: track elapsed avoid steps separately from a countdown,
        # so avoidance can clear as soon as front is clear AND at least
        # AVOID_MIN_STEPS have passed (instead of always waiting for the
        # full countdown to hit zero).
        self._avoid_on     = False
        self._avoid_steps  = 0     # how many steps we have been avoiding
        self._avoid_dir    = 1
        self._avoid_back   = 0

        # Timers
        self._map_t  = 0
        self._cam_t  = 0
        self._step   = 0

        print("=" * 64)
        print("  INTELLIGENT SWEEPER  --  Explore -> Plan -> Sweep")
        print("=" * 64)

    # ── Motor helpers ─────────────────────────────────────────────────

    def _drive(self, l, r):
        cap = Config.MAX_SPEED
        self._lm.setVelocity(max(-cap, min(cap, l)))
        self._rm.setVelocity(max(-cap, min(cap, r)))

    def _stop(self):
        self._drive(0.0, 0.0)

    def _vel_from_vw(self, v, omega):
        """
        Convert (linear m/s, angular rad/s) → (left rad/s, right rad/s).

        FIX #8: renamed inner variable from 'ri' to 'r_vel' to avoid
        visual confusion with the wheel radius variable 'r'.
        FIX #9: v is now a true linear speed [m/s]; no longer need to
        pre-multiply by WHEEL_RADIUS before calling this function.
        """
        r     = Config.WHEEL_RADIUS
        tw    = Config.TRACK_WIDTH
        l_vel = (v - omega * tw / 2.0) / r
        r_vel = (v + omega * tw / 2.0) / r
        return l_vel, r_vel

    # ── Goal setting & BFS re-planning ────────────────────────────────

    def _set_goal(self, wx, wy, explore=True):
        self._goal_wx = wx
        self._goal_wy = wy
        sx, sy = self._grid.w2g(self._odom.x, self._odom.y)
        gx, gy = self._grid.w2g(wx, wy)
        path   = self._grid.bfs_path(sx, sy, gx, gy, allow_unknown=explore)
        self._nav_path = path if path else []
        self._nav_idx  = 0

    # ── Navigation step ───────────────────────────────────────────────

    def _nav_step(self, obs):
        """
        One step toward the current goal.  Returns True when reached.

        FIX #9: v is computed directly in m/s (SPD_FORWARD is now m/s).
        The original multiplied SPD_FORWARD (rad/s) by WHEEL_RADIUS here
        and then divided by WHEEL_RADIUS inside _vel_from_vw, which
        cancelled out.  Now both sides use consistent units.
        """
        # Determine the immediate sub-goal from BFS path
        if self._nav_path and self._nav_idx < len(self._nav_path):
            tgx, tgy = self._nav_path[self._nav_idx]
            twx, twy = self._grid.g2w(tgx, tgy)
        else:
            twx, twy = self._goal_wx, self._goal_wy

        dx   = twx - self._odom.x
        dy   = twy - self._odom.y
        dist = math.hypot(dx, dy)

        # Reached current path sub-goal → advance
        if dist < Config.GOAL_TOL:
            if self._nav_path and self._nav_idx < len(self._nav_path):
                self._nav_idx += 1
                if self._nav_idx >= len(self._nav_path):
                    self._stop()
                    return True
                tgx, tgy = self._nav_path[self._nav_idx]
                twx, twy = self._grid.g2w(tgx, tgy)
                dx = twx - self._odom.x;  dy = twy - self._odom.y
                dist = math.hypot(dx, dy)
            else:
                self._stop()
                return True

        # Proportional heading controller
        target_angle = math.atan2(dy, dx)
        h_err = _wrap(target_angle - self._odom.theta)

        # FIX #9: SPD_FORWARD is already in m/s — no WHEEL_RADIUS multiply
        align_frac = max(0.15, 1.0 - abs(h_err) / math.pi)
        dist_frac  = min(1.0, dist / 0.5)
        v     = Config.SPD_FORWARD * align_frac * dist_frac
        omega = Config.K_ANG * h_err

        # Soft side-wall repulsion
        repel_l = repel_r = 0.0
        if obs['right'] < 0.35:
            repel_r = -min(2.0, 1.0 / max(0.05, obs['right'] - 0.20))
        elif obs['front_right'] < 0.40:
            repel_r = -0.5
        if obs['left'] < 0.35:
            repel_l = -min(2.0, 1.0 / max(0.05, obs['left'] - 0.20))
        elif obs['front_left'] < 0.40:
            repel_l = -0.5

        l_vel, r_vel = self._vel_from_vw(v, omega)
        self._drive(l_vel + repel_l, r_vel + repel_r)
        return False

    # ── Obstacle avoidance (interrupt layer) ──────────────────────────

    def _check_avoid(self, obs):
        """
        If a front sector is within DANGER_DIST: activate avoidance.
        Returns True while avoidance is active (caller skips normal code).

        FIX #7: The original used a countdown timer (avoid_t) and only
        cleared avoidance when BOTH front_clear AND avoid_t <= 0.  This
        meant the robot always rotated for the full AVOID_STEPS count even
        if the obstacle cleared after 2 steps, wasting time and corrupting
        heading.

        New logic: track elapsed steps (_avoid_steps).  Clear as soon as
        front is clear AND at least AVOID_MIN_STEPS (= 10) have elapsed.
        This allows early exit while still preventing single-step glitches.
        """
        in_danger = (obs['front']       < Config.DANGER_DIST or
                     obs['front_left']  < Config.DANGER_DIST or
                     obs['front_right'] < Config.DANGER_DIST)

        if not self._avoid_on and in_danger:
            if obs['left'] > obs['right'] + 0.1:
                self._avoid_dir = 1
            elif obs['right'] > obs['left'] + 0.1:
                self._avoid_dir = -1
            else:
                self._avoid_dir = random.choice([-1, 1])

            self._avoid_on    = True
            self._avoid_steps = 0
            self._avoid_back  = 6
            print(f"  [AVOID] obstacle — front:{obs['front']:.2f} m")

        if self._avoid_on:
            self._avoid_steps += 1

            if self._avoid_back > 0:
                self._drive(-Config.SPD_TURN * Config.WHEEL_RADIUS / Config.WHEEL_RADIUS,
                             -Config.SPD_TURN * Config.WHEEL_RADIUS / Config.WHEEL_RADIUS)
                # Simplified: brief backup at fixed slow speed
                self._drive(-1.0, -1.0)
                self._avoid_back -= 1
            else:
                spd = Config.SPD_TURN * self._avoid_dir
                self._drive(spd, -spd)

            front_clear = (obs['front']       > Config.WARN_DIST and
                           obs['front_left']  > Config.WARN_DIST and
                           obs['front_right'] > Config.WARN_DIST)

            # FIX #7: clear as soon as path is open + min steps elapsed
            if front_clear and self._avoid_steps >= Config.AVOID_MIN_STEPS:
                self._avoid_on    = False
                self._avoid_steps = 0
                print("  [AVOID] cleared — replanning")
                self._set_goal(self._goal_wx, self._goal_wy,
                               explore=(self._state == self.EXPLORE))
            return True

        return False

    # ── EXPLORE ───────────────────────────────────────────────────────

    def _run_explore(self, obs):
        self._rescan_t += 1
        if self._rescan_t >= Config.RESCAN_INT or not self._frontiers:
            self._frontiers = self._grid.find_frontiers()
            self._rescan_t  = 0

            if not self._frontiers:
                print("  [EXPLORE] all frontiers exhausted → PLAN")
                self._state = self.PLAN
                self._stop()
                return

            ox, oy = self._odom.x, self._odom.y
            self._frontiers.sort(key=lambda f: math.hypot(f[0]-ox, f[1]-oy))
            self._front_idx = 0
            fx, fy, sz = self._frontiers[0]
            self._set_goal(fx, fy, explore=True)
            print(f"  [EXPLORE] → frontier ({fx:+.2f},{fy:+.2f})  "
                  f"{len(self._frontiers)} clusters  size={sz}")

        # Stuck detection
        self._stuck_t += 1
        if self._stuck_t >= Config.STUCK_CHECK:
            moved = math.hypot(self._odom.x - self._stuck_ref[0],
                               self._odom.y - self._stuck_ref[1])
            self._stuck_ref = (self._odom.x, self._odom.y)
            self._stuck_t   = 0
            if moved < Config.STUCK_THRESH:
                self._front_idx = (self._front_idx + 1) % max(1, len(self._frontiers))
                if self._frontiers:
                    fx, fy, _ = self._frontiers[self._front_idx]
                    self._set_goal(fx, fy, explore=True)
                    print(f"  [EXPLORE] stuck — skip to frontier {self._front_idx}")
                else:
                    self._state = self.PLAN
                    return

        reached = self._nav_step(obs)
        if reached:
            self._rescan_t = Config.RESCAN_INT   # force rescan on next tick

    # ── PLAN ──────────────────────────────────────────────────────────

    def _run_plan(self):
        print("  [PLAN] computing coverage waypoints ...")
        self._sweep_plan = self._grid.plan_coverage()
        self._sweep_idx  = 0
        n = len(self._sweep_plan)
        print(f"  [PLAN] {n} waypoints  "
              f"(approx {n * Config.COV_STRIDE * Config.CELL_M:.1f} m of sweep track)")
        if n == 0:
            print("  [PLAN] nothing to sweep → DONE")
            self._state = self.DONE
        else:
            wx, wy = self._sweep_plan[0]
            self._set_goal(wx, wy, explore=False)
            self._state = self.SWEEP

    # ── SWEEP ─────────────────────────────────────────────────────────

    def _run_sweep(self, obs):
        if self._sweep_idx >= len(self._sweep_plan):
            print("  [SWEEP] coverage complete → DONE")
            self._state = self.DONE
            self._stop()
            return

        reached = self._nav_step(obs)

        if reached:
            gx, gy = self._grid.w2g(self._odom.x, self._odom.y)
            self._grid.set_swept(gx, gy)
            for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
                self._grid.set_swept(gx+dx, gy+dy)

            self._sweep_idx += 1
            if self._sweep_idx < len(self._sweep_plan):
                nx, ny = self._sweep_plan[self._sweep_idx]
                self._set_goal(nx, ny, explore=False)

            if self._sweep_idx % 25 == 0:
                pct = 100 * self._sweep_idx // len(self._sweep_plan)
                print(f"  [SWEEP] {self._sweep_idx}/{len(self._sweep_plan)}  {pct}%")

    # ── Main loop ─────────────────────────────────────────────────────

    def run(self):
        ts = Config.TIME_STEP

        while self._robot.step(ts) != -1:
            self._step += 1
            s = self._step

            # 1. Odometry
            self._odom.update()

            # 2. LiDAR sectors  (obstacle avoidance, every step)
            obs = self._lidar.read_sectors()

            # 3. Occupancy grid update  (every MAP_INTERVAL steps)
            self._map_t += 1
            if self._map_t >= Config.MAP_INTERVAL:
                self._lidar.update_map(self._grid, self._odom)
                self._map_t = 0

            # 4. Camera  (every 60 steps)
            self._cam_t += 1
            if self._cam_t >= 60:
                self._cam_t = 0
                if self._cam.is_floor_dirty():
                    print(f"  [CAM] dirty floor near "
                          f"({self._odom.x:+.2f},{self._odom.y:+.2f})")

            # 5. Obstacle avoidance interrupt
            if self._state in (self.EXPLORE, self.SWEEP):
                if self._check_avoid(obs):
                    continue

            # 6. State machine
            if   self._state == self.EXPLORE: self._run_explore(obs)
            elif self._state == self.PLAN:    self._run_plan()
            elif self._state == self.SWEEP:   self._run_sweep(obs)
            elif self._state == self.DONE:    self._stop()

            # 7. Periodic status log  (every 100 steps)
            if s % 100 == 0:
                sim_t       = self._robot.getTime()
                frontiers_n = len(self._frontiers)
                print(
                    f"[{sim_t:6.1f}s]  "
                    f"State:{self._state:<8}  "
                    f"Pos:({self._odom.x:+.2f},{self._odom.y:+.2f})  "
                    f"θ:{math.degrees(self._odom.theta):+5.1f}deg  "
                    f"F:{obs['front']:.2f}  "
                    f"L:{obs['left']:.2f}  "
                    f"R:{obs['right']:.2f}  "
                    f"Fronts:{frontiers_n:3d}  "
                    f"Swept:{self._sweep_idx:4d}"
                )


# ═══════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════
if __name__ == '__main__':
    RobotBrain().run()
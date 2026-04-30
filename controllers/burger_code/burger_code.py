"""
AMR Hospital Sanitizer Controller (TurtleBot3 Burger, Webots)
Phases: EXPLORE -> PLAN -> SWEEP -> RETURN
        (RESCUE is a side-trip that can interrupt EXPLORE and resume it afterwards)

"""

from collections import deque
import heapq
import math
import random

from controller import Supervisor, Camera


# ============================================================================
# Config
# ============================================================================

class Config:
    # ── Geometry ─────────────────────────────────────────────────────────────
    WHEEL_RADIUS = 0.033
    TRACK_WIDTH  = 0.160
    MAX_SPEED    = 6.67

    # ── Speeds / control ─────────────────────────────────────────────────────
    SPD_FORWARD = 0.18   # m/s  cruise speed
    SPD_SLOW    = 0.07   # m/s  recovery backup speed
    SPD_TURN    = 3.1    # rad/s  in-place turn wheel speed
    K_ANG       = 5.0    # proportional heading gain

    # ── Obstacle thresholds (m) ───────────────────────────────────────────────
    DANGER_DIST = 0.28
    WARN_DIST   = 0.45

    # ── Robot footprint ───────────────────────────────────────────────────────
    ROBOT_RADIUS_M  = 0.09
    SAFETY_MARGIN_M = 0.02
    CLEARANCE_CELLS = 0   # 0 → auto from (ROBOT_RADIUS_M + SAFETY_MARGIN_M) / CELL_M

    # ── Occupancy grid ────────────────────────────────────────────────────────
    CELL_M  = 0.10
    GRID_N  = 500
    GRID_OX = 250
    GRID_OY = 250

    # ── Localisation ─────────────────────────────────────────────────────────
    USE_SUPERVISOR_POSE_SYNC = True

    # ── LiDAR map update ──────────────────────────────────────────────────────
    MAP_LAYER_IDS            = (5, 7, 9)
    MAP_H_STEP               = 8
    MAP_INTERVAL             = 3
    MAX_RANGE                = 5.5
    LIDAR_FORWARD_OFFSET_DEG = None   # None → auto (LDS=0°, Velodyne=180°)

    # ── Obstacle sectors ──────────────────────────────────────────────────────
    SEC_LAYER_MIN = 2
    SEC_LAYER_MAX = 13
    SECTOR_DEFS = {
        "front":       [(340, 359), (0, 20)],
        "front_left":  [(21,  60)],
        "front_right": [(300, 339)],
        "left":        [(61,  119)],
        "right":       [(240, 299)],
    }

    # ── Navigation ────────────────────────────────────────────────────────────
    GOAL_TOL         = 0.18
    HEAD_TOL         = 0.15
    HOME_TOL         = 0.12
    UNKNOWN_COST     = 1.5
    NAV_REPLAN_INT   = 18
    NAV_STUCK_STEPS  = 120
    NAV_MIN_PROGRESS = 0.07

    # ── Frontier exploration ──────────────────────────────────────────────────
    FRONT_MIN               = 3
    RESCAN_INT              = 120
    STUCK_CHECK             = 90
    STUCK_THRESH            = 0.15
    FRONTIER_USE_NAV        = True
    FRONTIER_TARGET_TIMEOUT = 240

    FRONTIER_MIN_ADVANCE_M  = 0.50

    # ── Reactive wander) ───────────────────────────────────────────────
    EXPLORE_MIN_STEPS  = 1200
    EXPLORE_MAX_STEPS  = 12000
    EXPLORE_CHECK_INT  = 120
    EXPLORE_MIN_GAIN   = 25
    EXPLORE_IDLE_LIMIT = 8
    EXPLORE_FRONTIER_STOP_MAX = 2
    REACTIVE_BIAS_GAIN = 1.6
    WANDER_OMEGA_MAX   = 0.40   # FIX 1 – was 0.90
    WANDER_HOLD_MIN    = 20
    WANDER_HOLD_MAX    = 35     # FIX 1 – was 80

    # idle intervals where map gain is zero and all frontiers are failed
    FAILED_TARGET_FORCE_CLEAR_IDLE = 3

    # ── Rescue ────────────────────────────────────────────────────────────────
    # Hard-coded safe point the robot drives to when it detects it is stuck in
    # a furniture-heavy area (too many consecutive EXPLORE recoveries).
    RESCUE_WX                  = -1.51392
    RESCUE_WY                  =  0.746655
    RESCUE_TRIGGER_RECOVERIES  =  2   # recoveries before rescue is triggered
    # Rescue counter resets whenever the robot travels STUCK_THRESH metres
    # between consecutive stuck checks, indicating good forward progress.

    # ── Robust sweep fallbacks ────────────────────────────────────────────────
    TRACE_SAMPLE_STEP        = 8
    TRACE_MIN_DIST_M         = 0.20
    EXPLORE_TRACE_MIN_STEP_M = 0.12
    PLAN_MIN_WAYPOINTS       = 8
    PLAN_RESEED_RADIUS       = 2
    SWEEP_TRACE_POINT_LIMIT  = EXPLORE_MAX_STEPS

    # ── Coverage ──────────────────────────────────────────────────────────────
    COV_STRIDE              = 1
    SWEEP_MARK_RADIUS_CELLS = 2
    ENABLE_DIRTY_DETECTION  = False

    # ── Camera ────────────────────────────────────────────────────────────────
    ENABLE_CAMERA_SEMANTICS = True
    CAMERA_RECOG_INTERVAL   = 8
    PERSON_AVOID_DIST       = 0.85
    PERSON_HOLD_STEPS       = 16
    SEMANTIC_MARK_MAX_DIST  = 3.0
    SEMANTIC_BLOCK_MAP      = False

    # ── Odometry fusion ───────────────────────────────────────────────────────
    GYRO_WEIGHT   = 0.70
    ENC_WEIGHT    = 0.30
    COMPASS_ALPHA = 0.015

    # ── Avoidance / recovery ──────────────────────────────────────────────────
    AVOID_MIN_STEPS          = 10
    RECOVERY_BACKUP_M        = 0.12
    RECOVERY_TURN_MIN_DEG    = 40.0
    RECOVERY_TURN_MAX_DEG    = 140.0
    RECOVERY_COOLDOWN_STEPS  = 100

    # ── Diagnostics ───────────────────────────────────────────────────────────
    SELF_CHECK_INTERVAL = 120

    # ── Grid cell states ──────────────────────────────────────────────────────
    UNKNOWN  = 0
    FREE     = 1
    OCCUPIED = 2
    SWEPT    = 3


# ============================================================================
# Utility functions
# ============================================================================

def _wrap(a):
    """Wrap angle to (-π, π]."""
    return math.atan2(math.sin(a), math.cos(a))


def _compress_grid_path(path):
    """Remove collinear intermediate waypoints from an A* path."""
    if not path or len(path) <= 2:
        return path
    reduced  = [path[0]]
    prev     = path[0]
    cur      = path[1]
    prev_dir = (cur[0] - prev[0], cur[1] - prev[1])
    for i in range(2, len(path)):
        nxt     = path[i]
        cur_dir = (nxt[0] - cur[0], nxt[1] - cur[1])
        if cur_dir != prev_dir:
            reduced.append(cur)
            prev_dir = cur_dir
        prev = cur
        cur  = nxt
    reduced.append(path[-1])
    return reduced


def _yaw_from_axis_angle(rotation):
    """Convert Webots axis-angle rotation to yaw (rad)."""
    ax, ay, az, ang = rotation
    half = 0.5 * ang
    s    = math.sin(half)
    qw   = math.cos(half)
    qx, qy, qz = ax * s, ay * s, az * s
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


# ============================================================================
# OccupancyGrid
# ============================================================================

class OccupancyGrid:
    def __init__(self):
        self._N = Config.GRID_N
        self._g = bytearray(self._N * self._N)

        # Pre-compute clearance offsets (robot footprint inflation)
        self._clear_offsets = []
        r = Config.CLEARANCE_CELLS
        if r <= 0:
            r = max(1, int(round(
                (Config.ROBOT_RADIUS_M + Config.SAFETY_MARGIN_M) / Config.CELL_M
            )))
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy <= r * r:
                    self._clear_offsets.append((dx, dy))

    # ── Coordinate transforms ─────────────────────────────────────────────────
    @staticmethod
    def w2g(wx, wy):
        return (
            int(round(wx / Config.CELL_M)) + Config.GRID_OX,
            int(round(wy / Config.CELL_M)) + Config.GRID_OY,
        )

    @staticmethod
    def g2w(gx, gy):
        return (
            (gx - Config.GRID_OX) * Config.CELL_M,
            (gy - Config.GRID_OY) * Config.CELL_M,
        )

    def in_bounds(self, gx, gy):
        return 0 <= gx < self._N and 0 <= gy < self._N

    def _idx(self, gx, gy):
        return gy * self._N + gx

    def _set(self, gx, gy, value):
        if self.in_bounds(gx, gy):
            self._g[self._idx(gx, gy)] = value

    def get(self, gx, gy):
        if not self.in_bounds(gx, gy):
            return Config.OCCUPIED
        return self._g[self._idx(gx, gy)]

    # ── Navigability ─────────────────────────────────────────────────────────
    def is_navigable(self, gx, gy, allow_unknown):
        if not self.in_bounds(gx, gy):
            return False
        center = self.get(gx, gy)
        if center == Config.OCCUPIED:
            return False
        if not allow_unknown and center == Config.UNKNOWN:
            return False
        for dx, dy in self._clear_offsets:
            nx, ny = gx + dx, gy + dy
            if not self.in_bounds(nx, ny):
                return False
            if self.get(nx, ny) == Config.OCCUPIED:
                return False
        return True

    def nearest_navigable(self, gx, gy, allow_unknown, max_radius=8):
        if self.is_navigable(gx, gy, allow_unknown):
            return gx, gy
        for radius in range(1, max_radius + 1):
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if abs(dx) != radius and abs(dy) != radius:
                        continue
                    nx, ny = gx + dx, gy + dy
                    if self.is_navigable(nx, ny, allow_unknown):
                        return nx, ny
        return None

    # ── Cell state writers ────────────────────────────────────────────────────
    def seed_free_zone(self, wx, wy, radius_cells=3):
        cx, cy = self.w2g(wx, wy)
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                self._set(cx + dx, cy + dy, Config.FREE)

    def set_free(self, gx, gy):
        if self.in_bounds(gx, gy):
            i = self._idx(gx, gy)
            if self._g[i] == Config.UNKNOWN:
                self._g[i] = Config.FREE

    def set_occupied(self, gx, gy):
        if self.in_bounds(gx, gy):
            self._g[self._idx(gx, gy)] = Config.OCCUPIED

    def set_swept(self, gx, gy):
        if self.in_bounds(gx, gy):
            i = self._idx(gx, gy)
            if self._g[i] == Config.FREE:
                self._g[i] = Config.SWEPT

    # ── Ray casting ───────────────────────────────────────────────────────────
    @staticmethod
    def _bresenham(x0, y0, x1, y1):
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        while True:
            yield x0, y0
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy; x0 += sx
            if e2 <  dx:
                err += dx; y0 += sy

    def update_ray(self, robot_wx, robot_wy, angle_world, dist):
        if not math.isinf(dist) and dist <= 0.05:
            return
        is_hit  = (not math.isinf(dist)) and (dist < Config.MAX_RANGE)
        ray_len = min(
            dist if not math.isinf(dist) else Config.MAX_RANGE,
            Config.MAX_RANGE,
        )
        rx, ry = self.w2g(robot_wx, robot_wy)
        ex_w   = robot_wx + ray_len * math.cos(angle_world)
        ey_w   = robot_wy + ray_len * math.sin(angle_world)
        ex, ey = self.w2g(ex_w, ey_w)
        for cx, cy in self._bresenham(rx, ry, ex, ey):
            if cx == ex and cy == ey:
                break
            self.set_free(cx, cy)
        if is_hit:
            self.set_occupied(ex, ey)
        else:
            self.set_free(ex, ey)

    # ── Frontier detection ────────────────────────────────────────────────────
    def find_frontiers(self):
        N        = self._N
        g        = self._g
        walkable = (Config.FREE, Config.SWEPT)
        U        = Config.UNKNOWN
        candidates = set()
        for gy in range(1, N - 1):
            row = gy * N
            for gx in range(1, N - 1):
                if g[row + gx] not in walkable:
                    continue
                if (
                    g[(gy - 1) * N + gx] == U
                    or g[(gy + 1) * N + gx] == U
                    or g[row + gx - 1]     == U
                    or g[row + gx + 1]     == U
                ):
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
                for dx, dy in (
                    (1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,1),(1,-1),(-1,-1)
                ):
                    nb = (cx + dx, cy + dy)
                    if nb in remaining and nb not in seen:
                        seen.add(nb); q.append(nb)
            if len(group) >= Config.FRONT_MIN:
                sx = sum(p[0] for p in group)
                sy = sum(p[1] for p in group)
                cgx, cgy = int(sx / len(group)), int(sy / len(group))
                # Use the cluster cell nearest to the centroid (not the centroid
                # itself, which can fall on unreachable grid positions)
                rep_gx, rep_gy = min(
                    group,
                    key=lambda c: (abs(c[0]-cgx) + abs(c[1]-cgy), c[0], c[1]),
                )
                wx, wy = self.g2w(rep_gx, rep_gy)
                clusters.append((wx, wy, len(group)))
        return clusters

    # ── A* path planning ──────────────────────────────────────────────────────
    def bfs_path(self, sx, sy, gx, gy, allow_unknown=True):
        start = self.nearest_navigable(sx, sy, allow_unknown, max_radius=8)
        goal  = self.nearest_navigable(gx, gy, allow_unknown, max_radius=20)
        if start is None or goal is None:
            return None
        sx, sy = start
        gx, gy = goal

        open_heap = [(0.0, 0.0, sx, sy)]
        came      = {(sx, sy): None}
        g_cost    = {(sx, sy): 0.0}
        closed    = set()

        while open_heap:
            _, cur_g, cx, cy = heapq.heappop(open_heap)
            if (cx, cy) in closed:
                continue
            closed.add((cx, cy))
            if cx == gx and cy == gy:
                break
            for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
                nx, ny = cx + dx, cy + dy
                if (nx, ny) in closed:
                    continue
                if not self.is_navigable(nx, ny, allow_unknown):
                    continue
                step_cost = 1.0
                if self.get(nx, ny) == Config.UNKNOWN:
                    step_cost += Config.UNKNOWN_COST
                new_g = cur_g + step_cost
                if new_g >= g_cost.get((nx, ny), float("inf")):
                    continue
                g_cost[(nx, ny)] = new_g
                came[(nx, ny)]   = (cx, cy)
                h = abs(gx - nx) + abs(gy - ny)
                heapq.heappush(open_heap, (new_g + h, new_g, nx, ny))

        if (gx, gy) not in came:
            return None
        path = []
        node = (gx, gy)
        while node is not None:
            path.append(node)
            node = came[node]
        path.reverse()
        return path

    # ── Coverage planning ─────────────────────────────────────────────────────
    def known_cell_count(self):
        return sum(1 for v in self._g if v != Config.UNKNOWN)

    def reachable_walkable(self, start_wx, start_wy):
        start_gx, start_gy = self.w2g(start_wx, start_wy)
        start = self.nearest_navigable(
            start_gx, start_gy, allow_unknown=False, max_radius=10
        )
        if start is None:
            return set()
        walkable = {Config.FREE, Config.SWEPT}
        q    = deque([start])
        seen = {start}
        while q:
            cx, cy = q.popleft()
            for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
                nx, ny = cx + dx, cy + dy
                if (nx, ny) in seen or not self.in_bounds(nx, ny):
                    continue
                if self.get(nx, ny) not in walkable:
                    continue
                seen.add((nx, ny)); q.append((nx, ny))
        return seen

    def plan_coverage(self, start_wx=None, start_wy=None):
        if start_wx is None or start_wy is None:
            start_wx, start_wy = 0.0, 0.0
        reachable = self.reachable_walkable(start_wx, start_wy)
        if not reachable:
            return []

        rows = {}
        for gx, gy in reachable:
            if self.is_navigable(gx, gy, allow_unknown=False):
                rows.setdefault(gy, []).append(gx)
        if not rows:
            return []

        row_keys = sorted(rows.keys())[::max(1, Config.COV_STRIDE)]
        if not row_keys:
            return []

        min_gy = row_keys[0]; max_gy = row_keys[-1]
        min_gx = min(min(xs) for xs in rows.values())
        max_gx = max(max(xs) for xs in rows.values())

        start_gx, start_gy = self.w2g(start_wx, start_wy)
        mid_gy = (min_gy + max_gy) // 2
        scan_rows = row_keys if start_gy <= mid_gy else list(reversed(row_keys))
        direction = 1 if start_gx <= (min_gx + max_gx) // 2 else -1

        sweep_cells = []
        for gy in scan_rows:
            xs = sorted(rows.get(gy, []))
            if not xs:
                continue
            segments = []
            seg_start = xs[0]; prev = xs[0]
            for x in xs[1:]:
                if x == prev + 1:
                    prev = x; continue
                segments.append((seg_start, prev, gy))
                seg_start = x; prev = x
            segments.append((seg_start, prev, gy))

            for seg_start, seg_end, gy in (segments if direction > 0 else list(reversed(segments))):
                entry = (seg_start, gy) if direction > 0 else (seg_end,   gy)
                exit_ = (seg_end,   gy) if direction > 0 else (seg_start, gy)
                if not sweep_cells or sweep_cells[-1] != entry:
                    sweep_cells.append(entry)
                if sweep_cells[-1] != exit_:
                    sweep_cells.append(exit_)
            direction *= -1

        return [self.g2w(gx, gy) for gx, gy in sweep_cells]

    def mark_clean_local(self, wx, wy, radius_cells):
        cx, cy = self.w2g(wx, wy)
        if not self.in_bounds(cx, cy) or self.get(cx, cy) == Config.OCCUPIED:
            return
        r  = max(1, int(radius_cells))
        r2 = r * r
        q    = deque([(cx, cy)])
        seen = {(cx, cy)}
        while q:
            gx, gy = q.popleft()
            if (gx-cx)**2 + (gy-cy)**2 > r2:
                continue
            if self.get(gx, gy) == Config.OCCUPIED:
                continue
            self._set(gx, gy, Config.SWEPT)
            for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
                nx, ny = gx + dx, gy + dy
                if (nx, ny) in seen or not self.in_bounds(nx, ny):
                    continue
                if (nx-cx)**2 + (ny-cy)**2 > r2:
                    continue
                if self.get(nx, ny) == Config.OCCUPIED:
                    continue
                seen.add((nx, ny)); q.append((nx, ny))


# ============================================================================
# Odometry
# ============================================================================

class Odometry:
    def __init__(self, left_enc, right_enc, gyro, compass, dt_s):
        self._le = left_enc; self._re = right_enc
        self._gy = gyro;     self._cp = compass
        self._dt = dt_s;     self._inited = False
        self.x = 0.0; self.y = 0.0; self.theta = 0.0
        self._pl = 0.0; self._pr = 0.0

    def set_pose(self, x, y, theta):
        self.x = float(x); self.y = float(y); self.theta = _wrap(theta)

    def update(self):
        l_pos = self._le.getValue()
        r_pos = self._re.getValue()
        if not self._inited:
            self._pl = l_pos; self._pr = r_pos; self._inited = True; return

        dl = (l_pos - self._pl) * Config.WHEEL_RADIUS
        dr = (r_pos - self._pr) * Config.WHEEL_RADIUS
        self._pl = l_pos; self._pr = r_pos

        max_d = Config.MAX_SPEED * Config.WHEEL_RADIUS * self._dt * 1.5
        dl = max(-max_d, min(max_d, dl))
        dr = max(-max_d, min(max_d, dr))

        d_theta = (
            Config.GYRO_WEIGHT * self._gy.getValues()[1] * self._dt
            + Config.ENC_WEIGHT * (dr - dl) / Config.TRACK_WIDTH
        )
        d_dist = 0.5 * (dl + dr)
        mid    = self.theta + 0.5 * d_theta
        self.x     += d_dist * math.cos(mid)
        self.y     += d_dist * math.sin(mid)
        self.theta  = _wrap(self.theta + d_theta)

        cv = self._cp.getValues()
        self.theta = _wrap(
            self.theta + Config.COMPASS_ALPHA * _wrap(math.atan2(cv[0], cv[2]) - self.theta)
        )


# ============================================================================
# LidarInterface
# ============================================================================

class LidarInterface:
    def __init__(self, device, device_name=""):
        self._dev = device
        self._H   = int(device.getHorizontalResolution())
        self._L   = int(device.getNumberOfLayers())

        lname = (device_name or "").lower()
        if Config.LIDAR_FORWARD_OFFSET_DEG is not None:
            offset_deg = float(Config.LIDAR_FORWARD_OFFSET_DEG)
        elif "lds" in lname:
            offset_deg = 0.0
        else:
            offset_deg = 180.0
        self._ray0_offset = (
            int(round((offset_deg % 360.0) * self._H / 360.0)) % max(1, self._H)
        )

        if self._H <= 0 or self._L <= 0:
            raise RuntimeError(
                f"Invalid lidar resolution/layers: H={self._H}, L={self._L}"
            )

        self._map_layers = [l for l in Config.MAP_LAYER_IDS if 0 <= l < self._L]
        if not self._map_layers:
            self._map_layers = [self._L // 2]
            print(f"  [LIDAR] fallback to layer {self._map_layers[0]} ({self._L} total)")

        self._sector_idx = self._build_sector_indices()

    @staticmethod
    def _valid(r):
        return 0.05 < r < 20.0 and not math.isinf(r) and not math.isnan(r)

    def _build_sector_indices(self):
        built = {}
        pts   = max(1, self._H // 360)
        for name, ranges in Config.SECTOR_DEFS.items():
            idxs = []
            for sd, ed in ranges:
                start = (sd * pts + self._ray0_offset) % self._H
                h = start
                for _ in range((ed - sd + 1) * pts):
                    idxs.append(h); h = (h + 1) % self._H
            built[name] = idxs
        return built

    def read_sectors(self):
        ranges = self._dev.getRangeImage()
        lo = min(max(0, Config.SEC_LAYER_MIN), self._L - 1)
        hi = min(max(lo, Config.SEC_LAYER_MAX), self._L - 1)
        result = {}
        for name, idxs in self._sector_idx.items():
            best = float("inf")
            for layer in range(lo, hi + 1):
                base = layer * self._H
                for h in idxs:
                    r = ranges[base + h]
                    if self._valid(r) and r < best:
                        best = r
            result[name] = best
        return result

    def update_map(self, grid, odom):
        ranges = self._dev.getRangeImage()
        tau    = 2 * math.pi
        for layer in self._map_layers:
            base = layer * self._H
            for h in range(0, self._H, Config.MAP_H_STEP):
                r = ranges[base + h]
                if not math.isinf(r) and not self._valid(r):
                    continue
                angle_robot = (h - self._ray0_offset) * (tau / self._H)
                grid.update_ray(odom.x, odom.y, odom.theta + angle_robot, r)


# ============================================================================
# CameraProcessor
# ============================================================================

class CameraProcessor:
    def __init__(self, device, ts):
        self._dev = device
        self._w   = device.getWidth()
        self._h   = device.getHeight()
        try:
            self._fov = float(device.getFov())
        except Exception:
            self._fov = 1.096752

        self._recognition_on = False
        if Config.ENABLE_CAMERA_SEMANTICS and hasattr(device, "recognitionEnable"):
            try:
                device.recognitionEnable(ts)
                self._recognition_on = True
            except Exception:
                pass

    @staticmethod
    def _classify(label):
        s = (label or "").lower()
        if not s:
            return "unknown"
        if any(w in s for w in ("pedestrian", "person", "human")):
            return "person"
        if any(w in s for w in ("wall", "door")):
            return "wall"
        if any(w in s for w in ("bed","sofa","toilet","chair","desk","table","furniture")):
            return "furniture"
        return "unknown"

    @staticmethod
    def _safe_call(obj, method_name, default):
        fn = getattr(obj, method_name, None)
        if fn is None:
            return default
        try:
            return fn()
        except Exception:
            return default

    def scan_semantics(self):
        if not self._recognition_on:
            return []
        detections = []
        half_w = max(1.0, 0.5 * self._w)
        for obj in self._dev.getRecognitionObjects():
            model    = self._safe_call(obj, "get_model", "")
            category = self._classify(model)
            if category == "unknown":
                continue
            px = 0.5 * self._w
            pos_img = self._safe_call(obj, "get_position_on_image", None)
            if pos_img is not None and len(pos_img) >= 1:
                px = float(pos_img[0])
            nx      = max(-1.0, min(1.0, (px - half_w) / half_w))
            bearing = nx * (0.5 * self._fov)
            size_img   = self._safe_call(obj, "get_size_on_image", None)
            size_score = 0.0
            if size_img is not None and len(size_img) >= 2:
                size_score = float(size_img[0]) * float(size_img[1]) / max(1.0, float(self._w * self._h))
            pos      = self._safe_call(obj, "get_position", None)
            distance = None
            if pos is not None and len(pos) >= 3:
                distance = math.sqrt(float(pos[0])**2 + float(pos[2])**2)
            detections.append({
                "label": model, "category": category,
                "bearing": bearing, "distance": distance, "size_score": size_score,
            })
        return detections

    def is_floor_dirty(self):
        img = self._dev.getImage()
        if not img:
            return False
        w, h    = self._w, self._h
        start_y = (2 * h) // 3
        dark = total = 0
        for y in range(start_y, h, 8):
            for x in range(0, w, 8):
                r   = Camera.imageGetRed(img,   w, x, y)
                g   = Camera.imageGetGreen(img,  w, x, y)
                b   = Camera.imageGetBlue(img,   w, x, y)
                lum = 0.299 * r + 0.587 * g + 0.114 * b
                total += 1
                if lum < 60:
                    dark += 1
        return total > 0 and (dark / total) > 0.15


# ============================================================================
# RobotBrain
# ============================================================================

class RobotBrain:
    EXPLORE = "EXPLORE"
    PLAN    = "PLAN"
    SWEEP   = "SWEEP"
    RETURN  = "RETURN"
    RESCUE  = "RESCUE"   # ← new state: navigate to safe point, then resume EXPLORE
    DONE    = "DONE"

    # ── Device helpers ────────────────────────────────────────────────────────
    def _try_get_device(self, names):
        for name in names:
            try:
                dev = self._robot.getDevice(name)
                if dev is not None:
                    return dev, name
            except Exception:
                pass
        return None, None

    # ── Pose initialisation ───────────────────────────────────────────────────
    def _read_initial_pose(self):
        if self._tr_field is not None and self._rot_field is not None:
            tr  = self._tr_field.getSFVec3f()
            rot = self._rot_field.getSFRotation()
            self._home_webots_xyz = (tr[0], tr[1], tr[2])
            return tr[0], tr[1], _yaw_from_axis_angle(rot)

        print(
            "  [INIT] WARNING: Supervisor pose unavailable — home set to "
            "odometry frame origin (0,0,0).  Enable USE_SUPERVISOR_POSE_SYNC "
            "for world-frame homing."
        )
        self._home_webots_xyz = (0.0, 0.0, 0.0)
        return 0.0, 0.0, 0.0

    def _get_pose_fields(self):
        tr_field = rot_field = None
        try:
            if not bool(self._robot.getSupervisor()):
                return None, None
            self_node = self._robot.getSelf()
            if self_node is not None:
                tr_field  = self_node.getField("translation")
                rot_field = self_node.getField("rotation")
        except Exception:
            pass
        return tr_field, rot_field

    def _sync_pose_with_supervisor(self):
        if not self._use_supervisor_pose:
            return
        try:
            tr  = self._tr_field.getSFVec3f()
            rot = self._rot_field.getSFRotation()
            # Fix here as well
            self._odom.set_pose(tr[0], tr[1], _yaw_from_axis_angle(rot))
        except Exception:
            pass

    # ── Self-check ────────────────────────────────────────────────────────────
    def _run_self_check(self):
        gx, gy = self._grid.w2g(self._odom.x, self._odom.y)
        checks = [
            ("wheel geometry",  Config.WHEEL_RADIUS > 0.0 and Config.TRACK_WIDTH > 0.0),
            ("grid geometry",   Config.CELL_M > 0.0 and Config.GRID_N > 20),
            ("clearance model", len(self._grid._clear_offsets) > 0),
            ("coverage stride", Config.COV_STRIDE >= 1),
            ("origin pose",     math.isfinite(self._home_wx) and math.isfinite(self._home_wy)),
            ("pose-in-grid",    self._grid.in_bounds(gx, gy)),
            ("rescue point",    math.isfinite(Config.RESCUE_WX) and math.isfinite(Config.RESCUE_WY)),
        ]
        failed = [n for n, ok in checks if not ok]
        if failed:
            print(f"  [SELF-CHECK] FAILED: {', '.join(failed)}")
        else:
            print("  [SELF-CHECK] passed")
        src = "supervisor" if self._use_supervisor_pose else "odometry"
        wx, wy, wz = getattr(self, "_home_webots_xyz", (self._home_wx, 0.0, self._home_wy))
        print(
            f"  [ORIGIN] Webots XYZ=({wx:+.5f},{wy:+.5f},{wz:+.5f})  "
            f"nav=({self._home_wx:+.5f},{self._home_wy:+.5f})  "
            f"θ={math.degrees(self._home_theta):+.1f}° ({src})"
        )
        print(f"  [RESCUE POINT] nav=({Config.RESCUE_WX:+.5f},{Config.RESCUE_WY:+.5f})")

    # ── Replan / watchdog ─────────────────────────────────────────────────────
    def _runtime_replan_check(self):
        if self._state not in (self.SWEEP, self.RETURN):
            return
        if not self._nav_path or self._nav_idx >= len(self._nav_path):
            return
        if self._step % Config.NAV_REPLAN_INT != 0:
            return
        next_gx, next_gy = self._nav_path[self._nav_idx]
        if self._grid.is_navigable(next_gx, next_gy, allow_unknown=False):
            return
        fallback_unknown = (self._state == self.RETURN)
        if self._set_goal(self._goal_wx, self._goal_wy,
                          explore=False, fallback_unknown=fallback_unknown):
            print("  [CHECK] path became invalid, replanned")

    def _progress_watchdog(self):
        if self._recover_on:
            return
        if self._state not in (self.SWEEP, self.RETURN):
            self._progress_t   = 0
            self._progress_ref = (self._odom.x, self._odom.y)
            return
        if not self._nav_path or self._nav_idx >= len(self._nav_path):
            self._progress_t   = 0
            self._progress_ref = (self._odom.x, self._odom.y)
            return

        self._progress_t += 1
        if self._progress_t < Config.NAV_STUCK_STEPS:
            return

        moved = math.hypot(
            self._odom.x - self._progress_ref[0],
            self._odom.y - self._progress_ref[1],
        )
        self._progress_ref = (self._odom.x, self._odom.y)
        self._progress_t   = 0
        if moved >= Config.NAV_MIN_PROGRESS:
            return

        if self._start_recovery(f"low progress in {self._state} ({moved:.2f} m)"):
            return

        print(f"  [CHECK] low progress ({moved:.2f} m), replanning")
        fallback_unknown = (self._state == self.RETURN)
        if self._set_goal(self._goal_wx, self._goal_wy,
                          explore=False, fallback_unknown=fallback_unknown):
            return

        if self._state == self.SWEEP:
            self._sweep_idx += 1
            while self._sweep_idx < len(self._sweep_plan):
                nx, ny = self._sweep_plan[self._sweep_idx]
                if self._set_goal(nx, ny, explore=False, fallback_unknown=True):
                    print("  [CHECK] skipped blocked sweep waypoint")
                    return
                self._sweep_idx += 1
            self._start_return("watchdog exhausted sweep waypoints")
            return
        self._start_return("watchdog could not maintain return path")

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _mark_swept_footprint(self):
        self._grid.mark_clean_local(
            self._odom.x, self._odom.y, max(1, Config.SWEEP_MARK_RADIUS_CELLS)
        )

    def _sample_explore_trace(self, force=False):
        gx, gy = self._grid.w2g(self._odom.x, self._odom.y)
        if not self._grid.in_bounds(gx, gy):
            return
        if self._grid.get(gx, gy) == Config.OCCUPIED:
            return
        moved = math.hypot(
            self._odom.x - self._last_trace_pose[0],
            self._odom.y - self._last_trace_pose[1],
        )
        cell = (gx, gy)
        if not force and moved < Config.EXPLORE_TRACE_MIN_STEP_M and cell == self._last_trace_cell:
            return
        if self._explore_trace and self._grid.w2g(*self._explore_trace[-1]) == cell:
            self._last_trace_pose = (self._odom.x, self._odom.y)
            self._last_trace_cell = cell
            return
        cell_wx, cell_wy = self._grid.g2w(gx, gy)
        self._explore_trace.append((cell_wx, cell_wy))
        self._explore_trace_cells.add(cell)
        self._last_trace_pose = (self._odom.x, self._odom.y)
        self._last_trace_cell = cell
        if len(self._explore_trace) > Config.SWEEP_TRACE_POINT_LIMIT:
            self._explore_trace = self._explore_trace[-Config.SWEEP_TRACE_POINT_LIMIT:]
            self._explore_trace_cells = {
                self._grid.w2g(wx, wy) for wx, wy in self._explore_trace
            }

    def _bearing_to_sector_range(self, bearing, obs):
        deg = (math.degrees(bearing) + 360.0) % 360.0
        if deg >= 340.0 or deg <= 20.0:  return obs["front"]
        if 20.0  < deg <= 70.0:          return obs["front_left"]
        if 290.0 <= deg < 340.0:         return obs["front_right"]
        if 70.0  < deg < 180.0:          return obs["left"]
        return obs["right"]

    def _update_semantic_model(self, obs):
        if self._cam is None or not Config.ENABLE_CAMERA_SEMANTICS:
            return
        self._cam_t += 1
        if self._cam_t < Config.CAMERA_RECOG_INTERVAL:
            return
        self._cam_t = 0
        detections = self._cam.scan_semantics()
        if not detections:
            self._person_hold = max(0, self._person_hold - 1)
            return
        person_near = False
        seen_cats   = set()
        for det in detections:
            cat     = det["category"]
            bearing = float(det["bearing"])
            dist    = det["distance"]
            self._semantic_counts[cat] = self._semantic_counts.get(cat, 0) + 1
            seen_cats.add(cat)
            if dist is None or not math.isfinite(dist):
                dist = self._bearing_to_sector_range(bearing, obs)
            if cat == "person":
                if math.isfinite(dist) and dist < Config.PERSON_AVOID_DIST:
                    self._person_bearing = bearing
                    person_near = True
                continue
            if cat not in ("wall", "furniture") or not math.isfinite(dist):
                continue
            if not Config.SEMANTIC_BLOCK_MAP:
                continue
            d       = max(0.05, min(float(dist), Config.SEMANTIC_MARK_MAX_DIST))
            world_a = self._odom.theta + bearing
            wx      = self._odom.x + d * math.cos(world_a)
            wy      = self._odom.y + d * math.sin(world_a)
            gx, gy  = self._grid.w2g(wx, wy)
            if self._grid.in_bounds(gx, gy):
                self._grid.set_occupied(gx, gy)
        self._person_hold = Config.PERSON_HOLD_STEPS if person_near else max(0, self._person_hold - 1)
        if seen_cats and self._step % 200 == 0:
            print(f"  [SEM] seen: {', '.join(sorted(seen_cats))}")

    # ── Frontier management ───────────────────────────────────────────────────
    def _refresh_frontiers(self):
        self._frontiers = self._grid.find_frontiers()
        self._frontiers.sort(
            key=lambda f: (
                math.hypot(f[0] - self._odom.x, f[1] - self._odom.y) - 0.03 * f[2]
            )
        )

    def _try_set_frontier_goal(self, min_advance_m=0.0):
        """
        Pick the nearest non-failed frontier and navigate to it.
        """
        if not Config.FRONTIER_USE_NAV or not self._frontiers:
            return False

        for wx, wy, _size in self._frontiers:
            gx, gy = self._grid.w2g(wx, wy)
            if (gx, gy) in self._failed_targets:
                continue

            # FIX 3 – skip clusters too close to the last failed frontier
            if min_advance_m > 0.0 and self._last_failed_frontier_pos is not None:
                if math.hypot(
                    wx - self._last_failed_frontier_pos[0],
                    wy - self._last_failed_frontier_pos[1],
                ) < min_advance_m:
                    continue

            if self._set_goal(wx, wy, explore=True, fallback_unknown=True):
                self._frontier_goal     = (wx, wy)
                self._frontier_goal_age = 0
                print(f"  [FRONTIER] targeting ({wx:.2f},{wy:.2f}) size:{_size}")
                return True

            # Fallback: try navigable cells around this frontier
            fgx, fgy = self._grid.w2g(wx, wy)
            for rad in (1, 2, 4, 6):
                for dx, dy in (
                    (rad,0),(-rad,0),(0,rad),(0,-rad),
                    (rad,rad),(rad,-rad),(-rad,rad),(-rad,-rad)
                ):
                    cgx, cgy = fgx + dx, fgy + dy
                    if not self._grid.is_navigable(cgx, cgy, allow_unknown=True):
                        continue
                    cwx, cwy = self._grid.g2w(cgx, cgy)
                    if self._set_goal(cwx, cwy, explore=True, fallback_unknown=True):
                        self._frontier_goal     = (cwx, cwy)
                        self._frontier_goal_age = 0
                        print(
                            f"  [FRONTIER] nearby cell ({cwx:.2f},{cwy:.2f}) "
                            f"near ({wx:.2f},{wy:.2f})"
                        )
                        return True

            self._failed_targets.add((gx, gy))

        return False

    # ── Recovery ──────────────────────────────────────────────────────────────
    def _start_recovery(self, reason):
        if self._recover_on or self._recover_cooldown > 0:
            return False

        backup_speed = max(0.05, Config.SPD_SLOW)
        back_steps   = int(round(
            Config.RECOVERY_BACKUP_M / max(1e-6, backup_speed * self._dt_s)
        ))
        turn_deg     = random.uniform(
            Config.RECOVERY_TURN_MIN_DEG, Config.RECOVERY_TURN_MAX_DEG
        )
        robot_omega  = (
            2.0 * Config.SPD_TURN * Config.WHEEL_RADIUS / max(1e-6, Config.TRACK_WIDTH)
        )
        turn_steps   = int(round(
            math.radians(turn_deg) / max(1e-6, robot_omega * self._dt_s)
        ))

        self._recover_on         = True
        self._recover_reason     = reason
        self._recover_back_steps = max(3, back_steps)
        self._recover_turn_steps = max(5, turn_steps)
        self._recover_dir = (
            (-1 if self._person_bearing > 0.0 else 1)
            if self._person_hold > 0
            else random.choice([-1, 1])
        )
        self._avoid_on    = False
        self._avoid_steps = 0
        self._avoid_back  = 0

        # ── Rescue counter: only track recoveries that happen during EXPLORE ──
        if self._state == self.EXPLORE:
            self._rescue_recovery_count += 1
            print(
                f"  [RECOVERY] {reason} → reverse+turn  "
                f"(rescue count: {self._rescue_recovery_count}/{Config.RESCUE_TRIGGER_RECOVERIES})"
            )
        else:
            print(f"  [RECOVERY] {reason} → reverse+turn")

        return True

    def _run_recovery(self):
        if not self._recover_on:
            return False

        if self._recover_back_steps > 0:
            lv, rv = self._vel_from_vw(-max(0.05, Config.SPD_SLOW), 0.0)
            self._drive(lv, rv)
            self._recover_back_steps -= 1
            return True

        if self._recover_turn_steps > 0:
            spd = Config.SPD_TURN * self._recover_dir
            self._drive(spd, -spd)
            self._recover_turn_steps -= 1
            return True

        # Recovery finished – clean up and re-set the appropriate goal
        self._recover_on       = False
        self._recover_cooldown = Config.RECOVERY_COOLDOWN_STEPS
        self._wander_hold      = 0

        if self._state in (self.SWEEP, self.RETURN):
            fallback_unknown = (self._state == self.RETURN)
            self._set_goal(self._goal_wx, self._goal_wy,
                           explore=False, fallback_unknown=fallback_unknown)
        elif self._state == self.RESCUE:
            # BUG FIX: re-path to rescue point after recovery in RESCUE state.
            # Previously fell into EXPLORE branch, clearing the rescue nav path.
            self._set_goal(
                Config.RESCUE_WX, Config.RESCUE_WY,
                explore=True, fallback_unknown=True,
            )
        elif self._state == self.EXPLORE:
            self._nav_path      = []
            self._nav_idx       = 0
            self._frontier_goal = None

        print("  [RECOVERY] completed")
        return False

    # ── Coverage sweep trace ──────────────────────────────────────────────────
    def _full_sweep_from_trace(self):
        if not self._explore_trace:
            return []
        seen = set(); waypoints = []
        for wx, wy in reversed(self._explore_trace):
            gx, gy = self._grid.w2g(wx, wy)
            if not self._grid.in_bounds(gx, gy): continue
            if (gx, gy) in seen: continue
            if self._grid.get(gx, gy) == Config.OCCUPIED: continue
            seen.add((gx, gy)); waypoints.append((wx, wy))
            if len(waypoints) >= Config.SWEEP_TRACE_POINT_LIMIT:
                break
        return waypoints

    # ── Init ──────────────────────────────────────────────────────────────────
    def __init__(self):
        self._robot = Supervisor()
        self._ts    = int(self._robot.getBasicTimeStep())
        self._dt_s  = self._ts / 1000.0

        self._lm = self._robot.getDevice("left wheel motor")
        self._rm = self._robot.getDevice("right wheel motor")
        for m in (self._lm, self._rm):
            m.setPosition(float("inf")); m.setVelocity(0.0)

        ls = self._robot.getDevice("left wheel sensor")
        rs = self._robot.getDevice("right wheel sensor")
        ls.enable(self._ts); rs.enable(self._ts)

        gyro,  gyro_name  = self._try_get_device(("gyro","gyro(1)","Gyro","Gyro(1)"))
        comp,  comp_name  = self._try_get_device(("compass","compass(1)","Compass","Compass(1)"))
        accel, accel_name = self._try_get_device(("accelerometer","Accelerometer"))
        if gyro is None or comp is None:
            raise RuntimeError("Missing required IMU sensors (gyro/compass).")
        gyro.enable(self._ts); comp.enable(self._ts)
        if accel is not None:
            accel.enable(self._ts)
            print(f"  [INIT] accelerometer: {accel_name}")
        else:
            print("  [INIT] accelerometer not found")
        self._accel = accel
        print(f"  [INIT] gyro: {gyro_name}, compass: {comp_name}")

        self._tr_field, self._rot_field = self._get_pose_fields()
        self._use_supervisor_pose = (
            Config.USE_SUPERVISOR_POSE_SYNC
            and self._tr_field  is not None
            and self._rot_field is not None
        )
        self._pose_source = "supervisor" if self._use_supervisor_pose else "odometry"
        if not self._use_supervisor_pose and Config.USE_SUPERVISOR_POSE_SYNC:
            print("  [INIT] supervisor pose unavailable → falling back to odometry")

        lidar_dev, lidar_name = self._try_get_device(
            ("Velodyne VLP-16","LDS-01","lidar","Lidar")
        )
        if lidar_dev is None:
            raise RuntimeError("No lidar found. Expected: Velodyne VLP-16, LDS-01, lidar")
        lidar_dev.enable(self._ts)
        print(f"  [INIT] lidar: {lidar_name}")

        cam_dev, cam_name = self._try_get_device(("camera","Camera"))
        if cam_dev is not None:
            cam_dev.enable(self._ts); print(f"  [INIT] camera: {cam_name}")
        else:
            print("  [INIT] camera not found; dirty-floor detection disabled")

        self._odom  = Odometry(ls, rs, gyro, comp, self._dt_s)
        self._lidar = LidarInterface(lidar_dev, lidar_name)
        self._grid  = OccupancyGrid()
        self._cam   = CameraProcessor(cam_dev, self._ts) if cam_dev is not None else None

        self._state = self.EXPLORE

        self._home_wx, self._home_wy, self._home_theta = self._read_initial_pose()
        self._odom.set_pose(self._home_wx, self._home_wy, self._home_theta)
        self._grid.seed_free_zone(self._home_wx, self._home_wy, radius_cells=4)

        # ── Navigation state ──────────────────────────────────────────────────
        # If these fire before any sweep/return goal has been explicitly set
        # (e.g. on the very first stuck event) they would have navigated to
        # world-origin (0, 0) instead of home.  Using home as the default
        # means the robot always heads somewhere safe.
        self._goal_wx        = self._home_wx
        self._goal_wy        = self._home_wy
        self._nav_path       = []
        self._nav_idx        = 0
        self._failed_targets = set()

        # ── Frontier / explore state ──────────────────────────────────────────
        self._frontiers               = []
        self._frontier_goal           = None
        self._frontier_goal_age       = 0
        self._rescan_t                = 0
        self._stuck_t                 = 0
        self._stuck_ref               = (self._home_wx, self._home_wy)
        self._explore_steps           = 0
        self._explore_known_prev      = self._grid.known_cell_count()
        self._explore_idle_checks     = 0
        self._wander_omega            = 0.0
        self._wander_hold             = 0
        self._explore_trace           = []
        self._explore_trace_cells     = set()
        self._last_trace_pose         = (self._home_wx, self._home_wy)
        self._last_trace_cell         = self._grid.w2g(self._home_wx, self._home_wy)

        self._failed_targets_idle_count = 0
        self._last_failed_frontier_pos  = None

        # ── Rescue state ──────────────────────────────────────────────────────
        # Incremented each time _start_recovery is called during EXPLORE.
        # Resets when the robot makes good forward progress or arrives at the
        # rescue point.
        self._rescue_recovery_count = 0

        # ── Sweep state ───────────────────────────────────────────────────────
        self._sweep_plan = []
        self._sweep_idx  = 0

        # ── Avoidance ─────────────────────────────────────────────────────────
        self._avoid_on    = False
        self._avoid_steps = 0
        self._avoid_dir   = 1
        self._avoid_back  = 0

        # ── Camera / person state ─────────────────────────────────────────────
        self._person_hold     = 0
        self._person_bearing  = 0.0
        self._semantic_counts = {"person": 0, "wall": 0, "furniture": 0}

        # ── Recovery state ────────────────────────────────────────────────────
        self._recover_on          = False
        self._recover_reason      = ""
        self._recover_back_steps  = 0
        self._recover_turn_steps  = 0
        self._recover_dir         = 1
        self._recover_cooldown    = 0

        # ── Timers / diagnostics ──────────────────────────────────────────────
        self._map_t          = 0
        self._cam_t          = 0
        self._dirty_t        = 0
        self._step           = 0
        self._did_self_check = False
        self._progress_t     = 0
        self._progress_ref   = (self._home_wx, self._home_wy)

        self._sample_explore_trace(force=True)

        print("=" * 64)
        print("  INTELLIGENT SWEEPER  --  Explore → Plan → Sweep → Return")
        print(f"  LOCALIZATION         --  {self._pose_source}")
        wx, wy, wz = getattr(self, "_home_webots_xyz", (self._home_wx, 0.0, self._home_wy))
        print(f"  HOME (Webots XYZ)    --  X={wx:+.5f} Y={wy:+.5f} Z={wz:+.5f}")
        print(
            f"  HOME (nav)           --  "
            f"x={self._home_wx:+.5f} y={self._home_wy:+.5f} "
            f"θ={math.degrees(self._home_theta):+.1f}°"
        )
        print(
            f"  RESCUE POINT         --  "
            f"x={Config.RESCUE_WX:+.5f} y={Config.RESCUE_WY:+.5f}  "
            f"(trigger: {Config.RESCUE_TRIGGER_RECOVERIES} recoveries)"
        )
        print("=" * 64)

    # ── Low-level drive ───────────────────────────────────────────────────────
    def _drive(self, left_rad_s, right_rad_s):
        cap = Config.MAX_SPEED
        self._lm.setVelocity(max(-cap, min(cap, left_rad_s)))
        self._rm.setVelocity(max(-cap, min(cap, right_rad_s)))

    def _stop(self):
        self._drive(0.0, 0.0)

    def _vel_from_vw(self, v_mps, omega_rad_s):
        r  = Config.WHEEL_RADIUS
        tw = Config.TRACK_WIDTH
        return (
            (v_mps - omega_rad_s * tw / 2.0) / r,
            (v_mps + omega_rad_s * tw / 2.0) / r,
        )

    def _set_goal(self, wx, wy, explore=True, fallback_unknown=False):
        self._goal_wx = wx
        self._goal_wy = wy
        sx, sy = self._grid.w2g(self._odom.x, self._odom.y)
        gx, gy = self._grid.w2g(wx, wy)
        path = self._grid.bfs_path(sx, sy, gx, gy, allow_unknown=explore)
        if not path and fallback_unknown and not explore:
            path = self._grid.bfs_path(sx, sy, gx, gy, allow_unknown=True)
        if not path:
            self._failed_targets.add((gx, gy))
            self._nav_path = []; self._nav_idx = 0
            return False
        self._failed_targets.discard((gx, gy))
        self._nav_path = _compress_grid_path(path)
        self._nav_idx  = 0
        return True

    # ── RETURN helper ─────────────────────────────────────────────────────────
    def _start_return(self, reason):
        print(f"  [RETURN] {reason}")
        self._grid.seed_free_zone(self._home_wx, self._home_wy, radius_cells=8)
        self._grid.seed_free_zone(self._odom.x,  self._odom.y,  radius_cells=8)
        sx, sy    = self._grid.w2g(self._odom.x, self._odom.y)
        gx, gy    = self._grid.w2g(self._home_wx, self._home_wy)
        print(
            f"  [RETURN DEBUG] robot ({sx},{sy}) → home ({gx},{gy}), "
            f"{self._grid.known_cell_count()} known cells"
        )
        for explore, fallback_unknown, desc in (
            (False, False, "safe (no unknown)"),
            (False, True,  "safe (with unknown)"),
            (True,  False, "explore (no unknown)"),
            (True,  True,  "explore (with unknown)"),
        ):
            if self._set_goal(self._home_wx, self._home_wy,
                              explore=explore, fallback_unknown=fallback_unknown):
                self._state = self.RETURN
                print(f"  [RETURN] path found via {desc} ({len(self._nav_path)} waypoints)")
                return
            print(f"  [RETURN] strategy failed: {desc}")

        nhg = self._grid.nearest_navigable(
            self._grid.w2g(self._home_wx, self._home_wy)[0],
            self._grid.w2g(self._home_wx, self._home_wy)[1],
            allow_unknown=True, max_radius=50,
        )
        if nhg is not None:
            nhx, nhy = self._grid.g2w(nhg[0], nhg[1])
            if self._set_goal(nhx, nhy, explore=True, fallback_unknown=True):
                self._state = self.RETURN
                print(f"  [RETURN] going to nearest home-adjacent point ({nhx:.2f},{nhy:.2f})")
                return
        print("  [RETURN] no path home → DONE")
        self._state = self.DONE; self._stop()

    # ── Nav step (proportional heading controller) ────────────────────────────
    def _nav_step(self, obs):
        if not self._nav_path or self._nav_idx >= len(self._nav_path):
            self._stop(); return True

        tgx, tgy = self._nav_path[self._nav_idx]
        twx, twy = self._grid.g2w(tgx, tgy)
        dx = twx - self._odom.x
        dy = twy - self._odom.y
        dist = math.hypot(dx, dy)

        if dist < Config.GOAL_TOL:
            self._nav_idx += 1
            if self._nav_idx >= len(self._nav_path):
                self._stop(); return True
            tgx, tgy = self._nav_path[self._nav_idx]
            twx, twy = self._grid.g2w(tgx, tgy)
            dx = twx - self._odom.x
            dy = twy - self._odom.y
            dist = math.hypot(dx, dy)

        h_err = _wrap(math.atan2(dy, dx) - self._odom.theta)
        align_frac = max(0.15, 1.0 - abs(h_err) / math.pi)
        dist_frac  = min(1.0, dist / 0.5)
        v          = Config.SPD_FORWARD * align_frac * dist_frac

        front_clearance = min(obs["front"], obs["front_left"], obs["front_right"])
        if not math.isinf(front_clearance):
            span    = max(0.05, Config.WARN_DIST - Config.DANGER_DIST)
            obs_frac = max(0.25, min(1.0, (front_clearance - Config.DANGER_DIST) / span))
            v       *= obs_frac

        omega = Config.K_ANG * h_err
        repel_l = repel_r = 0.0
        if obs["right"] < 0.35:
            repel_r = -min(2.0, 1.0 / max(0.05, obs["right"] - 0.20))
        elif obs["front_right"] < 0.40:
            repel_r = -0.5
        if obs["left"] < 0.35:
            repel_l = -min(2.0, 1.0 / max(0.05, obs["left"] - 0.20))
        elif obs["front_left"] < 0.40:
            repel_l = -0.5

        lv, rv = self._vel_from_vw(v, omega)
        self._drive(lv + repel_l, rv + repel_r)
        return False

    # ── Obstacle avoidance ────────────────────────────────────────────────────
    def _check_avoid(self, obs):
        left_open    = max(obs["left"],  obs["front_left"])  > Config.WARN_DIST
        right_open   = max(obs["right"], obs["front_right"]) > Config.WARN_DIST
        front_blocked   = obs["front"] < Config.DANGER_DIST
        side_too_close  = obs["left"]  < 0.16 or obs["right"] < 0.16
        person_blocking = self._person_hold > 0 and obs["front"] < Config.PERSON_AVOID_DIST
        in_danger = (
            side_too_close
            or (front_blocked and not (left_open or right_open))
            or person_blocking
        )

        if not self._avoid_on and in_danger:
            if person_blocking:
                self._avoid_dir = -1 if self._person_bearing > 0.0 else 1
            elif obs["left"] > obs["right"] + 0.1:
                self._avoid_dir = 1
            elif obs["right"] > obs["left"] + 0.1:
                self._avoid_dir = -1
            else:
                self._avoid_dir = random.choice([-1, 1])
            self._avoid_on    = True
            self._avoid_steps = 0
            self._avoid_back  = 8 if person_blocking else 6
            print(f"  [AVOID] {'person' if person_blocking else 'obstacle'} front:{obs['front']:.2f} m")

        if not self._avoid_on:
            return False

        self._avoid_steps += 1
        if self._avoid_back > 0:
            self._drive(-1.0, -1.0); self._avoid_back -= 1
        else:
            spd = Config.SPD_TURN * self._avoid_dir
            self._drive(spd, -spd)

        front_clear = (
            obs["front"]       > Config.WARN_DIST
            and obs["front_left"]  > Config.WARN_DIST
            and obs["front_right"] > Config.WARN_DIST
        )
        if front_clear and self._avoid_steps >= Config.AVOID_MIN_STEPS:
            self._avoid_on    = False
            self._avoid_steps = 0
            print("  [AVOID] cleared")
            # BUG FIX: replan after avoidance for RESCUE too (was only SWEEP/RETURN)
            if self._state in (self.SWEEP, self.RETURN):
                self._set_goal(self._goal_wx, self._goal_wy,
                               explore=False,
                               fallback_unknown=(self._state == self.RETURN))
            elif self._state == self.RESCUE:
                self._set_goal(Config.RESCUE_WX, Config.RESCUE_WY,
                               explore=True, fallback_unknown=True)

        return True

    # ── Reactive wander (FIX 1) ───────────────────────────────────────────────
    def _reactive_cruise(self, obs):
        """
        Obstacle-avoiding random wander used as a fallback during EXPLORE when
        no frontier nav path is active.

        FIX 1: WANDER_OMEGA_MAX=0.40 (was 0.90) and WANDER_HOLD_MAX=35 (was 80).
        At SPD_FORWARD=0.18 m/s, ω=0.9 held for 80 steps (2.56 s) gave a 2.3 rad
        arc (more than a half-circle).  New values cap each arc to ≈0.45 rad (25°).
        """
        if self._wander_hold <= 0:
            self._wander_hold  = random.randint(Config.WANDER_HOLD_MIN, Config.WANDER_HOLD_MAX)
            self._wander_omega = random.uniform(-Config.WANDER_OMEGA_MAX, Config.WANDER_OMEGA_MAX)
        self._wander_hold -= 1

        front = min(obs["front"], obs["front_left"], obs["front_right"])
        side_balance = (
            (obs["left"] + 0.5 * obs["front_left"])
            - (obs["right"] + 0.5 * obs["front_right"])
        )
        side_omega = max(-1.4, min(1.4, Config.REACTIVE_BIAS_GAIN * side_balance))

        v = Config.SPD_FORWARD
        if not math.isinf(front):
            span = max(0.05, Config.WARN_DIST - Config.DANGER_DIST)
            v   *= max(0.25, min(1.0, (front - Config.DANGER_DIST) / span))

        lv, rv = self._vel_from_vw(v, side_omega + self._wander_omega)
        self._drive(lv, rv)

    # ── EXPLORE phase ─────────────────────────────────────────────────────────
    def _run_explore(self, obs):
        self._explore_steps += 1
        self._sample_explore_trace()
        known = self._grid.known_cell_count()

        # ── Local stuck check ────────────────────────────────────────────────
        self._stuck_t += 1
        if self._stuck_t >= Config.STUCK_CHECK:
            moved = math.hypot(
                self._odom.x - self._stuck_ref[0],
                self._odom.y - self._stuck_ref[1],
            )
            self._stuck_ref = (self._odom.x, self._odom.y)
            self._stuck_t   = 0
            if moved < Config.STUCK_THRESH:
                if self._start_recovery(f"explore stuck ({moved:.2f} m)"):
                    # Rescue check immediately after incrementing the counter
                    if self._rescue_recovery_count >= Config.RESCUE_TRIGGER_RECOVERIES:
                        self._trigger_rescue()
                    return
            else:
                # Good progress → reset rescue counter
                if self._rescue_recovery_count > 0:
                    self._rescue_recovery_count = 0
                    print("  [RESCUE] recovery count reset (good progress)")

        # ── Rescue threshold check (catches any recovery path) ────────────────
        # (Also reached when recovery was already in progress and didn't re-fire)
        if self._rescue_recovery_count >= Config.RESCUE_TRIGGER_RECOVERIES:
            self._trigger_rescue()
            return

        # ── Follow active frontier nav path ───────────────────────────────────
        exploring_frontier = bool(self._nav_path) and self._nav_idx < len(self._nav_path)
        if exploring_frontier:
            self._frontier_goal_age += 1
            reached = self._nav_step(obs)
            if reached:
                self._frontier_goal     = None
                self._frontier_goal_age = 0
                self._nav_path          = []
                self._nav_idx           = 0
            elif self._frontier_goal_age > Config.FRONTIER_TARGET_TIMEOUT:
                # FIX 3 – record timed-out position, refresh, pick far target
                if self._frontier_goal is not None:
                    fgx, fgy = self._grid.w2g(
                        self._frontier_goal[0], self._frontier_goal[1]
                    )
                    self._failed_targets.add((fgx, fgy))
                    self._last_failed_frontier_pos = self._frontier_goal
                    print(
                        f"  [FRONTIER] timeout at "
                        f"({self._frontier_goal[0]:.2f},{self._frontier_goal[1]:.2f})"
                    )
                self._frontier_goal     = None
                self._frontier_goal_age = 0
                self._nav_path          = []
                self._nav_idx           = 0
                self._refresh_frontiers()
                self._try_set_frontier_goal(
                    min_advance_m=Config.FRONTIER_MIN_ADVANCE_M
                )
                return
            else:
                return  # still navigating toward frontier

        # ── Periodic map-gain / frontier rescan ───────────────────────────────
        self._rescan_t += 1
        if self._rescan_t >= Config.EXPLORE_CHECK_INT:
            self._rescan_t = 0
            gain = known - self._explore_known_prev
            self._explore_known_prev = known
            self._refresh_frontiers()

            if gain > 0 and self._failed_targets:
                self._failed_targets.clear()
                self._failed_targets_idle_count = 0

            if gain < Config.EXPLORE_MIN_GAIN:
                self._explore_idle_checks += 1
            else:
                self._explore_idle_checks = 0

            if (
                self._explore_idle_checks >= max(2, Config.EXPLORE_IDLE_LIMIT // 2)
                and len(self._frontiers) > Config.EXPLORE_FRONTIER_STOP_MAX
            ):
                self._wander_hold = 0  # force new random heading

            if not exploring_frontier:
                frontier_cells = {
                    self._grid.w2g(wx, wy) for wx, wy, _ in self._frontiers
                }
                all_frontiers_failed = (
                    bool(frontier_cells)
                    and frontier_cells.issubset(self._failed_targets)
                )
                if all_frontiers_failed:
                    if gain > 0:
                        # Map grew → stale failures
                        self._failed_targets.clear()
                        self._failed_targets_idle_count = 0
                        self._try_set_frontier_goal()
                    else:
                        # FIX 2 – force-clear after repeated stalls
                        self._failed_targets_idle_count += 1
                        if (
                            self._failed_targets_idle_count
                            >= Config.FAILED_TARGET_FORCE_CLEAR_IDLE
                        ):
                            print(
                                f"  [EXPLORE] force-clearing failed targets after "
                                f"{self._failed_targets_idle_count} idle intervals "
                                f"(FIX 2)"
                            )
                            self._failed_targets.clear()
                            self._failed_targets_idle_count = 0
                            self._last_failed_frontier_pos  = None
                            self._try_set_frontier_goal()
                else:
                    self._failed_targets_idle_count = 0
                    self._try_set_frontier_goal()

            if self._explore_steps % (4 * Config.EXPLORE_CHECK_INT) == 0:
                print(
                    f"  [EXPLORE] gain:{gain:4d} known:{known:5d} "
                    f"fronts:{len(self._frontiers):3d} "
                    f"idle:{self._explore_idle_checks} "
                    f"failed:{len(self._failed_targets)} "
                    f"deadlock:{self._failed_targets_idle_count} "
                    f"rescue_count:{self._rescue_recovery_count}"
                )

        # ── Convergence / timeout check ───────────────────────────────────────
        timed_out = self._explore_steps >= Config.EXPLORE_MAX_STEPS
        converged = (
            self._explore_steps >= Config.EXPLORE_MIN_STEPS
            and self._explore_idle_checks >= Config.EXPLORE_IDLE_LIMIT
            and len(self._frontiers) <= Config.EXPLORE_FRONTIER_STOP_MAX
        )
        if timed_out or converged:
            reason = "time budget" if timed_out else "map converged"
            print(
                f"  [EXPLORE] {reason} → PLAN "
                f"(steps:{self._explore_steps}, known:{known})"
            )
            self._rescue_recovery_count = 0   # clean up before leaving EXPLORE
            self._state = self.PLAN
            self._stop()
            return

        self._reactive_cruise(obs)

    # ── RESCUE trigger helper ─────────────────────────────────────────────────
    def _trigger_rescue(self):
        """
        Called when _rescue_recovery_count reaches the threshold.
        Attempts to path-plan to the rescue point and transitions to RESCUE.
        If the rescue point is unreachable, resets the counter and continues
        exploring rather than hard-stopping.
        """
        print(
            f"  [RESCUE] {self._rescue_recovery_count} recoveries → "
            f"navigating to rescue point ({Config.RESCUE_WX:.5f},{Config.RESCUE_WY:.5f})"
        )
        # Seed the rescue point as free so A* can reach it even if it was
        # previously unknown or partially blocked.
        self._grid.seed_free_zone(Config.RESCUE_WX, Config.RESCUE_WY, radius_cells=3)

        if self._set_goal(Config.RESCUE_WX, Config.RESCUE_WY,
                          explore=True, fallback_unknown=True):
            self._state = self.RESCUE
            print(
                f"  [RESCUE] path found ({len(self._nav_path)} waypoints), "
                f"transitioning to RESCUE state"
            )
        else:
            # Rescue point unreachable from here – reset counter and keep exploring
            print("  [RESCUE] rescue point unreachable, resetting recovery count")
            self._rescue_recovery_count = 0
            self._failed_targets.clear()
            self._wander_hold = 0

    # ── RESCUE phase ──────────────────────────────────────────────────────────
    def _run_rescue(self, obs):
        """
        Navigate to the rescue point, then resume EXPLORE with a clean slate.
        """
        reached = self._nav_step(obs)
        if not reached:
            return

        dist = math.hypot(
            self._odom.x - Config.RESCUE_WX,
            self._odom.y - Config.RESCUE_WY,
        )
        print(
            f"  [RESCUE] arrived at rescue point "
            f"(dist={dist:.2f} m) → resuming EXPLORE"
        )

        # ── Full explore state reset ──────────────────────────────────────────
        self._rescue_recovery_count      = 0
        self._frontier_goal              = None
        self._frontier_goal_age          = 0
        self._nav_path                   = []
        self._nav_idx                    = 0
        self._failed_targets.clear()
        self._last_failed_frontier_pos   = None
        self._failed_targets_idle_count  = 0
        # BUG FIX: also reset idle/wander state so stale convergence counts from
        # the furniture-stuck period do not immediately trigger PLAN.
        self._explore_idle_checks        = 0
        self._wander_hold                = 0
        self._stuck_ref                  = (self._odom.x, self._odom.y)
        self._stuck_t                    = 0

        # Seed the area around the rescue point as free to give A* a clean
        # starting region for the resumed exploration.
        self._grid.seed_free_zone(self._odom.x, self._odom.y, radius_cells=4)

        # Immediately try to pick a frontier from the new vantage point
        self._refresh_frontiers()
        self._try_set_frontier_goal()

        self._state = self.EXPLORE

    # ── PLAN phase ────────────────────────────────────────────────────────────
    def _run_plan(self):
        print("  [PLAN] computing coverage waypoints ...")
        self._grid.seed_free_zone(
            self._odom.x, self._odom.y, radius_cells=Config.PLAN_RESEED_RADIUS
        )
        trace_plan = self._full_sweep_from_trace()
        grid_plan  = self._grid.plan_coverage(self._odom.x, self._odom.y)

        if grid_plan and len(grid_plan) >= max(10, len(trace_plan)):
            self._sweep_plan = grid_plan
            print(f"  [PLAN] grid-based: {len(grid_plan)} waypoints")
        elif trace_plan:
            self._sweep_plan = trace_plan
            print(f"  [PLAN] trace-based: {len(trace_plan)} waypoints")
        else:
            self._sweep_plan = trace_plan if trace_plan else []
            print(f"  [PLAN] fallback trace: {len(self._sweep_plan)} waypoints")

        self._sweep_idx = 0
        n = len(self._sweep_plan)
        print(f"  [PLAN] {n} waypoints  (≈{n * Config.CELL_M:.1f} m)")

        if n == 0:
            self._start_return("nothing to sweep")
            return

        found_start = False
        while self._sweep_idx < n:
            wx, wy = self._sweep_plan[self._sweep_idx]
            if self._set_goal(wx, wy, explore=False, fallback_unknown=True):
                self._state = self.SWEEP
                found_start = True
                print(f"  [PLAN] starting at waypoint {self._sweep_idx}")
                return
            self._sweep_idx += 1

        if not found_start:
            self._start_return("no reachable sweep waypoint")

    # ── SWEEP phase ───────────────────────────────────────────────────────────
    def _run_sweep(self, obs):
        if self._sweep_idx >= len(self._sweep_plan):
            self._start_return("coverage complete")
            return

        self._mark_swept_footprint()
        reached = self._nav_step(obs)
        if not reached:
            return

        self._sweep_idx += 1
        skipped = 0
        while self._sweep_idx < len(self._sweep_plan) and skipped < 5:
            nx, ny = self._sweep_plan[self._sweep_idx]
            if self._set_goal(nx, ny, explore=False, fallback_unknown=True):
                break
            if self._set_goal(nx, ny, explore=True, fallback_unknown=True):
                print(f"  [SWEEP] explore-mode for waypoint {self._sweep_idx}")
                break
            skipped += 1
            print(f"  [SWEEP] skipped waypoint {self._sweep_idx} ({skipped}/5)")
            self._sweep_idx += 1

        if skipped >= 5 and self._sweep_idx >= len(self._sweep_plan):
            self._start_return("too many unreachable waypoints")
            return

        if self._sweep_idx % 25 == 0 and self._sweep_idx > 0:
            pct = 100 * self._sweep_idx // len(self._sweep_plan)
            print(f"  [SWEEP] {self._sweep_idx}/{len(self._sweep_plan)} {pct}%")

    # ── RETURN phase ──────────────────────────────────────────────────────────
    def _run_return(self, obs):
        reached = self._nav_step(obs)
        if not reached:
            return

        dist = math.hypot(self._odom.x - self._home_wx, self._odom.y - self._home_wy)

        if dist > Config.HOME_TOL:
            self._grid.seed_free_zone(self._home_wx, self._home_wy, radius_cells=10)
            for explore, fallback_unknown in ((False, True), (True, True)):
                if self._set_goal(self._home_wx, self._home_wy,
                                  explore=explore, fallback_unknown=fallback_unknown):
                    print(
                        f"  [RETURN] path ended {dist:.2f} m from home — "
                        f"replanning to close the gap"
                    )
                    return
            print(f"  [RETURN] {dist:.2f} m from home, no closer path → DONE")
            self._state = self.DONE
            self._stop()
            return

        # Within HOME_TOL — align heading, then finish.
        h_err = _wrap(self._home_theta - self._odom.theta)
        if abs(h_err) > Config.HEAD_TOL:
            lv, rv = self._vel_from_vw(0.0, 2.0 * h_err)
            self._drive(lv, rv)
            return

        print("  [RETURN] reached origin → DONE")
        self._state = self.DONE
        self._stop()

    # ── Main loop ─────────────────────────────────────────────────────────────
    def run(self):
        while self._robot.step(self._ts) != -1:
            self._step += 1
            s = self._step

            if self._recover_cooldown > 0:
                self._recover_cooldown -= 1

            self._odom.update()
            self._sync_pose_with_supervisor()
            obs = self._lidar.read_sectors()

            self._map_t += 1
            if self._map_t >= Config.MAP_INTERVAL:
                self._lidar.update_map(self._grid, self._odom)
                self._grid.seed_free_zone(self._odom.x, self._odom.y, radius_cells=1)
                self._map_t = 0

            if not self._did_self_check and self._step >= 3:
                self._run_self_check()
                self._did_self_check = True

            self._update_semantic_model(obs)

            if Config.ENABLE_DIRTY_DETECTION and self._cam is not None:
                self._dirty_t += 1
                if self._dirty_t >= 60:
                    self._dirty_t = 0
                    if self._cam.is_floor_dirty():
                        print(
                            f"  [CAM] dirty floor near "
                            f"({self._odom.x:+.2f},{self._odom.y:+.2f})"
                        )

            if self._state in (self.SWEEP, self.RETURN):
                self._progress_watchdog()
                if self._step % Config.SELF_CHECK_INTERVAL == 0:
                    self._runtime_replan_check()

            if self._run_recovery():
                continue

            # Avoidance runs in all active navigation states
            if self._state in (
                self.EXPLORE, self.SWEEP, self.RETURN, self.RESCUE
            ) and self._check_avoid(obs):
                continue

            if   self._state == self.EXPLORE: self._run_explore(obs)
            elif self._state == self.PLAN:    self._run_plan()
            elif self._state == self.SWEEP:   self._run_sweep(obs)
            elif self._state == self.RETURN:  self._run_return(obs)
            elif self._state == self.RESCUE:  self._run_rescue(obs)
            elif self._state == self.DONE:    self._stop()

            if s % 100 == 0:
                sim_t = self._robot.getTime()
                print(
                    f"[{sim_t:6.1f}s] "
                    f"State:{self._state:<8} "
                    f"Pos:({self._odom.x:+.2f},{self._odom.y:+.2f}) "
                    f"θ:{math.degrees(self._odom.theta):+5.1f}° "
                    f"F:{obs['front']:.2f} L:{obs['left']:.2f} R:{obs['right']:.2f} "
                    f"Fronts:{len(self._frontiers):3d} "
                    f"Swept:{self._sweep_idx:4d} "
                    f"RescueCnt:{self._rescue_recovery_count}"
                )


if __name__ == "__main__":
    RobotBrain().run()
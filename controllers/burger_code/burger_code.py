"""
AMR Hospital Sanitizer Controller (TurtleBot3 Burger, Webots)
3 phases: EXPLORE -> PLAN -> SWEEP
"""

from collections import deque
import math
import random

from controller import Robot, Camera


class Config:
    # Geometry
    WHEEL_RADIUS = 0.033
    TRACK_WIDTH = 0.160
    MAX_SPEED = 6.67

    # Speeds / control
    SPD_FORWARD = 0.12  # m/s
    SPD_SLOW = 0.05     # m/s
    SPD_TURN = 2.5      # rad/s (wheel speed for in-place turn)
    K_ANG = 4.5

    # Obstacle thresholds [m]
    DANGER_DIST = 0.28
    WARN_DIST = 0.45

    # Occupancy grid
    CELL_M = 0.10
    GRID_N = 200
    GRID_OX = 100
    GRID_OY = 100

    # LiDAR map update
    MAP_LAYER_IDS = (5, 7, 9)
    MAP_H_STEP = 12
    MAP_INTERVAL = 4
    MAX_RANGE = 5.5

    # Obstacle sector layers (inclusive)
    SEC_LAYER_MIN = 2
    SEC_LAYER_MAX = 13

    # Sector definitions in degrees (0 = forward, CCW+)
    SECTOR_DEFS = {
        "front": [(340, 359), (0, 20)],
        "front_left": [(21, 60)],
        "front_right": [(300, 339)],
        "left": [(61, 119)],
        "right": [(240, 299)],
    }

    # Navigation
    GOAL_TOL = 0.18
    HEAD_TOL = 0.15

    # Frontier exploration
    FRONT_MIN = 4
    RESCAN_INT = 200
    STUCK_CHECK = 60
    STUCK_THRESH = 0.08

    # Coverage
    COV_STRIDE = 3

    # Odometry fusion
    GYRO_WEIGHT = 0.70
    ENC_WEIGHT = 0.30
    COMPASS_ALPHA = 0.015

    # Avoidance
    AVOID_MIN_STEPS = 10

    # Grid states
    UNKNOWN = 0
    FREE = 1
    OCCUPIED = 2
    SWEPT = 3


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class OccupancyGrid:
    def __init__(self):
        self._N = Config.GRID_N
        self._g = bytearray(self._N * self._N)

        ox, oy = Config.GRID_OX, Config.GRID_OY
        for dy in range(-3, 4):
            for dx in range(-3, 4):
                self._set(ox + dx, oy + dy, Config.FREE)

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

    def _idx(self, gx, gy):
        return gy * self._N + gx

    def _set(self, gx, gy, value):
        if self.in_bounds(gx, gy):
            self._g[self._idx(gx, gy)] = value

    def get(self, gx, gy):
        if not self.in_bounds(gx, gy):
            return Config.OCCUPIED
        return self._g[self._idx(gx, gy)]

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

    @staticmethod
    def _bresenham(x0, y0, x1, y1):
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        while True:
            yield x0, y0
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

    def update_ray(self, robot_wx, robot_wy, angle_world, dist):
        if not math.isinf(dist) and dist <= 0.05:
            return

        is_hit = (not math.isinf(dist)) and (dist < Config.MAX_RANGE)
        ray_len = min(dist if not math.isinf(dist) else Config.MAX_RANGE, Config.MAX_RANGE)

        rx, ry = self.w2g(robot_wx, robot_wy)
        ex_w = robot_wx + ray_len * math.cos(angle_world)
        ey_w = robot_wy + ray_len * math.sin(angle_world)
        ex, ey = self.w2g(ex_w, ey_w)

        for cx, cy in self._bresenham(rx, ry, ex, ey):
            if cx == ex and cy == ey:
                break
            self.set_free(cx, cy)

        if is_hit:
            self.set_occupied(ex, ey)
        else:
            self.set_free(ex, ey)

    def find_frontiers(self):
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
                if (
                    g[(gy - 1) * N + gx] == U
                    or g[(gy + 1) * N + gx] == U
                    or g[row + gx - 1] == U
                    or g[row + gx + 1] == U
                ):
                    candidates.add((gx, gy))

        return self._cluster_frontiers(candidates)

    def _cluster_frontiers(self, cells):
        clusters = []
        remaining = set(cells)
        while remaining:
            seed = next(iter(remaining))
            group = []
            q = deque([seed])
            seen = {seed}
            while q:
                cx, cy = q.popleft()
                group.append((cx, cy))
                remaining.discard((cx, cy))
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, 1), (1, -1), (-1, -1)):
                    nb = (cx + dx, cy + dy)
                    if nb in remaining and nb not in seen:
                        seen.add(nb)
                        q.append(nb)

            if len(group) >= Config.FRONT_MIN:
                sx = sy = 0
                for px, py in group:
                    sx += px
                    sy += py
                cgx = int(sx / len(group))
                cgy = int(sy / len(group))
                wx, wy = self.g2w(cgx, cgy)
                clusters.append((wx, wy, len(group)))
        return clusters

    def bfs_path(self, sx, sy, gx, gy, allow_unknown=True):
        if not self.in_bounds(gx, gy):
            return None

        passable = {Config.FREE, Config.SWEPT}
        if allow_unknown:
            passable.add(Config.UNKNOWN)

        came = {(sx, sy): None}
        q = deque([(sx, sy)])

        while q:
            cx, cy = q.popleft()
            if cx == gx and cy == gy:
                break
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = cx + dx, cy + dy
                if (
                    (nx, ny) not in came
                    and self.in_bounds(nx, ny)
                    and self.get(nx, ny) in passable
                ):
                    came[(nx, ny)] = (cx, cy)
                    q.append((nx, ny))

        if (gx, gy) not in came:
            return None

        path = []
        node = (gx, gy)
        while node is not None:
            path.append(node)
            node = came[node]
        path.reverse()
        return path

    def plan_coverage(self):
        N = self._N
        g = self._g
        st = Config.COV_STRIDE
        valid = {Config.FREE, Config.SWEPT}

        found = False
        min_gx = max_gx = Config.GRID_OX
        min_gy = max_gy = Config.GRID_OY

        for gy in range(N):
            for gx in range(N):
                if g[gy * N + gx] in valid:
                    if not found:
                        min_gx = max_gx = gx
                        min_gy = max_gy = gy
                        found = True
                    else:
                        min_gx = min(min_gx, gx)
                        max_gx = max(max_gx, gx)
                        min_gy = min(min_gy, gy)
                        max_gy = max(max_gy, gy)

        if not found:
            return []

        waypoints = []
        row_idx = 0
        for scan_gy in range(min_gy, max_gy + 1, st):
            row = [
                (scan_gx, scan_gy)
                for scan_gx in range(min_gx, max_gx + 1, st)
                if g[scan_gy * N + scan_gx] in valid
            ]
            if row_idx % 2 == 1:
                row.reverse()
            row_idx += 1

            for gx, gy in row:
                waypoints.append(self.g2w(gx, gy))

        return waypoints


class Odometry:
    def __init__(self, left_enc, right_enc, gyro, compass, dt_s):
        self._le = left_enc
        self._re = right_enc
        self._gy = gyro
        self._cp = compass
        self._dt = dt_s
        self._inited = False

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self._pl = 0.0
        self._pr = 0.0

    def update(self):
        l_pos = self._le.getValue()
        r_pos = self._re.getValue()

        if not self._inited:
            self._pl = l_pos
            self._pr = r_pos
            self._inited = True
            return

        dl = (l_pos - self._pl) * Config.WHEEL_RADIUS
        dr = (r_pos - self._pr) * Config.WHEEL_RADIUS
        self._pl = l_pos
        self._pr = r_pos

        max_d = Config.MAX_SPEED * Config.WHEEL_RADIUS * self._dt * 1.5
        dl = max(-max_d, min(max_d, dl))
        dr = max(-max_d, min(max_d, dr))

        d_theta_enc = (dr - dl) / Config.TRACK_WIDTH
        d_theta_gyro = self._gy.getValues()[1] * self._dt
        d_theta = Config.GYRO_WEIGHT * d_theta_gyro + Config.ENC_WEIGHT * d_theta_enc

        d_dist = 0.5 * (dl + dr)
        mid = self.theta + 0.5 * d_theta
        self.x += d_dist * math.cos(mid)
        self.y += d_dist * math.sin(mid)
        self.theta = _wrap(self.theta + d_theta)

        cv = self._cp.getValues()
        t_c = math.atan2(cv[0], cv[2])
        self.theta = _wrap(self.theta + Config.COMPASS_ALPHA * _wrap(t_c - self.theta))


class LidarInterface:
    def __init__(self, device):
        self._dev = device
        self._H = int(device.getHorizontalResolution())
        self._L = int(device.getNumberOfLayers())
        self._ray0_offset = self._H // 2

        if self._H <= 0 or self._L <= 0:
            raise RuntimeError(f"Invalid lidar resolution/layer count: H={self._H}, L={self._L}")

        self._sector_idx = self._build_sector_indices()

    @staticmethod
    def _valid(r):
        return 0.05 < r < 20.0 and not math.isinf(r) and not math.isnan(r)

    def _build_sector_indices(self):
        built = {}
        pts_per_deg = max(1, self._H // 360)

        for name, ranges in Config.SECTOR_DEFS.items():
            idxs = []
            for sd, ed in ranges:
                count = (ed - sd + 1) * pts_per_deg
                start = (sd * pts_per_deg + self._ray0_offset) % self._H
                h = start
                for _ in range(count):
                    idxs.append(h)
                    h = (h + 1) % self._H
            built[name] = idxs
        return built

    def read_sectors(self):
        ranges = self._dev.getRangeImage()
        lo = max(0, Config.SEC_LAYER_MIN)
        hi = min(self._L - 1, Config.SEC_LAYER_MAX)
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
        step = Config.MAP_H_STEP
        tau = 2 * math.pi

        valid_layers = [l for l in Config.MAP_LAYER_IDS if 0 <= l < self._L]
        if not valid_layers:
            fallback = self._L // 2
            print(f"  [LIDAR] warning: MAP_LAYER_IDS {Config.MAP_LAYER_IDS} invalid for {self._L} layers; using layer {fallback}")
            valid_layers = [fallback]

        for layer in valid_layers:
            base = layer * self._H
            for h in range(0, self._H, step):
                r = ranges[base + h]
                if not math.isinf(r) and not self._valid(r):
                    continue
                angle_robot = (h - self._ray0_offset) * (tau / self._H)
                grid.update_ray(odom.x, odom.y, odom.theta + angle_robot, r)


class CameraProcessor:
    def __init__(self, device):
        self._dev = device
        self._w = device.getWidth()
        self._h = device.getHeight()

    def is_floor_dirty(self):
        img = self._dev.getImage()
        if not img:
            return False

        w, h = self._w, self._h
        start_y = (2 * h) // 3
        step = 8
        dark = 0
        total = 0

        for y in range(start_y, h, step):
            for x in range(0, w, step):
                r = Camera.imageGetRed(img, w, x, y)
                g = Camera.imageGetGreen(img, w, x, y)
                b = Camera.imageGetBlue(img, w, x, y)
                lum = 0.299 * r + 0.587 * g + 0.114 * b
                total += 1
                if lum < 60:
                    dark += 1

        return total > 0 and (dark / total) > 0.15


class RobotBrain:
    EXPLORE = "EXPLORE"
    PLAN = "PLAN"
    SWEEP = "SWEEP"
    DONE = "DONE"

    def __init__(self):
        self._robot = Robot()
        self._ts = int(self._robot.getBasicTimeStep())
        dt_s = self._ts / 1000.0

        self._lm = self._robot.getDevice("left wheel motor")
        self._rm = self._robot.getDevice("right wheel motor")
        for m in (self._lm, self._rm):
            m.setPosition(float("inf"))
            m.setVelocity(0.0)

        ls = self._robot.getDevice("left wheel sensor")
        rs = self._robot.getDevice("right wheel sensor")
        ls.enable(self._ts)
        rs.enable(self._ts)

        gyro = self._robot.getDevice("gyro")
        comp = self._robot.getDevice("compass")
        accel = self._robot.getDevice("accelerometer")
        gyro.enable(self._ts)
        comp.enable(self._ts)
        accel.enable(self._ts)
        self._accel = accel

        lidar_dev = self._robot.getDevice("Velodyne VLP-16")
        lidar_dev.enable(self._ts)

        cam_dev = self._robot.getDevice("camera")
        cam_dev.enable(self._ts)

        self._odom = Odometry(ls, rs, gyro, comp, dt_s)
        self._lidar = LidarInterface(lidar_dev)
        self._grid = OccupancyGrid()
        self._cam = CameraProcessor(cam_dev)

        self._state = self.EXPLORE

        self._goal_wx = 0.0
        self._goal_wy = 0.0
        self._nav_path = []
        self._nav_idx = 0

        self._frontiers = []
        self._front_idx = 0
        self._rescan_t = 0
        self._stuck_t = 0
        self._stuck_ref = (0.0, 0.0)

        self._sweep_plan = []
        self._sweep_idx = 0

        self._avoid_on = False
        self._avoid_steps = 0
        self._avoid_dir = 1
        self._avoid_back = 0

        self._map_t = 0
        self._cam_t = 0
        self._step = 0

        print("=" * 64)
        print("  INTELLIGENT SWEEPER  --  Explore -> Plan -> Sweep")
        print("=" * 64)

    def _drive(self, left_rad_s, right_rad_s):
        cap = Config.MAX_SPEED
        self._lm.setVelocity(max(-cap, min(cap, left_rad_s)))
        self._rm.setVelocity(max(-cap, min(cap, right_rad_s)))

    def _stop(self):
        self._drive(0.0, 0.0)

    def _vel_from_vw(self, v_mps, omega_rad_s):
        r = Config.WHEEL_RADIUS
        tw = Config.TRACK_WIDTH
        left = (v_mps - omega_rad_s * tw / 2.0) / r
        right = (v_mps + omega_rad_s * tw / 2.0) / r
        return left, right

    def _set_goal(self, wx, wy, explore=True):
        self._goal_wx = wx
        self._goal_wy = wy

        sx, sy = self._grid.w2g(self._odom.x, self._odom.y)
        gx, gy = self._grid.w2g(wx, wy)

        path = self._grid.bfs_path(sx, sy, gx, gy, allow_unknown=explore)
        if not path:
            self._nav_path = []
            self._nav_idx = 0
            return False

        self._nav_path = path
        self._nav_idx = 0
        return True

    def _nav_step(self, obs):
        if not self._nav_path or self._nav_idx >= len(self._nav_path):
            self._stop()
            return True

        tgx, tgy = self._nav_path[self._nav_idx]
        twx, twy = self._grid.g2w(tgx, tgy)

        dx = twx - self._odom.x
        dy = twy - self._odom.y
        dist = math.hypot(dx, dy)

        if dist < Config.GOAL_TOL:
            self._nav_idx += 1
            if self._nav_idx >= len(self._nav_path):
                self._stop()
                return True
            tgx, tgy = self._nav_path[self._nav_idx]
            twx, twy = self._grid.g2w(tgx, tgy)
            dx = twx - self._odom.x
            dy = twy - self._odom.y
            dist = math.hypot(dx, dy)

        target_angle = math.atan2(dy, dx)
        h_err = _wrap(target_angle - self._odom.theta)

        align_frac = max(0.15, 1.0 - abs(h_err) / math.pi)
        dist_frac = min(1.0, dist / 0.5)
        v = Config.SPD_FORWARD * align_frac * dist_frac
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

    def _check_avoid(self, obs):
        in_danger = (
            obs["front"] < Config.DANGER_DIST
            or obs["front_left"] < Config.DANGER_DIST
            or obs["front_right"] < Config.DANGER_DIST
        )

        if not self._avoid_on and in_danger:
            if obs["left"] > obs["right"] + 0.1:
                self._avoid_dir = 1
            elif obs["right"] > obs["left"] + 0.1:
                self._avoid_dir = -1
            else:
                self._avoid_dir = random.choice([-1, 1])

            self._avoid_on = True
            self._avoid_steps = 0
            self._avoid_back = 6
            print(f"  [AVOID] obstacle — front:{obs['front']:.2f} m")

        if not self._avoid_on:
            return False

        self._avoid_steps += 1

        if self._avoid_back > 0:
            self._drive(-1.0, -1.0)
            self._avoid_back -= 1
        else:
            spd = Config.SPD_TURN * self._avoid_dir
            self._drive(spd, -spd)

        front_clear = (
            obs["front"] > Config.WARN_DIST
            and obs["front_left"] > Config.WARN_DIST
            and obs["front_right"] > Config.WARN_DIST
        )

        if front_clear and self._avoid_steps >= Config.AVOID_MIN_STEPS:
            self._avoid_on = False
            self._avoid_steps = 0
            print("  [AVOID] cleared — replanning")
            self._set_goal(self._goal_wx, self._goal_wy, explore=(self._state == self.EXPLORE))

        return True

    def _run_explore(self, obs):
        self._rescan_t += 1
        if self._rescan_t >= Config.RESCAN_INT or not self._frontiers:
            self._frontiers = self._grid.find_frontiers()
            self._rescan_t = 0

            if not self._frontiers:
                print("  [EXPLORE] all frontiers exhausted → PLAN")
                self._state = self.PLAN
                self._stop()
                return

            ox, oy = self._odom.x, self._odom.y
            self._frontiers.sort(key=lambda f: math.hypot(f[0] - ox, f[1] - oy))
            chosen = None
            for i, (fx, fy, sz) in enumerate(self._frontiers):
                if self._set_goal(fx, fy, explore=True):
                    self._front_idx = i
                    chosen = (fx, fy, sz)
                    break

            if chosen is None:
                print("  [EXPLORE] all candidate frontiers unreachable → PLAN")
                self._state = self.PLAN
                self._stop()
                return

            fx, fy, sz = chosen
            print(f"  [EXPLORE] → frontier ({fx:+.2f},{fy:+.2f})  {len(self._frontiers)} clusters  size={sz}")

        self._stuck_t += 1
        if self._stuck_t >= Config.STUCK_CHECK:
            moved = math.hypot(self._odom.x - self._stuck_ref[0], self._odom.y - self._stuck_ref[1])
            self._stuck_ref = (self._odom.x, self._odom.y)
            self._stuck_t = 0

            if moved < Config.STUCK_THRESH and self._frontiers:
                attempted = 0
                while attempted < len(self._frontiers):
                    self._front_idx = (self._front_idx + 1) % len(self._frontiers)
                    fx, fy, _ = self._frontiers[self._front_idx]
                    attempted += 1
                    if self._set_goal(fx, fy, explore=True):
                        print(f"  [EXPLORE] stuck — switching to frontier {self._front_idx}")
                        break
                else:
                    print("  [EXPLORE] no reachable frontier after stuck check → PLAN")
                    self._state = self.PLAN
                    self._stop()
                    return

        reached = self._nav_step(obs)
        if reached:
            self._rescan_t = Config.RESCAN_INT

    def _run_plan(self):
        print("  [PLAN] computing coverage waypoints ...")
        self._sweep_plan = self._grid.plan_coverage()
        self._sweep_idx = 0

        n = len(self._sweep_plan)
        print(f"  [PLAN] {n} waypoints  (approx {n * Config.COV_STRIDE * Config.CELL_M:.1f} m)")

        if n == 0:
            print("  [PLAN] nothing to sweep → DONE")
            self._state = self.DONE
            return

        while self._sweep_idx < n:
            wx, wy = self._sweep_plan[self._sweep_idx]
            if self._set_goal(wx, wy, explore=False):
                self._state = self.SWEEP
                return
            self._sweep_idx += 1

        print("  [PLAN] no reachable sweep waypoint → DONE")
        self._state = self.DONE

    def _run_sweep(self, obs):
        if self._sweep_idx >= len(self._sweep_plan):
            print("  [SWEEP] coverage complete → DONE")
            self._state = self.DONE
            self._stop()
            return

        reached = self._nav_step(obs)
        if not reached:
            return

        gx, gy = self._grid.w2g(self._odom.x, self._odom.y)
        self._grid.set_swept(gx, gy)
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            self._grid.set_swept(gx + dx, gy + dy)

        self._sweep_idx += 1
        while self._sweep_idx < len(self._sweep_plan):
            nx, ny = self._sweep_plan[self._sweep_idx]
            if self._set_goal(nx, ny, explore=False):
                break
            self._sweep_idx += 1

        if self._sweep_idx % 25 == 0 and self._sweep_idx > 0:
            pct = 100 * self._sweep_idx // len(self._sweep_plan)
            print(f"  [SWEEP] {self._sweep_idx}/{len(self._sweep_plan)}  {pct}%")

    def run(self):
        while self._robot.step(self._ts) != -1:
            self._step += 1
            s = self._step

            self._odom.update()
            obs = self._lidar.read_sectors()

            self._map_t += 1
            if self._map_t >= Config.MAP_INTERVAL:
                self._lidar.update_map(self._grid, self._odom)
                self._map_t = 0

            self._cam_t += 1
            if self._cam_t >= 60:
                self._cam_t = 0
                if self._cam.is_floor_dirty():
                    print(f"  [CAM] dirty floor near ({self._odom.x:+.2f},{self._odom.y:+.2f})")

            if self._state in (self.EXPLORE, self.SWEEP) and self._check_avoid(obs):
                continue

            if self._state == self.EXPLORE:
                self._run_explore(obs)
            elif self._state == self.PLAN:
                self._run_plan()
            elif self._state == self.SWEEP:
                self._run_sweep(obs)
            elif self._state == self.DONE:
                self._stop()

            if s % 100 == 0:
                sim_t = self._robot.getTime()
                print(
                    f"[{sim_t:6.1f}s]  "
                    f"State:{self._state:<8}  "
                    f"Pos:({self._odom.x:+.2f},{self._odom.y:+.2f})  "
                    f"θ:{math.degrees(self._odom.theta):+5.1f}deg  "
                    f"F:{obs['front']:.2f}  "
                    f"L:{obs['left']:.2f}  "
                    f"R:{obs['right']:.2f}  "
                    f"Fronts:{len(self._frontiers):3d}  "
                    f"Swept:{self._sweep_idx:4d}"
                )


if __name__ == "__main__":
    RobotBrain().run()

from controller import Robot
import math
import random

class Config:
    TIME_STEP  = 32
    WHEEL_RADIUS = 0.033
    TRACK_WIDTH  = 0.160
    MAX_SPEED  = 6.67
    FORWARD    = 5.5
    TURN       = 2.5
    SHIFT      = 3.5
    SWEEP_TIME  = 200
    UTURN_TIME  = 48
    SHIFT_TIME  = 55
    DANGER_DIST = 0.28
    WARN_DIST   = 0.45
    AVOID_STEPS = 40

class LidarProcessor:
    SECTORS = {
        'front':       [(0, 20), (340, 359)],
        'front_left':  [(21,  60)],
        'left':        [(61, 119)],
        'right':       [(240, 299)],
        'front_right': [(300, 339)],
    }

    def __init__(self, device):
        self._dev = device

    @staticmethod
    def _valid(r):
        return r > 0.0 and not math.isinf(r) and not math.isnan(r)

    def _sector_min(self, ranges, name):
        return min(
            (ranges[i]
             for s, e in self.SECTORS[name]
             for i in range(s, e + 1)
             if self._valid(ranges[i])),
            default=float('inf')
        )

    def read(self):
        ranges = self._dev.getRangeImage()
        return {n: self._sector_min(ranges, n) for n in self.SECTORS}

class SweepRobot:
    SWEEP      = 'SWEEP'
    UTURN_1    = 'UTURN_1'
    LANE_SHIFT = 'LANE_SHIFT'
    UTURN_2    = 'UTURN_2'
    AVOID      = 'AVOID'

    def __init__(self):
        self.robot = Robot()
        self._init_devices()
        self.state      = self.SWEEP
        self.timer      = Config.SWEEP_TIME
        self.direction  = 1
        self.turn_side  = -1
        self.avoid_dir      = 1
        self.avoid_timer    = 0
        self.resume_state   = self.SWEEP
        self.resume_timer   = 0
        self.row = 0

    def _init_devices(self):
        ts = Config.TIME_STEP
        self.lm = self.robot.getDevice('left wheel motor')
        self.rm = self.robot.getDevice('right wheel motor')
        for m in (self.lm, self.rm):
            m.setPosition(float('inf'))
            m.setVelocity(0.0)
        lidar_dev = self.robot.getDevice('LDS-01')
        lidar_dev.enable(ts)
        self.lidar = LidarProcessor(lidar_dev)

    def _drive(self, l, r):
        cap = Config.MAX_SPEED
        self.lm.setVelocity(max(-cap, min(cap, l)))
        self.rm.setVelocity(max(-cap, min(cap, r)))

    def _stop(self):
        self._drive(0.0, 0.0)

    def _go(self, new_state, timer, reason=''):
        self.state = new_state
        self.timer = timer

    def _run_sweep(self, obs):
        self.timer -= 1
        if self.timer <= 0:
            self.row += 1
            self._go(self.UTURN_1, Config.UTURN_TIME)
            return
        spd = Config.FORWARD * self.direction
        l = r = spd
        if obs['right'] < 0.30:
            l = spd
            r = spd * 0.5
        elif obs['left'] < 0.30:
            l = spd * 0.5
            r = spd
        self._drive(l, r)

    def _run_uturn_1(self):
        self.timer -= 1
        if self.timer <= 0:
            self._go(self.LANE_SHIFT, Config.SHIFT_TIME)
            return
        spd = Config.TURN * self.turn_side
        self._drive(spd, -spd)

    def _run_lane_shift(self):
        self.timer -= 1
        if self.timer <= 0:
            self._go(self.UTURN_2, Config.UTURN_TIME)
            return
        self._drive(Config.SHIFT, Config.SHIFT)

    def _run_uturn_2(self):
        self.timer -= 1
        if self.timer <= 0:
            self.direction *= -1
            self.turn_side *= -1
            self._go(self.SWEEP, Config.SWEEP_TIME)
            return
        spd = Config.TURN * self.turn_side
        self._drive(spd, -spd)

    def _run_avoid(self, obs):
        self.avoid_timer -= 1
        front_clear = (obs['front']         > Config.WARN_DIST and
                       obs['front_left']  > Config.WARN_DIST and
                       obs['front_right'] > Config.WARN_DIST)
        if front_clear and self.avoid_timer <= 0:
            self.state = self.resume_state
            self.timer = self.resume_timer
            return
        if not front_clear and self.avoid_timer <= 0:
            self.avoid_dir   *= -1
            self.avoid_timer  = Config.AVOID_STEPS
        spd = Config.TURN * self.avoid_dir
        self._drive(spd, -spd)

    def _check_obstacle(self, obs):
        if self.state == self.AVOID:
            return False
        if self.state not in (self.SWEEP, self.LANE_SHIFT):
            return False
        in_danger = (obs['front']       < Config.DANGER_DIST or
                     obs['front_left']  < Config.DANGER_DIST or
                     obs['front_right'] < Config.DANGER_DIST)
        if not in_danger:
            return False
        if obs['left'] > obs['right'] + 0.05:
            self.avoid_dir = 1
        elif obs['right'] > obs['left'] + 0.05:
            self.avoid_dir = -1
        else:
            self.avoid_dir = random.choice([-1, 1])
        self.resume_state = self.state
        self.resume_timer = self.timer
        self.avoid_timer  = Config.AVOID_STEPS
        self.state = self.AVOID
        return True

    def run(self):
        step = 0
        while self.robot.step(Config.TIME_STEP) != -1:
            step += 1
            obs = self.lidar.read()
            if self._check_obstacle(obs):
                self._run_avoid(obs)
                continue
            if   self.state == self.SWEEP:      self._run_sweep(obs)
            elif self.state == self.UTURN_1:    self._run_uturn_1()
            elif self.state == self.LANE_SHIFT: self._run_lane_shift()
            elif self.state == self.UTURN_2:    self._run_uturn_2()
            elif self.state == self.AVOID:      self._run_avoid(obs)

if __name__ == '__main__':
    SweepRobot().run()
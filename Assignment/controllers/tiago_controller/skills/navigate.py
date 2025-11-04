import math
from controller import Supervisor, Motor
from knowledge.kg import KG, AREAS

class NavigateSkill:
    def __init__(self, robot: Supervisor, kg: KG):
        self.robot = robot
        self.kg = kg
        self.stop_distance = 0.25
        self.approach_speed = 0.8
        self.turn_speed = 2.0
        self.k_r = 1.2
        self.k_th = 2.5
        self.wheel_radius = 0.0625
        self.axle_length = 0.40
        self.max_wheel_speed = 8.0

        self.left_motor = None
        self.right_motor = None
        self.me = None
        self.current_goal_place = None
        self.goal_xy = None

    def setup(self):
        self.me = self.robot.getSelf()
        self.left_motor, self.right_motor = self._detect_wheels()
        if not self.left_motor or not self.right_motor:
            raise RuntimeError("NavigateSkill: geen wielmotoren gevonden.")
        for m in (self.left_motor, self.right_motor):
            m.setPosition(float("inf"))
            m.setVelocity(0.0)

    def start(self, target_place: str):
        self.current_goal_place = target_place
        self.goal_xy = self._center_of_place(target_place)

    def step(self):
        if not self.goal_xy:
            return "failed"
        rpos = self.me.getPosition()
        rrot = self.me.getOrientation()
        yaw = self._yaw_from_R(rrot)

        dx, dy = self.goal_xy[0] - rpos[0], self.goal_xy[1] - rpos[1]
        dist = math.hypot(dx, dy)
        if dist <= self.stop_distance:
            self.left_motor.setVelocity(0.0)
            self.right_motor.setVelocity(0.0)
            return "done"

        heading_des = math.atan2(dy, dx)
        heading_err = self._wrap_pi(heading_des - yaw)
        v = self._clamp(self.k_r * dist, -self.approach_speed, self.approach_speed)
        om = self._clamp(self.k_th * heading_err, -self.turn_speed, self.turn_speed)

        wl, wr = self._vw_to_wheels(v, om)
        wl = self._clamp(wl, -self.max_wheel_speed, self.max_wheel_speed)
        wr = self._clamp(wr, -self.max_wheel_speed, self.max_wheel_speed)
        self.left_motor.setVelocity(wl)
        self.right_motor.setVelocity(wr)
        return "busy"

    def _center_of_place(self, place: str):
        if place not in AREAS:
            return None
        xs, ys = AREAS[place]["x"], AREAS[place]["y"]
        return ((xs[0] + xs[1]) * 0.5, (ys[0] + ys[1]) * 0.5)

    def _detect_wheels(self):
        left = right = None
        for i in range(self.robot.getNumberOfDevices()):
            d = self.robot.getDeviceByIndex(i)
            if isinstance(d, Motor):
                nm = d.getName().lower()
                if "wheel" in nm or "base" in nm:
                    if "left" in nm and left is None:
                        left = d
                    elif "right" in nm and right is None:
                        right = d
        if left is None:
            for nm in ["wheel_left_joint", "wheel_left", "base_wheel_left_joint"]:
                dev = self.robot.getDevice(nm)
                if dev: left = dev; break
        if right is None:
            for nm in ["wheel_right_joint", "wheel_right", "base_wheel_right_joint"]:
                dev = self.robot.getDevice(nm)
                if dev: right = dev; break
        return left, right

    @staticmethod
    def _yaw_from_R(R):
        r00, r01, r02, r10, r11, r12, r20, r21, r22 = R
        return math.atan2(r10, r00)

    @staticmethod
    def _wrap_pi(a): return (a + math.pi) % (2*math.pi) - math.pi
    @staticmethod
    def _clamp(x, lo, hi): return max(lo, min(hi, x))
    def _vw_to_wheels(self, v, omega):
        wl = (v - 0.5 * omega * self.axle_length) / self.wheel_radius
        wr = (v + 0.5 * omega * self.axle_length) / self.wheel_radius
        return wl, wr
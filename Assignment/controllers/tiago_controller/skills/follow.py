import math
from controller import Supervisor, Motor

class FollowSkill:
    def __init__(self, robot: Supervisor, target_def="TARGET"):
        self.robot = robot
        self.target_def = target_def

        # Tunables
        self.stop_distance = 0.35
        self.approach_speed = 0.8    # m/s
        self.turn_speed = 2.0        # rad/s
        self.k_r = 1.2               # P-gain positie
        self.k_th = 2.5              # P-gain heading

        # Ruw ingeschatte basisparameters TIAGo (pas aan indien nodig)
        self.wheel_radius = 0.0625   # m
        self.axle_length = 0.40      # m
        self.max_wheel_speed = 8.0   # rad/s

        self.left_motor = None
        self.right_motor = None
        self.me = None
        self.target = None

    def setup(self):
        self.me = self.robot.getSelf()
        self.target = self.robot.getFromDef(self.target_def)
        if self.target is None:
            raise RuntimeError(f"DEF {self.target_def} niet gevonden in de world.")

        self.left_motor, self.right_motor = self._detect_wheels()
        if not self.left_motor or not self.right_motor:
            raise RuntimeError("Kon de linker/rechter wielmotor niet detecteren. "
                               "Pas fallback-namen aan in _detect_wheels().")

        # Velocity control
        for m in (self.left_motor, self.right_motor):
            m.setPosition(float("inf"))
            m.setVelocity(0.0)

    def step(self):
        # Robotpose (ground-truth, Supervisor API)
        rpos = self.me.getPosition()          # [x,y,z]
        rrot = self.me.getOrientation()       # 3x3 rotatiematrix (list[9])
        yaw = self._yaw_from_R(rrot)

        # Targetpose
        tpos = self.target.getPosition()

        # 2D fout
        dx, dy = tpos[0] - rpos[0], tpos[1] - rpos[1]
        dist = math.hypot(dx, dy)
        heading_des = math.atan2(dy, dx)
        heading_err = self._wrap_pi(heading_des - yaw)

        # Unicycle-regelaar
        if dist <= self.stop_distance:
            v, om = 0.0, 0.0
        else:
            v = self._clamp(self.k_r * dist, -self.approach_speed, self.approach_speed)
            om = self._clamp(self.k_th * heading_err, -self.turn_speed, self.turn_speed)

        wl, wr = self._vw_to_wheels(v, om)
        wl = self._clamp(wl, -self.max_wheel_speed, self.max_wheel_speed)
        wr = self._clamp(wr, -self.max_wheel_speed, self.max_wheel_speed)

        self.left_motor.setVelocity(wl)
        self.right_motor.setVelocity(wr)

    # ---------- helpers ----------
    def _detect_wheels(self):
        # Heuristiek op basis van naam; zo nodig: pas de lijsten hieronder aan je TIAGo-variant
        left = None
        right = None
        for i in range(self.robot.getNumberOfDevices()):
            d = self.robot.getDeviceByIndex(i)
            if isinstance(d, Motor):
                name = d.getName().lower()
                if "wheel" in name or "base" in name:
                    if "left" in name and left is None:
                        left = d
                    elif "right" in name and right is None:
                        right = d

        # Fallback namen
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
    def _wrap_pi(a):
        return (a + math.pi) % (2*math.pi) - math.pi

    @staticmethod
    def _clamp(x, lo, hi):
        return max(lo, min(hi, x))

    def _vw_to_wheels(self, v, omega):
        # unicycle -> differential drive
        wl = (v - 0.5 * omega * self.axle_length) / self.wheel_radius
        wr = (v + 0.5 * omega * self.axle_length) / self.wheel_radius
        return wl, wr
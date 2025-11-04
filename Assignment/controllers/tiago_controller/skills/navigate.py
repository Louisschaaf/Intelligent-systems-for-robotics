# navigate.py
import math
from controller import Supervisor, Motor
from knowledge.kg import KG

class NavigateSkill:
    def __init__(self, robot: Supervisor, kg: KG):
        self.robot = robot
        self.kg = kg
        self.left_motor = self.right_motor = None
        self.me = None
        self.current_goal_place = None
        self.goal_xy = None

        # conservatieve gains
        self.stop_distance  = 0.30
        self.approach_speed = 0.5
        self.turn_speed     = 1.2
        self.k_r            = 0.8
        self.k_th           = 1.2

        self.wheel_radius   = 0.0625
        self.axle_length    = 0.40
        self.max_wheel_speed = 8.0

        # tuning switches
        self.forward_axis = "x"    # zet op "y" als lokale Y bij jou 'vooruit' is
        self.invert_omega = False  # omkeer draaiteken indien nodig
        self.swap_wheels  = False  # wissel L/R indien bekabeling/namen omgekeerd zijn

    def setup(self):
        self.me = self.robot.getSelf()
        self.left_motor, self.right_motor = self._detect_wheels()
        if not self.left_motor or not self.right_motor:
            raise RuntimeError("NavigateSkill: geen wielmotoren gevonden.")
        for m in (self.left_motor, self.right_motor):
            m.setPosition(float("inf"))
            m.setVelocity(0.0)

    def start_place(self, place_symbol: str):
        self.current_goal_place = place_symbol
        def_name = self.kg.get_waypoint_def(place_symbol)
        node = self.robot.getFromDef(def_name)
        if node is None:
            raise RuntimeError(f"DEF '{def_name}' (voor plaats {place_symbol}) niet gevonden.")
        x, y, z = node.getPosition()   # z is hoogte; rijden op x–y
        self.goal_xy = (x, y)

    def step(self):
        if not self.goal_xy:
            return "failed"

        rpos = self.me.getPosition()         # (x,y,z)
        rrot = self.me.getOrientation()      # 3x3 flatten (len=9)
        yaw = self._yaw_from_R_z_up(rrot, self.forward_axis)

        gx, gy = self.goal_xy
        dx, dy = gx - rpos[0], gy - rpos[1]
        dist = math.hypot(dx, dy)
        if dist <= self.stop_distance:
            self.left_motor.setVelocity(0.0)
            self.right_motor.setVelocity(0.0)
            return "done"

        heading_des = math.atan2(dy, dx)     # 2D op x–y
        heading_err = self._wrap_pi(heading_des - yaw)

        v  = self._clamp(self.k_r  * dist,        -self.approach_speed, self.approach_speed)
        om = self._clamp(self.k_th * heading_err, -self.turn_speed,     self.turn_speed)
        if self.invert_omega: om = -om

        wl, wr = self._vw_to_wheels(v, om)
        if self.swap_wheels: wl, wr = wr, wl

        wl = self._clamp(wl, -self.max_wheel_speed, self.max_wheel_speed)
        wr = self._clamp(wr, -self.max_wheel_speed, self.max_wheel_speed)
        self.left_motor.setVelocity(wl)
        self.right_motor.setVelocity(wr)
        return "busy"

    # helpers
    def _detect_wheels(self):
        left = right = None
        for i in range(self.robot.getNumberOfDevices()):
            d = self.robot.getDeviceByIndex(i)
            if isinstance(d, Motor):
                nm = d.getName().lower()
                if "wheel" in nm or "base" in nm:
                    if "left" in nm and left is None:  left = d
                    elif "right" in nm and right is None: right = d
        if left is None:
            for nm in ["wheel_left_joint","wheel_left","base_wheel_left_joint"]:
                dev = self.robot.getDevice(nm)
                if dev: left = dev; break
        if right is None:
            for nm in ["wheel_right_joint","wheel_right","base_wheel_right_joint"]:
                dev = self.robot.getDevice(nm)
                if dev: right = dev; break
        return left, right

    @staticmethod
    def _yaw_from_R_z_up(R, forward_axis="x"):
        r00,r01,r02, r10,r11,r12, r20,r21,r22 = R
        if forward_axis == "x":
            return math.atan2(r10, r00)  # yaw = atan2(R[1,0], R[0,0])
        else:
            return math.atan2(r11, r01)  # alternatief: lokale Y is vooruit

    @staticmethod
    def _wrap_pi(a): return (a + math.pi) % (2*math.pi) - math.pi
    @staticmethod
    def _clamp(x, lo, hi): return max(lo, min(hi, x))
    def _vw_to_wheels(self, v, omega):
        wl = (v - 0.5 * omega * self.axle_length) / self.wheel_radius
        wr = (v + 0.5 * omega * self.axle_length) / self.wheel_radius
        return wl, wr
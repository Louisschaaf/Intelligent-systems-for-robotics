import math
from controller import Supervisor, Motor
from knowledge.kg import KG

# De CameraDetection klasse wordt aangenomen dat deze beschikbaar is in de hoofdcontroller,
# zoals in je main() functie.

class NavigateSkill:
    def __init__(self, robot: Supervisor, kg: KG, cam_detection):
        self.robot = robot
        self.kg = kg
        self.cam = cam_detection # CameraDetection instantie
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
        
        # Parameters voor veiligheidsvertraging op basis van diepte
        self.safe_distance     = 0.40 # Minimale afstand waarop nog heel langzaam mag worden gereden
        self.max_slow_distance = 1.00 # Afstand waarboven de snelheid NIET wordt beperkt

        self.wheel_radius   = 0.0625
        self.axle_length    = 0.40
        self.max_wheel_speed = 8.0

        # tuning switches
        self.forward_axis = "x"
        self.invert_omega = False 
        self.swap_wheels  = False 

    def setup(self):
        """
        FIX: Deze methode was de oorzaak van de AttributeError. 
        Initialiseert de robot node en detecteert de wielmotoren.
        """
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
        # ... (De volledige stap-logica die je meestuurde, inclusief V_safe) ...
        if not self.goal_xy:
            return "failed"

        rpos = self.me.getPosition()         
        rrot = self.me.getOrientation()      
        yaw = self._yaw_from_R_z_up(rrot, self.forward_axis)

        gx, gy = self.goal_xy
        dx, dy = gx - rpos[0], gy - rpos[1]
        dist = math.hypot(dx, dy)
        
        if dist <= self.stop_distance:
            self.left_motor.setVelocity(0.0)
            self.right_motor.setVelocity(0.0)
            return "done"

        heading_des = math.atan2(dy, dx)     
        heading_err = self._wrap_pi(heading_des - yaw)

        # ----------------------------------------------------
        # STAP 1: Bepaal de maximale gewenste snelheid (V_goal)
        # ----------------------------------------------------
        v_goal  = self._clamp(self.k_r  * dist, 0.0, self.approach_speed)
        
        # ----------------------------------------------------
        # STAP 2: Bepaal de maximale veilige snelheid (V_safe)
        # ----------------------------------------------------
        v_safe = self.approach_speed
        min_dist = self.cam.get_min_distance()

        if min_dist is not None:
            if min_dist < self.max_slow_distance:
                d = self._clamp(min_dist, self.safe_distance, self.max_slow_distance)
                range_size = self.max_slow_distance - self.safe_distance
                if range_size > 0:
                    scale_factor = (d - self.safe_distance) / range_size
                    v_safe = self.approach_speed * scale_factor
                else:
                    v_safe = 0.1 * self.approach_speed 

        # ----------------------------------------------------
        # STAP 3 & 4: Eind V-snelheid en Rotatie (Omega)
        # ----------------------------------------------------
        v = min(v_goal, v_safe) 
        om = self._clamp(self.k_th * heading_err, -self.turn_speed, self.turn_speed)
        
        # Optioneel: Stop met draaien als je een obstakel TE dichtbij hebt
        if min_dist is not None and min_dist < self.safe_distance:
             om = 0.0 # Stop zowel V als Omega als te dichtbij

        if self.invert_omega: om = -om

        # ... (Kinematica en motoraansturing) ...
        wl, wr = self._vw_to_wheels(v, om)
        if self.swap_wheels: wl, wr = wr, wl

        wl = self._clamp(wl, -self.max_wheel_speed, self.max_wheel_speed)
        wr = self._clamp(wr, -self.max_wheel_speed, self.max_wheel_speed)
        self.left_motor.setVelocity(wl)
        self.right_motor.setVelocity(wr)
        return "busy"
    
    # --- HELPERS (Uit eerdere correcties) ---
    def _detect_wheels(self):
        """FIX: Deze methode moest in de klasse staan voor setup()."""
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
            return math.atan2(r10, r00)
        else:
            return math.atan2(r11, r01)

    @staticmethod
    def _wrap_pi(a): return (a + math.pi) % (2*math.pi) - math.pi
    @staticmethod
    def _clamp(x, lo, hi): return max(lo, min(hi, x))
    def _vw_to_wheels(self, v, omega):
        wl = (v - 0.5 * omega * self.axle_length) / self.wheel_radius
        wr = (v + 0.5 * omega * self.axle_length) / self.wheel_radius
        return wl, wr
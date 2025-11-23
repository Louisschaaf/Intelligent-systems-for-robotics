from controller import Supervisor
import numpy as np
import matplotlib.pyplot as plt

class Lidar:
    def __init__(self, robot: Supervisor, lidar_def: str, lidar_offset=(0.0, 0.0)):
        self.robot = robot
        self.lidar_def = lidar_def
        self.lidar = robot.getDevice(lidar_def)
        if self.lidar is None:
            raise RuntimeError(f"Lidar met DEF {lidar_def} niet gevonden.")
        self.lidar.enablePointCloud()

        # offset LiDAR t.o.v. robotcentrum in robotframe (x vooruit, y zijwaarts)
        self.lidar_offset = np.array(lidar_offset, dtype=float)

        self.fig, self.ax, self.sc = None, None, None
        self.global_points = np.empty((0, 2), dtype=float)

        self.node = self.robot.getSelf()
        self.translation_field = self.node.getField("translation")
        self.rotation_field = self.node.getField("rotation")

    def enable(self, timestep: int):
        self.lidar.enable(timestep)

    def get_range_image(self):
        return self.lidar.getRangeImage()

    def get_horizontal_resolution(self):
        return self.lidar.getHorizontalResolution()

    def get_number_of_points(self):
        return self.lidar.getNumberOfPoints()

    def get_fov(self):
        return self.lidar.getFov()

    def angle_robot_to_world(self):
        # Haal as-hoek uit Webots
        ax, ay, az, angle = self.rotation_field.getSFRotation()

        c = np.cos(angle)
        s = np.sin(angle)
        t = 1.0 - c

        # As-hoek -> 3x3 rotatiematrix
        R = np.array([
            [t*ax*ax + c,     t*ax*ay - s*az, t*ax*az + s*ay],
            [t*ax*ay + s*az,  t*ay*ay + c,    t*ay*az - s*ax],
            [t*ax*az - s*ay,  t*ay*az + s*ax, t*az*az + c   ]
        ])

        # BELANGRIJK verschil:
        # - Als R wereld->robot is, dan is de eerste RIJ de robot-x-as in wereldcoördinaten.
        ex_world = R[0, :]          # (ex, ey, ez) in wereldframe

        yaw = np.arctan2(ex_world[1], ex_world[0])

        # Als de kaart nog gespiegeld lijkt, kun je hier eventueel een min-teken proberen:
        # yaw = -yaw
        return yaw

    def transform_lidar(self):
        ranges = np.array(self.get_range_image(), dtype=float)
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=0.0, neginf=0.0)
        if ranges.size == 0:
            return np.empty((0, 2))

        fov = self.get_fov()
        res = self.get_horizontal_resolution()

        # Lokale LiDAR hoeken (in LiDAR-frame)
        angles = np.linspace(-fov/2, fov/2, res)

        # Robotpositie in wereld
        x_r, y_r, z_r = self.translation_field.getSFVec3f()

        # Robot yaw via correcte matrix-afleiding
        yaw = self.angle_robot_to_world()
        cos_y = np.cos(yaw)
        sin_y = np.sin(yaw)

        pts = []

        for theta, d in zip(angles, ranges):
            if d <= 0.01:
                continue

            # Punt in LiDAR/robot-frame
            x_l = d * np.cos(theta) + self.lidar_offset[0]
            y_l = d * np.sin(theta) + self.lidar_offset[1]

            # Transformeer naar wereld-frame
            x_w = x_r + cos_y * x_l - sin_y * y_l
            y_w = y_r + sin_y * x_l + cos_y * y_l

            pts.append([x_w, y_w])

        return np.array(pts)

    def update_global_map(self):
        pts_world = self.transform_lidar()
        if pts_world.shape[0] == 0:
            return
        self.global_points = np.vstack([self.global_points, pts_world])

    def visualize_global_map(self, xlim=(-5, 5), ylim=(-5, 5)):
        if self.global_points.shape[0] == 0:
            return

        xs = self.global_points[:, 0]
        ys = self.global_points[:, 1]

        if self.fig is None:
            plt.ion()
            self.fig, self.ax = plt.subplots()
            self.sc = self.ax.scatter(xs, ys, s=1)
            self.ax.set_xlim(*xlim)
            self.ax.set_ylim(*ylim)
            self.ax.set_aspect('equal')
            self.ax.set_title("Globale LiDAR-kaart")
            plt.show(block=False)
        else:
            self.sc.set_offsets(np.column_stack([xs, ys]))
            self.ax.set_xlim(*xlim)
            self.ax.set_ylim(*ylim)
            self.fig.canvas.draw_idle()
            plt.pause(0.001)
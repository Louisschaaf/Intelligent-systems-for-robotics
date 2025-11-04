from controller import Supervisor
from controller import Lidar
import numpy as np
import matplotlib.pyplot as plt

class Lidar:
    def __init__(self, robot: Supervisor, lidar_def: str):
        self.robot = robot
        self.lidar_def = lidar_def
        self.lidar = robot.getDevice(lidar_def)
        if self.lidar is None:
            raise RuntimeError(f"Lidar met DEF {lidar_def} niet gevonden.")
        self.lidar.enablePointCloud()
        self.fig, self.ax, self.sc = None, None, None


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
    
    # visualisatie van 2D Lidar data
    def visualize_2d(self):
        ranges = np.array(self.get_range_image())
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=0.0, neginf=0.0)
        if ranges.size == 0:
            return
        
        fov = self.get_fov()
        res = self.get_horizontal_resolution()
        angles = np.linspace(-fov/2, fov/2, res)

        x = np.cos(angles) * ranges
        y = np.sin(angles) * ranges

        if self.fig is None:
            plt.ion()
            self.fig, self.ax = plt.subplots()
            self.sc = self.ax.scatter(x, y, s=2)
            self.ax.set_xlim(-5, 5)
            self.ax.set_ylim(-5, 5)
            self.ax.set_aspect('equal')
            self.ax.set_title("Webots LiDAR Visualization")
            plt.show(block=False)
        else:
            self.sc.set_offsets(np.c_[x, y])
            self.fig.canvas.draw_idle()
            plt.pause(0.001)
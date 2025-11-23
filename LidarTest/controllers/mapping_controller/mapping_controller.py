from controller import Robot
import math

# Simulation setup
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Devices
lidar = robot.getDevice("lidar")
lidar.enable(timestep)

gps = robot.getDevice("gps")
gps.enable(timestep)

compass = robot.getDevice("compass")
compass.enable(timestep)

display = robot.getDevice("display")
W = display.getWidth()
H = display.getHeight()

# World map (persistent global occupancy grid)
MAP_SIZE = 512
world_map = [[0 for _ in range(MAP_SIZE)] for _ in range(MAP_SIZE)]

# Scale: 1 meter = 20 pixels in world map
SCALE = 20.0
MAP_CENTER = MAP_SIZE // 2

# Optional: robot moves forward slowly (you may remove this)
left_motor = robot.getDevice("left wheel")
right_motor = robot.getDevice("right wheel")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(2.0)
right_motor.setVelocity(2.0)

# Helper: convert world (x,z) to map indices
def world_to_map(wx, wz):
    gx = int(MAP_CENTER + wx * SCALE)
    gz = int(MAP_CENTER - wz * SCALE)
    return gx, gz

while robot.step(timestep) != -1:

    # 1. read sensors
    ranges = lidar.getRangeImage()
    res = lidar.getHorizontalResolution()
    fov = lidar.getFov()

    pos = gps.getValues()       # pos[0] = x, pos[2] = z
    north = compass.getValues() # vector pointing north

    # yaw: robot orientation
    yaw = math.atan2(north[0], north[2])

    # 2. Integrate LiDAR into persistent world map
    for i in range(res):
        r = ranges[i]
        if r >= lidar.getMaxRange():
            continue

        angle = -fov / 2.0 + i * (fov / res)
        world_angle = angle + yaw

        # convert to world coordinates
        wx = pos[0] + r * math.cos(world_angle)
        wz = pos[2] + r * math.sin(world_angle)

        gx, gz = world_to_map(wx, wz)
        if 0 <= gx < MAP_SIZE and 0 <= gz < MAP_SIZE:
            world_map[gz][gx] = 255

    # 3. Draw the world map to the display (downsampled)
    step_x = MAP_SIZE / W
    step_y = MAP_SIZE / H

    display.setColor(0x000000)
    display.fillRectangle(0, 0, W, H)

    display.setColor(0xFFFFFF)
    for py in range(H):
        sy = int(py * step_y)
        for px in range(W):
            sx = int(px * step_x)
            if world_map[sy][sx] > 0:
                display.drawPixel(px, py)
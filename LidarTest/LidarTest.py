from controller import Robot
import math

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Devices
lidar = robot.getDevice("lidar")
lidar.enable(timestep)

gps = robot.getDevice("gps")
gps.enable(timestep)

compass = robot.getDevice("compass")
compass.enable(timestep)

display = robot.getDevice("map_display")

# Precompute field of view
fov = lidar.getFov()
res = lidar.getHorizontalResolution()

# Map settings
W, H = display.getWidth(), display.getHeight()
scale = 10.0  # pixels per meter

while robot.step(timestep) != -1:
    ranges = lidar.getRangeImage()

    # Robot absolute position
    pos = gps.getValues()
    rx = pos[0]
    rz = pos[2]

    # Robot orientation (yaw)
    north = compass.getValues()
    yaw = math.atan2(north[0], north[2])  # heading

    # Clear display
    display.setColor(0x000000)
    display.fillRectangle(0, 0, W, H)
    display.setColor(0xFFFFFF)

    for i, r in enumerate(ranges):
        if r >= lidar.getMaxRange():
            continue

        angle = -fov/2 + i * (fov / res)
        global_angle = angle + yaw

        # Local point
        local_x = r * math.cos(global_angle)
        local_z = r * math.sin(global_angle)

        # World coordinates
        world_x = rx + local_x
        world_z = rz + local_z

        # Convert to display pixel coordinates
        px = int(W/2 + world_x * scale)
        pz = int(H/2 - world_z * scale)

        # Draw point
        if 0 <= px < W and 0 <= pz < H:
            display.drawPixel(px, pz)
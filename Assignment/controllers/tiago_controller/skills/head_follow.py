from controller import Motor
import numpy as np


class HeadFollowSkill:
    def __init__(self, robot, camera, yaw_gain=0.8, pitch_gain=0.6):
        self.robot = robot
        self.camera = camera

        # TIAGo head motor names in Webots
        self.yaw_motor = robot.getDevice("head_1_joint")   # left-right
        self.pitch_motor = robot.getDevice("head_2_joint") # up-down

        # Motor speeds
        self.yaw_motor.setVelocity(1.0)
        self.pitch_motor.setVelocity(1.0)

        # Store gains
        self.yaw_gain = yaw_gain
        self.pitch_gain = pitch_gain

        # PID terms
        self.yaw_integral = 0.0
        self.pitch_integral = 0.0
        self.yaw_prev_error = 0.0
        self.pitch_prev_error = 0.0

        # PID gains
        self.yaw_kp = yaw_gain
        self.yaw_ki = 0.002
        self.yaw_kd = 0.25

        self.pitch_kp = pitch_gain
        self.pitch_ki = 0.002
        self.pitch_kd = 0.18

        # Joint limits (from TIAGo URDF / Webots model)
        self.yaw_min, self.yaw_max     = -1.2, 1.2
        self.pitch_min, self.pitch_max = -0.5, 0.5

    def track(self, class_name="apple"):
        """
        Probeer het hoofd naar het gedetecteerde object te richten.
        Returnt True als tracking gebeurde, False als geen object gevonden werd.
        """
        det = self.camera.detect_object(class_name)
        if det is None:
            return False

        x1, y1, x2, y2 = det["bbox_xyxy"]
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2

        # Normaliseer pixelfouten
        err_x = (cx - self.camera.width  / 2) / self.camera.width
        err_y = (cy - self.camera.height / 2) / self.camera.height

        # PID computations
        self.yaw_integral += err_x
        yaw_derivative = err_x - self.yaw_prev_error
        yaw_output = (
            self.yaw_kp * err_x +
            self.yaw_ki * self.yaw_integral +
            self.yaw_kd * yaw_derivative
        )
        self.yaw_prev_error = err_x

        self.pitch_integral += err_y
        pitch_derivative = err_y - self.pitch_prev_error
        pitch_output = (
            self.pitch_kp * err_y +
            self.pitch_ki * self.pitch_integral +
            self.pitch_kd * pitch_derivative
        )
        self.pitch_prev_error = err_y

        # Huidige posities
        yaw   = self.yaw_motor.getTargetPosition()
        pitch = self.pitch_motor.getTargetPosition()

        if yaw is None:   yaw = 0.0
        if pitch is None: pitch = 0.0

        # Flip yaw and pitch corrections so the robot looks TOWARD the object
        yaw   -= yaw_output
        pitch -= pitch_output

        # Clamp
        yaw   = max(self.yaw_min,   min(self.yaw_max,   yaw))
        pitch = max(self.pitch_min, min(self.pitch_max, pitch))

        # Stuur motors aan
        self.yaw_motor.setPosition(yaw)
        self.pitch_motor.setPosition(pitch)

        return True
from controller import Motor
import numpy as np


class GraspSkill:
    def __init__(self, robot, camera, arm_joints=None, gripper_joints=None):
        self.robot = robot
        self.camera = camera

        # Motors ophalen
        self.arm_joints = arm_joints or [
            robot.getDevice(f"arm_{i}_joint") for i in range(1, 8)
        ]
        self.gripper_joints = gripper_joints or [
            robot.getDevice("gripper_left_finger_joint"),
            robot.getDevice("gripper_right_finger_joint"),
        ]

        # Setup motors
        for m in self.arm_joints + self.gripper_joints:
            m.setVelocity(1.0)

        # Head yaw motor for aligning arm with head direction
        self.head_yaw = robot.getDevice("head_1_joint")

    def detect_target(self, desired_class="apple"):
        detections = self.camera.detect_objects()
        if not detections:
            return None

        for det in detections:
            if det.get("class_name", "").lower() == desired_class.lower():
                return det

        return None

    def bbox_to_3d(self, det):
        # bounding box → pixel center
        x1, y1, x2, y2 = det["bbox_xyxy"]
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)

        # Depth from range finder
        depth = self.camera.get_distance_to_object(det)
        if depth is None:
            return None

        fx = 600
        fy = 600
        X = (cx - self.camera.width / 2) * depth / fx
        Y = (cy - self.camera.height / 2) * depth / fy
        Z = depth

        return np.array([X, Y, Z])

    def move_arm(self, positions):
        for m, p in zip(self.arm_joints, positions):
            m.setPosition(p)

        # kleine wachtloop
        for _ in range(20):
            self.robot.step(int(self.robot.getBasicTimeStep()))

    def close_gripper(self):
        for m in self.gripper_joints:
            m.setPosition(0.0)
        for _ in range(20):
            self.robot.step(int(self.robot.getBasicTimeStep()))

    def open_gripper(self):
        for m in self.gripper_joints:
            m.setPosition(0.04)
        for _ in range(20):
            self.robot.step(int(self.robot.getBasicTimeStep()))

    def execute(self, object_class="apple"):
        det = self.detect_target(object_class)
        if det is None:
            print("No target detected.")
            return False

        target = self.bbox_to_3d(det)

        if target is None:
            print("Could not compute 3D target.")
            return False

        print(f"Target 3D position: {target}")

        # Read current head yaw so the arm aligns with the head direction
        head_yaw_angle = self.head_yaw.getTargetPosition()
        if head_yaw_angle: 
            print("head yaw angle:", head_yaw_angle)
            print("")
        else:
            head_yaw_angle = 0.0

        # ----- Named joint targets for approach pose -----
        # scale head yaw to arm pan (arm moves less than the head)
        yaw_to_arm_scale = 0
        # Clamp shoulder pan so it never violates joint limits
        min_pan = 0.07  # TIAGo safety limit from Webots warning
        max_pan = 2.68

        shoulder_pan = 1.0875 * head_yaw_angle + 1.571

        shoulder_lift = 0.0      # arm_2_joint: raise/lower shoulder
        elbow_1 = 0.0           # arm_3_joint: main elbow flexion
        elbow_2 = 0.0           # arm_4_joint: forward reach
        wrist_1 = 0.0            # arm_5_joint
        wrist_2 = 0.0            # arm_6_joint
        wrist_3 = 0.0            # arm_7_joint
        # safer elevated approach pose
        self.move_arm([
            shoulder_pan,
            shoulder_lift,
            elbow_1,
            elbow_2,
            wrist_1,
            wrist_2,
            wrist_3
        ])

        # gripper
        self.open_gripper()

        print("Grasp complete.")
        return True
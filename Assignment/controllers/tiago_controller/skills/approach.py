import numpy as np

class ApproachSkill:
    def __init__(self, robot, camera, target_distance=0.4,
                 forward_speed=5.0, turn_speed=3.0):
        self.robot = robot
        self.camera = camera
        self.target_distance = target_distance
        self.forward_speed = forward_speed
        self.turn_speed = turn_speed

        # Motors
        self.left_motor  = robot.getDevice("wheel_left_joint")
        self.right_motor = robot.getDevice("wheel_right_joint")

        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))

    def stop(self):
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

    def approach(self, class_name="apple"):
        """
        Beweeg naar een object tot de doelafstand bereikt is.
        Retourneert True tijdens beweging, False wanneer voltooid of object verloren.
        """

        det = self.camera.detect_object(class_name)

        if det is None:
            print("Geen object gedetecteerd")
            self.stop()
            return False

        distance = self.camera.get_distance_to_object(det)

        if distance is None or np.isnan(distance):
            print("No valid depth")
            self.stop()
            return False

        x1, y1, x2, y2 = det["bbox_xyxy"]
        cx = (x1 + x2) / 2

        norm_x = (cx - self.camera.width / 2) / self.camera.width

        # check of we in de buurt zijn van de target distance
        error = distance - self.target_distance

        if error < 0.05:
            print("object bereikt")
            self.stop()
            return False

        forward = np.clip(error * 2.0, 0.0, self.forward_speed)
        turn = np.clip(-norm_x * 4.0, -self.turn_speed, self.turn_speed)

        v_l = forward - turn
        v_r = forward + turn

        self.left_motor.setVelocity(v_l)
        self.right_motor.setVelocity(v_r)

        return True

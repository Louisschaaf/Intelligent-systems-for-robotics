from controller import Supervisor, Keyboard
from knowledge.kg import KG
from perception.detect import CameraDetection
from perception.lidar import Lidar
import numpy as np
from skills.grasp import GraspSkill
from skills.head_follow import HeadFollowSkill
from skills.approach import ApproachSkill


def main():
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())

    # --- Keyboard ---
    kb = robot.getKeyboard()
    kb.enable(timestep)

    # --- Perception (mag blijven werken) ---
    cam = CameraDetection(robot, camera_name="Astra rgb", model_path="yolo11n.pt")
    cam.enable(timestep)

    # --- Skills ---
    grasp = GraspSkill(robot, cam)
    head_follow = HeadFollowSkill(robot, cam)
    approach = ApproachSkill(robot, cam, target_distance=0.4)

    lidar = Lidar(
        robot,
        lidar_def="Hokuyo URG-04LX-UG01",
        lidar_offset=(0.202, 0.0)
    )
    lidar.enable(timestep)

    # --- Lidar hoogte fixen ---
    tiago_node = robot.getFromDef("TIAGo")
    lidar_slot_field = tiago_node.getField("lidarSlot")
    lidar_node = lidar_slot_field.getMFNode(0)
    translation_field = lidar_node.getField("translation")
    x, y, z = translation_field.getSFVec3f()
    translation_field.setSFVec3f([x, y, z + 0.29])

    # --- Motors ophalen ---
    left_motor  = robot.getDevice("wheel_left_joint")
    right_motor = robot.getDevice("wheel_right_joint")

    left_motor.setPosition(float("inf"))
    right_motor.setPosition(float("inf"))

    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    print("Keyboard control active.")

    v_forward = 5.0     # vooruit snelheid
    v_turn    = 3.0     # draaiverschil

    while robot.step(timestep) != -1:

        cam.get_image()
        lidar.update_global_map()
        key = kb.getKey()

        # --- Skills ---
        if key == ord('G'):
            grasp.execute("apple")

        head_follow.track("apple")   

        approaching = False
        approaching = approach.approach("apple")

        # --- Keyboard control ---
        if not approaching:
            v_l = 0.0
            v_r = 0.0

            if key == Keyboard.UP:
                v_l = v_forward
                v_r = v_forward
            elif key == Keyboard.DOWN:
                v_l = -v_forward
                v_r = -v_forward
            elif key == Keyboard.LEFT:
                v_l = -v_turn
                v_r =  v_turn
            elif key == Keyboard.RIGHT:
                v_l =  v_turn
                v_r = -v_turn

            left_motor.setVelocity(v_l)
            right_motor.setVelocity(v_r)



if __name__ == "__main__":
    main()


from controller import Supervisor
from knowledge.kg import KG
from planning.pddl_problem import ProblemBuilder
from planning.planner import Planner
from exec.executor import Executor
from perception.detect import CameraDetection
from perception.lidar import Lidar
import numpy as np


def main():
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())

    # --- Perception devices ---
    cam = CameraDetection(robot, camera_name="Astra rgb", model_path="yolo11n.pt")
    cam.enable(timestep)

    # LiDAR: offset in het horizontale vlak in het robotframe
    # (hier gebruik je dus alleen de horizontale componenten van je 3D-offset)
    lidar = Lidar(
        robot,
        lidar_def="Hokuyo URG-04LX-UG01",
        lidar_offset=(0.202, 0.0)    # eventueel aanpassen naar (0.202, 0.286) afhankelijk van je assenkeuze
    )
    lidar.enable(timestep)

    # Zoek de Lidar-node op basis van de DEF-naam (zonder spaties!)
    # 1) Haal de TIAGo-robot op (DEF-naam in de Scene Tree)
    tiago_node = robot.getFromDef("TIAGo")
    if tiago_node is None:
        raise RuntimeError("Robot node met DEF 'TIAGo' niet gevonden.")

    # 2) Pak het 'lidarSlot' field (MFNode)
    lidar_slot_field = tiago_node.getField("lidarSlot")
    if lidar_slot_field is None:
        raise RuntimeError("Field 'lidarSlot' niet gevonden in TIAGo-node.")

    # 3) In lidarSlot[0] zit jouw HokuyoUrg04lxug01-node
    lidar_node = lidar_slot_field.getMFNode(0)
    if lidar_node is None:
        raise RuntimeError("Geen LiDAR-node gevonden in 'lidarSlot'.")

    translation_field = lidar_node.getField("translation")
    if translation_field is None:
        raise RuntimeError("Lidar node heeft geen 'translation' field.")

    # Huidige translation uitlezen
    x, y, z = translation_field.getSFVec3f()
    # 0.29 m hoger langs de hoogterichting (bij jou z)
    translation_field.setSFVec3f([x, y, z + 0.29])

    # --- Knowledge graph / planning / executor zoals jij die had ---
    kg = KG()
    kg.add_waypoint("wp_a", "WP_A")
    kg.add_waypoint("wp_b", "WP_B")
    kg.add_waypoint("wp_c", "WP_C")
    kg.set_path(["wp_a", "wp_b", "wp_c"])

    executor = Executor(robot=robot, kg=kg)
    planner  = Planner()

    domain_path_hint = "controllers/tiago_controller/planning/pddl_domain.pddl"
    pb = ProblemBuilder(kg, domain_path=domain_path_hint)
    domain_path, problem_path = pb.build_from_path()

    plan = planner.solve(domain_path, problem_path)
    executor.set_plan(plan)

    tick = 0
    while robot.step(timestep) != -1:
        tick += 1

        # 1) Perception
        cam.get_image()
        if tick % 3 == 0:
            _ = cam.detect_objects()

        lidar.update_global_map()
        lidar.visualize_global_map(xlim=(-15, 15), ylim=(-15, 15))

        print(f"Min distance to all objects: {cam.get_min_distance()} meters")

        # 2) Replanning indien nodig
        if executor.needs_plan():
            domain_path, problem_path = pb.build_from_path()
            plan = planner.solve(domain_path, problem_path)
            executor.set_plan(plan)

        # 3) Planstap uitvoeren
        executor.step()


if __name__ == "__main__":
    main()
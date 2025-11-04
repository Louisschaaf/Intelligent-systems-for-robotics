from controller import Supervisor
from knowledge.kg import KG
from planning.pddl_problem import ProblemBuilder
from planning.planner import Planner
from exec.executor import Executor
from perception.detect import CameraDetection
from perception.lidar import Lidar

def main():
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())

    # --- Perception devices ---
    cam = CameraDetection(robot, camera_name="Astra rgb", model_path="yolo11n.pt")
    cam.enable(timestep)

    # Zorg dat de device-naam exact overeenkomt met je LiDAR in de .wbt
    lidar = Lidar(robot, lidar_def="Hokuyo URG-04LX-UG01")
    lidar.enable(timestep)

    # --- Knowledge graph met symbolische plaatsen (waypoints) ---
    kg = KG()
    # Koppel symbolen aan DEF-namen in je .wbt (LET OP: DEF-namen hoofdlettergevoelig)
    kg.add_waypoint("wp_a", "WP_A")
    kg.add_waypoint("wp_b", "WP_B")
    kg.add_waypoint("wp_c", "WP_C")
    # Definieer het pad A -> B -> C
    kg.set_path(["wp_a", "wp_b", "wp_c"])

    # --- Planning en executie ---
    executor = Executor(robot=robot, kg=kg)
    planner  = Planner()

    # Bouw PDDL-problem op basis van de ketting uit de KG
    # Pas het pad hieronder aan naar jouw pddl_domain.pddl
    domain_path_hint = "controllers/tiago_controller/planning/pddl_domain.pddl"
    pb = ProblemBuilder(kg, domain_path=domain_path_hint)
    domain_path, problem_path = pb.build_from_path()

    # Vraag een plan en zet het klaar
    plan = planner.solve(domain_path, problem_path)
    executor.set_plan(plan)

    # Timestep loop
    tick = 0
    while robot.step(timestep) != -1:
        tick += 1

        # 1) Perception (non-blocking)
        cam.get_image()              # YOLO inference kan je intern throttlen (bv. every_n_frames=3)
        if tick % 3 == 0:
            _ = cam.detect_objects() # eventueel gebruiken om later KG te vullen

        # LiDAR eenvoudige 2D-visualisatie (throttle om UI te sparen)
        if tick % 5 == 0:
            lidar.visualize_2d()

        # 2) Replanning indien nodig (hier niet verwacht: pad is statisch)
        if executor.needs_plan():
            # In dit A->B->C scenario normaal niet nodig, maar contract blijven respecteren
            domain_path, problem_path = pb.build_from_path()
            plan = planner.solve(domain_path, problem_path)
            executor.set_plan(plan)

        # 3) Planstap uitvoeren via Navigate-skill
        executor.step()

if __name__ == "__main__":
    main()
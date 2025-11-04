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

    kg = KG()
    cam = CameraDetection(robot, "Astra rgb")
    cam.enable(timestep)

    lidar = Lidar(robot, "Hokuyo URG-04LX-UG01")
    lidar.enable(timestep)

    me = robot.getSelf()

    executor = Executor(robot=robot, kg=kg)
    planner  = Planner()

    goal = {"type": "move", "place": "room2"}  # start simpel; later place_on

    while robot.step(timestep) != -1:
        # 1) Perception
        cam.get_image()
        kg.assert_robot_at("tiago", kg.map_position_to_area(me.getPosition()))
        lidar.visualize_2d()
        
        # 2) Planning trigger
        if executor.needs_plan():
            pb = ProblemBuilder(kg, goal)
            domain, problem = pb.build()
            plan = planner.solve(domain, problem)
            executor.set_plan(plan)

        # 3) Execute
        executor.step()

if __name__ == "__main__":
    main()
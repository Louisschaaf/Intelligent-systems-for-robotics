from controller import Supervisor
from skills.follow import FollowSkill
from knowledge.kg import KG
from perception.detect import CameraDetection

def main():
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())
    kg = KG()
    camera = CameraDetection(robot, "Astra")
    camera.enable(timestep)

    # TIAGo moet supervisor zijn in je .wbt
    me = robot.getSelf()
    if me is None:
        raise RuntimeError("Zet 'supervisor TRUE' op de Tiago node in je world.")

    # Start de follow-skill: volg DEF TARGET
    skill = FollowSkill(robot=robot, kg=kg, target_def="TARGET")
    skill.setup()

    while robot.step(timestep) != -1:
        camera.get_image()
        skill.step()
        detection = camera.detect_object("chair")
        if detection:
            print("Gevonden stoel:", detection)

if __name__ == "__main__":
    main()
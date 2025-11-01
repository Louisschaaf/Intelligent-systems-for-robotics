from controller import Supervisor
from skills.follow import FollowSkill

def main():
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())

    # TIAGo moet supervisor zijn in je .wbt
    me = robot.getSelf()
    if me is None:
        raise RuntimeError("Zet 'supervisor TRUE' op de Tiago node in je world.")

    # Start de follow-skill: volg DEF TARGET
    skill = FollowSkill(robot=robot, target_def="TARGET")
    skill.setup()

    while robot.step(timestep) != -1:
        skill.step()

if __name__ == "__main__":
    main()
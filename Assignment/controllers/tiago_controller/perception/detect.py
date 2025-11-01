# Volledige, minimale stub: geeft ground-truth pose van DEF-nodes terug.
from controller import Supervisor

def get_pose_from_def(robot: Supervisor, def_name: str):
    node = robot.getFromDef(def_name)
    if node is None:
        raise RuntimeError(f"DEF {def_name} niet gevonden.")
    return node.getPosition(), node.getOrientation()
from typing import List, Dict
from knowledge.kg import KG

class Planner:
    """
    Dummy planner: genereert een simpel plan op kamer-niveau.
    Vervang later door een echte FD/FF wrapper die planregels parst.
    """
    def __init__(self):
        pass

    def solve(self, domain_path: str, problem_path: str) -> List[Dict]:
        # Minimalistisch: als goal type 'move' of 'place_on', produceer enkele 'move' stappen.
        # Je kunt dit vervangen door subprocess aanroep naar Fast Downward.
        # Voor nu: navigeer naar goal.place, en voeg evt. pickup/place stappen toe.
        # NB: we hebben hier geen toegang tot KG; daarom is dit puur "naar goal room".
        # In je controller weet je welk goal je gaf.
        # Om toch iets zinnigs te doen: parse problem.pddl heel grof om goal-room te vinden.
        plan: List[Dict] = []
        txt = open(problem_path, "r", encoding="utf-8").read()
        goal_room = None
        # simpel parse: zoek "(at tiago roomX)"
        for tok in txt.replace("(", " ").replace(")", " ").split():
            if tok.startswith("room"):
                goal_room = tok
                break
        if goal_room:
            plan.append({"name": "move", "args": {"to": goal_room}})
        return plan
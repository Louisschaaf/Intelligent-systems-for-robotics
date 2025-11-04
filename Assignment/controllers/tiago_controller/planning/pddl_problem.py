import tempfile
from pathlib import Path
from rdflib import RDF
from knowledge.kg import HOME, AREAS

class ProblemBuilder:
    def __init__(self, kg, goal: dict, domain_path: str = None):
        self.kg = kg
        self.goal = goal
        self.domain_path = domain_path or str(Path(__file__).with_name("pddl_domain.pddl"))

    def build(self):
        robots, places, items, supports = set(), set(), set(), set()

        # rooms uit AREAS
        places.update(AREAS.keys())

        # type-inferentie uit KG
        for s, p, o in self.kg.g:
            s_name = s.split("#")[-1]
            if (s, RDF.type, HOME.Robot) in self.kg.g:
                robots.add(s_name)
            if (s, RDF.type, HOME.Place) in self.kg.g:
                places.add(s_name)
            if (s, RDF.type, HOME.Item) in self.kg.g:
                items.add(s_name)
            if (s, RDF.type, HOME.Support) in self.kg.g:
                supports.add(s_name)

        if not robots:
            robots.add("tiago")

        init_facts = self.kg.to_pddl_init()
        # simpele adjacency (pas aan als je echte graaf hebt)
        init_facts += [
            "(adjacent room1 room2)", "(adjacent room2 room1)",
            "(adjacent room1 room3)", "(adjacent room3 room1)",
            "(adjacent room2 room4)", "(adjacent room4 room2)",
            "(adjacent room3 room4)", "(adjacent room4 room3)"
        ]
        if "(freehand tiago)" not in init_facts:
            init_facts.append("(freehand tiago)")

        goal_str = self._goal_to_pddl(self.goal)

        def objs(names, typ):
            return f"{' '.join(sorted(names))} - {typ}" if names else ""

        objects_block = "\n            ".join(filter(None, [
            objs(robots, "robot"), objs(places, "place"),
            objs(supports, "support"), objs(items, "item")
        ]))

        problem = f"""(define (problem auto_problem)
  (:domain home_assistant)
  (:objects
            {objects_block}
  )
  (:init
            {' '.join(init_facts)}
  )
  (:goal {goal_str})
)
"""
        tmp = tempfile.NamedTemporaryFile(delete=False, suffix=".pddl")
        tmp.write(problem.encode("utf-8"))
        tmp.flush()
        return self.domain_path, tmp.name

    def _goal_to_pddl(self, goal: dict) -> str:
        t = goal.get("type")
        if t == "place_on":
            i = goal["item"]; p = goal["place"]; s = goal["support"]
            return f"(and (on {i} {s}) (at {i} {p}) (at tiago {p}))"
        elif t == "move":
            p = goal["place"]
            return f"(at tiago {p})"
        return "(at tiago room2)"
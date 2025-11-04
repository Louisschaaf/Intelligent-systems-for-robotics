import tempfile

class ProblemBuilder:
    def __init__(self, kg, domain_path: str):
        self.kg = kg
        self.domain_path = domain_path

    def build_from_path(self):
        places = self.kg.get_path()
        if not places or len(places) < 2:
            raise ValueError("Pad moet minstens 2 plaatsen bevatten, bv. ['wp_a','wp_b'].")

        objects = f"tiago - robot {' '.join(places)} - place"
        init_facts = [f"(at tiago {places[0]})"]
        for a, b in zip(places, places[1:]):
            init_facts.append(f"(next {a} {b})")
        goal = f"(at tiago {places[-1]})"

        problem = f"""(define (problem hall_path)
  (:domain hall_nav)
  (:objects {objects})
  (:init {' '.join(init_facts)})
  (:goal {goal})
)"""

        tmp = tempfile.NamedTemporaryFile(delete=False, suffix=".pddl")
        tmp.write(problem.encode("utf-8"))
        tmp.flush()
        return self.domain_path, tmp.name
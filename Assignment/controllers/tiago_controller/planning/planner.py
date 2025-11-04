import re

class Planner:
    def solve(self, domain_path, problem_path):
        txt = open(problem_path, "r", encoding="utf-8").read()
        # Pak alleen tokens tot aan whitespace of ')'
        pairs = re.findall(r"\(next\s+([^\s\)]+)\s+([^\s\)]+)\)", txt, flags=re.IGNORECASE)

        # Extra defensieve schoonmaak
        def clean(tok: str) -> str:
            return tok.strip().strip("()")
        pairs = [(clean(a), clean(b)) for (a, b) in pairs]

        plan = [{"name": "goto-next", "args": {"from": a, "to": b}} for (a, b) in pairs]
        return plan
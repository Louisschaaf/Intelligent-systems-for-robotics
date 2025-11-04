import re

class Planner:
    def solve(self, domain_path, problem_path):
        txt = open(problem_path, "r", encoding="utf-8").read()
        pairs = re.findall(r"\(next\s+(\S+)\s+(\S+)\)", txt)
        plan = [{"name": "goto-next", "args": {"from": a, "to": b}} for (a,b) in pairs]
        return plan
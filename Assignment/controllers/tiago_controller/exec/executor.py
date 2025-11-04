# executor.py
from typing import List, Dict, Any, Optional

from skills.navigate import NavigateSkill

class Executor:
    def __init__(self, robot, kg):
        self.robot = robot
        self.kg = kg
        self.plan: List[Dict[str, Any]] = []
        self.idx = -1
        self.active_skill = None
        self.active_action: Optional[Dict[str, Any]] = None
        self._navigate: Optional[NavigateSkill] = None

    def needs_plan(self) -> bool:
        return not self.plan

    def set_plan(self, plan: List[Dict[str, Any]]):
        self.plan = plan or []
        self.idx = -1
        self.active_skill = None
        self.active_action = None

    def _ensure_navigate(self):
        if self._navigate is None:
            self._navigate = NavigateSkill(robot=self.robot, kg=self.kg)
            self._navigate.setup()

    def _start_next(self):
        self.idx += 1
        if self.idx >= len(self.plan):
            self.active_action = None
            self.active_skill = None
            self.plan = []
            return
        self.active_action = self.plan[self.idx]
        name = self.active_action.get("name", "").lower()
        args = self.active_action.get("args", {})

        if name == "goto-next":
            self._ensure_navigate()
            to_symbol = args.get("to")
            if not to_symbol:
                print("[Executor] goto-next zonder 'to'")
                self._start_next()
                return
            self._navigate.start_place(to_symbol)
            self.active_skill = self._navigate
        else:
            print(f"[Executor] Onbekende actie: {name}, sla over.")
            self._start_next()

    def step(self):
        if not self.plan:
            return
        if self.active_skill is None:
            self._start_next()
            return

        status = self.active_skill.step() or "busy"

        if status == "done":
            # optioneel KG-updates
            if isinstance(self.active_skill, NavigateSkill):
                place = self.active_skill.current_goal_place
                if place:
                    self.kg.assert_robot_at("tiago", place)
            self.active_skill = None
            self._start_next()

        elif status == "failed":
            print("[Executor] Skill fail → replanning noodzakelijk.")
            self.plan = []
            self.active_skill = None
            self.active_action = None
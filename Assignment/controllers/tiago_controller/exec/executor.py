from typing import List, Dict, Any, Optional
from skills.follow import FollowSkill

try:
    from skills.navigate import NavigateSkill
except Exception:
    NavigateSkill = None  # als navigate nog niet bestaat

class Executor:
    """
    Zet geplande acties om naar skills en bewaakt pre/post-condities heel minimaal.
    Verwacht plan in vorm van list[{"name": "move", "args": {"to": "room2"}} , ...]
    """
    def __init__(self, robot, kg):
        self.robot = robot
        self.kg = kg
        self.plan: List[Dict[str, Any]] = []
        self.idx = -1
        self.active_skill = None
        self.active_action: Optional[Dict[str, Any]] = None

        # Lazy init; follow gebruiken we enkel als er een TARGET is
        self._follow = None
        self._navigate = None

    def needs_plan(self) -> bool:
        return not self.plan

    def set_plan(self, plan: List[Dict[str, Any]]):
        self.plan = plan or []
        self.idx = -1
        self.active_skill = None
        self.active_action = None

    def _ensure_follow(self):
        if self._follow is None:
            self._follow = FollowSkill(robot=self.robot, kg=self.kg, target_def="TARGET")
            self._follow.setup()

    def _ensure_navigate(self):
        if self._navigate is None:
            if NavigateSkill is None:
                raise RuntimeError("NavigateSkill ontbreekt. Voeg skills/navigate.py toe.")
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

        if name in ("move", "navigate"):
            self._ensure_navigate()
            target_place = args.get("to") or args.get("place")
            self._navigate.start(target_place)
            self.active_skill = self._navigate

        elif name == "follow":
            self._ensure_follow()
            # FollowSkill volgt continu; geen extra args nodig in deze minimal
            self.active_skill = self._follow

        else:
            # Onbekend: sla stap over (of raise)
            print(f"[Executor] Onbekende actie: {name}, sla over.")
            self._start_next()

    def step(self):
        # Geen plan? niets te doen
        if not self.plan:
            return

        # Als geen actieve skill: start volgende
        if self.active_skill is None:
            self._start_next()
            return

        # Run actieve skill 1 timestep
        status = getattr(self.active_skill, "step")()

        # Conventie: skill.step() geeft "busy" | "done" | "failed" of None (interpreteer als busy)
        if status is None:
            status = "busy"

        if status == "done":
            # heel eenvoudige post-effect update:
            if isinstance(self.active_skill, type(self._navigate)):
                # nav: als we net klaar zijn, zet robot "at" in KG
                place = self.active_skill.current_goal_place
                if place:
                    self.kg.assert_robot_at("tiago", place)
            self.active_skill = None
            self._start_next()

        elif status == "failed":
            print("[Executor] Skill fail → vraag om replanning.")
            # maak plan leeg zodat controller een replan triggert
            self.plan = []
            self.active_skill = None
            self.active_action = None
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

        # fase: None | "to_from" | "to_to"
        self._phase: Optional[str] = None

    def needs_plan(self) -> bool:
        return not self.plan

    def set_plan(self, plan: List[Dict[str, Any]]):
        self.plan = plan or []
        self.idx = -1
        self.active_skill = None
        self.active_action = None
        self._phase = None

    def _ensure_navigate(self):
        if self._navigate is None:
            self._navigate = NavigateSkill(robot=self.robot, kg=self.kg)
            self._navigate.setup()

    def _start_next(self):
        """Start volgende planstap. Elke goto-next wordt in twee fasen uitgevoerd:
           1) 'to_from': navigeer naar 'from'
           2) 'to_to'  : navigeer naar 'to'
        """
        self.idx += 1
        if self.idx >= len(self.plan):
            self.active_action = None
            self.active_skill = None
            self._phase = None
            self.plan = []
            return

        self.active_action = self.plan[self.idx]
        name = self.active_action.get("name", "").lower()
        args = self.active_action.get("args", {})

        if name == "goto-next":
            self._ensure_navigate()
            from_sym = args.get("from")
            if not from_sym:
                print("[Executor] goto-next zonder 'from'")
                self._start_next()
                return
            # Fase 1: altijd eerst naar 'from'
            self._navigate.start_place(from_sym)
            self.active_skill = self._navigate
            self._phase = "to_from"
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
            # optioneel: KG bijwerken met waar we net heen reden
            if isinstance(self.active_skill, NavigateSkill):
                place = self.active_skill.current_goal_place
                if place:
                    self.kg.assert_robot_at("tiago", place)

            # Fase management
            if self._phase == "to_from":
                # Start fase 2: navigeer naar 'to' van dezelfde actie (idx NIET verhogen)
                to_sym = self.active_action.get("args", {}).get("to")
                if not to_sym:
                    print("[Executor] goto-next zonder 'to'")
                    # sla deze actie over en ga door naar volgende
                    self.active_skill = None
                    self._phase = None
                    self._start_next()
                    return
                self._navigate.start_place(to_sym)
                self.active_skill = self._navigate
                self._phase = "to_to"
                return

            elif self._phase == "to_to":
                # Volledige goto-next is klaar → volgende planstap
                self.active_skill = None
                self._phase = None
                self._start_next()
                return

            else:
                # Onverwachte state; ga naar volgende stap
                self.active_skill = None
                self._phase = None
                self._start_next()
                return

        elif status == "failed":
            print("[Executor] Skill fail → replanning noodzakelijk.")
            self.plan = []
            self.active_skill = None
            self.active_action = None
            self._phase = None
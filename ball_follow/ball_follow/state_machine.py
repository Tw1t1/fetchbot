from enum import Enum
from typing import Dict, Callable, Any

class StateMachine:
    def __init__(self, initial_state: Enum, state_actions: Dict[Enum, Callable[[], Any]]):
        self.current_state = initial_state
        self.state_actions = state_actions

    def update(self) -> Any:
        if self.current_state in self.state_actions:
            return self.state_actions[self.current_state]()
        else:
            raise ValueError(f"No action defined for state {self.current_state}")

    def transition_to(self, new_state: Enum):
        self.current_state = new_state

    def get_current_state(self) -> Enum:
        return self.current_state
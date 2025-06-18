from autonomous.auto_base import AutoBase
from components.chute import Chute
from components.drivetrain import Drivetrain
from components.arm import Arm

from magicbot import AutonomousStateMachine, state, timed_state




class test_L(AutoBase):
    MODE_NAME = "testL"

    def __init__(self):
        super().__init__(["test-testl"])

class l1(AutoBase):
    MODE_NAME = "Coral Drop"

    def __init__(self):
        super().__init__([
            "mstart-l1",
            "state:drop_coral",
            ]
        )
        

class DelayLeave(AutonomousStateMachine):
    MODE_NAME = "delay_leave"

    drivetrain: Drivetrain

    @timed_state(duration=10, first=True, next_state="going")
    def staying(self):
        self.drivetrain.drive(0, 0)

    @timed_state(duration=1, next_state="stopping")
    def going(self):
        self.drivetrain.drive(1, 0)

    @state
    def stopping(self):
        self.drivetrain.drive(0, 0)

class Leave(AutonomousStateMachine):
    MODE_NAME = "Leave"

    drivetrain: Drivetrain

    @timed_state(duration=1, first=True, next_state="stopped")
    def leaving(self):
        self.drivetrain.drive(1, 0)

    @state
    def stopped(self):
        self.drivetrain.drive(0, 0)
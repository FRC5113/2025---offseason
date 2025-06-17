from phoenix6.hardware import TalonFX
from phoenix5 import TalonSRX, TalonSRXControlMode
from phoenix6.configs import TalonFXConfiguration
from phoenix6.signals import NeutralModeValue
from magicbot import will_reset_to
from rev import SparkAbsoluteEncoder
import enum
from lemonlib.smart import SmartProfile
from lemonlib import fms_feedback
from lemonlib.util import Alert, AlertManager, AlertType
from wpimath import units
from .arm import Arm, ArmAngle
from magicbot import StateMachine, will_reset_to
from magicbot.state_machine import state


class ArmController(StateMachine):
    arm: Arm

    arm_setpoint = will_reset_to(ArmAngle.UP.value)
    intake_request = will_reset_to(False)
    shoot_request = will_reset_to(False)

    def setup(self):
        self.engage()

    def request_angle(self, angle):
        self.arm_setpoint = angle

    def request_intake(self):
        self.intake_request = True

    def request_shoot(self):
        self.shoot_request = True

    @state(first=True)
    def standby(self):
        self.arm.request_brake()
        self.arm.set_target_angle(self.arm_setpoint)
        if self.intake_request:
            self.arm.set_arm_speed(1)
            self.next_state("intake_fall")
        if not self.arm.at_setpoint():
            self.next_state("positioning_arm")

    @state
    def positioning_arm(self):
        self.arm.request_brake()
        self.arm.set_target_angle(self.arm_setpoint)
        if self.arm.at_setpoint():
            self.next_state("standby")

    @state
    def shooting(self):
        self.arm.request_brake()
        self.arm.set_target_angle(ArmAngle.UP.value)
        if self.arm.at_setpoint():
            self.arm.set_intake_speed(1)
        if not self.shoot_request:
            self.next_state("standby")

    @state
    def intake_fall(self):
        self.arm.request_coast()
        self.arm.set_arm_speed(0)
        self.arm.set_intake_speed(-1)
        if not self.intake_request:
            self.next_state("standby")

    @state
    def arm_failsafe(self):
        self.arm.request_brake()

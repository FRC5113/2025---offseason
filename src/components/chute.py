from phoenix6.hardware import TalonFX
from phoenix5 import TalonSRX, TalonSRXControlMode
from phoenix6.configs import TalonFXConfiguration
from phoenix6.signals import NeutralModeValue
from magicbot import will_reset_to
from wpilib import DutyCycleEncoder
import enum
from lemonlib.smart import SmartProfile
from wpimath import units


class ChuteAngle(enum.IntEnum):
    CLOSED = 90
    FLAT = 135
    OPEN = 180


class Chute:
    hinge_motor: TalonSRX
    encoder: DutyCycleEncoder
    profile: SmartProfile

    hinge_voltage = will_reset_to(0)
    requested_angle = will_reset_to(ChuteAngle.CLOSED)

    def setup(self):
        pass

    def on_enable(self):
        self.controller = self.profile.create_arm_controller("Chute")

    def open(self):
        self.requested_angle = ChuteAngle.OPEN

    def request_angle(self, angle: units.degrees):
        self.requested_angle = angle

    def execute(self):
        self.hinge_voltage = self.controller.calculate(
            self.encoder.get(), self.requested_angle
        )

        if (
            self.encoder.get() > ChuteAngle.OPEN
            or self.encoder.get() < ChuteAngle.CLOSED
        ):
            self.hinge_voltage = 0

        self.hinge_motor.set(TalonSRXControlMode.PercentOutput, self.hinge_voltage)

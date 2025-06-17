from phoenix6.hardware import TalonFX
from phoenix5 import TalonSRX, TalonSRXControlMode, NeutralMode
from phoenix6.configs import TalonFXConfiguration
from phoenix6.signals import NeutralModeValue
from magicbot import will_reset_to
from wpilib import DutyCycleEncoder
import enum
from lemonlib.smart import SmartPreference
from wpimath import units


class Chute:
    motor: TalonSRX

    speed = will_reset_to(0)
    chute_sped = SmartPreference(1)

    def setup(self):
        self.motor.setNeutralMode(NeutralMode.Brake)

    def request_speed(self, speed):
        self.speed = speed * self.chute_sped

    def execute(self):
        self.motor.set(TalonSRXControlMode.PercentOutput, self.speed)

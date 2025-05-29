from phoenix6.hardware import TalonFX
from phoenix5 import TalonSRX, TalonSRXControlMode
from phoenix6.configs import TalonFXConfiguration
from phoenix6.signals import NeutralModeValue
from magicbot import will_reset_to
from wpilib import DutyCycleEncoder
import enum
from lemonlib.smart import SmartProfile
from wpimath import units


class ArmAngle(enum.IntEnum):
    UP = 90
    DOWN = 0
    INTAKE = 45
    SAFESTART = -1
    SAFEEND = 91


class Arm:
    motor: TalonFX
    encoder: DutyCycleEncoder
    intake_motor: TalonSRX
    profile: SmartProfile

    intake_speed = will_reset_to(0)
    arm_speed = will_reset_to(0)
    target_angle = will_reset_to(90)
    manual_control = will_reset_to(False)

    def setup(self):
        self.motor_configs = TalonFXConfiguration()
        self.motor_configs.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.motor.configurator.apply(self.motor_configs)

    def on_enable(self):
        self.controller = self.profile.create_arm_controller("arm")

    """
    CONTROL METHODS"""

    def set_intake_speed(self, intake_speed):
        self.intake_speed = intake_speed

    def set_arm_speed(self, speed):
        self.arm_speed = speed

    def set_target_angle(self, angle: units.degrees):
        self.target_angle = angle
        self.manual_control = True

    def execute(self):
        self.arm_speed = self.controller.calculate(
            self.encoder.get(), self.target_angle
        )
        print(
            f"Arm Speed: {self.arm_speed}, Target Angle: {self.target_angle}, Encoder: {self.encoder.get()}"
        )
        # if (
        #     self.encoder.get() > ArmAngle.SAFEEND.value
        #     or self.encoder.get() < ArmAngle.SAFEEND.value
        # ):
        #     self.arm_speed = 0
        self.motor.set(self.arm_speed)
        self.intake_motor.set(TalonSRXControlMode.PercentOutput, self.intake_speed)

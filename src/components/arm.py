from phoenix6.hardware import TalonFX
from phoenix5 import TalonSRX, TalonSRXControlMode
from phoenix6.configs import TalonFXConfiguration
from phoenix6.signals import NeutralModeValue
from magicbot import will_reset_to
from rev import SparkAbsoluteEncoder
import enum
from lemonlib.smart import SmartProfile
from lemonlib import fms_feedback
from wpimath import units


class ArmAngle(enum.IntEnum):
    UP = 0
    DOWN = 48
    INTAKE = 45


class Arm:
    motor: TalonFX
    encoder: SparkAbsoluteEncoder
    intake_motor: TalonSRX
    profile: SmartProfile

    intake_speed = will_reset_to(0)
    arm_speed = will_reset_to(0)
    target_angle = will_reset_to(ArmAngle.UP.value)
    manual_control = will_reset_to(False)

    def setup(self):
        self.motor_configs = TalonFXConfiguration()
        self.motor_configs.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.motor.configurator.apply(self.motor_configs)

    def on_enable(self):
        self.controller = self.profile.create_arm_controller("arm")

    @fms_feedback
    def get_angle(self) -> units.degrees:
        """Return the angle of the hinge normalized to [-180,180].
        An angle of 0 refers to the claw in the up/stowed position.
        """
        angle = self.encoder.getPosition() * 360
        if angle > 180:
            angle -= 360
        return angle

    """
    CONTROL METHODS
    """

    def set_intake_speed(self, intake_speed):
        self.intake_speed = intake_speed

    def set_arm_speed(self, speed):
        self.arm_speed = speed

    def set_target_angle(self, angle: units.degrees):
        self.target_angle = angle
        self.manual_control = True

    def execute(self):
        self.arm_speed = self.controller.calculate(
            self.get_angle(), self.target_angle
        )
        if (
            self.get_angle() > ArmAngle.DOWN.value
        ):
            self.arm_speed = 0
        self.motor.set(self.arm_speed)
        self.intake_motor.set(TalonSRXControlMode.PercentOutput, self.intake_speed)

from phoenix6.hardware import TalonFX
from phoenix5 import TalonSRX, TalonSRXControlMode
from phoenix6.configs import TalonFXConfiguration
from phoenix6.signals import NeutralModeValue
from magicbot import will_reset_to
from rev import SparkAbsoluteEncoder
import enum
from lemonlib.smart import SmartProfile, SmartPreference
from lemonlib import fms_feedback
from lemonlib.util import AlertManager, Alert, AlertType
from wpimath import units


class ArmAngle(enum.IntEnum):
    UP = 88
    DOWN = 43
    INTAKE = 55
    SAFEBOTTOM = 38
    SAFETOP = 90


class Arm:
    motor: TalonFX
    encoder: SparkAbsoluteEncoder
    intake_motor: TalonSRX
    profile: SmartProfile
    tolerance: units.degrees

    intake_speed = will_reset_to(0)
    arm_speed = will_reset_to(0)
    target_angle = will_reset_to(ArmAngle.UP.value)
    manual_control = will_reset_to(False)

    intake_mult = SmartPreference(1)
    taut_voltage = SmartPreference(0.05)

    def setup(self):
        self.motor_configs = TalonFXConfiguration()
        self.motor_configs.motor_output.neutral_mode = NeutralModeValue.COAST
        self.motor.configurator.apply(self.motor_configs)

    def on_enable(self):
        self.controller = self.profile.create_arm_controller("arm")
        self.controller.setTolerance(self.tolerance)

    @fms_feedback
    def get_angle(self) -> units.degrees:
        """Return the angle of the hinge normalized to [-180,180].
        An angle of 0 refers to the claw in the up/stowed position.
        """
        angle = self.encoder.getPosition() * 360
        if angle > 180:
            angle -= 360
        return 90 - angle

    def at_point(self, angle):
        if abs(self.get_angle - angle) < self.tolerance:
            return True
        return False

    def at_setpoint(self):
        if abs(self.get_angle() - self.target_angle) < self.tolerance:
            return True
        return False

    """
    CONTROL METHODS
    """

    def set_intake_speed(self, intake_speed):
        self.intake_speed = intake_speed * self.intake_mult

    def set_arm_speed(self, voltage):
        self.manual_control = True
        self.arm_speed = voltage

    def set_target_angle(self, angle: units.degrees):
        self.target_angle = angle


    def execute(self):
        current_angle = self.get_angle()

        if not self.manual_control:
            # Only calculate full PID output if below setpoint (needs to pull up)
            if current_angle < self.target_angle:
                self.arm_speed = self.controller.calculate(current_angle, self.target_angle)
            else:
                self.arm_speed = self.taut_voltage

        if (self.get_angle() < ArmAngle.SAFEBOTTOM.value) and (self.arm_speed < 0.0):
            self.arm_speed = 0
        elif (self.get_angle() > ArmAngle.SAFETOP.value) and (self.arm_speed > 0.0):
            self.arm_speed = 0

        if self.arm_speed < 0 and current_angle >= self.target_angle:
            self.arm_speed = 0  # Prevent wrapping rope backward

        self.motor.setVoltage(self.arm_speed)
        self.intake_motor.set(TalonSRXControlMode.PercentOutput, self.intake_speed)

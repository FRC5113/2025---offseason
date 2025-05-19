from phoenix6.hardware import TalonFX
from phoenix5 import TalonSRX
import wpilib
from phoenix6 import controls
from phoenix6.units import ampere
from phoenix6.configs import TalonFXConfiguration, CurrentLimitsConfigs
from phoenix6.hardware import CANcoder, TalonFX
from phoenix6.signals import NeutralModeValue
from magicbot import will_reset_to

class arm:
    motor: TalonFX
    encoder: wpilib.Encoder
    intake_motor: TalonSRX

    intake_speed = will_reset_to(0)
    arm_speed = will_reset_to(0)

    def setup(self):
        self.motor_configs = TalonFXConfiguration()
        self.motor_configs.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.motor.configurator.apply(self.motor_configs)

    def intake(self,intake_speed):
        self.intake_speed = intake_speed

    def move_arm(self,voltage):
        self.arm_speed = voltage

    def execute(self):
        self.motor.set(self.arm_speed)
        self.intake_motor.set(self.intake_speed)

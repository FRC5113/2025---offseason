from rev import SparkMax, SparkRelativeEncoder
from lemonlib import LemonInput, LemonRobot
import navx
from phoenix6.hardware import TalonFX
from phoenix5 import TalonSRX
from wpilib import DutyCycleEncoder, DigitalInput, Field2d, SmartDashboard
from wpimath import units, applyDeadband
from lemonlib.smart import SmartProfile

from components.drivetrain import Drivetrain
from components.arm import Arm, ArmAngle

from lemonlib import fms_feedback
from autonomous.auto_base import AutoBase


class MyRobot(LemonRobot):
    drivetrain: Drivetrain
    arm: Arm

    def createObjects(self):

        """
        DRIVETRAIN
        """
        self.right_front_motor = SparkMax(11, SparkMax.MotorType.kBrushless)
        self.left_front_motor = SparkMax(12, SparkMax.MotorType.kBrushless)
        self.right_back_motor = SparkMax(13, SparkMax.MotorType.kBrushless)
        self.left_back_motor = SparkMax(14, SparkMax.MotorType.kBrushless)

        self.left_drive_encoder = self.left_front_motor.getEncoder()
        self.right_drive_encoder = self.right_front_motor.getEncoder()

        self.track_width: units.meters = 0.6
        self.gear_ratio = 8.45
        self.wheel_radius: units.meters = 0.0762

        self.drivetrain_left_profile = SmartProfile(
            "left",
            {
                "kP": 1.0,
                "kI": 0.0,
                "kD": 0.0,
                "kS": 3.0,
                "kV": 1.0,
                "kA": 0.0,
            },
            not self.low_bandwidth,
        )
        self.drivetrain_right_profile = SmartProfile(
            "right",
            {
                "kP": 1.0,
                "kI": 0.0,
                "kD": 0.0,
                "kS": 3.0,
                "kV": 1.0,
                "kA": 0.0,
            },
            not self.low_bandwidth,
        )

        self.drivetrain_translation_profile = SmartProfile(
            "translation",
            {
                "kP": 1.0,
                "kI": 0.0,
                "kD": 0.0,
            },
            not self.low_bandwidth,
        )
        self.drivetrain_rotation_profile = SmartProfile(
            "rotation",
            {
                "kP": 1.0,
                "kI": 0.0,
                "kD": 0.0,
            },
            not self.low_bandwidth,
        )

        """
        ARM
        """

        self.arm_motor = TalonFX(21)
        self.arm_intake_motor = TalonSRX(22)
        self.arm_encoder = DutyCycleEncoder(DigitalInput(1))

        self.arm_profile = SmartProfile(
            "arm",
            {
                "kP": 0.15,
                "kI": 0.0,
                "kD": 0.0,
                "kS": 0.0,
                "kV": 0.0,
                "kA": 0.0,
                "kG": 0.0,
                "kMaxV": 150.0,
                "kMaxA": 500.0,
            },
            not self.low_bandwidth,
        )

        """
        MISC
        """

        self.navx = navx.AHRS.create_spi()
        SmartDashboard.putData("Navx", self.navx)
        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)
        self.mass = 100
        self.moi = 6.41

    def teleopInit(self):
        self.primaryController = LemonInput(0)
        self.secondaryController = LemonInput(1)

    def teleopPeriodic(self):
        self.drivetrain.drive(
            -applyDeadband(self.primaryController.getLeftY(), 0.1),
            -applyDeadband(self.primaryController.getLeftX(), 0.1),
        )

        if self.primaryController.getAButton():
            self.arm.set_target_angle(ArmAngle.UP)
            self.arm.set_intake_speed(-1)
        elif self.primaryController.getBButton():
            self.arm.set_target_angle(ArmAngle.DOWN)
            self.arm.set_intake_speed(1)


    def _display_auto_trajectory(self) -> None:
        selected_auto = self._automodes.chooser.getSelected()
        if isinstance(selected_auto, AutoBase):
            selected_auto.display_trajectory()

    @fms_feedback
    def display_auto_state(self) -> None:
        selected_auto = self._automodes.chooser.getSelected()
        if isinstance(selected_auto, AutoBase):
            return selected_auto.current_state
        return "No Auto Selected"

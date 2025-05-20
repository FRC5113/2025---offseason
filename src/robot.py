from rev import SparkMax
from lemonlib import LemonInput, LemonRobot
import navx
from phoenix6.hardware import TalonFX
from phoenix5 import TalonSRX
from wpilib import DutyCycleEncoder, DigitalInput,Field2d,SmartDashboard
from wpimath import units
from lemonlib.smart import SmartProfile

from components.drivetrain import Drivetrain
from components.arm import Arm, ArmAngle

from lemonlib import fms_feedback
from autonomous.auto_base import AutoBase


class myRobot(LemonRobot):
    drivetrain: Drivetrain
    arm: Arm

    def createObjects(self):

        """
        DRIVETRAIN
        """
        self.rfMotor = SparkMax(11, SparkMax.MotorType.kBrushless)
        self.lfMotor = SparkMax(12, SparkMax.MotorType.kBrushless)
        self.rbMotor = SparkMax(13, SparkMax.MotorType.kBrushless)
        self.lbMotor = SparkMax(14, SparkMax.MotorType.kBrushless)

        self.left_encoder = self.lfMotor.getEncoder()
        self.right_encoder = self.rfMotor.getEncoder()

        self.track_width: units.meters = 0.6
        self.gear_ratio = 8.45
        self.wheel_diameter: units.meters = units.inchesToMeters(6.0)
        
        self.drivetrain_left_profile = SmartProfile(
            "left",
            {
                "kP": 0.15,
                "kI": 0.0,
                "kD": 0.0,
                "kS": 0.0,
                "kV": 0.0,
                "kA": 0.0,
            },
            not self.low_bandwidth,
        )
        self.drivetrain_right_profile = SmartProfile(
            "right",
            {
                "kP": 0.15,
                "kI": 0.0,
                "kD": 0.0,
                "kS": 0.0,
                "kV": 0.0,
                "kA": 0.0,
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
        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)

    def teleopInit(self):
        self.primaryController = LemonInput(0)
        self.secondaryController = LemonInput(1)

    def teleopPeriodic(self):
        self.drivetrain.drive(
            self.primaryController.getLeftY(), self.primaryController.getRightX()
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
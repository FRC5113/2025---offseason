from rev import SparkMax, SparkRelativeEncoder
from lemonlib import LemonInput, LemonRobot
import navx
from phoenix6.hardware import TalonFX
from phoenix5 import TalonSRX, NeutralMode
from wpilib import DutyCycleEncoder, DigitalInput, Field2d, SmartDashboard
from wpimath import units, applyDeadband
from wpimath.geometry import Transform3d
from wpimath.kinematics import ChassisSpeeds
from lemonlib.smart import SmartProfile, SmartPreference

from components.drivetrain import Drivetrain
from components.arm import Arm, ArmAngle
from components.odometry import Odometry
from components.chute import Chute

from lemonlib import fms_feedback, LemonCamera
from lemonlib.util import get_file
from autonomous.auto_base import AutoBase
from robotpy_apriltag import AprilTagFieldLayout


class MyRobot(LemonRobot):
    drivetrain: Drivetrain
    arm: Arm
    odometry: Odometry
    chute: Chute

    intake_speed = SmartPreference(1)

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
                "kD": 1.0,
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
                "kD": 1.0,
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

        self.kv_linear = 1.98  # Volts per (m/s)
        self.ka_linear = 0.2  # Volts per (m/s^2)
        self.kv_angular = 1.5  # Volts per (rad/s)
        self.ka_angular = 0.3  # Volts per (rad/s^2)

        """
        ARM
        """

        self.arm_motor = TalonFX(21)
        self.arm_intake_motor = TalonSRX(22)
        self.arm_intake_motor.setNeutralMode(NeutralMode.Brake)
        self.arm_gear_ratio = 73
        self.arm_encoder = self.right_front_motor.getAbsoluteEncoder()

        self.arm_tolerance = 3.0

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
        CHUTE
        """
        self.chute_motor = TalonSRX(31)

        """
        CAMERA
        """
        self.field_layout = AprilTagFieldLayout(get_file("2025_test_field.json"))
        self.robot_to_front_cam = Transform3d()
        self.front_camera = LemonCamera(
            "ThriftyCam", self.robot_to_front_cam, self.field_layout
        )
        self.estimated_field = Field2d()

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
            applyDeadband(self.primaryController.getLeftY(), 0.1),
            applyDeadband(self.primaryController.getRightX(), 0.1),
        )

        # if self.secondaryController.getBButton():
        #     self.arm.set_arm_speed(5)
        if self.secondaryController.getAButton():
            self.arm.set_target_angle(ArmAngle.UP)
            self.arm.set_intake_speed(1 * self.intake_speed)
        elif self.secondaryController.getBButton():
            self.arm.set_target_angle(ArmAngle.INTAKE)
            self.arm.set_intake_speed(-1 * self.intake_speed)

        if self.secondaryController.getLeftTriggerAxis() > 0.1:
            self.chute.request_speed(self.secondaryController.getLeftTriggerAxis())
        if self.secondaryController.getRightTriggerAxis() > 0.1:
            self.chute.request_speed(-self.secondaryController.getRightTriggerAxis())

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

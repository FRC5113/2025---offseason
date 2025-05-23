from rev import SparkMax, SparkMaxConfig, SparkRelativeEncoder
from wpilib.drive import DifferentialDrive
from wpilib import SmartDashboard
from wpimath.kinematics import (
    DifferentialDriveKinematics,
    DifferentialDriveWheelSpeeds,
    ChassisSpeeds,
    DifferentialDriveOdometry,
)
from wpimath import units
from lemonlib.smart import SmartProfile, SmartPreference
import math
from choreo.trajectory import DifferentialSample
from wpiutil import Sendable, SendableBuilder
from wpimath.geometry import Pose2d, Rotation2d
from wpilib import Field2d
from navx import AHRS
from magicbot import will_reset_to


class Drivetrain(Sendable):
    right_front_motor: SparkMax
    right_back_motor: SparkMax
    left_front_motor: SparkMax
    left_back_motor: SparkMax
    navx: AHRS
    field: Field2d

    right_drive_encoder: SparkRelativeEncoder
    left_drive_encoder: SparkRelativeEncoder

    track_width: units.meters
    gear_ratio: float
    wheel_radius: units.meters

    right_profile: SmartProfile
    left_profile: SmartProfile

    translation_profile: SmartProfile
    rotation_profile: SmartProfile

    top_speed = SmartPreference(4.0)
    top_omega = SmartPreference(3.0)
    wheel_speeds = DifferentialDriveWheelSpeeds(0, 0)

    left_voltage = will_reset_to(0)
    right_voltage = will_reset_to(0)

    stopped = will_reset_to(True)

    def __init__(self):
        Sendable.__init__(self)

    def setup(self):
        self.right_back_motor.configure(
            SparkMaxConfig().follow(self.right_front_motor.getDeviceId(), True),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.left_back_motor.configure(
            SparkMaxConfig().follow(self.left_front_motor.getDeviceId(), True),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.left_front_motor.configure(
            SparkMaxConfig().setIdleMode(SparkMaxConfig.IdleMode.kCoast),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.left_back_motor.configure(
            SparkMaxConfig().setIdleMode(SparkMaxConfig.IdleMode.kCoast),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.right_front_motor.configure(
            SparkMaxConfig().setIdleMode(SparkMaxConfig.IdleMode.kCoast),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.right_back_motor.configure(
            SparkMaxConfig().setIdleMode(SparkMaxConfig.IdleMode.kCoast),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )

        self.right_front_motor.configure(
            SparkMaxConfig().inverted(True),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )

        self.right_back_motor.configure(
            SparkMaxConfig().inverted(True),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )

        self.kinematics = DifferentialDriveKinematics(self.track_width)

        self.chassis_speeds = ChassisSpeeds()

        self.odometry = DifferentialDriveOdometry(
            self.navx.getRotation2d(),
            self.left_drive_encoder.getPosition(),
            self.right_drive_encoder.getPosition(),
            Pose2d(0, 0, Rotation2d()),
        )

        SmartDashboard.putData("Drivetrain", self)

    def on_enable(self):
        self.right_back_motor.configure(
            SparkMaxConfig().setIdleMode(SparkMaxConfig.IdleMode.kBrake),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.left_back_motor.configure(
            SparkMaxConfig().setIdleMode(SparkMaxConfig.IdleMode.kBrake),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.left_front_motor.configure(
            SparkMaxConfig().setIdleMode(SparkMaxConfig.IdleMode.kBrake),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.right_front_motor.configure(
            SparkMaxConfig().setIdleMode(SparkMaxConfig.IdleMode.kBrake),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.right_controller = self.right_profile.create_flywheel_controller("Right")
        self.left_controller = self.left_profile.create_flywheel_controller("Left")

        self.translation_controller = self.translation_profile.create_pid_controller(
            "Translation"
        )
        self.rotation_controller = self.rotation_profile.create_pid_controller(
            "Rotation"
        )

    def on_disable(self):
        self.right_back_motor.configure(
            SparkMaxConfig().setIdleMode(SparkMaxConfig.IdleMode.kCoast),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.left_back_motor.configure(
            SparkMaxConfig().setIdleMode(SparkMaxConfig.IdleMode.kCoast),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.left_front_motor.configure(
            SparkMaxConfig().setIdleMode(SparkMaxConfig.IdleMode.kCoast),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.right_front_motor.configure(
            SparkMaxConfig().setIdleMode(SparkMaxConfig.IdleMode.kCoast),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.stopped = True

    def drive(self, vY: float, omega: float):
        self.chassis_speeds = ChassisSpeeds(vY, 0.0, omega)
        self.wheel_speeds = self.kinematics.toWheelSpeeds(self.chassis_speeds)
        self.wheel_speeds.desaturate(self.top_speed)
        self.stopped = False

    def drive_sample(self, sample: DifferentialSample):
        speeds = sample.get_chassis_speeds()
        self.chassis_speeds = ChassisSpeeds(
            speeds.vx
            + self.translation_controller.calculate(
                self.odometry.getPose().X(), sample.x
            ),
            0.0,
            speeds.omega
            + self.rotation_controller.calculate(
                self.odometry.getPose().rotation().radians(), sample.heading
            ),
        )
        self.wheel_speeds = self.kinematics.toWheelSpeeds(self.chassis_speeds)
        self.wheel_speeds.desaturate(self.top_speed)
        self.stopped = False

    def get_velocity(self):

        return self.chassis_speeds

    def get_pose(self):
        return self.odometry.getPose()

    def set_pose(self, pose: Pose2d):
        self.odometry.resetPose(pose)
        self.field.setRobotPose(pose)

    def initSendable(self, builder: SendableBuilder) -> None:
        builder.setSmartDashboardType("DifferentialDrive")
        builder.setActuator(True)
        builder.setSafeState(self.on_disable)
        builder.addDoubleProperty(
            "Left Motor Speed",
            lambda: self.left_front_motor.get() * 12.0,
            lambda speed: self.left_front_motor.set(speed),
        )
        builder.addDoubleProperty(
            "Right Motor Speed",
            lambda: self.right_front_motor.get() * 12.0,
            lambda speed: self.right_front_motor.set(speed),
        )

    def execute(self):
        self.left_voltage = self.left_controller.calculate(
            self.left_drive_encoder.getVelocity()
            / self.gear_ratio
            * self.wheel_radius
            * math.tau,
            self.wheel_speeds.left,
        )
        self.right_voltage = self.right_controller.calculate(
            self.right_drive_encoder.getVelocity()
            / self.gear_ratio
            * self.wheel_radius
            * math.tau,
            self.wheel_speeds.right,
        )
        if self.stopped:
            self.left_voltage = 0
            self.right_voltage = 0
            return

        self.left_front_motor.setVoltage(
            self.left_voltage
        )

        self.right_front_motor.setVoltage(
            self.right_voltage
        )

        self.odometry.update(
            self.navx.getRotation2d(),
            self.left_drive_encoder.getPosition(),
            self.right_drive_encoder.getPosition(),
        )
        pose = self.odometry.getPose()
        self.field.setRobotPose(pose)

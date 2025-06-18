from rev import SparkMax, SparkMaxConfig, SparkRelativeEncoder
from wpilib.drive import DifferentialDrive
from wpilib import SmartDashboard
from wpimath.kinematics import (
    DifferentialDriveKinematics,
    DifferentialDriveWheelSpeeds,
    ChassisSpeeds,
)
from wpimath import units
from lemonlib.smart import SmartProfile, SmartPreference
from lemonlib import LemonCamera,fms_feedback
import math
from choreo.trajectory import DifferentialSample
from wpiutil import Sendable, SendableBuilder
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.estimator import DifferentialDrivePoseEstimator
from wpimath.controller import LTVDifferentialDriveController
from wpimath.system.plant import LinearSystemId
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
    estimated_field: Field2d

    right_drive_encoder: SparkRelativeEncoder
    left_drive_encoder: SparkRelativeEncoder

    kv_linear: float  
    ka_linear: float  
    kv_angular: float  
    ka_angular: float
    track_width: units.meters
    gear_ratio: float
    wheel_radius: units.meters
    loop_time: units.seconds = 0.02
    
    translation_profile: SmartProfile
    rotation_profile: SmartProfile

    max_Vx = SmartPreference(1.0)
    max_omega = SmartPreference(1.0)

    left_voltage = will_reset_to(0)
    right_voltage = will_reset_to(0)


    wheel_speeds = DifferentialDriveWheelSpeeds(0, 0)
    chassis_speeds = ChassisSpeeds(0, 0, 0)

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
        self.diff = DifferentialDrive(self.left_front_motor,self.right_front_motor)

        self.odometry = DifferentialDrivePoseEstimator(
            self.kinematics,
            self.navx.getRotation2d(),
            self.left_drive_encoder.getPosition(),
            self.right_drive_encoder.getPosition(),
            Pose2d(0, 0, Rotation2d()),
        )

        SmartDashboard.putData("Drivetrain", self)



    def drive(self, vY: float, omega: float):

        self.wheel_speeds = self.diff.arcadeDriveIK(vY,omega)

    def drive_sample(self, sample: DifferentialSample):
        pose = self.odometry.getEstimatedPosition()


    def get_velocity(self):
        return self.chassis_speeds

    def get_pose(self):
        return self.odometry.getEstimatedPosition()

    def set_pose(self, pose: Pose2d):
        self.odometry.resetPose(pose)
        self.estimated_field.setRobotPose(pose)

    def add_vision_measurement(self, mesurement: Pose2d, timestamp: units.seconds):
        self.odometry.addVisionMeasurement(mesurement, timestamp)

    def initSendable(self, builder: SendableBuilder) -> None:
        builder.setSmartDashboardType("DifferentialDrive")
        builder.setActuator(True)
        builder.addDoubleProperty(
            "Left Motor Speed",
            lambda: self.left_front_motor.get(),
            lambda speed: self.left_front_motor.set(speed),
        )
        builder.addDoubleProperty(
            "Right Motor Speed",
            lambda: self.right_front_motor.get(),
            lambda speed: self.right_front_motor.set(speed),
        )

    def execute(self):
        self.left_front_motor.set(self.wheel_speeds.left)

        self.right_front_motor.set(self.wheel_speeds.right)
        self.diff.feed()

        self.odometry.update(
            self.navx.getRotation2d(),
            self.left_drive_encoder.getPosition(),
            self.right_drive_encoder.getPosition(),
        )

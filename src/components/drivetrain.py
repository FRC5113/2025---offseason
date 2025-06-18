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
from lemonlib import LemonCamera
import math
from choreo.trajectory import DifferentialSample
from wpiutil import Sendable, SendableBuilder
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.estimator import DifferentialDrivePoseEstimator
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

    right_profile: SmartProfile
    left_profile: SmartProfile

    ltv_profile: SmartProfile

    omega_mult = SmartPreference(1.0)
    speed_mult = SmartPreference(1.0)

    left_voltage = will_reset_to(0)
    right_voltage = will_reset_to(0)

    stopped = will_reset_to(True)

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

        self.odometry = DifferentialDrivePoseEstimator(
            self.kinematics,
            self.navx.getRotation2d(),
            self.left_drive_encoder.getPosition(),
            self.right_drive_encoder.getPosition(),
            Pose2d(0, 0, Rotation2d()),
        )

        SmartDashboard.putData("Drivetrain", self)

    def on_enable(self):
        self.right_controller = self.right_profile.create_flywheel_controller("Right")
        self.right_controller.setTolerance(0.01)
        self.left_controller = self.left_profile.create_flywheel_controller("Left")
        self.left_controller.setTolerance(0.01)

        self.ltv_controller = self.ltv_profile.create_ltv_unicycle_controller(
            LinearSystemId.identifyDrivetrainSystem(
                self.kv_linear,
                self.ka_linear,
                self.kv_angular,
                self.ka_angular,
                self.track_width,
            ),
            self.track_width,
            self.loop_time,
        )


    def on_disable(self):
        self.stopped = True

    def drive(self, vY: float, omega: float):
        self.chassis_speeds = ChassisSpeeds(
            vY * self.speed_mult, 0.0, omega * self.omega_mult
        )
        self.wheel_speeds = self.kinematics.toWheelSpeeds(self.chassis_speeds)

        self.stopped = False

    def drive_sample(self, sample: DifferentialSample):
        pose = self.odometry.getEstimatedPosition()

        ff = sample.get_chassis_speeds()

        # Generate the next speeds for the robot
        self.chassis_speeds = self.ltv_controller.calculate(
            pose,
            sample.get_pose(),
            ff.vx,
            ff.omega
        )
        self.wheel_speeds = self.kinematics.toWheelSpeeds(self.chassis_speeds)
        self.stopped = False

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
        builder.setSafeState(self.on_disable)
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
        self.left_voltage = self.left_controller.calculate(
            (
                (self.left_drive_encoder.getVelocity() / 60.0)
                / self.gear_ratio
                * self.wheel_radius
                * math.tau
            ),
            self.wheel_speeds.left,
        )
        self.right_voltage = self.right_controller.calculate(
            (
                (self.right_drive_encoder.getVelocity() / 60.0)
                / self.gear_ratio
                * self.wheel_radius
                * math.tau
            ),
            self.wheel_speeds.right,
        )
        if self.stopped:
            self.left_voltage = 0
            self.right_voltage = 0
            self.left_front_motor.stopMotor()
            self.right_front_motor.stopMotor()
            return
        else:
            self.left_front_motor.setVoltage(self.left_voltage)

            self.right_front_motor.setVoltage(self.right_voltage)

        self.odometry.update(
            self.navx.getRotation2d(),
            self.left_drive_encoder.getPosition(),
            self.right_drive_encoder.getPosition(),
        )

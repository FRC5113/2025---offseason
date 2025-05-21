from rev import SparkMax, SparkMaxConfig, SparkRelativeEncoder
from wpilib.drive import DifferentialDrive
from wpilib import SmartDashboard
from wpimath.kinematics import DifferentialDriveKinematics, DifferentialDriveWheelSpeeds
from wpimath.kinematics import ChassisSpeeds
from wpimath import units
from lemonlib.smart import SmartProfile, SmartPreference
import math
from choreo.trajectory import DifferentialSample
from wpiutil import Sendable,SendableBuilder

class Drivetrain:
    right_front_motor: SparkMax
    right_back_motor: SparkMax
    left_front_motor: SparkMax
    left_back_motor: SparkMax

    right_drive_encoder: SparkRelativeEncoder
    left_drive_encoder: SparkRelativeEncoder


    track_width: units.meters
    gear_ratio: float
    wheel_radius: units.meters

    right_profile: SmartProfile
    left_profile: SmartProfile

    top_speed = SmartPreference(4.0)
    top_omega = SmartPreference(3.0)
    wheel_speeds = DifferentialDriveWheelSpeeds(0, 0)

    def setup(self):
        self.right_back_motor.configure(
            SparkMaxConfig().follow(self.right_front_motor.getDeviceId()),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.left_back_motor.configure(
            SparkMaxConfig().follow(self.left_front_motor.getDeviceId()),
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


        self.kinematics = DifferentialDriveKinematics(
            self.track_width
        )

        self.chassis_speeds = ChassisSpeeds()

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

    def drive(self, vY: float, omega: float):
        self.chassis_speeds = ChassisSpeeds(vY, 0.0, omega)
        self.wheel_speeds = self.kinematics.toWheelSpeeds(self.chassis_speeds)
        self.wheel_speeds.desaturate(self.top_speed)

    def drive_sample(self, sample: DifferentialSample):
        self.chassis_speeds = sample.get_chassis_speeds()
        self.wheel_speeds = self.kinematics.toWheelSpeeds(self.chassis_speeds)
        self.wheel_speeds.desaturate(self.top_speed)

    def get_velocity(self):

        return self.chassis_speeds
    
    def initSendable(self, builder: SendableBuilder) -> None:
        builder.setSmartDashboardType("DifferentialDrive")
        builder.setActuator(True)
        builder.setSafeState(self.on_disable)
        builder.addDoubleProperty(
            "Left Motor Speed",
            lambda: self.left_front_motor.get(),
            lambda speed: self.left_front_motor.set(speed)
        )
        builder.addDoubleProperty(
            "Right Motor Speed",
            lambda: self.right_front_motor.get(),
            lambda speed: self.right_front_motor.set(speed)
        )

    def execute(self):
        
        self.left_front_motor.setVoltage(
            self.left_controller.calculate(
                self.right_drive_encoder.getVelocity()
                / self.gear_ratio
                * self.wheel_radius
                * math.tau,
                self.wheel_speeds.left,
            )
        )

        self.right_back_motor.setVoltage(
            self.right_controller.calculate(
                self.right_drive_encoder.getVelocity()
                / self.gear_ratio
                * self.wheel_radius
                * math.tau,
                -self.wheel_speeds.right,
            )
        )

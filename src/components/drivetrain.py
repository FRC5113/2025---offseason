from rev import SparkMax, SparkMaxConfig,SparkRelativeEncoder
from wpilib.drive import DifferentialDrive
from wpilib import SmartDashboard
from wpimath.kinematics import DifferentialDriveKinematics,DifferentialDriveWheelSpeeds
from wpimath.kinematics import ChassisSpeeds
from wpimath import units
from lemonlib.smart import SmartProfile,SmartPreference
import math


class Drivetrain:
    rbMotor: SparkMax
    lbMotor: SparkMax
    lfMotor: SparkMax
    rfMotor: SparkMax

    left_encoder: SparkRelativeEncoder
    right_encoder: SparkRelativeEncoder
    
    track_width: units.meters
    gear_ratio: float
    wheel_diameter: units.meters

    right_profile: SmartProfile
    left_profile: SmartProfile

    top_speed = SmartPreference(4.0)
    top_omega = SmartPreference(3.0)
    wheel_speeds = DifferentialDriveWheelSpeeds(0, 0)

    def setup(self):
        self.rfMotor.configure(
            SparkMaxConfig().follow(self.rbMotor.getDeviceId()),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.lfMotor.configure(
            SparkMaxConfig().follow(self.lbMotor.getDeviceId()),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.rfMotor.configure(
            SparkMaxConfig().setIdleMode(SparkMaxConfig.IdleMode.kCoast),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.rfMotor.configure(
            SparkMaxConfig().setIdleMode(SparkMaxConfig.IdleMode.kCoast),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.lfMotor.configure(
            SparkMaxConfig().setIdleMode(SparkMaxConfig.IdleMode.kCoast),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.rfMotor.configure(
            SparkMaxConfig().setIdleMode(SparkMaxConfig.IdleMode.kCoast),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )

        self.drivetrain = DifferentialDrive(self.lfMotor, self.rfMotor)
        self.kinematics = DifferentialDriveKinematics(
            units.inchesToMeters(self.track_width)
        )

        self.velocity_factor = (self.wheel_diameter * math.pi) / 60

    def on_enable(self):
        self.rbMotor.configure(
            SparkMaxConfig().setIdleMode(SparkMaxConfig.IdleMode.kBrake),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.lbMotor.configure(
            SparkMaxConfig().setIdleMode(SparkMaxConfig.IdleMode.kBrake),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.lfMotor.configure(
            SparkMaxConfig().setIdleMode(SparkMaxConfig.IdleMode.kBrake),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.rfMotor.configure(
            SparkMaxConfig().setIdleMode(SparkMaxConfig.IdleMode.kBrake),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.right_controller = self.right_profile.create_flywheel_controller("Right")
        self.left_controller = self.left_profile.create_flywheel_controller("Left")

    def on_disable(self):
        self.rbMotor.configure(
            SparkMaxConfig().setIdleMode(SparkMaxConfig.IdleMode.kCoast),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.lbMotor.configure(
            SparkMaxConfig().setIdleMode(SparkMaxConfig.IdleMode.kCoast),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.lfMotor.configure(
            SparkMaxConfig().setIdleMode(SparkMaxConfig.IdleMode.kCoast),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.rfMotor.configure(
            SparkMaxConfig().setIdleMode(SparkMaxConfig.IdleMode.kCoast),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )

    def drive(self, vX: float,omega: float):
        self.chassis_speeds = ChassisSpeeds(
            vX,
            0.0,
            omega
        )
        self.wheel_speeds = self.kinematics.toWheelSpeeds(self.chassis_speeds)

    def get_velocity(self):
        return self.chassis_speeds

    def execute(self):
        self.lfMotor.setVoltage(self.left_controller.calculate(
            self.left_encoder.getVelocity() * self.velocity_factor, self.wheel_speeds.left
        ))
        self.rfMotor.setVoltage(self.right_controller.calculate(
            self.right_encoder.getVelocity() * self.velocity_factor, self.wheel_speeds.right
        ))

        self.drivetrain.feed()

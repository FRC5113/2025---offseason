from wpimath.kinematics import DifferentialDriveOdometry
from wpimath.geometry import Pose2d
from wpimath.geometry import Rotation2d
from wpilib import SmartDashboard,Field2d
from navx import AHRS
from rev import SparkRelativeEncoder


class Odometry:
    navx: AHRS
    left_encoder: SparkRelativeEncoder
    right_encoder: SparkRelativeEncoder

    def setup(self):
        self.odometry = DifferentialDriveOdometry(
            self.navx.getRotation2d(),
            self.left_encoder.getPosition(),
            self.right_encoder.getPosition(),
            Pose2d(0, 0, Rotation2d()),
        )
        self.field = Field2d()

    def execute(self):
        self.odometry.update(
            self.navx.getRotation2d(),
            self.left_encoder.getPosition(),
            self.right_encoder.getPosition(),
        )
        pose = self.odometry.getPose()
        self.field.setRobotPose(pose)

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
    field: Field2d

    def setup(self):
        self.odometry = DifferentialDriveOdometry(
            self.navx.getRotation2d(),
            self.left_encoder.getPosition(),
            self.right_encoder.getPosition(),
            Pose2d(0, 0, Rotation2d()),
        )

    def get_pose(self):
        return self.odometry.getPose()
    def set_pose(self, pose: Pose2d):
        self.odometry.resetPose(pose)
        self.field.setRobotPose(pose)

    def execute(self):
        self.odometry.update(
            self.navx.getRotation2d(),
            self.left_encoder.getPosition(),
            self.right_encoder.getPosition(),
        )
        pose = self.odometry.getPose()
        self.field.setRobotPose(pose)

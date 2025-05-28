from wpilib import Field2d, SmartDashboard, Timer
from wpimath.geometry import Transform3d, Pose2d
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy
from robotpy_apriltag import AprilTagFieldLayout

from lemonlib.vision import LemonCamera

from components.drivetrain import Drivetrain


class Odometry:
    front_camera: LemonCamera
    robot_to_front_cam: Transform3d
    field_layout: AprilTagFieldLayout
    drivetrain: Drivetrain
    estimated_field: Field2d

    def setup(self):
        self.camera_pose_estimator_front = PhotonPoseEstimator(
            self.field_layout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            self.front_camera,
            self.robot_to_front_cam,
        )

        SmartDashboard.putData("Estimated Field", self.estimated_field)

    def execute(self):
        # may need to tweak timestamp to match system time
        self.front_camera.update()
        camera_estimator_result_front = self.camera_pose_estimator_front.update()
        if camera_estimator_result_front is not None:
            self.drivetrain.add_vision_measurement(
                camera_estimator_result_front.estimatedPose.toPose2d(),
                self.front_camera.getLatestResult().getTimestampSeconds(),
            )

        self.estimated_field.setRobotPose(self.drivetrain.get_pose())

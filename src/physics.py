#
# See the documentation for more details on how this works
#
# Documentation can be found at https://robotpy.readthedocs.io/projects/pyfrc/en/latest/physics.html
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#
# Examples can be found at https://github.com/robotpy/examples

import wpilib.simulation

from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics import motor_cfgs, tankmodel
from pyfrc.physics.units import units
from wpilib.simulation import DifferentialDrivetrainSim, DutyCycleEncoderSim
from wpimath.system.plant import DCMotor
from wpimath.geometry import Rotation2d
from wpimath.system import LinearSystem_2_2_2
from wpimath.system.plant import DCMotor, LinearSystemId
from rev import SparkMaxSim, SparkRelativeEncoderSim, SparkMax, SparkAbsoluteEncoderSim
import typing
from wpilib import DriverStation, Mechanism2d, SmartDashboard, Color8Bit
from wpilib.simulation import (
    SingleJointedArmSim,
    ElevatorSim,
    DIOSim,
    DutyCycleEncoderSim,
)

from lemonlib.simulation import LemonCameraSim
from lemonlib.simulation import FalconSim

if typing.TYPE_CHECKING:
    from robot import MyRobot


class PhysicsEngine:
    """
    Simulates a 4-wheel robot using Tank Drive joystick control
    """

    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        """
        :param physics_controller: `pyfrc.physics.core.Physics` object
                                   to communicate simulation effects to
        :param robot: your robot object
        """

        self.physics_controller = physics_controller

        self.left_drive_motor_sim = SparkMaxSim(robot.left_front_motor, DCMotor.NEO(2))
        self.right_drive_motor_sim = SparkMaxSim(
            robot.right_front_motor, DCMotor.NEO(2)
        )

        self.right_drive_encoder_sim = (
            self.right_drive_motor_sim.getRelativeEncoderSim()
        )
        self.left_drive_encoder_sim = self.left_drive_motor_sim.getRelativeEncoderSim()

        self.navx = robot.navx

        self.drive_sim = DifferentialDrivetrainSim(
            DCMotor.NEO(4),
            robot.gear_ratio,
            robot.moi,
            robot.mass,
            robot.wheel_radius,
            robot.track_width,
        )

        # Claw Simulation
        self.arm_gearbox = DCMotor.falcon500(1)
        self.arm_sim = SingleJointedArmSim(
            self.arm_gearbox,
            robot.arm_gear_ratio,
            0.1,  # gross estimate
            0.2,  # estimate
            -0.52,
            1.57,
            True,
            1.57,
            [0, 0],
        )
        self.arm_encoder_sim = DutyCycleEncoderSim(robot.arm_encoder)
        self.arm_motor_sim = FalconSim(robot.arm_motor, 0.1, robot.arm_gear_ratio)

        # Mechanism2d Visualization for Arm
        self.arm_sim_vis = Mechanism2d(20, 50)
        self.arm_root = self.arm_sim_vis.getRoot("Arm Root", 10, 0)
        self.arm_ligament = self.arm_root.appendLigament(
            "Arm", 10, 0, color=Color8Bit(0, 150, 0)
        )

        # Put Mechanism to SmartDashboard
        SmartDashboard.putData("Arm Sim", self.arm_sim_vis)

        self.vision_sim = LemonCameraSim(
            robot.front_camera,
            robot.field_layout,
        )

        self.field = robot.field

    def update_sim(self, now: float, tm_diff: float) -> None:
        self.drive_sim.setInputs(
            self.left_drive_motor_sim.getSetpoint() * 12.0,
            self.right_drive_motor_sim.getSetpoint() * 12.0,
        )

        self.right_drive_encoder_sim.setPosition(self.drive_sim.getRightPosition())
        self.right_drive_encoder_sim.setVelocity(self.drive_sim.getRightVelocity())

        self.left_drive_encoder_sim.setPosition(self.drive_sim.getLeftPosition())
        self.left_drive_encoder_sim.setVelocity(self.drive_sim.getLeftVelocity())

        self.navx.setAngleAdjustment(-self.drive_sim.getHeading().degrees())

        self.physics_controller.field.setRobotPose(self.drive_sim.getPose())

        self.arm_sim.setInput(0, self.arm_motor_sim.getSetpoint())
        self.arm_sim.update(tm_diff)
        self.arm_encoder_sim.set(self.arm_sim.getAngleDegrees())

        self.arm_ligament.setAngle(self.arm_sim.getAngleDegrees())

        self.vision_sim.update(self.drive_sim.getPose())

        self.drive_sim.update(tm_diff)

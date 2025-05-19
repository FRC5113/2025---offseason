import lemonlib
from rev import SparkMax, SparkMaxConfig
from lemonlib import LemonInput,LemonRobot
import navx
from phoenix6.hardware import TalonFX
from phoenix5 import TalonSRX
from wpilib import Encoder

from components.drivetrain import Drivetrain


class myRobot(LemonRobot):
    drivetrain: Drivetrain

    def createObjects(self):
        self.rfMotor = SparkMax(1, SparkMax.MotorType.kBrushless)
        self.lfMotor = SparkMax(2, SparkMax.MotorType.kBrushless)
        self.rbMotor = SparkMax(3, SparkMax.MotorType.kBrushless)
        self.lbMotor = SparkMax(4, SparkMax.MotorType.kBrushless)

        self.arm_motor = TalonFX(5)
        self.arm_intake_motor = TalonSRX(6)
        self.arm_encoder = Encoder(0,1)

        self.primaryController = LemonInput(0)
        self.secondaryController = LemonInput(1)

        self.navx = navx.AHRS.create_spi()

    def teleopPeriodic(self):
        self.drivetrain.drive(self.primaryController.getLeftY(), self.primaryController.getRightX())

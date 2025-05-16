from rev import SparkMax, SparkMaxConfig
from wpilib.drive import DifferentialDrive
from wpilib import SmartDashboard


class Drivetrain:
    rbMotor: SparkMax
    lbMotor: SparkMax
    lfMotor: SparkMax
    rfMotor: SparkMax
    
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
        self.vForward = 0
        self.vRot = 0

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

    def drive(self, forward: float, rot: float):
        self.vForward = forward
        self.vRot = rot
    
    def execute(self):
        self.drivetrain.arcadeDrive(self.vForward, self.vRot)

        SmartDashboard.putData("DriveTrain", self.drivetrain)



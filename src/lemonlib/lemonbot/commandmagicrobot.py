import magicbot
import commands2
from wpilib import DriverStation
from wpilib import SmartDashboard
from robotpy_ext.autonomous import AutonomousModeSelector
from .commandcomponent import LemonComponent


class LemonRobot(magicbot.MagicRobot):
    """
    Wrapper for the magicbot robot class to allow for command-based
    functionality. This class is used to create a robot that can be
    controlled using commands, while still using the magicbot framework.
    """

    low_bandwidth = DriverStation.isFMSAttached()
    commandscheduler = commands2.CommandScheduler.getInstance()

    def __init__(self):
        super().__init__()

        self.loop_time = self.control_loop_wait_time

    def autonomousPeriodic(self):
        """
        Periodic code for autonomous mode should go here.
        Runs when not enabled for trajectory display.

        Users should override this method for code which will be called
        periodically at a regular rate while the robot is in autonomous mode.

        This code executes before the ``execute`` functions of all
        components are called.
        """
        pass

    # def endCompetition(self) -> None:
    #     self.__done = True
    #     if self._automodes:
    #         self._automodes.endCompetition()

    def autonomous(self):
        super().autonomous()
        self.autonomousPeriodic()

    def enabledperiodic(self) -> None:
        """Periodic code for when the bot is enabled should go here.
        Runs when not enabled for trajectory display.

        Users should override this method for code which will be called"""
        pass

    def _enabled_periodic(self) -> None:
        """Run components and all periodic methods."""
        watchdog = self.watchdog
        self.commandscheduler.run()

        for name, component in self._components:
            if commands2.Subsystem.getCurrentCommand(component) is None and issubclass(
                component.__class__, LemonComponent
            ):
                try:
                    component.execute()

                except Exception:
                    self.onException()
            else:
                try:
                    component.execute()

                except Exception:
                    self.onException()
            watchdog.addEpoch(name)
        SmartDashboard.putData("CommandScheduler", self.commandscheduler)
        if DriverStation.isEnabled():
            self.enabledperiodic()

        self._do_periodics()

        for reset_dict, component in self._reset_components:
            component.__dict__.update(reset_dict)

    def _do_periodics(self):
        super()._do_periodics()

        self.loop_time = max(self.control_loop_wait_time, self.watchdog.getTime())

    def get_period(self) -> float:
        """Get the period of the robot loop in seconds."""
        return self.loop_time

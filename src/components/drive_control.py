import numpy as np

from wpilib import DriverStation
import wpimath.controller
import magicbot
from magicbot.state_machine import state, timed_state
from magicbot import tunable, will_reset_to, feedback
import navx

from components.drivetrain import Drivetrain

from lemonlib import LemonCamera
from lemonlib.smart import SmartProfile, SmartPreference
from lemonlib import util



class DriveControl(magicbot.StateMachine):
    # other components
    drivetrain: Drivetrain
    front_camera: LemonCamera

    # variables to be injected
    navx: navx.AHRS

    translation_profile: SmartProfile
    rotation_profile: SmartProfile

    align_trigger = will_reset_to(False)
    turn_trigger = will_reset_to(False)

    drivetrain_kS = SmartPreference(0.0)

    drive_from_tag_setpoint = SmartPreference(1.05)
    drive_from_tag_tP = SmartPreference( 0.07)


    

    def setup(self):
        self.engage()
        self.turn_angle = 0.0

    def on_enable(self):
        self.translation_controller = self.translation_profile.create_pid_controller("Translation")
        self.rotation_controller = self.rotation_profile.create_pid_controller("Rotation")
        

    def request_align(self):
        self.align_trigger = True

    def request_turn(self):
        self.turn_trigger = True

    def set_angle(self, angle: float):
        """Changes the `turn_to_angle` PID controller's setpoint"""
        self.turn_angle = angle

    def arcade_drive(self, forward: float, turn: float):
        """Call this instead of `drivetrain.arcade_drive()` because
        this will only drive if the drivetrain is free to do so
        """
        if self.current_state == "free":
            self.drivetrain.drive(forward, turn)

    def turn_to_tag(self):
        """Changes the `turn_to_angle` setpoint to one such that the robot
        would face an AprilTag
        """
        if not self.front_camera.has_target():
            return
        theta = self.front_camera.get_best_pose().rotation().degrees()  # - latency * turn_rate
        if theta is None:
            return
        self.set_angle(theta)

    @state(first=True)
    def free(self):
        """First state -- arcade drive"""
        if self.align_trigger and self.front_camera.has_target():
            self.turn_to_tag()
            self.next_state("aligning")
        if self.turn_trigger:
            self.next_state("turning_to_angle")

    @timed_state(duration=0.5, next_state="free")
    def settling(self):
        self.drivetrain.drive(0, 0)

    @state
    def turning_to_angle(self):
        measurement = self.navx.getAngle()
        output = self.rotation_controller.calculate(
            measurement, self.turn_angle)
        if output > 0:
            output += 0.13
        elif output < 0:
            output -= 0.13

        """Here (and elsewhere) the output is negated because a positive turn
        value in `arcade_drive()` corresponds with a decrease in angle.
        This could also be fixed with negative PID values, but this is not
        recommended.
        """
        self.drivetrain.drive(0, util.clamp(-output, -0.5, 0.5))
        if self.rotation_controller.at_setpoint():
            self.next_state("settling")

    @state
    def aligning(self):
        """State in which robot uses a PID controller to turn to a certain
        angle using sensor data from the gyroscope.
        """
        if not self.front_camera.has_target():
            self.next_state("free")
            return
        measurement = self.navx.getAngle()
        output = self.rotation_controller.calculate(measurement,self.turn_angle)
        if output > 0:
            output += self.drivetrain_kS
        elif output < 0:
            output -= self.drivetrain_kS

        """Here (and elsewhere) the output is negated because a positive turn
        value in `arcade_drive()` corresponds with a decrease in angle.
        This could also be fixed with negative PID values, but this is not
        recommended.
        """
        self.drivetrain.drive(0, util.clamp(-output, -0.5, 0.5))
        if self.rotation_controller.at_setpoint():
            self.next_state("spacing")

    @state
    def spacing(self):
        """State in which robot drives forward or backward so that it is
        a set distance away from a detected Apriltag
        """
        if not self.front_camera.has_target():

            self.next_state("free")
            return
        measurement = self.front_camera.get_best_pose().X()
        if measurement is None:
            return
        output = self.translation_controller.calculate(measurement,self.drive_from_tag_setpoint)
        if output > 0:
            output += self.drivetrain_kS
        elif output < 0:
            output -= self.drivetrain_kS
        self.drivetrain.drive(util.clamp(-output, -0.5, 0.5), 0)
        error = self.translation_controller.getError()
        if abs(error) < self.drive_from_tag_tP:
            self.next_state("settling")
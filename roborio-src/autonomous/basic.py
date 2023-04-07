from logging import Logger

from components.drivetrain import SwerveChassis
from components.arm_control import ArmControl, GrabberControl
from magicbot import AutonomousStateMachine, state, timed_state
from wpimath.geometry import Transform2d, Pose2d, Rotation2d
from wpimath.units import degreesToRadians, inchesToMeters, feetToMeters
from pathplannerlib import PathPoint


class NoAuto(AutonomousStateMachine):
    MODE_NAME = "No Auto (default)"
    DEFAULT = True

    @state(first=True)
    def doNothing(self):
        pass


class placeCone(AutonomousStateMachine):
    MODE_NAME = "Simple - Back, Drop cone"

    swerveChassis: SwerveChassis
    armControl: ArmControl
    grabberControl: GrabberControl
    startX = 2.47
    endX = 1.81

    @state(first=True)
    def raiseArm(self, initial_call):
        if initial_call:
            self.logger.info(f"Raising arm to upper")
            self.armControl.moveToUpper()
        self.armControl.engage()
        if self.armControl.last_state == "atUpper":
            self.next_state("moveBack")

    @timed_state(duration=2, next_state="dropCone")
    def moveBack(self, initial_call):
        self.swerveChassis.fieldOrientedDrive(-0.125, 0, 0)
        # self.swerveChassis.autoDrive()
        # if self.swerveChassis.holonomicController.atReference():
        #     self.next_state("dropCone")

    @state()
    def dropCone(self, initial_call):
        if initial_call:
            self.grabberControl.next_state("dropping")
        self.grabberControl.engage()

        if not self.grabberControl.hasObject:
            self.done()


class placeConeDriveForward(AutonomousStateMachine):
    MODE_NAME = "Simple - Back, Drop Cone, Drive Forward"

    swerveChassis: SwerveChassis
    armControl: ArmControl
    grabberControl: GrabberControl
    startX = 2.47
    endX = 1.81

    @state(first=True)
    def raiseArm(self, initial_call):
        if initial_call:
            self.armControl.moveToUpper()
        self.armControl.engage()
        if self.armControl.last_state == "atUpper":
            self.next_state("moveBack")

    @timed_state(duration=2, next_state="dropCone")
    def moveBack(self, initial_call):
        self.swerveChassis.fieldOrientedDrive(-0.125, 0, 0)
        # self.swerveChassis.autoDrive()
        # if self.swerveChassis.holonomicController.atReference():
        #     self.next_state("dropCone")

    @state()
    def dropCone(self, initial_call):
        if initial_call:
            self.grabberControl.next_state("dropping")
        self.grabberControl.engage()
        if not self.grabberControl.hasObject:
            self.next_state("moveForward")

    @timed_state(duration=4, next_state="dropArm")
    def moveForward(self, initial_call):
        self.swerveChassis.fieldOrientedDrive(0.150, 0, 0)

    @state()
    def dropArm(self, initial_call):
        if initial_call:
            self.swerveChassis.fieldOrientedDrive(0, 0, 0)
            self.armControl.moveToHome()
        self.armControl.engage()
        if self.armControl.last_state == "atHome":
            self.next_state("end")

    @state()
    def end(self):
        self.done()

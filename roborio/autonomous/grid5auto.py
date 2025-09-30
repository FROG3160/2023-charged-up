from components.drivetrain import SwerveChassis
from components.drive_control import DriveControl
from components.arm_control import ArmControl, GrabberControl, ArmStates
from magicbot import AutonomousStateMachine, state, timed_state
from wpimath.geometry import Transform2d, Pose2d, Rotation2d
from wpimath.units import degreesToRadians, inchesToMeters, feetToMeters
from pathplannerlib import PathPoint
from components.field import FROGFieldLayout
from wpilib import Timer

class Grid5CubeDropThenCharge(AutonomousStateMachine):
    MODE_NAME = "Grid5CubeDropThenCharge"

    driveControl: DriveControl
    armControl: ArmControl
    grabberControl: GrabberControl
    fieldLayout: FROGFieldLayout

    def __init__(self):
        super().__init__()
        self.stateTimer = Timer()


    @timed_state(duration = 3, first=True, next_state='movingToGrid5')
    def raisingArm(self, initial_call):
        self.armControl.moveToUpper()
    
    @state()
    def movingToGrid5(self, initial_call):
        self.driveControl.holonomicDrivePath(
            'ChargeToGrid5'
        )
        if marker := self.driveControl.holonomic.getPastMarker():
            if 'dropObject' in marker.names:
                self.next_state('droppingObject1')

    @state()
    def droppingObject1(self, initial_call):
        if initial_call:
            self.grabberControl.next_state('dropping')
        self.grabberControl.engage()
        if not self.grabberControl.hasObject:
            self.next_state('driveToMid')

    @state()
    def driveToMid(self):
        self.driveControl.holonomicDrivePath(
            'Grid5ToMid'
        )
        if marker := self.driveControl.holonomic.getPastMarker():
            if 'armToHome' in marker.names:
                self.next_state('loweringArm')
    
    @timed_state(duration = 3, next_state='driveToCharging')
    def loweringArm(self):
        self.armControl.moveToHome()

    @state()
    def driveToCharging(self, initial_call):
        self.grabberControl.engage()
        if initial_call:
            self.driveControl.done()
        self.driveControl.driveToChargingForward()
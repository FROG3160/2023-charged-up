from components.drivetrain import SwerveChassis
from components.drive_control import DriveControl
from components.arm_control import ArmControl, GrabberControl, ArmStates
from magicbot import AutonomousStateMachine, state, timed_state
from wpimath.geometry import Transform2d, Pose2d, Rotation2d
from wpimath.units import degreesToRadians, inchesToMeters, feetToMeters
from pathplannerlib import PathPoint
from components.field import FROGFieldLayout
from wpilib import Timer

class ConeCubeChargeLoadSide(AutonomousStateMachine):
    MODE_NAME = "Cone-Cube-Charge-LoadSide"

    driveControl: DriveControl
    armControl: ArmControl
    grabberControl: GrabberControl
    fieldLayout: FROGFieldLayout
    #timer: Timer

    def __init__(self):
        super().__init__()
        self.stateTimer = Timer()


    @timed_state(duration = 1, first=True, next_state='movingToGrid9')
    def raisingArm(self, initial_call):
        self.armControl.moveToShelf()
    
    @state()
    def movingToGrid9(self, initial_call):
        self.driveControl.holonomicDrivePath(
            'ChargeToGrid9'
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
            self.next_state('movingToObject4')

    @state()
    def movingToObject4(self, initial_call):
        self.grabberControl.engage()
        self.driveControl.holonomicDrivePath(
            'Grid9ToObject4'
        )
        if initial_call:
            self.firstLoop = True
        if marker := self.driveControl.holonomic.getPastMarker():
            if 'dropArm' in marker.names:
                self.armControl.moveToHome()
            if 'armToFloor' in marker.names:
                if self.firstLoop:
                    self.armControl.done()
                    self.firstLoop = False
                self.armControl.moveToFloor()
            if 'driveToObject' in marker.names:
                self.driveControl.done()
                self.armControl.done()
                self.next_state('gettingCube')
    
    @state()
    def gettingCube(self, initial_call):
        if initial_call:
            self.grabberControl.next_state('looking')
        self.armControl.done()
        self.driveControl.autoDriveToCube()
        self.grabberControl.engage()

        if self.grabberControl.current_state == 'lifting':
            self.driveControl.done()
            self.next_state('movingToGrid8')
    
    @state()
    def movingToGrid8(self, initial_call):
        self.grabberControl.engage()
        self.driveControl.holonomicDrivePath(
            'Object4ToGrid8'
        )
        if marker := self.driveControl.holonomic.getPastMarker():
            if 'resetArm' in marker.names:
                self.armControl.done()
            if 'armToShelf' in marker.names:
                self.armControl.moveToShelf()
            if 'dropObject' in marker.names:
                self.driveControl.done()
                self.next_state('droppingCube1')

    @state()
    def droppingCube1(self, initial_call):
        if initial_call:
            self.grabberControl.next_state('dropping')
        self.grabberControl.engage()
        if not self.grabberControl.hasObject:
            self.next_state('movingToCharge')
## might want a check here to see if we should spend the time to get the 
## next object
    @state()
    def movingToCharge(self, initial_call):
        self.driveControl.holonomicDrivePath(
            'Grid8ToCharging'
        )
        if initial_call:
            self.firstLoop = True
        if marker := self.driveControl.holonomic.getPastMarker():
            if 'armToHome' in marker.names:
                self.armControl.moveToHome()
            if 'driveToCharging' in marker.names:
                self.driveControl.done()
                self.armControl.done()
                self.next_state('driveToCharging')

    @state()
    def driveToCharging(self, initial_call):
        self.grabberControl.engage()
        if initial_call:
            self.driveControl.done()
        self.driveControl.driveToChargingReverse()
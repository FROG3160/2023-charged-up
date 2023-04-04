from components.drivetrain import SwerveChassis
from components.drive_control import DriveControl
from components.arm_control import ArmControl, GrabberControl, ArmStates
from magicbot import AutonomousStateMachine, state, timed_state
from wpimath.geometry import Transform2d, Pose2d, Rotation2d
from wpimath.units import degreesToRadians, inchesToMeters, feetToMeters
from pathplannerlib import PathPoint
from components.field import FROGFieldLayout
from wpilib import Timer

class ThreePieceLoadSide(AutonomousStateMachine):
    MODE_NAME = "3 Piece Load Side"

    driveControl: DriveControl
    armControl: ArmControl
    grabberControl: GrabberControl
    fieldLayout: FROGFieldLayout
    #timer: Timer

    def __init__(self):
        super().__init__()
        self.stateTimer = Timer()


    @state(first=True)
    def raisingArm(self, initial_call):
        self.armControl.moveToShelf()
        self.logger.info(f'Arm Position: {self.armControl.getArmPosition()}')
        if self.armControl.getArmPosition() == 'shelf':
            self.next_state('movingToGrid9')
    
    @state()
    def movingToGrid9(self, initial_call):
        self.driveControl.holonomicDrivePath(
            'ChargeToGrid9'
        )
        if marker := self.driveControl.holonomic.getPastMarker():
            if 'dropObject' in marker.names:
        #if self.driveControl.holonomic.atReference():
                self.logger.info('hit marker')
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
        self.driveControl.holonomicDrivePath(
            'Grid9ToObject4'
        )

        if marker := self.driveControl.holonomic.getPastMarker():
            if 'dropArm' in marker.names:
                self.armControl.moveToHome()
            # if 'armToFloor' in marker.names:
            #     self.armControl.done()
            #     self.armControl.moveToFloor()
            if 'driveToObject' in marker.names:
                self.driveControl.done()
                self.armControl.done()
                self.next_state('gettingCube')
    
    @state()
    def gettingCube(self):
        self.armControl.moveToFloor()
        self.driveControl.autoDriveToCube()
        self.grabberControl.look()

        if self.grabberControl.current_state == 'lifting':
            self.driveControl.done()
            self.next_state('movingToGrid8')
    
    @state()
    def movingToGrid8(self, initial_call):
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
            self.next_state('movingToObject3')

    @state()
    def movingToObject3(self, initial_call):
        self.driveControl.holonomicDrivePath(
            'Grid8ToObject3'
        )
        if marker := self.driveControl.holonomic.getPastMarker():
            if 'armToHome' in marker.names:
                self.armControl.moveToHome()
            # if 'armToFloor' in marker.names():
            #     self.armControl.done()
            #     self.armControl.moveToFloor()
            if 'driveToCone' in marker.names:
                self.driveControl.done()
                self.armControl.done()
                self.next_state('gettingCone')

    @state()
    def gettingCone(self):
        self.armControl.moveToFloor()
        self.driveControl.autoDriveToCone()
        self.grabberControl.look()

        if self.grabberControl.current_state == 'lifting':
            self.driveControl.done()
            self.next_state('movingToGrid7')  

    @state()
    def movingToGrid7(self, initial_call):
        self.driveControl.holonomicDrivePath(
            'Object3ToGrid7'
        )
        if marker := self.driveControl.holonomic.getPastMarker():
            if 'resetArm' in marker.names:
                self.armControl.done()
            if 'armToShelf' in marker.names:
                self.armControl.moveToShelf()
            if 'dropObject' in marker.names:
                self.driveControl.done()
                self.next_state('droppingCone2')

    @state()
    def droppingCone2(self, initial_call):
        if initial_call:
            self.grabberControl.next_state('dropping')
        self.grabberControl.engage()
        if not self.grabberControl.hasObject:
            self.done()
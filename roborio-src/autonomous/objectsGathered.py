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
        if self.armControl.getArmPosition() == 'shelf':
            self.next_state('movingToGrid9')
    
    @state()
    def movingToGrid9(self, initial_call):
        self.driveControl.holonomicDriveToWaypoint(
            self.fieldLayout.getPosition(9).toPose2d()
        )
        if self.driveControl.holonomic.atReference():
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
        if initial_call:
            self.stateTimer.reset()
            self.stateTimer.start()
        self.driveControl.holonomicDrivePath(
            'Grid9ToObject4'
        )
        #after a quarter of a second, start dropping the arm
        if self.stateTimer.hasElapsed(0.25):
            self.armControl.moveToHome()
        if self.driveControl.holonomic.atReference():
            self.next_state('gettingCube')
    
    @state()
    def gettingCube(self):
        self.driveControl.autoDriveToCube()
        self.armControl.moveToFloor()
        self.grabberControl.intake()

        if self.grabberControl.current_state == 'holding':
            self.next_state('movingToGrid8')
    
    @state()
    def movingToGrid8(self, initial_call):
        if initial_call:
            self.stateTimer.reset()
            self.stateTimer.start()
        self.driveControl.holonomicDrivePath(
            'Object4ToGrid8'
        )
        if self.driveControl.holonomic.atReference():
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
        if initial_call:
            self.stateTimer.reset()
            self.stateTimer.start()
        self.driveControl.holonomicDrivePath(
            'Grid8ToObject3'
        )
        #after a quarter of a second, start dropping the arm
        if self.stateTimer.hasElapsed(0.25):
            self.armControl.moveToHome()
        if self.driveControl.holonomic.atReference():
            self.next_state('gettingCone')

    @state()
    def gettingCone(self):
        self.driveControl.autoDriveToCone()
        self.armControl.moveToFloor()
        self.grabberControl.intake()

        if self.grabberControl.current_state == 'holding':
            self.next_state('movingToGrid7')  

    @state()
    def movingToGrid7(self, initial_call):
        if initial_call:
            self.stateTimer.reset()
            self.stateTimer.start()
        self.driveControl.holonomicDrivePath(
            'Object3ToGrid7'
        )
        if self.driveControl.holonomic.atReference():
            self.next_state('droppingCone2')

    @state()
    def droppingCone2(self, initial_call):
        if initial_call:
            self.grabberControl.next_state('dropping')
        self.grabberControl.engage()
        if not self.grabberControl.hasObject:
            self.done()
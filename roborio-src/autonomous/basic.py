from logging import Logger

from components.drivetrain import SwerveChassis
from components.arm_control import GrabberControl, ArmControl
from magicbot import AutonomousStateMachine, state, timed_state
from pathplannerlib import PathPoint
from wpimath.units import feetToMeters
from wpimath.geometry import Pose2d, Transform2d


class PlaceHigh(AutonomousStateMachine):
    MODE_NAME = "Place High"

    swerveChassis: SwerveChassis
    grabberControl: GrabberControl
    armControl: ArmControl

    @state(first=True)
    def loadTrajectory(self, initial_call):
        if initial_call:
            if not self.swerveChassis.holonomicController.trajectoryType:
                self.startPose = self.swerveChassis.estimator.getEstimatedPosition()
                self.endPose = self.startPose.transformBy(Transform2d(feetToMeters(2), 0, 0))
                ##TODO figure out how to calculate heading
                startPoint = PathPoint(self.startPose.translation(), self.startPose.rotation())
               # gridPosition = self.fieldLayout.getPosition(self.gridPosition).toPose2d()
                endPoint = PathPoint(self.endPose.translation(), self.endPose.rotation())
                self.swerveChassis.holonomicController.initSimpleTrajectory(
                    startPoint, # Starting position
			        endPoint, # Ending position
                )
        self.swerveChassis.autoDrive()
        if self.swerveChassis.holonomicController.atReference():
            self.next_state('liftArm')

    @state
    def liftArm(self, initial_call):
        if initial_call:
            self.armControl.next_state('moveToUpper')
        if self.armControl.arm.atPosition():
            self.next_state('moveBack')
        self.armControl.engage()

    @state
    def moveBack(self, initial_call):
        if initial_call:
            if not self.swerveChassis.holonomicController.trajectoryType:
                ##TODO figure out how to calculate heading
                endPoint = PathPoint(self.startPose.translation(), self.startPose.rotation())
               # gridPosition = self.fieldLayout.getPosition(self.gridPosition).toPose2d()
                startPoint = PathPoint(self.endPose.translation(), self.endPose.rotation())
                self.swerveChassis.holonomicController.initSimpleTrajectory(
                    startPoint, # Starting position
			        endPoint, # Ending position
                )
        self.swerveChassis.autoDrive()
        if self.swerveChassis.holonomicController.atReference():
            self.next_state('dropObject')
        
    @state
    def dropObject(self, initial_call):
        if initial_call:
            self.grabberControl.next_state('dropping')
        self.grabberControl.engage()
        if self.grabberControl.current_state == 'empty':
            self.done()

    

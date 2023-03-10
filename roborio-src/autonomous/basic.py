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
class moveToGridRight(AutonomousStateMachine):
    MODE_NAME = "Place cone"

    swerveChassis: SwerveChassis
    armControl: ArmControl
    grabberControl: GrabberControl
    startX = 2.47
    endX = 1.81

    @state(first=True)
    def raiseArm(self, initial_call):
        if initial_call:
            self.logger.info(f'Vision Estimate: {self.swerveChassis.visionPoseEstimator.getEstimatedRobotPose()}')
            self.swerveChassis.setFieldPosition(Pose2d(self.startX,0,0))
            self.logger.info(f'Estimator Field Position is: {self.swerveChassis.estimator.getEstimatedPosition()}')
            startTrajectoryPose = self.swerveChassis.estimator.getEstimatedPosition()
            endTrajectoryPose = startTrajectoryPose.transformBy(
                Transform2d(self.endX - self.startX, 0 , 0)
            )
            self.logger.info(f'Starting at {startTrajectoryPose}')
            self.logger.info(f'Ending at: {endTrajectoryPose}')
            ##TODO figure out how to calculate heading
            startPoint = PathPoint(startTrajectoryPose.translation(), startTrajectoryPose.rotation())
            endPoint = PathPoint(endTrajectoryPose.translation(), endTrajectoryPose.rotation())
            self.swerveChassis.holonomicController.initSimpleTrajectory(
                startPoint, # Starting position
                endPoint, # Ending position
            )
            self.logger.info(f'Raising arm to upper')
            self.armControl.next_state('moveToUpper')
        self.armControl.engage()
        if self.armControl.last_state == 'atUpper':
            self.next_state('moveBack')

    @state()
    def moveBack(self, initial_call):
        self.swerveChassis.autoDrive()
        if self.swerveChassis.holonomicController.atReference():
            self.next_state("dropCone")

    @state()
    def dropCone(self, initial_call):
        if initial_call:
            self.logger.info(f'Vision Estimate: {self.swerveChassis.visionPoseEstimator.getEstimatedRobotPose()}')
            self.swerveChassis.setFieldPosition(Pose2d(self.startX,0,0))
            self.logger.info(f'Estimator Field Position is: {self.swerveChassis.estimator.getEstimatedPosition()}')
            startTrajectoryPose = self.swerveChassis.estimator.getEstimatedPosition()
            endTrajectoryPose = startTrajectoryPose.transformBy(
                Transform2d(self.startX - self.endX, 0 , 0)
            )
            self.logger.info(f'Starting at {startTrajectoryPose}')
            self.logger.info(f'Ending at: {endTrajectoryPose}')
            ##TODO figure out how to calculate heading
            startPoint = PathPoint(startTrajectoryPose.translation(), startTrajectoryPose.rotation())
            endPoint = PathPoint(endTrajectoryPose.translation(), endTrajectoryPose.rotation())
            self.swerveChassis.holonomicController.initSimpleTrajectory(
                startPoint, # Starting position
                endPoint, # Ending position
            )
            self.grabberControl.next_state('dropping')
        self.grabberControl.engage()
        if not self.grabberControl.hasObject:
            self.next_state('moveForward')
    
    @state()
    def moveForward(self, initial_call):
        self.swerveChassis.autoDrive()
        if self.swerveChassis.holonomicController.atReference():
            self.next_state('dropArm')
        
    @state()
    def dropArm(self, initial_call):
        if initial_call:
            self.armControl.next_state('moveToHome')
        self.armControl.engage()
        if self.armControl.last_state == 'atHome':
            self.next_state('end')

    @state()
    def end(self):
        self.done()



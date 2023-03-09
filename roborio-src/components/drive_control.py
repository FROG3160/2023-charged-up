from magicbot.state_machine import StateMachine
from magicbot import state, timed_state, feedback, tunable, default_state
from components.drivetrain import SwerveChassis
import config
from components.controllers import PPHolonomic
from pathplannerlib import PathPoint
from wpimath.geometry import Pose2d, Rotation2d, Transform2d
import math


class DriveControl(StateMachine):

    swerveChassis: SwerveChassis
    VERBOSE_LOGGING = True

    def __init__(self):
        self.vX = 0
        self.vY = 0
        self.vT = 0
        self.throttle = 0
        self._endpoint = Pose2d(math.inf, math.inf, math.inf)

    @default_state
    def fieldOrientedDrive(self):
        self.swerveChassis.fieldOrientedDrive(
            self.vX,
            self.vY,
            self.vT,
            self.throttle
        )

    @state(first=True)
    def driveToWayPoint(self, initial_call):
        if initial_call:
            self.logger.info(f'Initializing trajectory to {self._endpoint}')
            if self._endpoint != Pose2d(math.inf, math.inf, math.inf):
            #init the trajectory
                startPoint = self.createPathPoint(
                    self.swerveChassis.estimator.getEstimatedPosition()
                )
                ##TODO figure out how to calculate heading
                endPoint = self.createPathPoint(
                    self.fieldLayout.getPosition(self.gridPosition).toPose2d()
                )
                self.logger.info(f"AUTODRIVE TO WAYPOINT: {startPoint} to {endPoint}")
                self.swerveChassis.holonomicController.initSimpleTrajectory(
                    startPoint, # Starting position
                    endPoint, # Ending position
                )
            else:
                self.logger.warning('driveToWayPoint called without endpoint')
                self.next_state_now('fieldOrientedDrive')
        self.swerveChassis.autoDrive()
        if self.swerveChassis.holonomicController.atReference():
            self._endpoint = Pose2d(math.inf, math.inf, math.inf)
            self.next_state('fieldOrientedDrive')

    def createPathPoint(self, pose: Pose2d):
        return PathPoint(pose.translation(), pose.rotation())
    
    def setEndpoint(self, pose: Pose2d):
        self._endpoint = pose
    


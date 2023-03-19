from magicbot import state, timed_state, feedback, tunable, default_state
from magicbot.state_machine import StateMachine
from components.drivetrain import SwerveChassis
from pathplannerlib import PathPoint
from wpimath.geometry import Pose2d, Rotation2d, Transform2d
import math
from components.vision import FROGLimeLightVision

class DriveControl(StateMachine):
    
    swerveChassis: SwerveChassis
    limelight: FROGLimeLightVision
    VERBOSE_LOGGING = True

    def __init__(self) -> None:
        self._vX = 0
        self._vY = 0
        self._vT = 0
        self._throttle = 0
        self._endPose: Pose2d = None
        pass

    def autoDrive(self):
        self.engage()

    # State fieldOriented (as the default state) This will be the first state.
    @default_state
    def fieldOriented(self):
        self.swerveChassis.fieldOrientedDrive(self._vX, self._vY, self._vT, self._throttle)

    # State robotOriented (for driving to cones, cubes and posts).
    @state()
    def robotOriented(self):
        self.swerveChassis.robotOrientedDrive(self._vX, self._vY, self._vT)

    @state()
    def driveToObject(self):
        self.swerveChassis.robotOrientedDrive(*self.limelight.getVelocities())

    # State locked (for turning the wheels in to keep it from moving).
    @state()
    def locked(self):
        self.swerveChassis.lockChassis()

    @state(first=True)
    def driveToWayPoint(self, initial_call):
        if not self._endPose is None:
            if initial_call:
            #init the trajectory
                self.logger.info(f'Initializing trajectory to {self._endPose}')
                startPoint = self.createPathPoint(
                    self.swerveChassis.estimator.getEstimatedPosition()
                )
                ##TODO figure out how to calculate heading
                endPoint = self.createPathPoint(
                    self._endPose
                )
                self.logger.info(f"AUTODRIVE TO WAYPOINT: {startPoint} to {endPoint}")
                self.swerveChassis.holonomicController.initSimpleTrajectory(
                    startPoint, # Starting position
                    endPoint, # Ending position
                )
            else:
                self.logger.warning('driveToWayPoint called without endpoint')
                self.next_state_now('fieldOriented')
            self.swerveChassis.holonomicDrive()
            if self.swerveChassis.holonomicController.atReference():
                self._endPose = None
                self.done()
        else:
            self.next_state('fieldOriented')

    def createPathPoint(self, pose: Pose2d):
        return PathPoint(pose.translation(), pose.rotation())
    
    def setEndpoint(self, pose: Pose2d):
        self._endPose = pose

    def setVelocities(self, vX: float, vY: float, vT: float, throttle: float = 1.0) -> None:
        self._vX, self._vY, self._vT, self._throttle = (vX, vY, vT, throttle)

    


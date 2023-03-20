from magicbot import state, default_state
from magicbot.state_machine import StateMachine
from components.drivetrain import SwerveChassis
from pathplannerlib import PathPoint
from wpimath.geometry import Pose2d
from components.vision import FROGLimeLightVision, LL_CONE, LL_CUBE
from components.controllers import PPHolonomic, FROGXboxDriver
from components.field import FROGFieldLayout


class DriveControl(StateMachine):
    swerveChassis: SwerveChassis
    limelight: FROGLimeLightVision
    driverController: FROGXboxDriver

    VERBOSE_LOGGING = True

    def __init__(self) -> None:
        self._vX = 0
        self._vY = 0
        self._vT = 0
        self._throttle = 0
        self._endPose: Pose2d = None
        self._pathName = None

    def setup(self):
        self.holonomic = PPHolonomic(self.swerveChassis.kinematics)

    def autoDrive(self):
        self.engage()

    def autoDriveToCone(self):
        if self.limelight.getGrabberPipeline == LL_CUBE:
            self.limelight.setGrabberPipeline(LL_CONE)
        self.engage(initial_state="driveToObject", force=True)

    def autoDriveToCube(self):
        if self.limelight.getGrabberPipeline == LL_CONE:
            self.limelight.setGrabberPipeline(LL_CUBE)
        self.engage(initial_state="driveToObject", force=True)

    def holonomicDriveToWaypoint(self, waypoint:Pose2d):
        self._endPose = waypoint
        self.engage(initial_state='driveToWayPoint', force=True)

    def holonomicDrivePath(self, pathName:str):
        self._pathName = pathName
        


    # State fieldOriented (as the default state) This will be the first state.
    @default_state
    def fieldOriented(self):
        self.swerveChassis.fieldOrientedDrive(
            #self._vX, self._vY, self._vT, self._throttle
        self.driverController.getFieldForward(),
        self.driverController.getFieldLeft(),
        self.driverController.getFieldRotation(),
        self.driverController.getFieldThrottle(),
        )

    # State robotOriented (for driving to cones, cubes and posts).
    @state()
    def robotOriented(self):
        self.swerveChassis.robotOrientedDrive(self._vX, self._vY, self._vT)

    @state()
    def driveToObject(self):
        velocities = self.limelight.getVelocities()
        self.swerveChassis.robotOrientedDrive(*velocities)

    @state()
    def drivePath(self, initial_call):
        if initial_call:
            self.holonomic.loadPathPlanner(self._pathName)
        if not self.holonomic.atReference():
            self.swerveChassis.holonomicDrive(
                self.holonomic.getChassisSpeeds(
                    self.swerveChassis.estimator.getEstimatedPosition()
                )
            )

    # State locked (for turning the wheels in to keep it from moving).
    @state()
    def locked(self):
        self.swerveChassis.lockChassis()

    @state(first=True)
    def driveToWayPoint(self, initial_call):
        if initial_call:
            # init the trajectory
            self.logger.info(f"Initializing trajectory to {self._endPose}")
            startPoint = self.createPathPoint(
                self.swerveChassis.estimator.getEstimatedPosition()
            )
            ##TODO figure out how to calculate heading
            endPoint = self.createPathPoint(self._endPose)
            self.logger.info(f"AUTODRIVE TO WAYPOINT: {startPoint} to {endPoint}")
            self.holonomic.initSimpleTrajectory(
                startPoint,  # Starting position
                endPoint,  # Ending position
            )
        if not self.holonomic.atReference():
            newSpeeds = self.holonomic.getChassisSpeeds(
                self.swerveChassis.estimator.getEstimatedPosition()
            )
            self.logger.info(f'Speeds: {newSpeeds}')
            self.swerveChassis.holonomicDrive(
                newSpeeds
            )

    def createPathPoint(self, pose: Pose2d):
        return PathPoint(pose.translation(), pose.rotation())

    def setEndpoint(self, pose: Pose2d):
        self._endPose = pose

    def setVelocities(
        self, vX: float, vY: float, vT: float, throttle: float = 1.0
    ) -> None:
        self._vX, self._vY, self._vT, self._throttle = (vX, vY, vT, throttle)
    
    def done(self):
        self._endPose = None
        self._pathName = None
        super().done()

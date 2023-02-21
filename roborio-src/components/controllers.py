from wpilib import Joystick, XboxController,Timer
from wpilib.interfaces import GenericHID
from wpimath.controller import PIDController, ProfiledPIDControllerRadians, HolonomicDriveController
from utils.utils import remap
import wpimath
from wpimath.units import feetToMeters
from wpimath.trajectory import TrajectoryGenerator, TrajectoryConfig, TrapezoidProfileRadians, Trajectory
from wpimath.geometry import Pose2d, Translation2d, Transform2d, Rotation2d
import math
import config
from wpimath.kinematics import ChassisSpeeds
from pathplannerlib import PathConstraints, PathPlanner, PathPoint, PathPlannerTrajectory, controllers
from magicbot import feedback
from wpilib import SmartDashboard
import os

MAX_TRAJECTORY_SPEED = feetToMeters(5)
MAX_TRAJECTORY_ACCEL = feetToMeters(5)

RIGHT_RUMBLE = GenericHID.RumbleType.kRightRumble
LEFT_RUMBLE = GenericHID.RumbleType.kLeftRumble

     # config for saitek joystick
        # self.driverController = FROGStick(0, 0, 1, 3, 2)
        # config for Logitech Extreme 3D
        #self.driverController = FROGStick(0, 0, 1, 2, 3)
SAITEK_AXIS_CONFIG = {
    'xAxis': 0,
    'yAxis': 1,
    'rAxis': 3,
    'tAxis': 2
}
LOGITECH_EXTREME_AXIS_CONFIG = {
    'xAxis': 0,
    'yAxis': 1,
    'rAxis': 2,
    'tAxis': 3
}

class FROGStickDriver(Joystick):
    """Extended class of wpilib.Joystick

    Returns:
        FROGStick: Custom Joystick class
    """

    DEADBAND = 0.025
    SPEED_DIVISOR = 1
    ROTATION_DIVISOR = 1.6
    ROTATION_MIN = 0
    ROTATION_MAX = 0.5
    DEBOUNCE_PERIOD = 0.5

    def __init__(
        self, port: int, xAxis: int = 1, yAxis: int = 2, rAxis: int = 3, tAxis: int = 4
    ) -> None:
        """Constructor for FROGStick

        :param port: The port on the Driver Station that the joystick
        is plugged into (0-5).
        :param xAxis: channel for the X axis
        :param yAxis: channel for the Y axis
        :param rAxis: channel for the rotation (twist) axis
        :param tAxis: channel for the throttle axis
        """

        super().__init__(port)
        self.setThrottleChannel(tAxis)
        self.setTwistChannel(rAxis)
        self.setXChannel(xAxis)
        self.setYChannel(yAxis)
        self.button_latest = {}
        self.timer = Timer

    def getFieldForward(self):
        """Get's the joystick's Y axis and
        inverts it so pushing forward is positive
        and translates to chassis moving away from
        driver.

        Returns:
            float: -1 to 1
        """
        # inverts the joystick's Y axis so pushing
        # forward is positive and pulling back is
        # negative

        return wpimath.applyDeadband(-self.getY(), self.DEADBAND)

    def getFieldLeft(self):
        """Get's the joystick's X axis and
        inverts it so pushing left is positive
        and translates to chassis moving to the
        left of the driver.

        Returns:
            float: -1 to 1
        """
        # inverts the joystick's X axis so pushing
        # left is positive and pushing right is negative
        return wpimath.applyDeadband(-self.getX(), self.DEADBAND)

    def getFieldRotation(self):
        """Get's the joystick's Twist axis and
        inverts it so twisting CCW is positive
        and translates to chassis rotating CCW.

        Returns:
            float: -1 to 1
        """
        # inverts the joystick's twist axis so CCW
        # is positive and CW is negative
        return wpimath.applyDeadband(-self.getTwist(), self.DEADBAND)

    def get_speed(self):
        # Dampens the -1 to 1 values of the joystick to provide a smoothed acceleration
        speed = self.getY()
        speed = -1 * (
            speed**3 / self.SPEED_DIVISOR if abs(speed) > self.DEADBAND else 0
        )
        return speed

    def getFieldThrottle(self):
        val = -super().getThrottle()
        throttle = (val + 1) / 2
        return throttle
        
    def get_rotation(self):
        return (
            self.getTwist() / self.ROTATION_DIVISOR
            if abs(self.getTwist()) > self.DEADBAND
            else 0
        )

    def getRangedCubedRotation(self):
        return remap(
            self.getTwist() ** 3,
            self.SPEED_DIVISOR,
            1,
            self.ROTATION_MIN,
            self.ROTATION_MAX,
        )

    def getRangeRotation(self):
        return remap(
            self.getTwist(),
            self.SPEED_DIVISOR,
            1,
            self.ROTATION_MIN,
            self.ROTATION_MAX,
        )

    def get_button(self, num):
        val = self.getRawButton(num)
        return val

    def get_debounced_button(self, num):
        """Returns the value of the joystick button. If the button is held down, then
        True will only be returned once every ``debounce_period`` seconds"""
        val = False
        now = self.timer.getFPGATimestamp()
        if self.getRawButton(num):
            if (now - self.button_latest.get(num, 0)) > self.DEBOUNCE_PERIOD:
                self.button_latest[num] = now
                val = True
        return val

class FROGXboxDriver(XboxController):
    
    DEADBAND = 0.045
    ROTATION_DIVISOR = 1
    
    def __init__(self, channel):

        super().__init__(channel)
        self.button_latest = {}

    def getFieldRotation(self):
        
        return wpimath.applyDeadband(-self.getRightX(), self.DEADBAND)

    def getFieldForward(self):
        return wpimath.applyDeadband(-self.getLeftY(), self.DEADBAND)

    def getFieldLeft(self):
        return wpimath.applyDeadband(-self.getLeftX(), self.DEADBAND)

    def getFieldThrottle(self):
        return wpimath.applyDeadband(self.getRightTriggerAxis(), 0)


class FROGXboxOperator(XboxController):
    def __init__(self, channel):
        super().__init__(channel)





class FROGHolonomic(HolonomicDriveController):
    max_trajectory_speed = config.MAX_TRAJECTORY_SPEED
    max_trajectory_accel = config.MAX_TRAJECTORY_ACCEL

    def __init__(self, kinematics):
        # the holonomic controller
        self.xController = config.holonomicTranslationPIDController
        self.yController = config.holonomicTranslationPIDController
        self.angleController = config.holonomicAnglePIDController
        self.angleController.enableContinuousInput(-1 * math.pi, math.pi)
        super().__init__(self.xController, self.yController, self.angleController)
        self.timer = Timer()
        self.trajectoryType = False
        self.kinematics = kinematics
        SmartDashboard.putNumber('ControllerP', self.xController.getP())
        SmartDashboard.putNumber('angleControllerP', self.angleController.getP())

    def initialize(self, trajectoryType):
        self.firstCall = True
        self.trajectoryType = trajectoryType

    def loadPID(self):
        self.xController.setP(SmartDashboard.getNumber('ControllerP'))
        self.yController.setP(SmartDashboard.getNumber('ControllerP'))
        self.angleController.setP(SmartDashboard.getNumber('angleControllerP'))


    def initTrajectory(
        self, startPose: Pose2d, wayPoints: list[Translation2d], endPose: Pose2d
    ):
        # the trajectory setup
        trajectoryConfig = TrajectoryConfig(
            self.max_trajectory_speed, self.max_trajectory_accel
        )
        trajectoryConfig.setKinematics(self.kinematics)
        self.trajectory = TrajectoryGenerator.generateTrajectory(
            startPose,  # Starting position
            wayPoints,  # Pass through these points
            endPose,  # Ending position
            trajectoryConfig,
        )
        # Pose2d(0, 0, Rotation2d.fromDegrees(0)), # Starting position
        # [Translation2d(1,1), Translation2d(2,-1)], # Pass through these points
        # Pose2d(3, 0, Rotation2d.fromDegrees(0)), # Ending position
        self.initialize("wpilib")

    def initSimpleTrajectory(self, startPoint: PathPoint, endPoint: PathPoint):
        """Initializes a PathPlanner trajectory"""
        self.trajectory = PathPlanner.generatePath(
            PathConstraints(self.max_trajectory_speed, self.max_trajectory_accel),
            [
                # PathPoint(
                #     Translation2d(2.6, 4.6),
                #     Rotation2d.fromDegrees(90),
                #     Rotation2d.fromDegrees(0),
                # ),  # position, heading(direction of travel), holonomic rotation
                # PathPoint(
                #     Translation2d(9, 7),
                #     Rotation2d.fromDegrees(0),
                #     Rotation2d.fromDegrees(-90),
                # ),  # position, heading(direction of travel), holonomic rotation
                startPoint,
                endPoint
            ],
        )
        self.initialize("pathPlanner")

    def loadPathPlanner(self, pathName):
        """Loads a PathPlanner trajectory from a preconfigured path.

        Args:
            pathName (_type_): The name of the path, without the .path extension.
        """
        self.trajectory = PathPlanner.loadPath(
            os.path.join(os.path.dirname(__file__), r"..", r"paths", pathName),
            PathConstraints(self.max_trajectory_speed, self.max_trajectory_accel),
            False,
        )
        self.initialize("pathPlanner")

    def loadPathWeaver(self):
        self.trajectory = TrajectoryUtil.fromPathweaverJson(pathWeaverPath)
        self.initialize("pathWeaver")

    @feedback
    def getGoalPose(self):
        if type(self.trajectory) == PathPlannerTrajectory:
            return self.trajectory.sample(self.timer.get()).asWPILibState()
        else:
            return self.trajectory.sample(self.timer.get())
        

    def getChassisSpeeds(self, currentPose: Pose2d) -> ChassisSpeeds:
        """Calculates the chassis speeds of the trajectory at the current time.

        Args:
            currentPose (Pose2d): current pose of the Robot

        Returns:
            ChassisSpeeds: translation and rotational vectors desired
        """
        if not self.trajectoryType is None:
            if self.firstCall:
                self.timer.start()
                self.firstCall = False
            # get the pose of the trajectory at the current time
            goalPose = self.getGoalPose()
            self.logger.info(
                "Auto Update -- initial Pose: %s\n  goal Pose: %s\n time: %s",
                currentPose, goalPose, self.timer.get()
            )
            return self.calculate(currentPose, goalPose, goalPose.pose.rotation())


class PPHolonomic(controllers.PPHolonomicDriveController):
    max_trajectory_speed = config.MAX_TRAJECTORY_SPEED
    max_trajectory_accel = config.MAX_TRAJECTORY_ACCEL

    def __init__(self, kinematics):
        # the holonomic controller
        self.xController = config.ppTranslationPIDController
        self.yController = config.ppTranslationPIDController
        self.rotationController = config.ppRotationPIDController
        #self.angleController.enableContinuousInput(-1 * math.pi, math.pi)
        super().__init__(self.xController, self.yController, self.rotationController)
        self.timer = Timer()
        self.trajectoryType = False
        self.kinematics = kinematics
        SmartDashboard.putNumber('ControllerP', self.xController.getP())
        SmartDashboard.putNumber('angleControllerP', self.rotationController.getP())

    def initialize(self, trajectoryType):
        self.firstCall = True
        self.trajectoryType = trajectoryType

    def loadPID(self):
        self.xController.setP(SmartDashboard.getNumber('ControllerP'))
        self.yController.setP(SmartDashboard.getNumber('ControllerP'))
        self.rotationController.setP(SmartDashboard.getNumber('angleControllerP'))


    def initSimpleTrajectory(self, startPoint: PathPoint, endPoint: PathPoint):
        """Initializes a PathPlanner trajectory"""
        self.trajectory = PathPlanner.generatePath(
            PathConstraints(self.max_trajectory_speed, self.max_trajectory_accel),
            False,
            [
                startPoint,
                endPoint
            ],
        )
        self.initialize("pathPlanner")

    def loadPathPlanner(self, pathName):
        """Loads a PathPlanner trajectory from a preconfigured path.

        Args:
            pathName (_type_): The name of the path, without the .path extension.
        """
        self.trajectory = PathPlanner.loadPath(
            os.path.join(os.path.dirname(__file__), r"..", r"paths", pathName),
            PathConstraints(self.max_trajectory_speed, self.max_trajectory_accel),
            False,
        )
        self.initialize("pathPlanner")

    def getChassisSpeeds(self, currentPose: Pose2d) -> ChassisSpeeds:
        """Calculates the chassis speeds of the trajectory at the current time.

        Args:
            currentPose (Pose2d): current pose of the Robot

        Returns:
            ChassisSpeeds: translation and rotational vectors desired
        """
        if not self.trajectoryType is None:
            if self.firstCall:
                self.timer.start()
                self.firstCall = False
            # get the pose of the trajectory at the current time
            referenceState = self.trajectory.sample(self.timer.get())
            return self.calculate(currentPose, referenceState)

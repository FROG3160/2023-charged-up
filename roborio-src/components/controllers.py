from wpilib import Joystick, XboxController,Timer
from wpilib.interfaces import GenericHID
from wpimath.controller import PIDController, ProfiledPIDControllerRadians, HolonomicDriveController
from utils.utils import remap
import wpimath
from wpimath.units import feetToMeters
from wpimath.trajectory import TrajectoryGenerator, TrajectoryConfig, TrapezoidProfileRadians
from wpimath.geometry import Pose2d, Translation2d, Transform2d, Rotation2d
import math
import config
from commands2.button import CommandJoystick, CommandXboxController

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
    
    DEADBAND = 0.15
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
        return wpimath.applyDeadband(self.getRightTriggerAxis(), self.DEADBAND)


class FROGXboxOperator(XboxController):
    def __init__(self, channel):
        super().__init__(channel)





class FROGHolonomic(HolonomicDriveController):
    def __init__(self, kinematics):
        # the holonomic controller
        self.kinematics = kinematics
        self.xController = PIDController(1, 0, 0)
        self.yController = PIDController(1, 0, 0)
        self.angleController = ProfiledPIDControllerRadians(
            1, 0, 0, TrapezoidProfileRadians.Constraints(math.pi, math.pi)
        )
        self.angleController.enableContinuousInput(-1*math.pi, math.pi)
        super().__init__(self.xController, self.yController, self.angleController)
        self.timer = Timer()
        self.trajectoryLoaded = False



    def initTrajectory(self, startPose: Pose2d, wayPoints: list[Translation2d], endPose: Pose2d ):
        # the trajectory setup
        trajectoryConfig = TrajectoryConfig(MAX_TRAJECTORY_SPEED, MAX_TRAJECTORY_ACCEL)
        trajectoryConfig.setKinematics(self.kinematics)
        self.trajectory = TrajectoryGenerator.generateTrajectory(
			startPose, # Starting position
			wayPoints, # Pass through these points
			endPose, # Ending position
			trajectoryConfig
        )
        self.firstCall = True
        self.trajectoryLoaded = True

			# Pose2d(0, 0, Rotation2d.fromDegrees(0)), # Starting position
			# [Translation2d(1,1), Translation2d(2,-1)], # Pass through these points
			# Pose2d(3, 0, Rotation2d.fromDegrees(0)), # Ending position

    def getChassisSpeeds(self, chassisPose: Pose2d, finalRotation2d: Rotation2d):
        if self.firstCall:
            self.timer.start()
            self.firstCall = False
        # get the pose of the trajectory at the current time
        goalPose = self.trajectory.sample(self.timer.get())
        return self.calculate(chassisPose, goalPose, finalRotation2d)
from commands2 import SubsystemBase
from ctre import (
    FeedbackDevice,
    NeutralMode,
    RemoteSensorSource,
    WPI_TalonFX,
    WPI_CANCoder,
    TalonFXInvertType,
    ControlMode,
    SensorInitializationStrategy,
    AbsoluteSensorRange,
    StatusFrameEnhanced,
)
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.controller import PIDController, ProfiledPIDControllerRadians
from wpimath.trajectory import TrapezoidProfileRadians
from wpilib import Field2d
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath.kinematics import (
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
    ChassisSpeeds,
    SwerveModuleState,
    SwerveModulePosition
)
import math
from magicbot import feedback, tunable
from .sensors import FROGGyro
from utils.utils import DriveUnit, Rescale
from logging import Logger
import config
from sensors import FROGGyro

# Motor Control modes
VELOCITY_MODE = ControlMode.Velocity
POSITION_MODE = ControlMode.Position

class GearStages:
    def __init__(
        self, gear_stages:list
    ):
        """
        Constructs a DriveUnit object that stores data about the gear stages.
        Args:
            gear_stages (list): list of gear stages expressed as tuples of two integers e.g. [(10, 32), (9, 24)]
        """
        self.gearing = math.prod(gear_stages)

    def toMotor(self, rotations):
        """Calculates motor rotations given the rotation at the other end of the gears."""
        return rotations / self.gearing

    def fromMotor(self, rotations):
        """Calculates final gear rotation given the motor's rotation"""
        return rotations * self.gearing


class DriveUnit:
    def __init__(
        self, gear_stages: list, motor_rpm: int, diameter: float, cpr: int
    ):
        """Constructs a DriveUnit object that stores data about the drive, gear stages, and wheel.
        Args:
            gear_stages (list): list of gear stages expressed as tuples of two integers e.g. [(10, 32), (9, 24)]
            motor_rpm (int): Maximum rpm of the attached motor
            diameter (float): Diameter of the attached wheel in meters
            cpr (int): Number of encoder counts per revolution
        """
        self.gearing = GearStages(gear_stages)
        self.motor_rpm = motor_rpm
        self.cpr = cpr
        self.circumference = math.pi * diameter

    def speedToVelocity(self, speed: float) -> float:
        """Converts linear speed to Falcon velocity
        Args:
            speed (float): desired linear speed in meters per second
        Returns:
            float: velocity in encoder counts per 100ms
        """
        wheel_rotations_sec = speed / self.circumference
        motor_rotations_sec = self.gearing.toMotor(wheel_rotations_sec)
        ticks_per_sec = motor_rotations_sec * self.cpr
        return ticks_per_sec / 10

    def velocityToSpeed(self, velocity: float) -> float:
        """Converts Falcon velocity to linear speed
        Args:
            velocity (float): velocity in encoder counts per 100ms
        Returns:
            float: linear speed in meters per second
        """
        ticks_per_sec = velocity * 10
        motor_rotations_sec = ticks_per_sec / self.cpr
        wheel_rotations_sec = self.gearing.fromMotor(motor_rotations_sec)
        return wheel_rotations_sec * self.circumference


class SwerveModule(SubsystemBase):

    def __init__(self, drive_motor_id: int, steer_motor_id: int, steer_sensor_id: int, steer_sensor_offset: float, location: Translation2d):
        # set initial states for the component
        self.drive = WPI_TalonFX(drive_motor_id)
        self.steer = WPI_TalonFX(steer_motor_id)
        self.encoder = WPI_CANCoder(steer_sensor_id)
        self.steerOffset = steer_sensor_offset
        self.location = location
        
        self.velocity = 0
        self.angle = 0
        self.enabled = False
        self.requestedState = SwerveModuleState(0, Rotation2d.fromDegrees(0)) #zeroing swerve state  TODO: set to actual angle?
        self.drive_unit = DriveUnit(
            config.MODULE_DRIVE_GEARING,
            config.FALCON_MAX_RPM,
            config.MODULE_WHEEL_DIAMETER,
            config.FALCON_TICKS_PER_ROTATION,
        )
        self.calculated_velocity = 0
        # configure all swerve module components
        self.config()

    def disable(self):
        self.enabled = False

    def enable(self):
        self.enabled = True

    def getEncoderAbsolutePosition(self) -> float:
        """gets the absolute position from the CANCoder
        Returns:
            float: position of the sensor in degrees (-180 to 180)
        """
        return self.encoder.getAbsolutePosition()

    # TODO: Determine which way we want these
    # to read.  Right now they are inverted
    # to visually show positive angles to the
    # right (clockwise) to match the smartdashboard
    # @feedback()
    def getCommandedDegrees(self):
        return -self.requestedState.angle.degrees()

    # TODO: rewrite this whole thing so execute updates attributes
    # TODO: and the attributes are read by these methods.
    def getCurrentRotation(self) -> Rotation2d:
        if degrees := self.getEncoderAbsolutePosition():
            return Rotation2d.fromDegrees(degrees)
        else:
            return Rotation2d.fromDegrees(0)

    def getCommandedVelocity(self):
        return self.calculated_velocity

    def getActualVelocity(self):
        return self.drive.getSelectedSensorVelocity()

    def getCurrentSpeed(self) -> float:
        return self.drive_unit.velocityToSpeed(
            self.drive.getSelectedSensorVelocity()
        )

    def getDrivePosition(self) -> float:
        return self.drive.getSelectedSensorPosition()

    # TODO: see TODO on getCurrentRotation()
    def getCurrentState(self):
        return SwerveModuleState(
            self.getCurrentSpeed(),
            self.getCurrentRotation(),
        )

    def getSteerPosition(self):
        return self.steer.getSelectedSensorPosition(0)

    def getSwerveModulePosition(self):
        return SwerveModulePosition()

    def resetRemoteEncoder(self):
        self.encoder.setPositionToAbsolute()

    def config(self):
        # configure CANCoder
        # No worky: self.encoder.configAllSettings(cfgSteerEncoder)
        # TODO: ^^ see if we can use the configAllSettings method again
        # TODO:  Review all other config settings for the devices
        self.encoder.configAbsoluteSensorRange(
            AbsoluteSensorRange.Signed_PlusMinus180
        )
        self.encoder.configSensorDirection(False)
        # adjust 0 degree point with offset
        self.encoder.configMagnetOffset(self.steerOffset)
        self.encoder.configSensorInitializationStrategy(
            SensorInitializationStrategy.BootToAbsolutePosition
        )
        # set position to Absolute

        self.resetRemoteEncoder()

        self.steer.configAllSettings(config.cfgSteerMotor)
        self.steer.setStatusFramePeriod(
            StatusFrameEnhanced.Status_1_General, 250
        )
        # define the remote CANCoder as Remote Feedback 0
        self.steer.configRemoteFeedbackFilter(
            self.encoder.getDeviceNumber(), RemoteSensorSource.CANCoder, 0
        )
        self.steer.setInverted(TalonFXInvertType.Clockwise)
        # configure Falcon to use Remote Feedback 0
        # self.steer.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0)
        self.steer.configIntegratedSensorInitializationStrategy(
            SensorInitializationStrategy.BootToAbsolutePosition
        )
        self.steer.configIntegratedSensorAbsoluteRange(
            AbsoluteSensorRange.Signed_PlusMinus180
        )
        self.steer.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0)
        self.steer.setSensorPhase(True)
        self.steer.setNeutralMode(NeutralMode.Brake)
        # configure drive motor
        self.drive.configAllSettings(config.cfgDriveMotor)
        self.drive.setStatusFramePeriod(
            StatusFrameEnhanced.Status_1_General, 250
        )
        self.drive.setInverted(TalonFXInvertType.Clockwise)
        self.drive.configClosedloopRamp(0.25)

        self.current_states = None
        self.current_speeds = None

    def setState(self, state):
        self.requestedState = state

    # TODO: Figure out if this needs to be renamed.
    def execute(self):
        # execute is called each iteration
        # define what needs to happen if the
        # component is enabled/disabled
        # TODO: work out if the enabling/disabling is needed in command-based
        if self.enabled:

            # 
            # using built-in optimize method instead of our custom one from last year
            self.requestedState = SwerveModuleState.optimize(
                self.requestedState, 
                Rotation2d(self.getSteerPosition()/config.CANCODER_TICKS_PER_RADIAN)
            )
            # current_steer_position = self.getSteerPosition()
            # steer_adjust_radians, speed_inversion = optimize_steer_angle(
            #     self.state, current_steer_position / kCANCoderTicksPerRadian
            # )
            self.steer.set(
                POSITION_MODE,
                self.requestedState.angle.radians() * config.CANCODER_TICKS_PER_RADIAN
            )
            # self.steer.set(POSITION_MODE, self.getCommandedTicks())
            # self.steer.set(
            #     POSITION_MODE,
            #     current_steer_position
            #     + (steer_adjust_radians * kCANCoderTicksPerRadian),
            # )
            # set velocity for Falcon to ticks/100ms
            # self.calculated_velocity = self.drive_unit.speedToVelocity(
            #     self.state.speed * speed_inversion
            # )
            self.drive.set(
                VELOCITY_MODE,
                self.drive_unit.speedToVelocity(self.requestedState.speed)
            )
        else:
            self.drive.set(0)

class SwerveChassis:

    def __init__(self):
        self.center = Translation2d(0,0)
        self.swerveFrontLeft = SwerveModule(
            **config.MODULE_FRONT_LEFT
        )
        self.swerveFrontRight = SwerveModule(
            **config.MODULE_FRONT_RIGHT
        )
        self.swerveBackLeft = SwerveModule(
            **config.MODULE_BACK_LEFT
        )
        self.swerveBackRight = SwerveModule(
            **config.MODULE_BACK_RIGHT
        )

        self.modules = (
            self.swerveFrontLeft,
            self.swerveFrontRight,
            self.swerveBackLeft,
            self.swerveBackRight,
        )

        self.kinematics = SwerveDrive4Kinematics(
            # the splat operator (asterisk) below expands
            # the list into positional arguments for the
            # kinematics object.  We are taking the location
            # property of each swerveModule object and passing
            # it to SwerveDrive4Kinematics the order defined by
            # self.modules above.  Order is critical here.
            # We will receive back drive and steer values for
            # each SwerveModule in the same order we use here.
            *[m.location for m in self.modules]
        )

        self.gyro = FROGGyro()
        #self.field = 
        #self.logger =
        # 
        # initialize the drivetrain with zero movement
        self.speeds = ChassisSpeeds(0,0,0)
        # TODO: set values for pose depending on starting field position
        self.odometry = SwerveDrive4Odometry(
            self.kinematics,
            self.gyro.getRotation2d(),
            tuple([
                SwerveModulePosition(0, x.getCurrentRotation()) for x in self.modules
            ]),
            Pose2d()
        )


    def drive(self, vX, vy, vT, chassis_angle=None):
        pass

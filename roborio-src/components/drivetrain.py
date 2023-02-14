import math

import config
from commands2 import SubsystemBase, Swerve4ControllerCommand
from ctre import (
    AbsoluteSensorRange,
    ControlMode,
    FeedbackDevice,
    NeutralMode,
    RemoteSensorSource,
    SensorInitializationStrategy,
    StatusFrameEnhanced,
    TalonFXInvertType,
    WPI_CANCoder,
    WPI_TalonFX,
)
from wpimath.trajectory import (
    TrajectoryGenerator,
    TrajectoryConfig,
    TrapezoidProfileRadians,
)
from components.sensors import FROGGyro
from components.vision import FROGPhotonVision, FROGLimeLightVision
from utils.utils import DriveUnit
from wpilib import Field2d, SmartDashboard
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import (
    Pose2d,
    Rotation2d,
    Translation2d,
    Transform3d,
    Translation3d,
    Rotation3d,
    Transform2d,
)
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
    SwerveModulePosition,
    SwerveModuleState,
)

from .sensors import FROGGyro
from utils.utils import DriveUnit, Rescale
from wpimath.units import metersToInches, inchesToMeters, feetToMeters
from logging import Logger
import config
from wpimath.controller import (
    PIDController,
    ProfiledPIDControllerRadians,
    HolonomicDriveController,
)

# Motor Control modes
VELOCITY_MODE = ControlMode.Velocity
POSITION_MODE = ControlMode.Position

MAX_TRAJECTORY_SPEED = feetToMeters(5)
MAX_TRAJECTORY_ACCEL = feetToMeters(5)


def constrain_radians(rads):
    """Returns radians between -2*pi and 2*pi"""
    return math.atan2(math.sin(rads), math.cos(rads))


class FROGSwerveModuleState(SwerveModuleState):
    def optimize(self, current_rotation: Rotation2d):
        # def optimize_steer_angle(new_state: SwerveModuleState, current_radians):
        """This function takes the desired module state and the current
        angle of the wheel and calculates a new position that keeps the
        amount of rotation needed to under 90 degrees in either direction.
        Args:
            new_state (SwerveModuleState): the module state
            current_radians (float): current angle in radians.
                This value does not have to be between -pi and pi.
        Returns:
            SwerveModuleState

        """
        invert_speed = 1

        # all angles are in radians
        desired_angle = self.angle.radians()

        # we are taking the radians which may be < -pi or > pi and constraining
        # it to the range of -pi to pi for our calculations
        # current_angle = math.atan2(
        #     math.sin(current_rotation), math.cos(current_rotation)
        # )
        current_angle = constrain_radians(current_rotation.radians())

        n_offset = desired_angle - current_angle

        # if our offset is greater than 90 degrees, we need to flip 180 and reverse
        # speed.
        while abs(n_offset) > math.pi / 2:
            if n_offset < -math.pi / 2:
                n_offset += math.pi
                invert_speed *= -1
            elif n_offset > math.pi / 2:
                n_offset -= math.pi
                invert_speed *= -1
        new_angle = constrain_radians(current_angle + n_offset)

        if abs(n_offset) > math.pi / 2:
            print(">>>>>>>>>>>>>>>>>ERROR<<<<<<<<<<<<<<<<<<<<")
        self.speed *= invert_speed
        self.angle = Rotation2d(new_angle)
        # return FROGSwerveModuleState(self.speed * invert_speed, Rotation2d(new_angle))


class GearStages:
    def __init__(self, gear_stages: list):
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
    def __init__(self, gear_stages: list, motor_rpm: int, diameter: float, cpr: int):
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

    def positionToDistance(self, position: int) -> float:
        motor_rotations = position / self.cpr
        wheel_rotations = self.gearing.fromMotor(motor_rotations)
        return wheel_rotations * self.circumference


class SwerveModule:
    def __init__(
        self,
        name: str,
        drive_motor_id: int,
        steer_motor_id: int,
        steer_sensor_id: int,
        steer_sensor_offset: float,
        location: Translation2d,
    ):
        super().__init__()
        # set initial states for the component
        self.name = name
        self.drive = WPI_TalonFX(drive_motor_id)
        self.steer = WPI_TalonFX(steer_motor_id)
        self.encoder = WPI_CANCoder(steer_sensor_id)
        self.steerOffset = steer_sensor_offset
        self.location = location
        self.drive_unit = DriveUnit(
            config.MODULE_DRIVE_GEARING,
            config.FALCON_MAX_RPM,
            config.MODULE_WHEEL_DIAMETER,
            config.FALCON_TICKS_PER_ROTATION,
        )
        self.configModuleComponents()

        # self.velocity = 0
        # self.angle = 0
        self.enabled = False

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
    # def getCommandedDegrees(self):
    #     return -self.requestedState.angle.degrees()

    # TODO: rewrite this whole thing so execute updates attributes
    # TODO: and the attributes are read by these methods.
    def getCurrentRotation(self) -> Rotation2d:
        if degrees := self.getEncoderAbsolutePosition():
            return Rotation2d.fromDegrees(degrees)
        else:
            return Rotation2d.fromDegrees(0)

    # def getCommandedVelocity(self):
    #     return self.calculated_velocity

    # def getActualVelocity(self):
    #     return self.drive.getSelectedSensorVelocity()

    def getCurrentDistance(self) -> float:
        return self.drive_unit.positionToDistance(
            self.drive.getSelectedSensorPosition()
        )

    def getCurrentSpeed(self) -> float:
        return self.drive_unit.velocityToSpeed(self.drive.getSelectedSensorVelocity())

    # def getDrivePosition(self) -> float:
    #     return self.drive.getSelectedSensorPosition()

    # TODO: see TODO on getCurrentRotation()
    def getCurrentState(self):
        return SwerveModuleState(
            self.getCurrentSpeed(),
            self.getCurrentRotation(),
        )

    def getCurrentPosition(self):
        return SwerveModulePosition(
            self.getCurrentDistance(), self.getCurrentRotation()
        )

    def getSteerPosition(self):
        return self.steer.getSelectedSensorPosition(0)

    # def resetRemoteEncoder(self):
    #     self.encoder.setPositionToAbsolute()

    def configModuleComponents(self):
        # configure CANCoder
        # No worky: self.encoder.configAllSettings(cfgSteerEncoder)
        # TODO: ^^ see if we can use the configAllSettings method again
        # TODO:  Review all other config settings for the devices
        self.encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180)
        self.encoder.configSensorDirection(False)
        # adjust 0 degree point with offset
        self.encoder.configMagnetOffset(self.steerOffset)
        self.encoder.configSensorInitializationStrategy(
            SensorInitializationStrategy.BootToAbsolutePosition
        )
        # set position to Absolute

        self.steer.configAllSettings(config.cfgSteerMotor)
        self.steer.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 250)
        self.steer.setInverted(TalonFXInvertType.CounterClockwise)  # was Clockwise
        # define the remote CANCoder as Remote Feedback 0
        self.steer.configRemoteFeedbackFilter(
            self.encoder.getDeviceNumber(), RemoteSensorSource.CANCoder, 0
        )
        # configure Falcon to use Remote Feedback 0
        self.steer.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0)
        self.steer.configIntegratedSensorInitializationStrategy(
            SensorInitializationStrategy.BootToAbsolutePosition
        )
        self.steer.configIntegratedSensorAbsoluteRange(
            AbsoluteSensorRange.Signed_PlusMinus180
        )
        self.steer.setSensorPhase(False)
        self.steer.setNeutralMode(NeutralMode.Brake)

        # configure drive motor
        self.drive.configAllSettings(config.cfgDriveMotor)
        self.drive.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 250)
        self.drive.setInverted(TalonFXInvertType.Clockwise)
        self.drive.configClosedloopRamp(0.25)

        # self.current_states = None
        # self.current_speeds = None

    def setState(self, state: SwerveModuleState):
        self.requestedState = FROGSwerveModuleState(state.speed, state.angle)

        if self.enabled:
            #
            # using built-in optimize method instead of our custom one from last year
            self.requestedState.optimize(self.getCurrentRotation())
            self.steer.set(
                POSITION_MODE,
                self.requestedState.angle.radians() * config.CANCODER_TICKS_PER_RADIAN,
            )

            self.drive.set(
                VELOCITY_MODE,
                self.drive_unit.speedToVelocity(self.requestedState.speed),
            )
        else:
            self.drive.set(0)

    # def initSendable(self, builder) -> None:

    #     super().initSendable(builder)

    def periodic(self) -> None:
        SmartDashboard.putNumber(
            "{}_steerAngle".format(self.name), self.getCurrentRotation().degrees()
        )
        SmartDashboard.putNumber(
            "{}_driveSpeed".format(self.name), self.getCurrentSpeed()
        )


class SwerveChassis:
    moduleFrontLeft: SwerveModule
    moduleFrontRight: SwerveModule
    moduleBackLeft: SwerveModule
    swerveBackRight: SwerveModule

    def __init__(self):
        self.enabled = False
        # TODO: Adjust for field placement
        self.logger = Logger("SwerveChassis")
        self.starting_pose = Pose2d(0, 0, 0)
        self.visionPoseEstimator = FROGPhotonVision(
            "OV5647",
            Transform3d(
                Translation3d(inchesToMeters(13.75), 0, inchesToMeters(5.5)),
                Rotation3d(0, 0, math.pi),
            ),
        )
        self.limelightPoseEstimator = FROGLimeLightVision()
        self.center = Translation2d(0, 0)


    def setup(self):

        self.modules = (
            self.moduleFrontLeft,
            self.moduleFrontRight,
            self.moduleBackLeft,
            self.swerveBackRight,
        )

        self.moduleStates = (
            FROGSwerveModuleState(),
            FROGSwerveModuleState(),
            FROGSwerveModuleState(),
            FROGSwerveModuleState(),
        )
        self.current_speeds = ChassisSpeeds(0, 0, 0)

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

        self.trajectoryConfig = TrajectoryConfig(
            MAX_TRAJECTORY_SPEED, MAX_TRAJECTORY_ACCEL
        )
        self.trajectoryConfig.setKinematics(self.kinematics)

        self.gyro = FROGGyro()
        self.gyro.resetGyro()
        # self.field =
        # self.logger =
        #
        # initialize the drivetrain with zero movement
        self.chassisSpeeds = ChassisSpeeds(0, 0, 0)
        # TODO: set values for pose depending on starting field position
        self.odometry = SwerveDrive4Odometry(
            self.kinematics,
            self.gyro.getRotation2d(),
            tuple(
                [SwerveModulePosition(0, x.getCurrentRotation()) for x in self.modules]
            ),
            self.starting_pose,
        )
        self.estimator = SwerveDrive4PoseEstimator(
            self.kinematics,
            self.gyro.getRotation2d(),
            tuple(
                [SwerveModulePosition(0, x.getCurrentRotation()) for x in self.modules]
            ),
            self.starting_pose,
        )
        # TODO: Adjust the stdDevs
        self.estimator.setVisionMeasurementStdDevs((0.1, 0.1, 0.1))
        self.field = Field2d()

    def disable(self):
        self.enabled = False
        for module in self.modules:
            module.disable()

    def enable(self):
        self.enabled = True
        for module in self.modules:
            module.enable()

    def disableAuto(self):
        self.autoDrive = False

    def enableAuto(self):
        self.autoDrive = True

    def setModuleStates(self, states):
        self.moduleStates = states

    def setPosition(self, pose: Pose2d):
        self.starting_pose = pose
        self.estimator.resetPosition(
            self.gyro.getRotation2d(),
            tuple(self.getModulePositions()),
            self.starting_pose,
        )
        self.odometry.resetPosition(
            self.gyro.getRotation2d(),
            self.starting_pose,
            *self.getModulePositions()
        )

    def execute(self):
        if self.enabled:
            # if self.autoDrive:
            #     #apply holonomic states
            #     pass
            # else:
            self.setStatesFromSpeeds()#apply chassis Speeds

            for module, state in zip(self.modules, self.moduleStates):
                module.setState(state)
        self.periodic()

    def getSimpleTrajectory(self):
        self.startTrajectoryPose = self.estimator.getEstimatedPosition()
        self.endTrajectoryPose = self.startTrajectoryPose + Transform2d(
            feetToMeters(6), feetToMeters(3), 0
        )
        self.logger.info(
            "Auto Drive - Start Pose: %s\n End Pose:%s",
            self.startTrajectoryPose,
            self.endTrajectoryPose,
        )
        return TrajectoryGenerator.generateTrajectory(
            self.startTrajectoryPose,  # Starting position
            [],  # Pass through these points
            self.endTrajectoryPose,  # Ending position
            self.trajectoryConfig,
        )

    # def getSwerveCommand(self):
    #     self.xController = PIDController(1, 0, 0)
    #     self.yController = PIDController(1, 0, 0)
    #     self.angleController = ProfiledPIDControllerRadians(
    #         1, 0, 0, TrapezoidProfileRadians.Constraints(math.pi, math.pi)
    #     )
    #     self.angleController.enableContinuousInput(-1 * math.pi, math.pi)
    #     self.holonomicController = HolonomicDriveController(
    #         self.xController, self.yController, self.angleController
    #     )
    #     return Swerve4ControllerCommand(
    #         self.getSimpleTrajectory(),
    #         self.estimator.getEstimatedPosition,  # CALLABLE getPose
    #         self.kinematics,
    #         self.holonomicController,
    #         self.setModuleStates,
    #         [self],
    #     )

    def getModuleStates(self):
        return [module.getCurrentState() for module in self.modules]

    def getModulePositions(self):
        return [module.getCurrentPosition() for module in self.modules]

    def setStatesFromSpeeds(self):
        states = self.kinematics.toSwerveModuleStates(self.chassisSpeeds, self.center)
        states = self.kinematics.desaturateWheelSpeeds(
            states, config.MAX_METERS_PER_SEC
        )
        self.moduleStates = states

    def fieldOrientedDrive(self, vX: float, vY: float, vT: float, throttle=1.0):
        xSpeed = vX * config.MAX_METERS_PER_SEC * throttle
        ySpeed = vY * config.MAX_METERS_PER_SEC * throttle
        rotSpeed = vT * config.MAX_CHASSIS_RADIANS_SEC * throttle
        self.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, rotSpeed, self.gyro.getRotation2d()
        )

    def autoDrive(self, chassisSpeeds: ChassisSpeeds) -> None:
        self.chassisSpeeds = chassisSpeeds

    def periodic(self) -> None:
        self.estimatorPose = self.estimator.update(
            Rotation2d.fromDegrees(self.gyro.getAngleCCW()),
            tuple(self.getModulePositions()),
        )
        # visionPose, visionTime = self.visionPoseEstimator.getEstimatedRobotPose()
        # visionPose, visionTime = self.limelightPoseEstimator.getBotPoseAlliance()
        # if visionPose:
        #     if (
        #         abs(visionPose.x - self.estimatorPose.x) < 1
        #         and abs(visionPose.y - self.estimatorPose.y) < 1
        #     ):
        #         currentPose = self.estimator.getEstimatedPosition()

        #         self.estimator.addVisionMeasurement(visionPose.toPose2d(), visionTime)
        #         adjustedPose = self.estimator.getEstimatedPosition()
        #         self.logger.info(
        #             "Vision Update -- initial Pose: %s\n  vision Pose: %s\n final Pose: %s",
        #             currentPose, visionPose, adjustedPose
        #         )

        self.odometryPose = self.odometry.update(
            Rotation2d.fromDegrees(self.gyro.getYaw()),
            *self.getModulePositions(),
        )

        self.field.setRobotPose(self.odometryPose)

        SmartDashboard.putNumber(
            "Estimator_X_Inches", metersToInches(self.estimatorPose.X())
        )
        SmartDashboard.putNumber(
            "Estimator_Y_Inches", metersToInches(self.estimatorPose.Y())
        )
        SmartDashboard.putNumber(
            "Estimator_T_Degrees", self.estimatorPose.rotation().degrees()
        )
        SmartDashboard.putNumber("Gyro_Angle_CCW", self.gyro.getYaw())
        SmartDashboard.putNumber(
            "Odometry_X_Inches", metersToInches(self.odometryPose.X())
        )
        SmartDashboard.putNumber(
            "Odometry_Y_Inches", metersToInches(self.odometryPose.Y())
        )
        SmartDashboard.putNumber(
            "Odometry_T_Degrees", self.odometryPose.rotation().degrees()
        )

import math
from wpimath.geometry import Translation2d, Translation3d, Transform3d, Rotation3d
from wpimath.units import feetToMeters, metersToInches, degreesToRadians, inchesToMeters
from wpimath.controller import PIDController, ProfiledPIDControllerRadians
from wpimath.trajectory import TrapezoidProfileRadians
from ctre import (
    TalonFXConfiguration,
    SensorInitializationStrategy,
    BaseTalonPIDSetConfiguration,
    FeedbackDevice,
    CANCoderConfiguration,
    AbsoluteSensorRange,
)

TRACK_WIDTH = 20 / 12
WHEELBASE = 20 / 12

# SwerveDriveSpecialties modules have the following max speeds (in ft/sec):
# L1 - 13.5, L2 - 16.3, L3 - 18
MAX_FEET_PER_SEC = 16
MIN_FEET_PER_SEC = 0.55
MAX_METERS_PER_SEC = feetToMeters(MAX_FEET_PER_SEC)
MIN_METERS_PER_SEC = feetToMeters(MIN_FEET_PER_SEC)

MAX_CHASSIS_REV_SEC = 2
MAX_CHASSIS_RADIANS_SEC = MAX_CHASSIS_REV_SEC * math.tau

MODULE_DRIVE_GEARING = [(14.0 / 50.0), (28.0 / 16.0), (15.0 / 45.0)]  # Mk4 L3
MODULE_WHEEL_DIAMETER = 0.1000125  # 3 15/16 inches in meters

MAX_TRAJECTORY_SPEED = feetToMeters(4)
MAX_TRAJECTORY_ACCEL = feetToMeters(4)

MODULE_FRONT_LEFT = {
    "name": "FrontLeft",
    "drive_motor_id": 11,
    "steer_motor_id": 21,
    "steer_sensor_id": 31,
    "steer_sensor_offset": -5.185547,  #-4.13085938,  
    "location": Translation2d.fromFeet(WHEELBASE / 2, TRACK_WIDTH / 2),
}

MODULE_FRONT_RIGHT = {
    "name": "FrontRight",
    "drive_motor_id": 12,
    "steer_motor_id": 22,
    "steer_sensor_id": 32,
    "steer_sensor_offset": -150.820313, #-150.292969,  
    "location": Translation2d.fromFeet(WHEELBASE / 2, -TRACK_WIDTH / 2),
}
MODULE_BACK_LEFT = {
    "name": "BackLeft",
    "drive_motor_id": 13,
    "steer_motor_id": 23,
    "steer_sensor_id": 33,
    "steer_sensor_offset": -179.648438,  #-179.736328,  
    "location": Translation2d.fromFeet(-WHEELBASE / 2, TRACK_WIDTH / 2),
}
MODULE_BACK_RIGHT = {
    "name": "BackRight",
    "drive_motor_id": 14,
    "steer_motor_id": 24,
    "steer_sensor_id": 34,
    "steer_sensor_offset": 45.9667969,  ##47.2851563,  
    "location": Translation2d.fromFeet(-WHEELBASE / 2, -TRACK_WIDTH / 2),
}

#
# Holonomic PID values
#
holonomicTranslationPIDController = PIDController(1.5, 0, 0)
holonomicAnglePIDController = ProfiledPIDControllerRadians(
    2.5, 0, 0, TrapezoidProfileRadians.Constraints(math.pi, math.pi)
)
ppTranslationPIDController = PIDController(1,0,0)
ppRotationPIDController = PIDController(2,0,0)

#
# **Swerve Module Drive Motor Config
#
cfgDriveMotor = TalonFXConfiguration()
cfgDriveMotor.initializationStrategy = SensorInitializationStrategy.BootToZero
cfgDriveMotor.primaryPID = BaseTalonPIDSetConfiguration(FeedbackDevice.IntegratedSensor)
cfgDriveMotor.slot0.kP = 0.0  # TODO: Confirm PID
cfgDriveMotor.slot0.kI = 0.0
cfgDriveMotor.slot0.kD = 0.0
cfgDriveMotor.slot0.kF = 0.05 #0.04664  # 0.058

#
# **Swerve Module Steer Motor Config
#
cfgSteerMotor = TalonFXConfiguration()
cfgSteerMotor.primaryPID = BaseTalonPIDSetConfiguration(FeedbackDevice.RemoteSensor0)
cfgSteerMotor.slot0.kP = 1.2  # TODO: Confirm PID
cfgSteerMotor.slot0.kI = 0.0001
cfgSteerMotor.slot0.kD = 0.0
cfgSteerMotor.slot0.kF = 0.0
""" TODO: test if we can remove this.  The current value amounts to
  about .44 degrees allowed error. """
cfgSteerMotor.slot0.allowableClosedloopError = 5

#
# **Swerve Module CanCoder Config
#
cfgSteerEncoder = CANCoderConfiguration()
cfgSteerEncoder.sensorDirection = False  # CCW spin of magnet is positive
cfgSteerEncoder.initializationStrategy = (
    SensorInitializationStrategy.BootToAbsolutePosition
)
cfgSteerEncoder.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180

#
# Sweve Drive Constants
#
FIELD_ORIENTED = 0
ROBOT_ORIENTED = 1

FALCON_TICKS_PER_ROTATION = 2048
FALCON_MAX_RPM = 6380
CANCODER_TICKS_PER_ROTATION = 4096
CANCODER_TICKS_PER_DEGREE = CANCODER_TICKS_PER_ROTATION / 360
CANCODER_TICKS_PER_RADIAN = CANCODER_TICKS_PER_ROTATION / math.tau

#Boom motor config
BOOM_MOTOR_ID = 41
cfgBoomMotor = TalonFXConfiguration()
cfgBoomMotor.primaryPID = BaseTalonPIDSetConfiguration(FeedbackDevice.IntegratedSensor)
cfgBoomMotor.slot0.kP = 0.128#0.24 #0.16
cfgBoomMotor.slot0.kI = 0.0
cfgBoomMotor.slot0.kD = 0.0
cfgBoomMotor.slot0.kF = 0.05115
cfgBoomMotor.clearPositionOnLimitR = True
cfgBoomMotor.motionAcceleration = 40000
cfgBoomMotor.motionCruiseVelocity = 20000
cfgBoomMotor.motionCurveStrength = 3
cfgBoomMotor.slot0.allowableClosedloopError = 0
cfgBoomMotor.neutralDeadband = 0.02

#Stick motor config
STICK_MOTOR_ID = 42
cfgStickMotor = TalonFXConfiguration()
cfgStickMotor.primaryPID = BaseTalonPIDSetConfiguration(FeedbackDevice.IntegratedSensor)
cfgStickMotor.slot0.kP = 0.128#0.30 #0.096
cfgStickMotor.slot0.kI = 0.0
cfgStickMotor.slot0.kD = 0.0
cfgStickMotor.slot0.kF = 0.05115
cfgStickMotor.clearPositionOnLimitR = True
cfgStickMotor.motionAcceleration = 30000
cfgStickMotor.motionCruiseVelocity = 20000
cfgStickMotor.motionCurveStrength = 2
cfgStickMotor.slot0.allowableClosedloopError = 0
cfgStickMotor.neutralDeadband = 0.02


#MAX STICK 256000
BOOM_GRID_UPPER = 149505 #145000 #150400
STICK_GRID_UPPER = 254000 #320500
BOOM_GRID_MID = 4500
STICK_GRID_MID = 208400 #262000
BOOM_GRID_LOW = 95300
STICK_GRID_LOW = 400
BOOM_HOME = -20000
STICK_HOME = -20000
BOOM_FLOOR_PICKUP = 152639 #150000 #148000 #153000 #154000 max
STICK_FLOOR_PICKUP = 400
BOOM_FLOOR_MANIPULATE = 106000
STICK_FLOOR_MANIPULATE = 400
BOOM_SHELF = 4500
STICK_SHELF = 208400 #262000

#
#  MaxBotix MB1043-000 config
#
ULTRASONIC = {
    "port": 0,
    "mvPerInch": metersToInches((4.88 / 5) * 1000 )
}


#Vision Camera config
PHOTONVISION_CAMERA_NAME = "OV5647"
# TODO: check these values
PHOTONVISION_CAMERA_POSE = Transform3d(
    Translation3d(inchesToMeters(-8.75), 0, inchesToMeters(50.375)),
    Rotation3d(0, degreesToRadians(23), degreesToRadians(-178))
)

LIMELIGHT_GRABBER = 'limelight'
LIMELIGHT_UPPER = 'limelight-at'


pass

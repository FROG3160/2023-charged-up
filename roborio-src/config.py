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
MAX_FEET_PER_SEC = 8
MIN_FEET_PER_SEC = 0.5
MAX_METERS_PER_SEC = feetToMeters(MAX_FEET_PER_SEC)
MIN_METERS_PER_SEC = feetToMeters(MIN_FEET_PER_SEC)

MAX_CHASSIS_REV_SEC = 1
MAX_CHASSIS_RADIANS_SEC = MAX_CHASSIS_REV_SEC * math.tau

MODULE_DRIVE_GEARING = [(14.0 / 50.0), (28.0 / 16.0), (15.0 / 45.0)]  # Mk4 L3
MODULE_WHEEL_DIAMETER = 0.1000125  # 3 15/16 inches in meters

MAX_TRAJECTORY_SPEED = feetToMeters(2)
MAX_TRAJECTORY_ACCEL = feetToMeters(2)

MODULE_FRONT_LEFT = {
    "name": "FrontLeft",
    "drive_motor_id": 11,
    "steer_motor_id": 21,
    "steer_sensor_id": 31,
    "steer_sensor_offset": -4.13085938,  # 176.484375-180 ,
    "location": Translation2d.fromFeet(WHEELBASE / 2, TRACK_WIDTH / 2),
}

MODULE_FRONT_RIGHT = {
    "name": "FrontRight",
    "drive_motor_id": 12,
    "steer_motor_id": 22,
    "steer_sensor_id": 32,
    "steer_sensor_offset": -150.292969,  # 29.0917969 - 180,
    "location": Translation2d.fromFeet(WHEELBASE / 2, -TRACK_WIDTH / 2),
}
MODULE_BACK_LEFT = {
    "name": "BackLeft",
    "drive_motor_id": 13,
    "steer_motor_id": 23,
    "steer_sensor_id": 33,
    "steer_sensor_offset": -179.736328,  # 0,
    "location": Translation2d.fromFeet(-WHEELBASE / 2, TRACK_WIDTH / 2),
}
MODULE_BACK_RIGHT = {
    "name": "BackRight",
    "drive_motor_id": 14,
    "steer_motor_id": 24,
    "steer_sensor_id": 34,
    "steer_sensor_offset": 47.2851563,  # -131.484375+180,
    "location": Translation2d.fromFeet(-WHEELBASE / 2, -TRACK_WIDTH / 2),
}

#
# Holonomic PID values
#
holonomicTranslationPIDController = PIDController(1.5, 0, 0)
holonomicAnglePIDController = ProfiledPIDControllerRadians(
    2.5, 0, 0, TrapezoidProfileRadians.Constraints(math.pi, math.pi)
)
ppTranslationPIDController = PIDController(4,0,0)
ppRotationPIDController = PIDController(8,0,0)

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
cfgBoomMotor = TalonFXConfiguration()
cfgBoomMotor.primaryPID = BaseTalonPIDSetConfiguration(FeedbackDevice.RemoteSensor0)
cfgBoomMotor.slot0.kP = 0.16
cfgBoomMotor.slot0.kI = 0.0
cfgBoomMotor.slot0.kD = 0.0
cfgBoomMotor.slot0.kF = 0.0
cfgBoomMotor.clearPositionOnLimitR = True
cfgBoomMotor.motionAcceleration = 7500
cfgBoomMotor.motionCruiseVelocity = 15000

#Stick motor config
cfgStickMotor = TalonFXConfiguration()
cfgStickMotor.primaryPID = BaseTalonPIDSetConfiguration(FeedbackDevice.RemoteSensor0)
cfgStickMotor.slot0.kP = 0.096
cfgStickMotor.slot0.kI = 0.0
cfgStickMotor.slot0.kD = 0.0
cfgStickMotor.slot0.kF = 0.0
cfgStickMotor.clearPositionOnLimitR = True
cfgStickMotor.motionAcceleration = 10000
cfgStickMotor.motionCruiseVelocity = 20000



#
#  MaxBotix MB1043-000 config
#
ULTRASONIC = {
    "port": 0,
    "mvPerInch": metersToInches((4.88 / 5) * 1000 )
}

# >>> metersToInches((4.88/5) * 1000)
# 38425.1968503937
# >>> (4.88/5)
# 0.976
# >>> mvPerMM = (4.88/5)
# >>> mvPerMM
# 0.976
# >>> mvPerM = mvPerMM /1000
# >>> mvPerM
# 0.000976
# >>> mvPerInch = metersToInches(mvPerM) 
# >>> mvPerInch 
# 0.0384251968503937
# >>> 4.88 * mvPerInch
# 0.18751496062992126
# >>> VPerInch = mvPerInch / 1000
# >>> .00488 * VPerInch
# 1.8751496062992126e-07
# >>> VPerInch
# 3.8425196850393705e-05
# >>>

#Vision Camera config
PHOTONVISION_CAMERA_NAME = "OV5647"
PHOTONVISION_CAMERA_POSE = Transform3d(
                Translation3d(inchesToMeters(-9), 0, inchesToMeters(49.8)),
                Rotation3d(0, degreesToRadians(-27), degreesToRadians(180))
)
import math
from wpimath.geometry import Translation2d, Translation3d, Transform3d, Rotation3d, Pose2d, Rotation2d
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
VOLTAGE_COMPENSATION = 10.5

MAX_CHASSIS_REV_SEC = 2
MAX_CHASSIS_RADIANS_SEC = MAX_CHASSIS_REV_SEC * math.tau

MODULE_DRIVE_GEARING = [(14.0 / 50.0), (28.0 / 16.0), (15.0 / 45.0)]  # Mk4 L3
MODULE_WHEEL_DIAMETER = 0.1000125  # 3 15/16 inches in meters

#
# **Swerve Module Drive Motor Config
#
frDriveMotorPID = TalonFXConfiguration()
frDriveMotorPID.initializationStrategy = SensorInitializationStrategy.BootToZero
frDriveMotorPID.primaryPID = BaseTalonPIDSetConfiguration(FeedbackDevice.IntegratedSensor)
frDriveMotorPID.slot0.kP = 0.0  # TODO: Confirm PID
frDriveMotorPID.slot0.kI = 0.0
frDriveMotorPID.slot0.kD = 0.0
frDriveMotorPID.slot0.kF = 0.04548 #0.04664  # 0.058

flDriveMotorPID = TalonFXConfiguration()
flDriveMotorPID.initializationStrategy = SensorInitializationStrategy.BootToZero
flDriveMotorPID.primaryPID = BaseTalonPIDSetConfiguration(FeedbackDevice.IntegratedSensor)
flDriveMotorPID.slot0.kP = 0.0  # TODO: Confirm PID
flDriveMotorPID.slot0.kI = 0.0
flDriveMotorPID.slot0.kD = 0.0
flDriveMotorPID.slot0.kF = 0.0455 #0.04664  # 0.058

blDriveMotorPID = TalonFXConfiguration()
blDriveMotorPID.initializationStrategy = SensorInitializationStrategy.BootToZero
blDriveMotorPID.primaryPID = BaseTalonPIDSetConfiguration(FeedbackDevice.IntegratedSensor)
blDriveMotorPID.slot0.kP = 0.0  # TODO: Confirm PID
blDriveMotorPID.slot0.kI = 0.0
blDriveMotorPID.slot0.kD = 0.0
blDriveMotorPID.slot0.kF = 0.04489 #0.0447 #0.04664  # 0.058

brDriveMotorPID = TalonFXConfiguration()
brDriveMotorPID.initializationStrategy = SensorInitializationStrategy.BootToZero
brDriveMotorPID.primaryPID = BaseTalonPIDSetConfiguration(FeedbackDevice.IntegratedSensor)
brDriveMotorPID.slot0.kP = 0.0  # TODO: Confirm PID
brDriveMotorPID.slot0.kI = 0.0
brDriveMotorPID.slot0.kD = 0.0
brDriveMotorPID.slot0.kF = 0.044752, #0.04455 #0.04664  # 0.058

MODULE_FRONT_LEFT = {
    "name": "FrontLeft",
    "drive_motor_id": 11,
    "steer_motor_id": 21,
    "steer_sensor_id": 31,
    "steer_sensor_offset": -5.625, #-5.186, #-5.537,# -5.185547,  #-4.13085938,  
    "location": Translation2d.fromFeet(WHEELBASE / 2, TRACK_WIDTH / 2),
    "driveMotorPID": flDriveMotorPID
}

MODULE_FRONT_RIGHT = {
    "name": "FrontRight",
    "drive_motor_id": 12,
    "steer_motor_id": 22,
    "steer_sensor_id": 32,
    "steer_sensor_offset": -150.293, #-150.908, #-150.46875, #-150.381,#-150.820313, #-150.292969,  
    "location": Translation2d.fromFeet(WHEELBASE / 2, -TRACK_WIDTH / 2),
    "driveMotorPID": frDriveMotorPID
}
MODULE_BACK_LEFT = {
    "name": "BackLeft",
    "drive_motor_id": 13,
    "steer_motor_id": 23,
    "steer_sensor_id": 33,
    "steer_sensor_offset": 179.825, #-179.561, #179.912109, #179.736,#-179.648438,  #-179.736328,  
    "location": Translation2d.fromFeet(-WHEELBASE / 2, TRACK_WIDTH / 2),
    "driveMotorPID": blDriveMotorPID
}
MODULE_BACK_RIGHT = {
    "name": "BackRight",
    "drive_motor_id": 14,
    "steer_motor_id": 24,
    "steer_sensor_id": 34,
    "steer_sensor_offset": 46.230, #45.703, #44.91211, #45,#45.9667969,  ##47.2851563,  
    "location": Translation2d.fromFeet(-WHEELBASE / 2, -TRACK_WIDTH / 2),
    "driveMotorPID": brDriveMotorPID
}

#
# Holonomic PID values
#
# holonomicTranslationPIDController = PIDController(1.5, 0, 0)
# holonomicAnglePIDController = ProfiledPIDControllerRadians(
#     2.5, 0, 0, TrapezoidProfileRadians.Constraints(math.pi, math.pi)
# )
MAX_TRAJECTORY_SPEED = 3
MAX_TRAJECTORY_ACCEL = 3
ppXPIDController = PIDController(1,0,0)
ppYPIDController = PIDController(1,0,0)
ppRotationPIDController = PIDController(1,0,0)
ppTolerance = Pose2d(0.03, 0.03, Rotation2d.fromDegrees(2))
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

cfgProfiledMaxVelocity = math.pi*8
cfgProfiledMaxAccel = math.pi*4

cfgProfiledP = 0.4
cfgProfiledI = 0.0
cfgProfiledD = 0.0

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
BOOM_GRID_UPPER = 118500 #150000 #149505 #145000 #150400
STICK_GRID_UPPER = 252000 #250000 #320500
BOOM_GRID_MID = 8700 #4500
STICK_GRID_MID = 204600 #207000 #208400 #262000
BOOM_GRID_LOW = 82600 #95300
STICK_GRID_LOW = 0 #400
BOOM_HOME = -10000  # negative value so we always get to 0 and the limit switch
STICK_HOME = -10000
BOOM_FLOOR_PICKUP = 125000 #155000 #152639 #150000 #148000 #153000 #154000 max
STICK_FLOOR_PICKUP = 0 #400
BOOM_FLOOR_MANIPULATE = 82600 #99000 #106000
STICK_FLOOR_MANIPULATE = 0 #400
BOOM_SHELF = 8700 #4500
STICK_SHELF = 204600 #207000 #208400 #262000

boomPositions = {
    'upper': BOOM_GRID_UPPER,
    'shelf': BOOM_SHELF,
    'manipulate': BOOM_FLOOR_MANIPULATE,
    'floor': BOOM_FLOOR_PICKUP,
    'home': BOOM_HOME
}

stickPositions = {
    'upper': STICK_GRID_UPPER,
    'shelf': STICK_SHELF,
    'manipulate': STICK_FLOOR_MANIPULATE,
    'floor': STICK_FLOOR_PICKUP,
    'home': STICK_HOME
}
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

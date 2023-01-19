from wpimath.geometry import Translation2d
from ctre import (
    TalonFXConfiguration,
    SensorInitializationStrategy,
    BaseTalonPIDSetConfiguration,
    FeedbackDevice,
    RemoteSensorSource,
    CANCoderConfiguration,
    AbsoluteSensorRange
)

TRACK_WIDTH = 27.75 / 12
WHEELBASE = 21.75 / 12
# these are the values for the 2023 frame when it's built
#TRACK_WIDTH = 20 / 12 # track width in feet
#WHEELBASE = 20 / 12  # wheelbase in feet

MODULE_DRIVE_GEARING = [(14.0 / 50.0), (28.0 / 16.0), (15.0 / 45.0)] #Mk4 L3
MODULE_WHEEL_DIAMETER = 0.1000125  # 3 15/16 inches in meters

MODULE_FRONT_LEFT = {
    'drive_motor_id': 11,
    'steer_motor_id': 21,
    'steer_sensor_id': 31,
    'steer_sensor_offset': 13.008,
    'location': Translation2d.fromFeet(
        WHEELBASE/2,
        TRACK_WIDTH/2
    )
}

MODULE_FRONT_RIGHT = {
    'drive_motor_id': 12,
    'steer_motor_id': 22,
    'steer_sensor_id': 32,
    'steer_sensor_offset': 171.914,
    'location': Translation2d.fromFeet(
        WHEELBASE/2,
        -TRACK_WIDTH/2
    )
}
MODULE_BACK_LEFT = {
    'drive_motor_id': 13,
    'steer_motor_id': 23,
    'steer_sensor_id': 33,
    'steer_sensor_offset': 22.764,
    'location': Translation2d.fromFeet(
        -WHEELBASE/2,
        TRACK_WIDTH/2
    )
}
MODULE_BACK_RIGHT = {
    'drive_motor_id': 14,
    'steer_motor_id': 24,
    'steer_sensor_id': 34,
    'steer_sensor_offset': -43.242,
    'location': Translation2d.fromFeet(
        -WHEELBASE/2,
        -TRACK_WIDTH/2
    )
}


cfgDriveMotor = TalonFXConfiguration()
cfgDriveMotor.initializationStrategy = SensorInitializationStrategy.BootToZero
cfgDriveMotor.primaryPID = BaseTalonPIDSetConfiguration(
    FeedbackDevice.IntegratedSensor
)
cfgDriveMotor.slot0.kP = 0.0  # TODO: Confirm PID
cfgDriveMotor.slot0.kI = 0.0
cfgDriveMotor.slot0.kD = 0.0
cfgDriveMotor.slot0.kF = 0.04664  # 0.058

cfgSteerMotor = TalonFXConfiguration()
cfgSteerMotor.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder
cfgSteerMotor.primaryPID = BaseTalonPIDSetConfiguration(
    FeedbackDevice.RemoteSensor0
)
cfgSteerMotor.slot0.kP = 1.2  # TODO: Confirm PID
cfgSteerMotor.slot0.kI = 0.0001
cfgSteerMotor.slot0.kD = 0.0
cfgSteerMotor.slot0.kF = 0.0
""" TODO: test if we can remove this.  The current value amounts to
  about .44 degrees allowed error. """
cfgSteerMotor.slot0.allowableClosedloopError = 5

cfgSteerEncoder = CANCoderConfiguration()
cfgSteerEncoder.sensorDirection = False  # CCW spin of magnet is positive
cfgSteerEncoder.initializationStrategy = (
    SensorInitializationStrategy.BootToAbsolutePosition
)
cfgSteerEncoder.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180

FIELD_ORIENTED = 0
ROBOT_ORIENTED = 1


FALCON_TICKS_PER_ROTATION = 2048
FALCON_MAX_RPM = 6380
CANCODER_TICKS_PER_ROTATION = 4096
CANCODER_TICKS_PER_DEGREE = CANCODER_TICKS_PER_ROTATION / 360
CANCODER_TICKS_PER_RADIAN = CANCODER_TICKS_PER_ROTATION / math.tau
import wpilib
from navx import AHRS
from ctre import CANifier
from utils.utils import Buffer
from rev import ColorSensorV3
import math
from wpimath.geometry import Rotation2d
from magicbot import feedback
from wpilib import SmartDashboard

BUFFERLEN = 50

SENSORUNITS_IN_INCHES = 0.0394
SENSORUNITS_IN_FEET = 0.00328
SENSORUNITS_IN_METERS = 0.001


class FROGGyro:
    
    def __init__(self):
        # TODO Make sure if we need this.
        self.gyro = AHRS.create_spi()
        self.starting_angle = 0.0
        self.offset = 0
        # self.field_heading = 360-242
        # self.gyro.reset()
        self.gyro.setAngleAdjustment(self.offset)

    def getYawCCW(self):
        # returns gyro heading +180 to -180 degrees
        # and inverts it to change from bearing to
        # cartesian angles with CCW positive.
        # return -self.gyro.getYaw()
        return -self.gyro.getAngle()
    
    def getRoll(self):
        return self.gyro.getRoll()
    
    def getPitch(self):
        return self.gyro.getPitch()

    def setOffset(self, offset):
        self.offset = offset

    def getDegreesPerSecCCW(self):
        return -self.gyro.getRate()
    
    def getRadiansPerSecCCW(self):
        return math.radians(self.getDegreesPerSecCCW())

    def getRotation2d(self):
        return Rotation2d.fromDegrees(self.getAngleCCW())

    def getOffsetYaw(self):
        chassisYaw = self.getYawCCW()
        fieldYaw = Rotation2d.fromDegrees(chassisYaw + self.starting_angle)
        # Adding an angle to the current reading can cause the result
        # to be outside -180 to +180 degrees, so we utilize the atan2
        # function to give us the angle back inside the limits.
        return math.degrees(math.atan2(fieldYaw.sin(), fieldYaw.cos()))

    def resetGyro(self):
        # sets yaw reading to 0
        self.setAngleAdjustment(self.starting_angle)
        self.gyro.reset()

    def execute(self):
        wpilib.SmartDashboard.putNumber('Gyro_Pitch', self.gyro.getPitch())
        wpilib.SmartDashboard.putNumber('Gyro_YawCCW', -self.gyro.getYaw())
        wpilib.SmartDashboard.putNumber('Gyro_Roll', self.gyro.getRoll())
        wpilib.SmartDashboard.putNumber('Gyro_AngleCCW', -self.gyro.getAngle())

    def getAngle(self):
        return self.gyro.getAngle()

    def getAngleCCW(self):
        return -self.gyro.getAngle()

    def getAngleConstrained(self):
        angle = self.getAngle()
        return math.degrees(math.atan2(math.sin(angle), math.cos(angle)))

    def setAngleAdjustment(self, angle):
        self.gyro.setAngleAdjustment(angle)

    # TODO: Confirm this is incorrect.  getYaw returns CW positive?
    def getRadiansCCW(self):
        return math.radians(self.gyro.getYaw())

    def getAngleAdjustment(self):
        return self.gyro.getAngleAdjustment()


class FROGdar:
    canifier: CANifier

    def __init__(self):
        self.enabled = False
        self.targetRange = None
        self.rangeBuffer = Buffer(BUFFERLEN)

    def disable(self):
        self.enabled = False

    def enable(self):
        # clear range data to get fresh information
        self.rangeBuffer.clear()
        self.enabled = True

    def isValidData(self):
        return self.rangeBuffer._isValidData() and self.targetRange is not None

    def getSensorData(self):
        errorcode, (val1, val2) = self.canifier.getPWMInput(
            CANifier.PWMChannel.PWMChannel0
        )
        return val1

    def getBufferedSensorData(self):
        return self.targetRange

    def getDistance(self):
        if self.isValidData():
            return self.getBufferedSensorData() * SENSORUNITS_IN_INCHES

    def getDistanceFeet(self):
        if self.isValidData():
            return self.getBufferedSensorData() * SENSORUNITS_IN_FEET

    def getDistanceMeters(self):
        if self.isValidData():
            return self.getBufferedSensorData() * SENSORUNITS_IN_METERS

    def execute(self):
        if self.enabled:
            # stream data into our counter
            self.rangeBuffer.append(self.getSensorData())
            if self.rangeBuffer._getBufferLength() > 0:
                self.targetRange = self.rangeBuffer.average()
            else:
                self.targetRange = None
        else:
            self.rangeBuffer.clear()
            self.targetRange = None


class FROGColor:
    def __init__(self):
        self.enabled = False
        self.colorSensor = ColorSensorV3(wpilib.I2C.Port.kMXP)

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False

    def getRed(self):
        return self.colorSensor.getColor().red

    def getBlue(self):
        return self.colorSensor.getColor().blue
    
    def getGreen(self):
        return self.colorSensor.getColor().green

    @feedback
    def getProximity(self):
        return self.colorSensor.getProximity()
    
    def isCube(self):
        color = self.colorSensor.getColor()
        return color.blue > color.red and color.blue > color.green

    def execute(self):
        pass


class FROGsonic(wpilib.AnalogInput):



    def __init__(self, port: int, voltsPerInch: float):
        super().__init__(port)
        self.voltsPerInch = voltsPerInch


    def execute(self):
        pass

    def __call__(self):
        self.getInches()

    def getInches(self):
        self.volts = self.getVoltage()
        return self.volts / self.voltsPerInch
        #return (self.USVolt * self.mm / (self.mv / 1000)) * 0.039

from ctre import WPI_TalonSRX
from wpilib import PneumaticsModuleType, Solenoid
from components.sensors import FROGsonic, FROGColor
from components.vision import FROGLimeLightVision
from logging import Logger

# installed module type
# original robot used a new REV Power distribution hub
# PCMTYPE = PneumaticsModuleType.REVPH
PCMTYPE = PneumaticsModuleType.CTREPCM

class FROGGrabber:

    sensor: FROGColor
    limelight: FROGLimeLightVision
    logger: Logger

    def __init__(self):
        """Creates our FROGGrabber

        Args:
            motorID (int): Motor ID for the motor
            solenoidID (int): Solenoid ID for the grabber pneumatics.
        """

        self.motor = WPI_TalonSRX(43)
        self.jaws = Solenoid(1, PCMTYPE, 0)
        self.plate = Solenoid(1, PCMTYPE, 1)
        self.grabberOpen = False
        self.speed = 0
        # motors and stuff

    def openJaws(self):
        self.grabberOpen = True
        # pnumatics for this
        # open it to get ready to grab the cone/cube or
        # ungrab the cone/cube use same button to open/turn off

    def isClosed(self):
        return self.jaws.get()

    def closeJaws(self):
        self.grabberOpen = False

    def plateUp(self):
        self.plate.set(True)
    
    def plateDown(self):
        self.plate.set(False)

    def wheelsOn(self, speed):
        self.speed = speed
        # set a button to activate

    def wheelsOff(self):
        self.speed = 0
        # use same button to deactivate

    def getProximity(self):
        return self.sensor.getProximity()
    
    def isObjectClose(self):
        return self.limelight.ta >= 75
    
    def execute(self):
        self.jaws.set(self.grabberOpen)
        self.motor.set(self.speed)
        # if self.intake == 'Intake':
        #     self.logger.info("Checking Intake target")
        #     if self.limelight.hasTarget():
        #         if self.limelight.ta >= 75 and self.getProximity() < 1000:
        #             self.logger.info("Turning intake wheels on")
        #             self.wheelsOn(1)
        #         else:
        #             self.logger.info("Turning intake wheels off")
        #             self.wheelsOff()
        #     if self.getProximity() > 230:
        #         self.logger.info("Closing grabber")
        #         self.close()
        #     if self.getProximity() > 1000:
        #         self.logger.info("Shutting down intake wheels")
        #         self.wheelsOff()
        #         self.intake = False
        # if self.intake == 'Reverse':
        #     if self.getProximity()> 230:
        #         self.open()
        #         self.wheelsOn(-0.5)
        #     else:
        #         self.wheelsOff()
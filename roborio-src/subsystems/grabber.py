from wpilib import Solenoid
from ctre import WPI_TalonSRX
from commands2 import SubsystemBase


class FROGGrabber(SubsystemBase):
    def __init__(self, motorLeftID, motorRightID, solenoidID):
        """Creates our FROGGrabber

        Args:
            motorLeftID (int): Motor ID for the Left Motor
            motorRightID (int): Motor ID for the Right Motor
            solenoidID (int): Solenoid ID for the grabber pneumatics.
        """
        super().__init__(self)
        self.motorL = WPI_TalonSRX(motorLeftID)
        self.motorR = WPI_TalonSRX(motorRightID)
        self.pneumatics = Solenoid(solenoidID)
        # motors and stuff

    def open(self):
        self.pneumatics.set(False)
        # pnumatics for this
        # open it to get ready to grab the cone/cube or
        # ungrab the cone/cube use same button to open/turn off

    def close(self):
        self.pneumatics.set(True)
        # pnumatics for this
        # grab the cone/cube probably set a button to close

    def wheelsOn(self, speed):
        self.motorL.set(speed)
        self.motorR.set(speed)
        # set a button to activate

    def wheelsOff(self):
        self.motorL.disable()
        self.motorR.disable()
        # use same button to deactivate

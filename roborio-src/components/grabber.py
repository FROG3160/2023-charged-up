from ctre import WPI_TalonSRX
from wpilib import PneumaticsModuleType, Solenoid


class FROGGrabber:
    def __init__(self, motorID, solenoidID):
        """Creates our FROGGrabber

        Args:
            motorID (int): Motor ID for the motor
            solenoidID (int): Solenoid ID for the grabber pneumatics.
        """
        self.motor = WPI_TalonSRX(motorID)
        self.pneumatics = Solenoid(PneumaticsModuleType.REVPH, solenoidID)
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
        self.motor.set(speed)
        self.motorR.set(speed)
        # set a button to activate

    def wheelsOff(self):
        self.motor.disable()
        self.motorR.disable()
        # use same button to deactivate

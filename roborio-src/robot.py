import wpilib
from commands2 import TimedCommandRobot

class FROGbot(TimedCommandRobot):

    def robotInit(self):
        """
            This method is run when the robot is first started up and should be used for any
        initialization code.
        """
        pass

    def autonomousInit(self) -> None:
        pass

    def autonomousPeriodic(self) -> None:
        pass

    def teleopInit(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        pass

    def robotPeriodic(self) -> None:
        pass

if __name__ == "__main__":
    wpilib.run(FROGbot)
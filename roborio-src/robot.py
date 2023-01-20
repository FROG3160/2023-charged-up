import wpilib
from commands2 import TimedCommandRobot
from robotcontainer import RobotContainer

class FROGbot(TimedCommandRobot):

    def robotInit(self):
        """
            This method is run when the robot is first started up and should be used for any
        initialization code.
        """
        self.container = RobotContainer()

if __name__ == "__main__":
    wpilib.run(FROGbot)
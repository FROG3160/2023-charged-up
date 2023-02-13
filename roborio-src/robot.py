import wpilib
import config
from magicbot import MagicRobot, tunable, feedback
from components.drivetrain import SwerveChassis, SwerveModule
from components.controllers import FROGStickDriver, FROGXboxDriver, FROGXboxOperator

class FROGbot(MagicRobot):

    # def robotInit(self):
    #     """
    #         This method is run when the robot is first started up and should be used for any
    #     initialization code.
    #     """
    #     self.container = RobotContainer()

    # Any magicbot component needs to be listed here
    # in order for their "execute" method to be run
    # every loop
    swerveChassis: SwerveChassis



    def createObjects(self) -> None:
        self.moduleFrontLeft = SwerveModule(**config.MODULE_FRONT_LEFT)
        self.moduleFrontRight = SwerveModule(**config.MODULE_FRONT_RIGHT)
        self.moduleBackLeft = SwerveModule(**config.MODULE_BACK_LEFT)
        self.swerveBackRight = SwerveModule(**config.MODULE_BACK_RIGHT)

        self.driverController = FROGXboxDriver(0)
        self.operatorController = FROGXboxOperator(1)

        # declare buttons
        self.btnEnableAuto = self.driverController.getRightBumper

    def autonomousInit(self):
        pass

    def teleopInit(self):
        self.swerveChassis.enable()

    def teleopPeriodic(self):
        if self.btnEnableAuto():
            self.swerveChassis.enableAuto()
        else:
            self.swerveChassis.disableAuto()
        self.swerveChassis.fieldOrientedDrive(
            self.driverController.getFieldForward(),
            self.driverController.getFieldLeft(),
            self.driverController.getFieldRotation(),
            self.driverController.getFieldThrottle(),
        )
        

    def testInit(self):
        pass
        
    def testPeriodic(self):
        pass

if __name__ == "__main__":
    wpilib.run(FROGbot)
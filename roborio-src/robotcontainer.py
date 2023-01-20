from subsystems.drivetrain import SwerveChassis
from subsystems.controllers import FROGStick
from commands2 import RunCommand


class RobotContainer:

    def __init__(self):

        #driver interface
        self.driverController = FROGStick(0, 0, 1, 2, 3)

        # Robot Subsystems
        self.swerveChassis = SwerveChassis()
        self.swerveChassis.setDefaultCommand(
            RunCommand(
                lambda: self.swerveChassis.fieldOrientedDrive(
                    self.driverController.getX(),
                    self.driverController.getY(),
                    self.driverController.getFieldRotation()
                ),
                self.swerveChassis
            )

        )
        self.configureButtonBindings()

    def configureButtonBindings(self):
        pass
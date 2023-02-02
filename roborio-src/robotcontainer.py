from subsystems.drivetrain import SwerveChassis
from subsystems.controllers import FROGStick, LOGITECH_EXTREME_AXIS_CONFIG
from commands2.button import JoystickButton

from commands.swerve_commands import cmdFieldOrientedDrive, cmdZeroGyro


class RobotContainer:
    def __init__(self):

        self.driverController = FROGStick(port = 0, **LOGITECH_EXTREME_AXIS_CONFIG)

        # Robot Subsystems
        self.swerveChassis = SwerveChassis()
        self.swerveChassis.setDefaultCommand(
            cmdFieldOrientedDrive(self.driverController, self.swerveChassis)
        )
        self.btnZeroGyro = JoystickButton(self.driverController, 3)
        self.configureButtonBindings()

    def configureButtonBindings(self):
        self.btnZeroGyro.onTrue(
            cmdZeroGyro(self.swerveChassis)
        )

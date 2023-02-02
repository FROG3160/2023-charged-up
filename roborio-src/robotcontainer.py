from subsystems.drivetrain import SwerveChassis
from subsystems.controllers import FROGStick
from commands2.button import JoystickButton

from commands.swerve_commands import cmdFieldOrientedDrive, cmdZeroGyro


class RobotContainer:
    def __init__(self):

        # config for saitek joystick
        # self.driverController = FROGStick(0, 0, 1, 3, 2)
        # config for Logitech Extreme 3D
        self.driverController = FROGStick(0, 0, 1, 2, 3)

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

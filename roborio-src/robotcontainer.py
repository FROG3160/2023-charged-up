from subsystems.drivetrain import SwerveChassis
from subsystems.controllers import FROGStick
from commands2 import RunCommand
from commands2.button import JoystickButton


class RobotContainer:

    def __init__(self):

        # config for saitek joystick
        #self.driverController = FROGStick(0, 0, 1, 3, 2)
        # config for Logitech Extreme 3D
        self.driverController = FROGStick(0, 0, 1, 2, 3)

        # Robot Subsystems
        self.swerveChassis = SwerveChassis()
        self.swerveChassis.setDefaultCommand(
            RunCommand(
                lambda: self.swerveChassis.fieldOrientedDrive(
                    self.driverController.getFieldForward(),
                    self.driverController.getFieldLeft(),
                    self.driverController.getFieldRotation()
                ),
                self.swerveChassis
            )

        )
        self.zeroGyro = JoystickButton(self.driverController, 3);
        self.configureButtonBindings()

    def configureButtonBindings(self):
        self.zeroGyro.onTrue(
                RunCommand(
                    lambda: self.swerveChassis.gyro.resetGyro(),
                    self.swerveChassis
                )
            )   

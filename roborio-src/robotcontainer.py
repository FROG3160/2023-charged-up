
from subsystems.drivetrain import SwerveChassis
from subsystems.controllers import FROGStick, LOGITECH_EXTREME_AXIS_CONFIG, FROGXbox
from subsystems.vision import FROGPhotonVision
from subsystems.arm import Arm
from subsystems.grabber import FROGGrabber
from commands2.button import JoystickButton, CommandXboxController
from commands2 import RunCommand
from wpimath.units import feetToMeters
from wpimath.geometry import Transform2d

from commands.swerve_commands import cmdFieldOrientedDrive, cmdFieldOrientedThrottledDrive, cmdZeroGyro, cmdDriveTrajectory




class RobotContainer:
    def __init__(self):

        # self.driverController = FROGStick(port = 0, **LOGITECH_EXTREME_AXIS_CONFIG)
        self.driverController = FROGXbox(0)
        self.operatorController = CommandXboxController(1)

        # Robot Subsystems
        self.swerveChassis = SwerveChassis()
        self.swerveChassis.setDefaultCommand(
            cmdFieldOrientedThrottledDrive(self.driverController, self.swerveChassis)
        )
        self.arm = Arm(41, 42)
        self.arm.setDefaultCommand(
            RunCommand(
                lambda: self.arm.manual(
                    self.operatorController.getRightY(),
                    self.operatorController.getLeftY()
                    ),
                    [self.arm]
            )
        )
        
        self.grabber = FROGGrabber(51, 52, 1)
        self.grabber.setDefaultCommand(
            RunCommand(
                lambda: self.grabber.wheelsOn(
                    self.operatorController.getLeftTriggerAxis()
                    ),
                    [self.grabber]
            )
        )


        self.btnZeroGyro = JoystickButton(self.driverController, self.driverController.Button.kStart)
        self.btnDriveAuto = JoystickButton(self.driverController, self.driverController.Button.kRightBumper)

        self.configureButtonBindings()

    def configureButtonBindings(self):
        self.btnZeroGyro.onTrue(
            cmdZeroGyro(self.swerveChassis)
        )
        self.btnDriveAuto.whileTrue(
            cmdDriveTrajectory(
                self.swerveChassis, self.swerveChassis.getSimpleTrajectory()
            )
        )

        JoystickButton(self.driverController, self.driverController.Button.kLeftBumper).whileTrue(
            self.swerveChassis.getSwerveCommand
        )

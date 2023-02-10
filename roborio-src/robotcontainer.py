from wpimath.trajectory import TrajectoryGenerator, TrajectoryConfig, TrapezoidProfileRadians
from subsystems.drivetrain import SwerveChassis
from subsystems.controllers import FROGStick, LOGITECH_EXTREME_AXIS_CONFIG
from subsystems.vision import FROGPhotonVision
from subsystems.arm import Arm
from subsystems.grabber import FROGGrabber
from commands2.button import JoystickButton, CommandXboxController
from commands2 import RunCommand
from wpimath.units import feetToMeters
from wpimath.geometry import Transform2d

from commands.swerve_commands import cmdFieldOrientedDrive, cmdFieldOrientedThrottledDrive, cmdZeroGyro, cmdDriveTrajectory

MAX_TRAJECTORY_SPEED = feetToMeters(5)
MAX_TRAJECTORY_ACCEL = feetToMeters(5)

trajectoryConfig = TrajectoryConfig(MAX_TRAJECTORY_SPEED, MAX_TRAJECTORY_ACCEL)


class RobotContainer:
    def __init__(self):

        self.driverController = FROGStick(port = 0, **LOGITECH_EXTREME_AXIS_CONFIG)
        self.operatorController = CommandXboxController(2)

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
                    [self.arm]
            )
        )


        self.btnZeroGyro = JoystickButton(self.driverController, 3)
        self.btnDriveAuto = JoystickButton(self.driverController, 2)

        trajectoryConfig.setKinematics(self.kinematics)
        self.trajectory = TrajectoryGenerator.generateTrajectory(
			self.swerveChassis.estimatorPose, # Starting position
			[], # Pass through these points
			self.swerveChassis.estimatorPose + Transform2d(feetToMeters(6), feetToMeters(3)),
             # Ending position
			trajectoryConfig
        )

        self.configureButtonBindings()

    def configureButtonBindings(self):
        self.btnZeroGyro.onTrue(
            cmdZeroGyro(self.swerveChassis)
        )
        self.btnDriveAuto.whileTrue(
            cmdDriveTrajectory(
                self.swerveChassis, self.trajectory
            )
        )

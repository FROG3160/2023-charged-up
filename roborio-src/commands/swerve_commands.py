from commands2 import RunCommand, InstantCommand, Swerve4ControllerCommand
from wpimath.trajectory import Trajectory, TrapezoidProfileRadians
from wpimath.geometry import Pose2d
from subsystems.drivetrain import SwerveChassis
from subsystems.controllers import FROGStick
from wpimath.controller import PIDController, ProfiledPIDControllerRadians, HolonomicDriveController
import math

class cmdFieldOrientedDrive(RunCommand):
    def __init__(self, controller: FROGStick, drive: SwerveChassis) -> None:
        """Moves the robot using field oriented drive"""
        super().__init__(
            lambda: drive.fieldOrientedDrive(
                controller.getFieldForward(),
                controller.getFieldLeft(),
                controller.getFieldRotation(),
            ),  
            [drive],
        )

class cmdDriveTrajectory(Swerve4ControllerCommand):
    def __init__(self, drive: SwerveChassis, trajectory: Trajectory) -> None:
        self.xController = PIDController(1, 0, 0)
        self.yController = PIDController(1, 0, 0)
        self.angleController = ProfiledPIDControllerRadians(1, 0, 0, TrapezoidProfileRadians.Constraints(math.pi, math.pi))
        self.angleController.enableContinuousInput(-1*math.pi, math.pi)
        self.holonomicController = HolonomicDriveController(self.xController, self.yController, self.angleController)
        super().__init__(
            trajectory,
            drive.estimator.getEstimatedPosition,  #CALLABLE getPose
            drive.kinematics,
            self.holonomicController,
            drive.applyModuleStates,
            [drive]
        )

class cmdFieldOrientedThrottledDrive(RunCommand):
    def __init__(self, controller: FROGStick, drive: SwerveChassis) -> None:
        """Moves the robot using field oriented drive"""
        super().__init__(
            lambda: drive.fieldOrientedDrive(
                controller.getFieldForward(),
                controller.getFieldLeft(),
                controller.getFieldRotation(),
                controller.getFieldThrottle()
            ),  
            [drive],
        )

class cmdZeroGyro(InstantCommand):
    def __init__(self, drive: SwerveChassis) -> None:
        super().__init__(
            drive.gyro.resetGyro,
            [drive]
        )

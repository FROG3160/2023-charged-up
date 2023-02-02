from commands2 import RunCommand, InstantCommand
from subsystems.drivetrain import SwerveChassis
from subsystems.controllers import FROGStick

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


class cmdZeroGyro(InstantCommand):
    def __init__(self, drive: SwerveChassis) -> None:
        super().__init__(
            drive.gyro.resetGyro,
            [drive]
        )

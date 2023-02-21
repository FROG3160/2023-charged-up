#
# See the documentation for more details on how this works
#
# Documentation can be found at https://robotpy.readthedocs.io/projects/pyfrc/en/latest/physics.html
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#
# Examples can be found at https://github.com/robotpy/examples

import wpilib.simulation

from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics.drivetrains import four_motor_swerve_drivetrain, linear_deadzone



# import typing

# if typing.TYPE_CHECKING:
from robot import FROGbot


class PhysicsEngine:
    """
    Simulates a 4-wheel robot using Tank Drive joystick control
    """

    def __init__(self, physics_controller: PhysicsInterface, robot: "FROGbot"):
        """
        :param physics_controller: `pyfrc.physics.core.Physics` object
                                   to communicate simulation effects to
        :param robot: your robot objet
        """

        self.physics_controller = physics_controller
        self.robot = robot
        self.physics_controller.field.setRobotPose(self.robot.startingPose2d)


    def update_sim(self, now, tm_diff):
        pose = self.physics_controller.drive(
            self.robot.swerveChassis.chassisSpeeds, tm_diff
        )
        self.robot.swerveChassis.gyro.setAngleAdjustment(-pose.rotation().degrees())
        self.robot.swerveChassis.setFieldPosition(self.physics_controller.field.getRobotPose())

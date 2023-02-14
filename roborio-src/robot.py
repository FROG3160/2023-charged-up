import wpilib
import config
from magicbot import MagicRobot, tunable, feedback
from components.drivetrain import SwerveChassis, SwerveModule
from components.controllers import FROGStickDriver, FROGXboxDriver, FROGXboxOperator, FROGHolonomic
from components.field import FROGFieldLayout
from wpimath.geometry import Pose2d, Translation2d, Transform2d, Rotation2d
from wpimath.units import feetToMeters
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

        self.fieldLayout = FROGFieldLayout()
        


        # declare buttons
        self.btnEnableAutoDrive = self.driverController.getRightBumper
    
    # def robotInit(self) -> None:
    #     self.swerveChassis.setPosition(self.fieldLayout.getTagRelativePosition(7, 1))
  
    def autonomousInit(self):
        self.swerveChassis.setPosition(self.fieldLayout.getTagRelativePosition(7, 1))
        pass

    def teleopInit(self):
        self.holonomicController = FROGHolonomic(self.swerveChassis.kinematics)
        self.swerveChassis.enable()

    def teleopPeriodic(self):
        if self.btnEnableAutoDrive():
            #self.swerveChassis.enableAuto()
            if not self.holonomicController.trajectoryLoaded:

                startTrajectoryPose = self.swerveChassis.estimator.getEstimatedPosition()
                endTrajectoryPose = startTrajectoryPose + Transform2d(
                    feetToMeters(6), feetToMeters(3), 0
                )
                print(startTrajectoryPose,endTrajectoryPose)
                self.holonomicController.initTrajectory(
                    startTrajectoryPose, # Starting position
			        [], # Pass through these points
			        endTrajectoryPose, # Ending position
                )
            chassisSpeeds = self.holonomicController.getChassisSpeeds(
                self.swerveChassis.estimator.getEstimatedPosition(),
                Rotation2d.fromDegrees(0)
            )
            self.swerveChassis.autoDrive(chassisSpeeds)
        else:
            self.holonomicController.trajectoryLoaded = False
            #self.swerveChassis.disableAuto()
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
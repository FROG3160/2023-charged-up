import wpilib
import config
from magicbot import MagicRobot, tunable, feedback
from components.drivetrain import SwerveChassis, SwerveModule
from components.controllers import FROGStickDriver, FROGXboxDriver, FROGXboxOperator, FROGHolonomic
from components.field import FROGFieldLayout
from wpimath.geometry import Pose2d, Translation2d, Transform2d, Rotation2d
from wpimath.units import feetToMeters

RED_ALLIANCE = wpilib.DriverStation.Alliance.kRed
BLUE_ALLIANCE = wpilib.DriverStation.Alliance.kBlue
class FROGbot(MagicRobot):

    # Any magicbot component needs to be listed here
    # in order for their "execute" method to be run
    # every loop
    swerveChassis: SwerveChassis

    # tunable puts these values on NetworkTables for use with SmartDashboard
    gridNumber = tunable(default=1)
    gridPosition = tunable(default=1)
    gridLevel = tunable(default=3)

    def createObjects(self) -> None:
        self.moduleFrontLeft = SwerveModule(**config.MODULE_FRONT_LEFT)
        self.moduleFrontRight = SwerveModule(**config.MODULE_FRONT_RIGHT)
        self.moduleBackLeft = SwerveModule(**config.MODULE_BACK_LEFT)
        self.swerveBackRight = SwerveModule(**config.MODULE_BACK_RIGHT)

        self.driverController = FROGXboxDriver(0)
        self.operatorController = FROGXboxOperator(1)

        self.fieldLayout = FROGFieldLayout()
        if self.isSimulation():
            self.fieldLayout.setAlliance(BLUE_ALLIANCE)
            self.startingPose2d = self.fieldLayout.getTagRelativePosition(7, 1).toPose2d().transformBy(
                Transform2d(Translation2d(0,0),Rotation2d.fromDegrees(180))
            )
        else:
            self.startingPose2d = self.fieldLayout.getTagRelativePosition(7, 1).toPose2d().transformBy(
                Transform2d(Translation2d(0,0),Rotation2d.fromDegrees(180))
            )

        # declare buttons
        self.btnEnableAutoDrive = self.driverController.getRightBumper
        self.btnChangePosition = self.driverController.getPOV

    
    def robotInit(self) -> None:
        super().robotInit()
        self.swerveChassis.setPosition(self.startingPose2d)
  
    def autonomousInit(self):
        pass

    def teleopInit(self):

        self.swerveChassis.enable()

    def teleopPeriodic(self):
        changePosition = self.btnChangePosition()
        if changePosition > -1:
            print(changePosition)

        if self.btnEnableAutoDrive():
            #self.swerveChassis.enableAuto()
            if not self.swerveChassis.holonomicController.trajectoryType:
                startTrajectoryPose = self.swerveChassis.estimator.getEstimatedPosition()
                endTrajectoryPose = startTrajectoryPose + Transform2d(
                    feetToMeters(6), feetToMeters(3), 0
                )
                self.swerveChassis.holonomicController.initTrajectory(
                    startTrajectoryPose, # Starting position
			        [], # Pass through these points
			        endTrajectoryPose, # Ending position
                )
            self.swerveChassis.autoDrive()
        else:
            self.swerveChassis.holonomicController.trajectoryType = False
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
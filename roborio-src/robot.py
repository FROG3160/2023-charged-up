import wpilib
import config
from magicbot import MagicRobot, tunable, feedback
from components.drivetrain import SwerveChassis, SwerveModule
from components.controllers import FROGStickDriver, FROGXboxDriver, FROGXboxOperator, FROGHolonomic
from components.field import FROGFieldLayout
from wpimath.geometry import Pose2d, Translation2d, Transform2d, Rotation2d
from wpimath.units import feetToMeters
from pathplannerlib import PathPoint
from wpimath.units import inchesToMeters


RED_ALLIANCE = wpilib.DriverStation.Alliance.kRed
BLUE_ALLIANCE = wpilib.DriverStation.Alliance.kBlue

CTRE_PCM = wpilib.PneumaticsModuleType.CTREPCM
class FROGbot(MagicRobot):

    # Any magicbot component needs to be listed here
    # in order for their "execute" method to be run
    # every loop
    swerveChassis: SwerveChassis

    def createObjects(self) -> None:
        self.moduleFrontLeft = SwerveModule(**config.MODULE_FRONT_LEFT)
        self.moduleFrontRight = SwerveModule(**config.MODULE_FRONT_RIGHT)
        self.moduleBackLeft = SwerveModule(**config.MODULE_BACK_LEFT)
        self.swerveBackRight = SwerveModule(**config.MODULE_BACK_RIGHT)

        self.intake_rollerDeploy = wpilib.Solenoid(CTRE_PCM, 3)
        self.intake_rollerDeploy.set(False)

        self.driverController = FROGXboxDriver(0)
        self.operatorController = FROGXboxOperator(1)


        if self.isSimulation():
            self.alliance = BLUE_ALLIANCE
        else:
            self.alliance = wpilib.DriverStation.getAlliance()
        self.fieldLayout = FROGFieldLayout(self.alliance)
        self.logger.info(f"Alliance set to {self.alliance}")

        self.startingPose2d = self.fieldLayout.getTagRelativePosition(7, 2).toPose2d().transformBy(
            Transform2d(Translation2d(0,0),Rotation2d.fromDegrees(180))
        )


        # declare buttons
        self.btnEnableAutoDrive = self.driverController.getRightBumper
        self.btnResetEstimator = self.driverController.getStartButtonPressed
        self.btnResetGyro = self.driverController.getRightStickButtonPressed
        self.btnGoToPositionB = self.driverController.getBButton
        self.btnGoToPositionA = self.driverController.getAButton

        self.positionA = Pose2d( inchesToMeters(78), inchesToMeters(108.2), 0)
        self.positionB = Pose2d( inchesToMeters(203.5), inchesToMeters(174.2), 0)
    
    def robotInit(self) -> None:
        super().robotInit()  #calls createObjects()
        self.swerveChassis.setFieldPosition(self.startingPose2d)
  
    def autonomousInit(self):
        self.fieldLayout.setAlliance(self.alliance)
        pass

    def teleopInit(self):
        self.fieldLayout.setAlliance(self.alliance)
        self.swerveChassis.enable()

    def teleopPeriodic(self):
        if self.btnResetEstimator():
            print("Resetting Estimator to Vision Pose Estimate")
            self.swerveChassis.setFieldPosition(
                self.swerveChassis.visionPoseEstimator.getEstimatedRobotPose()[0].toPose2d())
        if self.btnResetGyro():
            self.swerveChassis.gyro.resetGyro()
        if self.btnEnableAutoDrive():
            #self.swerveChassis.enableAuto()
            if not self.swerveChassis.holonomicController.trajectoryType:
                startTrajectoryPose = self.swerveChassis.estimator.getEstimatedPosition()
                endTrajectoryPose = startTrajectoryPose + Transform2d(
                    feetToMeters(6), feetToMeters(3), 0
                )
                ##TODO figure out how to calculate heading
                startPoint = PathPoint(startTrajectoryPose.translation(), startTrajectoryPose.rotation())
                endPoint = PathPoint(endTrajectoryPose.translation(), endTrajectoryPose.rotation())
                self.swerveChassis.holonomicController.initSimpleTrajectory(
                    startPoint, # Starting position
			        endPoint, # Ending position
                )
            self.swerveChassis.autoDrive()

        elif self.btnGoToPositionA():
            #self.swerveChassis.enableAuto()
            if not self.swerveChassis.holonomicController.trajectoryType:
                currentPose = self.swerveChassis.estimator.getEstimatedPosition()
                ##TODO figure out how to calculate heading
                startPoint = PathPoint(currentPose.translation(), currentPose.rotation())
                endPoint = PathPoint(self.positionA.translation(), self.positionA.rotation())
                self.swerveChassis.holonomicController.initSimpleTrajectory(
                    startPoint, # Starting position
			        endPoint, # Ending position
                )
            self.swerveChassis.autoDrive()

        elif self.btnGoToPositionB():
            #self.swerveChassis.enableAuto()
            if not self.swerveChassis.holonomicController.trajectoryType:
                currentPose = self.swerveChassis.estimator.getEstimatedPosition()
                ##TODO figure out how to calculate heading
                startPoint = PathPoint(currentPose.translation(), currentPose.rotation())
                endPoint = PathPoint(self.positionB.translation(), self.positionB.rotation())
                self.swerveChassis.holonomicController.initSimpleTrajectory(
                    startPoint, # Starting position
			        endPoint, # Ending position
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
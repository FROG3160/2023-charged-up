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
from components.grabber import FROGGrabber
from components.arm import Arm
from wpimath.units import degreesToRadians
from ctre import ControlMode


RED_ALLIANCE = wpilib.DriverStation.Alliance.kRed
BLUE_ALLIANCE = wpilib.DriverStation.Alliance.kBlue


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

        self.grabber = FROGGrabber(43, 0)
        self.arm = Arm(41, 42)

        self.driverController = FROGXboxDriver(0)
        self.operatorController = FROGXboxOperator(1)

        self.fieldLayout = FROGFieldLayout()

        # declare buttons
        self.btnEnableAutoDrive = self.driverController.getRightBumper
        self.btnEnableAutoRotate = self.driverController.getLeftBumper
        self.btnResetEstimator = self.driverController.getStartButtonPressed
        self.btnResetGyro = self.driverController.getRightStickButtonPressed
        self.btnGoToPositionB = self.driverController.getBButton
        self.btnGoToPositionA = self.driverController.getAButton
        self.btnRunPathOut = self.driverController.getYButton

        self.btnCloseGrabber = self.operatorController.getRightBumper
        self.btnFloorPickup = self.operatorController.getAButtonPressed
        self.btnFloorManipulate = self.operatorController.getBButtonPressed
        self.btnHome = self.operatorController.getXButtonPressed

        self.positionA = Pose2d( inchesToMeters(78), inchesToMeters(108.2), 0)
        self.positionB = Pose2d( inchesToMeters(203.5), inchesToMeters(174.2), 0)

        self.startingPose2d = None
    
    def setAlliance(self):
        self.alliance = wpilib.DriverStation.getAlliance()
        self.fieldLayout.setAlliance(self.alliance)
        self.logger.info(f"FROGBot.fieldLayout alliance is {self.fieldLayout.alliance}")
        self.logger.info(f"SwerveChassis.fieldLayout alliance is {self.swerveChassis.visionPoseEstimator.fieldLayout.alliance}")
        self.logger.info(f'FROGPhotonVision.fieldlayout alliance is {self.swerveChassis.visionPoseEstimator.poseEstimator.getFieldLayout().alliance}')
    
    
    def robotInit(self) -> None:
        super().robotInit()  #calls createObjects()
  
    def autonomousInit(self):
        self.setAlliance()
        self.startingPose2d = self.fieldLayout.getTagRelativePosition(7, 2).toPose2d()
        self.swerveChassis.setFieldPosition(self.startingPose2d)

    def teleopInit(self):
        self.setAlliance()
        self.swerveChassis.enable()

    def teleopPeriodic(self):
        wpilib.SmartDashboard.putNumber('Ultrasonic Distance', self.grabber.ultrasonic.getInches())
        if self.btnCloseGrabber():
            self.grabber.close()
        else:
            self.grabber.open()
        self.grabber.motor.set(self.operatorController.getRightTriggerAxis())
        # self.arm.boom.run(self.operatorController.getRightY())
        # self.arm.stick.run(-self.operatorController.getLeftY())
        if self.btnFloorPickup():
            self.arm.boom.motor.set(ControlMode.Position, 146000)
            self.arm.stick.motor.set(ControlMode.Position, 0)
        elif self.btnFloorManipulate():
            self.arm.boom.motor.set(ControlMode.Position, 106000)
            self.arm.stick.motor.set(ControlMode.Position, 0)
        elif self.btnHome():
            self.arm.boom.motor.set(ControlMode.Position, 0)
            self.arm.stick.motor.set(ControlMode.Position, 0)


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
                    feetToMeters(6), 0 , -startTrajectoryPose.rotation().radians()
                )
                ##TODO figure out how to calculate heading
                startPoint = PathPoint(startTrajectoryPose.translation(), startTrajectoryPose.rotation())
                endPoint = PathPoint(endTrajectoryPose.translation(), endTrajectoryPose.rotation())
                self.logger.info(f"AUTODRIVE: {startTrajectoryPose} to {endTrajectoryPose}")
                self.swerveChassis.holonomicController.initSimpleTrajectory(
                    startPoint, # Starting position
			        endPoint, # Ending position
                )
            self.swerveChassis.autoDrive()

        if self.btnEnableAutoRotate():
            #self.swerveChassis.enableAuto()
            if not self.swerveChassis.holonomicController.trajectoryType:
                startTrajectoryPose = self.swerveChassis.estimator.getEstimatedPosition()
                endTrajectoryPose = startTrajectoryPose + Transform2d(
                    0, 0 , startTrajectoryPose.rotation().radians() + degreesToRadians(90)
                )
                ##TODO figure out how to calculate heading
                startPoint = PathPoint(startTrajectoryPose.translation(), startTrajectoryPose.rotation())
                endPoint = PathPoint(endTrajectoryPose.translation(), endTrajectoryPose.rotation())
                self.logger.info(f"AUTOROTATE: {startTrajectoryPose} to {endTrajectoryPose}")
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
                self.logger.info(f"AUTODRIVE TO A: {currentPose} to {self.positionA}")
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
                self.logger.info(f"AUTODRIVE TO B: {currentPose} to {self.positionB}")
                self.swerveChassis.holonomicController.initSimpleTrajectory(
                    startPoint, # Starting position
			        endPoint, # Ending position
                )
            self.swerveChassis.autoDrive()
        elif self.btnRunPathOut():
            if not self.swerveChassis.holonomicController.trajectoryType:
                self.swerveChassis.holonomicController.loadPathPlanner('pp_test1')
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
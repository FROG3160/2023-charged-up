import wpilib
import config
from magicbot import MagicRobot, tunable, feedback
from components.drivetrain import SwerveChassis, SwerveModule
from components.controllers import FROGXboxDriver, FROGXboxOperator
from components.field import FROGFieldLayout
from components.led import FROGLED
from components.vision import FROGLimeLightVision
from wpimath.geometry import Pose2d, Translation2d, Transform2d, Rotation2d
from wpimath import applyDeadband
from wpimath.units import feetToMeters
from pathplannerlib import PathPoint
from wpimath.units import inchesToMeters
from components.grabber import FROGGrabber
from components.arm import Arm
from components.sensors import FROGColor, FROGGyro
from wpimath.units import degreesToRadians
from components.arm_control import GrabberControl, ArmControl, ArmStates
from ctre import ControlMode
from wpilib import SmartDashboard
from wpilib.shuffleboard import  Shuffleboard
from wpilib.interfaces import GenericHID
from components.drive_control import DriveControl
# from components.led_control import LedControl


RED_ALLIANCE = wpilib.DriverStation.Alliance.kRed
BLUE_ALLIANCE = wpilib.DriverStation.Alliance.kBlue


class FROGbot(MagicRobot):

    # Any magicbot component needs to be listed here
    # in order for their "execute" method to be run
    # every loop
    grabberControl: GrabberControl
    armControl: ArmControl
    # ledControl: LedControl

    #Upper leve components first, lower level components last
    swerveChassis: SwerveChassis
    driveControl: DriveControl
    arm: Arm


    gyro: FROGGyro
    grabber: FROGGrabber
    sensor: FROGColor
    limelight: FROGLimeLightVision

    def createObjects(self) -> None:
        self.moduleFrontLeft = SwerveModule(**config.MODULE_FRONT_LEFT)
        self.moduleFrontRight = SwerveModule(**config.MODULE_FRONT_RIGHT)
        self.moduleBackLeft = SwerveModule(**config.MODULE_BACK_LEFT)
        self.moduleBackRight = SwerveModule(**config.MODULE_BACK_RIGHT)

        #self.sensor = FROGColor

        self.leds = FROGLED(35)

        self.driverController = FROGXboxDriver(0)
        self.operatorController = FROGXboxOperator(1)

        self.fieldLayout = FROGFieldLayout()

        # declare buttons for driver
        self.btnLockChassis = self.driverController.getRightBumper
        self.btnUnlockChassis = self.driverController.getLeftBumper
        self.btnResetEstimator = self.driverController.getBackButtonPressed
        self.btnResetGyro = self.driverController.getStartButtonPressed
        self.btnDrivePath = self.driverController.getBButton
        self.btnGoToGridPosition = self.driverController.getAButton
        self.btnDriveToCube = self.driverController.getXButton
        self.btnDriveToCone = self.driverController.getYButton
        self.btnDriveToCharging = self.driverController.getLeftTriggerAxis

        # declare buttons for operator
        self.btnToggleGrabber = self.operatorController.getLeftBumperPressed
        self.btnFloorPickup = self.operatorController.getAButtonPressed
        self.btnFloorManipulate = self.operatorController.getBButtonPressed
        self.btnHome = self.operatorController.getRightBumperPressed
        self.btnUpperPlace = self.operatorController.getYButtonPressed
        self.btnMidPlace = self.operatorController.getXButtonPressed
        self.btnRunArm = self.operatorController.getRightTriggerAxis
        self.btnRejectObject = self.operatorController.getLeftTriggerAxis
        self.btnGridSelect = self.operatorController.getPOVDebounced
        # self.btnOperatorManualChange = self.operatorController.getStartButtonPressed
        self.btnGrabberReset = self.operatorController.getBackButtonPressed

        self.positionGrid = Pose2d( 3.9, 2.1, 0)
        self.positionB = Pose2d( 7.1, 4.58, 0)

        self.startingPose2d = Pose2d(0,0,0)
        self.grabberIsOpen = False

        self.gridLevel = 1
        self.gridPosition = 1

        self.chassisLocked = False

        # self.aprilTagsTab = Shuffleboard.getTab('April Tags')
        # self.aprilTagsTab.add(title='botPoseBlue', defaultValue=self.limelight_at)
    
    def setAlliance(self):
        self.alliance = wpilib.DriverStation.getAlliance()
        self.fieldLayout.setAlliance(self.alliance)
        self.logger.info(f"FROGBot.fieldLayout alliance is {self.fieldLayout.alliance}")
        self.logger.info(f"SwerveChassis.fieldLayout alliance is {self.swerveChassis.fieldLayout.alliance}")
        self.logger.info(f"FROGLimeLight.fieldlayout alliance is {self.swerveChassis.limelight.fieldLayout.alliance}")

    
    
    def robotInit(self) -> None:
        """Runs at the startup of the robot code.  Anything that needs to be set
        before running Autonomous or Teleop modes should be added here"""
        super().robotInit()  #calls createObjects()
        self.setAlliance()
        #TODO: test if we can place the robot position setting here from AutonomousInit
        visionPose, visionTime = self.limelight.getBotPoseEstimateForAlliance()
        if visionPose:
            self.startingPose2d = visionPose.toPose2d()
        self.swerveChassis.setFieldPosition(self.startingPose2d)
        self.leds.green()
  
    def autonomousInit(self):
        """Runs at the beginning autonomous mode.  Add anything that is needed
        to put the robot in a known state to start Autonomous mode.
        """
        self.setAlliance()
        self.swerveChassis.enable()
        # TODO: Test and change this to use the initial bot post from vision?
        # this call gets the pose3d from the tuple returned by getBotPoseEstimateForAlliance
        # and then converts to a Pose2d for the setFieldPosition
        visionPose, visionTime = self.limelight.getBotPoseEstimateForAlliance()
        if visionPose:
            self.startingPose2d = visionPose.toPose2d()
        self.swerveChassis.setFieldPosition(self.startingPose2d)
        self.grabberControl.disableConeSupport()
        # self.armControl.next_state('leaveZero')

    def teleopInit(self):
        self.setAlliance()
        self.swerveChassis.enable()
        self.grabberControl.enableConeSupport()

    def teleopPeriodic(self):
        SmartDashboard.putNumber('Grid Position', self.gridPosition)
        SmartDashboard.putNumber('Grid Level', self.gridLevel)
        
                
        if self.btnGrabberReset():
            self.grabberControl.next_state('reset')
        # if self.btnOperatorManualChange():
        #     self.operatorController.changeMode()
        self.grabberControl.engage()
        if self.btnRunArm() > 0.5:
            self.armControl.engage()
        if self.btnRejectObject() > 0.5:
            self.grabber.motor.set(-0.5)
        
        if self.btnToggleGrabber():
            self.logger.info(f"Toggle Grabber (LB) pressed, current state is {self.grabberControl.current_state}")
            if self.grabberControl.current_state in ["holding", "stoppingIntake"]:
                self.grabberControl.next_state('dropping')
            elif self.grabberControl.current_state == "empty":
                self.grabberControl.next_state('looking')
        pov = self.btnGridSelect()
        if pov > -1:
            if pov == 0:
                if self.gridLevel < 3:
                    self.gridLevel += 1
            elif pov == 180:
                if self.gridLevel > 1:
                    self.gridLevel -= 1
            elif pov == 90:
                if self.gridPosition > 1:
                    self.gridPosition -= 1
            elif pov == 270:
                if self.gridPosition < 9:
                    self.gridPosition += 1

            self.logger.info(f'Grid Level: {self.gridLevel}, Grid Position: {self.gridPosition}')
        # else:
        #     self.arm.boom.run(-self.operatorController.getRightY())
        #     self.arm.stick.run(-self.operatorController.getLeftY())


        #self.grabber.motor.set(self.operatorController.getRightTriggerAxis())
        # self.arm.boom.run(self.operatorController.getRightY())
        # self.arm.stick.run(-self.operatorController.getLeftY())
        if self.btnFloorPickup():
            #self.armControl.moveToFloor()
            self.armControl.next_state(ArmStates.MOVING_TO_FLOOR)
            # self.arm.boom.toPosition(config.BOOM_FLOOR_PICKUP)
            # self.arm.stick.toPosition(config.STICK_FLOOR_PICKUP)
        elif self.btnFloorManipulate():
            #self.armControl.moveToManipulate()
            self.armControl.next_state(ArmStates.MOVING_TO_MANIPULATE)
            # self.arm.boom.toPosition(config.BOOM_FLOOR_MANIPULATE)
            # self.arm.stick.toPosition(config.STICK_FLOOR_MANIPULATE)
        elif self.btnHome():
            self.armControl.next_state(ArmStates.MOVING_TO_HOME)
            #self.armControl.moveToHome()
            # self.arm.boom.toPosition(config.BOOM_HOME)
            # self.arm.stick.toPosition(config.STICK_HOME)
        elif self.btnMidPlace():
            self.armControl.next_state(ArmStates.MOVING_TO_SHELF)
            #self.armControl.moveToShelf()
            # self.arm.boom.toPosition(config.BOOM_SHELF)
            # self.arm.stick.toPosition(config.STICK_SHELF)
        elif self.btnUpperPlace():
            self.armControl.next_state(ArmStates.MOVING_TO_UPPER)
            #self.armControl.moveToUpper()
            # self.arm.boom.toPosition(config.BOOM_GRID_UPPER)
            # self.arm.stick.toPosition(config.STICK_GRID_UPPER)


        if self.btnResetEstimator():
            print("Resetting Estimator to Vision Pose Estimate")
            visionPose, timestamp = self.limelight.getBotPoseEstimateForAlliance()
            if visionPose:
                self.swerveChassis.setFieldPosition(visionPose.toPose2d())
        if self.btnResetGyro():
            self.swerveChassis.gyro.resetGyro()
        # if self.btnEnableAutoDrive():
        #     #self.swerveChassis.enableAuto()
        #     if not self.swerveChassis.holonomicController.trajectoryType:
        #         startTrajectoryPose = self.swerveChassis.estimator.getEstimatedPosition()
        #         endTrajectoryPose = startTrajectoryPose + Transform2d(
        #             feetToMeters(6), 0 , -startTrajectoryPose.rotation().radians()
        #         )
        #         ##TODO figure out how to calculate heading
        #         startPoint = PathPoint(startTrajectoryPose.translation(), startTrajectoryPose.rotation())
        #         endPoint = PathPoint(endTrajectoryPose.translation(), endTrajectoryPose.rotation())
        #         self.logger.info(f"AUTODRIVE: {startTrajectoryPose} to {endTrajectoryPose}")
        #         self.swerveChassis.holonomicController.initSimpleTrajectory(
        #             startPoint, # Starting position
		# 	        endPoint, # Ending position
        #         )
        #     self.swerveChassis.autoDrive()
        if self.btnLockChassis():
            self.chassisLocked = True
            self.swerveChassis.enabled = False

        if self.btnUnlockChassis():
            self.chassisLocked = False
            self.swerveChassis.enabled = True

        if self.chassisLocked:
            self.driveControl.lockWheels()

        elif self.btnGoToGridPosition():
            self.logger.info(f'Driving to position {self.gridPosition}')
            self.driveControl.holonomicDriveToWaypoint(
                self.fieldLayout.getPosition(self.gridPosition).toPose2d()
            )

        elif self.btnDrivePath():
            self.driveControl.holonomicDrivePath()
            
        elif self.btnDriveToCone():
            self.driveControl.autoDriveToCone()


        elif self.btnDriveToCube():
            self.driveControl.autoDriveToCube()

        elif self.btnDriveToCharging() > 0.5:
            self.driveControl.driveToChargingReverse()

        else:
            pass


    def testInit(self):
        pass
        
    def testPeriodic(self):
        self.arm.boom.run(applyDeadband(-self.operatorController.getRightY(), 0.15))
        self.arm.stick.run(applyDeadband(-self.operatorController.getLeftY(), 0.15))

        if self.operatorController.getAButton():
            self.grabber.plateUp()
        else:
            self.grabber.plateDown()

        SmartDashboard.putNumber('Boom Position', self.arm.boom.getEncoderPosition())
        SmartDashboard.putNumber('Stick Position', self.arm.stick.getEncoderPosition())
        # self.leds.yellowPocketFast()

if __name__ == "__main__":
    wpilib.run(FROGbot)
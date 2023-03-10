import wpilib
import config
from magicbot import MagicRobot, tunable, feedback
from components.drivetrain import SwerveChassis, SwerveModule
from components.controllers import FROGStickDriver, FROGXboxDriver, FROGXboxOperator, FROGHolonomic
from components.field import FROGFieldLayout
from components.led import FROGLED
from components.vision import FROGLimeLightVision
from wpimath.geometry import Pose2d, Translation2d, Transform2d, Rotation2d
from wpimath.units import feetToMeters
from pathplannerlib import PathPoint
from wpimath.units import inchesToMeters
from components.grabber import FROGGrabber
from components.arm import Arm
from components.sensors import FROGColor, FROGGyro
from wpimath.units import degreesToRadians
from components.arm_control import GrabberControl, ArmControl
from ctre import ControlMode
from wpilib import SmartDashboard
from wpilib.interfaces import GenericHID


RED_ALLIANCE = wpilib.DriverStation.Alliance.kRed
BLUE_ALLIANCE = wpilib.DriverStation.Alliance.kBlue


class FROGbot(MagicRobot):

    # Any magicbot component needs to be listed here
    # in order for their "execute" method to be run
    # every loop
    grabberControl: GrabberControl
    armControl: ArmControl

    #Upper leve components first, lower level components last
    swerveChassis: SwerveChassis
    arm: Arm
    limelight: FROGLimeLightVision
    gyro: FROGGyro
    grabber: FROGGrabber
    sensor: FROGColor

    def createObjects(self) -> None:
        self.moduleFrontLeft = SwerveModule(**config.MODULE_FRONT_LEFT)
        self.moduleFrontRight = SwerveModule(**config.MODULE_FRONT_RIGHT)
        self.moduleBackLeft = SwerveModule(**config.MODULE_BACK_LEFT)
        self.swerveBackRight = SwerveModule(**config.MODULE_BACK_RIGHT)

        #self.sensor = FROGColor()

        self.leds = FROGLED(35)

        self.driverController = FROGXboxDriver(0)
        self.operatorController = FROGXboxOperator(1)

        self.fieldLayout = FROGFieldLayout()

        # declare buttons for driver
        self.btnEnableAutoDrive = self.driverController.getRightBumper
        self.btnEnableAutoRotate = self.driverController.getLeftBumper
        self.btnResetEstimator = self.driverController.getBackButtonPressed
        self.btnResetGyro = self.driverController.getStartButtonPressed
        self.btnGoToPositionB = self.driverController.getBButton
        self.btnGoToPositionA = self.driverController.getAButton
        self.btnDriveToCube = self.driverController.getXButton
        self.btnDriveToCone = self.driverController.getYButton

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
        self.btnOperatorManualChange = self.operatorController.getStartButtonPressed

        self.positionA = Pose2d( inchesToMeters(98.5), inchesToMeters(70), 0)
        self.positionB = Pose2d( inchesToMeters(203.5), inchesToMeters(174.2), 0)

        self.startingPose2d = None
        self.grabberIsOpen = False

        self.gridLevel = 1
        self.gridPosition = 1
    
    def setAlliance(self):
        self.alliance = wpilib.DriverStation.getAlliance()
        self.fieldLayout.setAlliance(self.alliance)
        self.logger.info(f"FROGBot.fieldLayout alliance is {self.fieldLayout.alliance}")
        self.logger.info(f"SwerveChassis.fieldLayout alliance is {self.swerveChassis.visionPoseEstimator.fieldLayout.alliance}")
        self.logger.info(f'FROGPhotonVision.fieldlayout alliance is {self.swerveChassis.visionPoseEstimator.poseEstimator.getFieldLayout().alliance}')
    
    
    def robotInit(self) -> None:
        super().robotInit()  #calls createObjects()
        self.leds.Fire()
  
    def autonomousInit(self):
        self.setAlliance()
        self.swerveChassis.enable()
        #self.startingPose2d = self.fieldLayout.getTagRelativePosition(7, 2).toPose2d()
        #self.swerveChassis.setFieldPosition(self.startingPose2d)
        self.armControl.next_state('leaveZero')

    def teleopInit(self):
        self.setAlliance()
        self.swerveChassis.enable()
        self.swerveChassis.visionPoseEstimator.camera.setDriverMode(False)

    def teleopPeriodic(self):
        SmartDashboard.putNumber('Grid Position', self.gridPosition)
        SmartDashboard.putNumber('Grid Level', self.gridLevel)
        SmartDashboard.putNumber('Operator Manual Mode', self.operatorController.getManualMode())
        if not self.operatorController.getManualMode():
                
            if self.btnOperatorManualChange:
                self.operatorController.changeMode()
            self.grabberControl.engage()
            if self.btnRunArm() > 0.5:
                self.armControl.engage()
            if self.btnRejectObject() > 0.5:
                self.grabber.motor.set(-0.5)
            wpilib.SmartDashboard.putNumber('Proximity', self.sensor.getProximity())
            if self.btnToggleGrabber():
                self.logger.info("Toggle Grabber pressed")
                self.logger.info(f"Current state is {self.grabberControl.current_state}")
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
        else:
            self.arm.boom.run(-self.operatorController.getRightY())
            self.arm.stick.run(-self.operatorController.getLeftY())


        #self.grabber.motor.set(self.operatorController.getRightTriggerAxis())
        # self.arm.boom.run(self.operatorController.getRightY())
        # self.arm.stick.run(-self.operatorController.getLeftY())
        if self.btnFloorPickup():
            self.armControl.setNextState('moveToFloor')
            # self.arm.boom.toPosition(config.BOOM_FLOOR_PICKUP)
            # self.arm.stick.toPosition(config.STICK_FLOOR_PICKUP)
        elif self.btnFloorManipulate():
            self.armControl.setNextState('moveToManipulate')
            # self.arm.boom.toPosition(config.BOOM_FLOOR_MANIPULATE)
            # self.arm.stick.toPosition(config.STICK_FLOOR_MANIPULATE)
        elif self.btnHome():
            self.armControl.setNextState('moveToHome')
            # self.arm.boom.toPosition(config.BOOM_HOME)
            # self.arm.stick.toPosition(config.STICK_HOME)
        elif self.btnMidPlace():
            self.armControl.setNextState('moveToShelf')
            # self.arm.boom.toPosition(config.BOOM_SHELF)
            # self.arm.stick.toPosition(config.STICK_SHELF)
        elif self.btnUpperPlace():
            self.armControl.setNextState('moveToUpper')
            # self.arm.boom.toPosition(config.BOOM_GRID_UPPER)
            # self.arm.stick.toPosition(config.STICK_GRID_UPPER)


        if self.btnResetEstimator():
            print("Resetting Estimator to Vision Pose Estimate")
            visionPose, timestamp = self.swerveChassis.visionPoseEstimator.getEstimatedRobotPose()
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
                startX = 2.47
                endX = 1.81
                self.swerveChassis.setFieldPosition(Pose2d(startX,0,0))
                self.logger.info(f'Estimator Field Position is: {self.swerveChassis.estimator.getEstimatedPosition()}')
                startTrajectoryPose = self.swerveChassis.estimator.getEstimatedPosition()
                endTrajectoryPose = Pose2d(endX, 0 , 0)
                self.logger.info(f'Starting at {startTrajectoryPose}')
                self.logger.info(f'Ending at: {endTrajectoryPose}')
                ##TODO figure out how to calculate heading
                self.swerveChassis.holonomicController.initPoseToPose(
                    startTrajectoryPose, # Starting position
                    endTrajectoryPose, # Ending position
                )
            self.swerveChassis.autoDrive()
        elif self.btnGoToPositionB():
        #     #self.swerveChassis.enableAuto()
            if not self.swerveChassis.holonomicController.trajectoryType:
                startX = 1.81
                endX = 2.47
                self.swerveChassis.setFieldPosition(Pose2d(startX,0,0))
                self.logger.info(f'Estimator Field Position is: {self.swerveChassis.estimator.getEstimatedPosition()}')
                startTrajectoryPose = self.swerveChassis.estimator.getEstimatedPosition()
                endTrajectoryPose = Pose2d(self.endX, 0 , 0)
                self.logger.info(f'Starting at {startTrajectoryPose}')
                self.logger.info(f'Ending at: {endTrajectoryPose}')
                ##TODO figure out how to calculate heading
                self.swerveChassis.holonomicController.initPoseToPose(
                    startTrajectoryPose, # Starting position
                    endTrajectoryPose, # Ending position
                )
            self.swerveChassis.autoDrive()
        # elif self.btnEnableAutoDrive():
        #     if not self.swerveChassis.holonomicController.trajectoryType:
        #         self.swerveChassis.holonomicController.loadPathPlanner('Position8toMidfield')
        #     self.swerveChassis.autoDrive()
            
        elif self.btnDriveToCone():
            self.limelight.findCones()
            self.swerveChassis.driveToObject()
            # if self.grabberControl.targetPresent:
            #     self.driverController.setRumble(GenericHID.RumbleType.kRightRumble)

        elif self.btnDriveToCube():
            self.limelight.findCubes()
            self.swerveChassis.driveToObject()

        else:
            self.swerveChassis.holonomicController.trajectoryType = False
            #self.swerveChassis.disableAuto()
            self.swerveChassis.fieldOrientedDrive(
                self.driverController.getFieldForward(),
                self.driverController.getFieldLeft(),
                self.driverController.getFieldRotation(),
                self.driverController.getFieldThrottle(),
            )
            # if self.armControl.current_state == 'atHome' and self.swerveChassis.getChassisVelocityFPS() > 3:
            #     self.operatorController.setRumble(GenericHID.RumbleType.kRightRumble)
            #     self.driverController.setRumble(GenericHID.RumbleType.kLeftRumble)
                
        

    def testInit(self):
        pass
        
    def testPeriodic(self):
        pass

if __name__ == "__main__":
    wpilib.run(FROGbot)
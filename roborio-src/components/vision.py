import os
import typing
import wpilib, logging
from photonvision import PhotonCamera, PoseStrategy, RobotPoseEstimator
from robotpy_apriltag import AprilTagFieldLayout
from wpimath.geometry import Pose3d, Quaternion, Rotation3d, Transform3d, Translation3d
from wpilib import SmartDashboard
from ntcore import NetworkTableInstance, NetworkTable
from wpimath.units import metersToInches, inchesToMeters, radiansToDegrees
from wpimath.filter import MedianFilter
from components.field import FROGFieldLayout
from photonvision import PhotonUtils
from wpilib.shuffleboard import Shuffleboard

RED_ALLIANCE = wpilib.DriverStation.Alliance.kRed
BLUE_ALLIANCE = wpilib.DriverStation.Alliance.kBlue

LL_CUBE = 1
LL_CONE = 0

def arrayToPose3d(poseArray) -> Pose3d:
    return Pose3d(
        Translation3d(poseArray[0], poseArray[1], poseArray[2]),
        Rotation3d(poseArray[3], poseArray[4], poseArray[5]),
    )

class FROGPhotonVision:
    def __init__(self, fieldLayout: FROGFieldLayout, cameraName: str, cameraTransform3d: Transform3d):
        self.logger = logging.getLogger("FROGPhotonVision")
        self.camera = PhotonCamera(cameraName = cameraName)
        self.camera.setDriverMode(False)
        self.fieldLayout = fieldLayout
        self.cameraTransform3d = cameraTransform3d
        self.logger.info(f'Initializing with fieldlayout origin: {self.fieldLayout.alliance}')
        self.poseEstimator = RobotPoseEstimator(
            self.fieldLayout,
            PoseStrategy.LOWEST_AMBIGUITY,
            [
                (
                    self.camera,
                    self.cameraTransform3d,
                ),
            ],
        )
        # self.photonEstimator = PhotonPoseEstimator(
        #     self.fieldLayout,
        #     PoseStrategy.LOWEST_AMBIGUITY,
        #     self.cameraTransform3d
        # )
        self.currentPose = Pose3d()
        self.previousEstimatedRobotPose = None
        self.poseFilter = MedianFilter(5)
        self.ambiguity = 10
        self.targetID = 0


    def getEstimatedRobotPose(self) -> typing.Tuple[Pose3d, float]:
        if not wpilib.RobotBase.isSimulation():
            self.currentPose, visionTime = self.poseEstimator.update()
            if self.camera.hasTargets():
                self.bestTarget = self.camera.getLatestResult().getBestTarget()
                self.ambiguity = self.bestTarget.getPoseAmbiguity()
                self.targetID = self.bestTarget.getFiducialId()
                if self.ambiguity < 0.02:
                    return (self.currentPose, visionTime)
        return (None, None)
        
    

    def periodic(self) -> None:
        if self.camera.hasTargets():
            result = self.camera.getLatestResult()
            # self.photonEstimatedPose = self.photonEstimator.update(result)
            bestTarget = result.getBestTarget()
            ambiguity = bestTarget.getPoseAmbiguity()
            bestTgtTransform = bestTarget.getBestCameraToTarget()
            bestTgtID = bestTarget.getFiducialId()
            tagPose = self.fieldLayout.getTagPose(bestTgtID)
            # PhotonUtils.estimateFieldToRobot
            # robotPose = PhotonUtils.estimateFieldToRobotAprilTag(bestTarget.getBestCameraToTarget(), self.fieldLayout.getTagPose(bestTarget.getFiducialId()), self.cameraTransform3d);
            #photonEstimatedPose = self.photonEstimatedPose.estimatedPose
            #timestamp = self.photonEstimatedPose.timestamp
            # SmartDashboard.putNumber("PEP_X", metersToInches(photonEstimatedPose.X()))
            # SmartDashboard.putNumber("PEP_Y", metersToInches(photonEstimatedPose.Y()))
            # SmartDashboard.putNumber("PEP_Degrees", metersToInches(photonEstimatedPose.rotation().toRotation2d().degrees()))
            SmartDashboard.putNumber("TgtTransform_X", metersToInches(bestTgtTransform.X()))
            SmartDashboard.putNumber("TgtTransform_Y", metersToInches(bestTgtTransform.Y()))
            SmartDashboard.putNumber("TgtTransform_Degrees", bestTgtTransform.rotation().toRotation2d().degrees())
            # cameraOnField = tagPose.transformBy(bestTgtTransform.inverse())
            # robotOnField = cameraOnField.transformBy(self.cameraTransform3d).toPose2d()
            # SmartDashboard.putNumber("PVRobot_X", metersToInches(robotOnField.X()))
            # SmartDashboard.putNumber("PVRobot_Y", metersToInches(robotOnField.Y()))
            # SmartDashboard.putNumber("PVRobot_Degrees", robotOnField.rotation().degrees())

            if self.ambiguity < 0.02:
                SmartDashboard.putNumber(
                    "PhotonVision_X_Inches", metersToInches(self.currentPose.X())
                )
                SmartDashboard.putNumber(
                    "PhotonVision_Y_Inches", metersToInches(self.currentPose.Y())
                )
                SmartDashboard.putNumber(
                    "PhotonVision_T_Degrees", self.currentPose.rotation().angle_degrees
                )
                SmartDashboard.putNumber(
                    "PhotonVision_Ambiguity", self.ambiguity
                )
                SmartDashboard.putNumber(
                    "PhotonVision_TargetID", self.targetID
                )


class FROGLimeLightVision:

    fieldLayout: FROGFieldLayout

    def __init__(self, fieldLayout: FROGFieldLayout, ntTableName):
        self.fieldLayout = fieldLayout
        self.limelightTable = NetworkTableInstance.getDefault().getTable(
            key=ntTableName
        )
        self.ntTclass = self.limelightTable.getStringTopic('tclass').subscribe('None')
        self.ntTa = self.limelightTable.getFloatTopic('ta').subscribe(0)
        self.ntTx = self.limelightTable.getFloatTopic('tx').subscribe(-999)
        self.ntTv = self.limelightTable.getIntegerTopic('tv').subscribe(0)
        self.ntPipe = self.limelightTable.getIntegerTopic('getPipe').subscribe(-1)

        self.ntCl = self.limelightTable.getFloatTopic('cl').subscribe(0)
        self.ntTl = self.limelightTable.getFloatTopic('tl').subscribe(0)

        self.botPose = self.limelightTable.getFloatArrayTopic("botpose").subscribe([-99, -99, -99, 0, 0, 0])
        
        self.botPoseBlue = self.limelightTable.getFloatArrayTopic(
            "botpose_wpiblue"
        ).subscribe([-99, -99, -99, 0, 0, 0])
        self.botPoseRed = self.limelightTable.getFloatArrayTopic(
            "botpose_wpired"
        ).subscribe([-99, -99, -99, 0, 0, 0])
        # create the timer that we can use to the the FPGA timestamp
        self.timer = wpilib.Timer()

    def getLatency(self):
        return (self.ntCl.get() + self.ntTl.get())/1000

    def findCubes(self):
        self.setPipeline(LL_CUBE)

    def findCones(self):
        self.setPipeline(LL_CONE)

    def getPipeline(self):
        return self.ntPipe.get()
    
    def getBotPoseAlliance(self) -> typing.Tuple[Pose3d, float]:
        if self.fieldLayout.alliance == RED_ALLIANCE:
            return (self.getBotPoseRed(), self.timer.getFPGATimestamp()-self.getLatency())
        elif self.fieldLayout.alliance == BLUE_ALLIANCE:
            return (self.getBotPoseBlue(), self.timer.getFPGATimestamp()-self.getLatency())
        
    def getBotPose(self) -> Pose3d:
        return arrayToPose3d(self.botPose.get())

    def getBotPoseBlue(self) -> Pose3d:
        return arrayToPose3d(self.botPoseBlue.get())

    def getBotPoseRed(self) -> Pose3d:
        return arrayToPose3d(self.botPoseRed.get())

    def getTID(self) -> float:
        return self.limelightTable.getNumber("tid", -1.0)

    def getTarget(self):
        if self.ntTv.get():
            self.tClass = self.ntTclass.get()
            self.ta = self.ntTa.get()
            self.tx = self.ntTx.get()
            self.tv = self.ntTv.get()
            self.drive_vRotate = -(self.tx / 40)
            self.drive_vX = -(self.ta * -0.0098 + 1.0293)
            self.drive_vY = 0
        else:
            self.tClass = self.ta = self.tx = self.tv = None
            self.drive_vRotate = self.drive_vX = self.drive_vY = None
    
    def hasTarget(self):
        return self.tv
    
    def execute(self) -> None:
        self.getTarget()
        if self.hasTarget():
            SmartDashboard.putNumber('LL TGT Area', self.ta)
            SmartDashboard.putNumber('LL TGT X', self.tx)
            SmartDashboard.putNumber('LL TGT Valid', self.tv)
            SmartDashboard.putString( 'LL TGT Class', self.tClass)

    def setPipeline(self, objectInt: int):
        self.limelightTable.putNumber('pipeline', objectInt)


if __name__ == '__main__':
    pass
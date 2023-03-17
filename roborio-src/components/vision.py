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
import config

RED_ALLIANCE = wpilib.DriverStation.Alliance.kRed
BLUE_ALLIANCE = wpilib.DriverStation.Alliance.kBlue

LL_CUBE = 1
LL_CONE = 0


def arrayToPose3d(poseArray) -> Pose3d:
    return Pose3d(
        Translation3d(poseArray[0], poseArray[1], poseArray[2]),
        Rotation3d.fromDegrees(poseArray[3], poseArray[4], poseArray[5]),
    )


class FROGPhotonVision:
    def __init__(
        self,
        fieldLayout: FROGFieldLayout,
        cameraName: str,
        cameraTransform3d: Transform3d,
    ):
        self.logger = logging.getLogger("FROGPhotonVision")
        self.camera = PhotonCamera(cameraName=cameraName)
        self.camera.setDriverMode(False)
        self.fieldLayout = fieldLayout
        self.cameraTransform3d = cameraTransform3d
        self.logger.info(
            f"Initializing with fieldlayout origin: {self.fieldLayout.alliance}"
        )
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
            # photonEstimatedPose = self.photonEstimatedPose.estimatedPose
            # timestamp = self.photonEstimatedPose.timestamp
            # SmartDashboard.putNumber("PEP_X", metersToInches(photonEstimatedPose.X()))
            # SmartDashboard.putNumber("PEP_Y", metersToInches(photonEstimatedPose.Y()))
            # SmartDashboard.putNumber("PEP_Degrees", metersToInches(photonEstimatedPose.rotation().toRotation2d().degrees()))
            SmartDashboard.putNumber(
                "TgtTransform_X", metersToInches(bestTgtTransform.X())
            )
            SmartDashboard.putNumber(
                "TgtTransform_Y", metersToInches(bestTgtTransform.Y())
            )
            SmartDashboard.putNumber(
                "TgtTransform_Degrees",
                bestTgtTransform.rotation().toRotation2d().degrees(),
            )
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
                SmartDashboard.putNumber("PhotonVision_Ambiguity", self.ambiguity)
                SmartDashboard.putNumber("PhotonVision_TargetID", self.targetID)


class FROGLimeLightVision:
    fieldLayout: FROGFieldLayout

    def __init__(self):
        # self.fieldLayout = fieldLayout
        self.ll_grabberTable = NetworkTableInstance.getDefault().getTable(
            key=config.LIMELIGHT_GRABBER
        )
        self.ll_upperTable = NetworkTableInstance.getDefault().getTable(
            key=config.LIMELIGHT_UPPER
        )
        self.grabberTclass = self.ll_grabberTable.getStringTopic("tclass").subscribe(
            "None"
        )
        self.grabberTa = self.ll_grabberTable.getFloatTopic("ta").subscribe(0)
        self.grabberTx = self.ll_grabberTable.getFloatTopic("tx").subscribe(-999)
        self.grabberTv = self.ll_grabberTable.getIntegerTopic("tv").subscribe(0)
        self.grabberPipe = self.ll_grabberTable.getIntegerTopic("getPipe").subscribe(-1)

        self.grabberCl = self.ll_grabberTable.getFloatTopic("cl").subscribe(0)
        self.grabberTl = self.ll_grabberTable.getFloatTopic("tl").subscribe(0)

        self.upperPose = self.ll_upperTable.getFloatArrayTopic("botpose").subscribe(
            [-99, -99, -99, 0, 0, 0]
        )
        self.upperPoseBlue = self.ll_upperTable.getFloatArrayTopic(
            "botpose_wpiblue"
        ).subscribe([-99, -99, -99, 0, 0, 0])
        self.upperPoseRed = self.ll_upperTable.getFloatArrayTopic(
            "botpose_wpired"
        ).subscribe([-99, -99, -99, 0, 0, 0])
        self.upperPipe = self.ll_upperTable.getIntegerTopic("getPipe").subscribe(-1)
        # create the timer that we can use to the the FPGA timestamp
        self.timer = wpilib.Timer()

    def getLatency(self):
        return (self.grabberCl.get() + self.grabberTl.get()) / 1000

    def findCubes(self):
        self.setGrabberPipeline(LL_CUBE)

    def findCones(self):
        self.setGrabberPipeline(LL_CONE)

    def getGrabberPipeline(self):
        return self.grabberPipe.get()
    
    def getUpperPipeline(self):
        return self.upperPipe.get()

    def getBotPoseEstimateForAlliance(self) -> typing.Tuple[Pose3d, float]:
        if self.fieldLayout.alliance == RED_ALLIANCE:
            return (
                self.getBotPoseRed(),
                self.timer.getFPGATimestamp() - self.getLatency(),
            )
        elif self.fieldLayout.alliance == BLUE_ALLIANCE:
            return (
                self.getBotPoseBlue(),
                self.timer.getFPGATimestamp() - self.getLatency(),
            )

    def getBotPose(self) -> Pose3d:
        return arrayToPose3d(self.upperPose.get())

    def getBotPoseBlue(self) -> Pose3d:
        return arrayToPose3d(self.upperPoseBlue.get())

    def getBotPoseRed(self) -> Pose3d:
        return arrayToPose3d(self.upperPoseRed.get())

    def getTID(self) -> float:
        return self.ll_grabberTable.getNumber("tid", -1.0)

    def getTarget(self):
        if self.grabberTv.get():
            self.tClass = self.grabberTclass.get()
            self.ta = self.grabberTa.get()
            self.tx = self.grabberTx.get()
            self.tv = self.grabberTv.get()
            self.drive_vRotate = -(self.tx / 40)
            self.drive_vX = -(self.ta * -0.0098 + 1.0293)
            self.drive_vY = 0
        else:
            self.tClass = self.ta = self.tx = self.tv = None
            self.drive_vRotate = self.drive_vX = self.drive_vY = None

    def hasGrabberTarget(self):
        return self.tv

    def execute(self) -> None:
        self.getTarget()
        if self.hasGrabberTarget():
            SmartDashboard.putNumber("LL TGT Area", self.ta)
            SmartDashboard.putNumber("LL TGT X", self.tx)
            SmartDashboard.putNumber("LL TGT Valid", self.tv)
            SmartDashboard.putString("LL TGT Class", self.tClass)

    def setGrabberPipeline(self, objectInt: int):
        self.ll_grabberTable.putNumber("pipeline", objectInt)

    def setUpperPipeline(self, pipeNum: int):
        self.ll_upperTable.putNumber("pipeline", pipeNum)


if __name__ == "__main__":
    pass

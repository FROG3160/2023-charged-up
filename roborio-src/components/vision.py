import os
import typing
import wpilib, logging
from photonvision import PhotonCamera, PoseStrategy, RobotPoseEstimator, SimPhotonCamera
from robotpy_apriltag import AprilTagFieldLayout
from wpimath.geometry import Pose3d, Quaternion, Rotation3d, Transform3d, Translation3d
from wpilib import SmartDashboard
from ntcore import NetworkTableInstance, NetworkTable
from wpimath.units import metersToInches, inchesToMeters, radiansToDegrees
from components.field import FROGFieldLayout

RED_ALLIANCE = wpilib.DriverStation.Alliance.kRed
BLUE_ALLIANCE = wpilib.DriverStation.Alliance.kBlue

def arrayToPose3d(poseArray) -> Pose3d:
    return Pose3d(
        Translation3d(poseArray[0], poseArray[1], poseArray[2]),
        Rotation3d(poseArray[3], poseArray[4], poseArray[5]),
    )

class FROGPhotonVision:
    def __init__(self, fieldLayout: FROGFieldLayout, cameraName: str, cameraTransform3d: Transform3d):
        self.logger = logging.getLogger("FROGPhotonVision")
        self.camera = PhotonCamera(cameraName = cameraName)
        self.fieldLayout = fieldLayout
        self.logger.info(f'Initializing with fieldlayout origin: {self.fieldLayout.alliance}')
        self.poseEstimator = RobotPoseEstimator(
            self.fieldLayout,
            PoseStrategy.LOWEST_AMBIGUITY,
            [
                (
                    self.camera,
                    cameraTransform3d,
                ),
            ],
        )
        self.currentPose = Pose3d()
        self.previousEstimatedRobotPose = None

    def getEstimatedRobotPose(self) -> typing.Tuple[Pose3d, float]:
        # if self.previousEstimatedRobotPose:
        #     self.poseEstimator.setReferencePose(self.previousEstimatedRobotPose)
        if not wpilib.RobotBase.isSimulation():
            self.currentPose, visionTime = self.poseEstimator.update()
            # if self.currentPose:
            #     self.previousEstimatedRobotPose = self.currentPose
            return (self.currentPose, visionTime)
        else:
            return (None, None)
    

    def periodic(self) -> None:
        self.logger.info(f'Getting pose from {self.fieldlayout.alliance}')
        self.getEstimatedRobotPose()
        SmartDashboard.putNumber(
            "PhotonVision_X_Inches", metersToInches(self.currentPose.X())
        )
        SmartDashboard.putNumber(
            "PhotonVision_Y_Inches", metersToInches(self.currentPose.Y())
        )
        SmartDashboard.putNumber(
            "PhotonVision_T_Degrees", self.currentPose.rotation().angle_degrees
        )


class FROGLimeLightVision:
    def __init__(self):
        self.fieldLayout = FROGFieldLayout()
        self.limelightTable = NetworkTableInstance.getDefault().getTable(
            key="limelight"
        )
        self.botPose = self.limelightTable.getFloatArrayTopic("botpose").subscribe([-99, -99, -99, 0, 0, 0])
        #self.botPose = self.limelightTable.getDoubleArrayTopic("botpose").subscribe([])
        self.botPoseBlue = self.limelightTable.getFloatArrayTopic(
            "botpose_wpiblue"
        ).subscribe([-99, -99, -99, 0, 0, 0])
        self.botPoseRed = self.limelightTable.getFloatArrayTopic(
            "botpose_wpired"
        ).subscribe([-99, -99, -99, 0, 0, 0])
        # create the timer that we can use to the the FPGA timestamp
        self.timer = wpilib.Timer()

    def getBotPoseAlliance(self) -> typing.Tuple[Pose3d, float]:
        if self.fieldLayout.alliance == RED_ALLIANCE:
            return (self.getBotPoseRed(), self.timer.getFPGATimestamp())
        elif self.fieldLayout.alliance == BLUE_ALLIANCE:
            return (self.getBotPoseBlue(), self.timer.getFPGATimestamp())
        else:
            return None

    def getBotPose(self) -> Pose3d:
        return arrayToPose3d(self.botPose.get())

    def getBotPoseBlue(self) -> Pose3d:
        return arrayToPose3d(self.botPoseBlue.get())

    def getBotPoseRed(self) -> Pose3d:
        return arrayToPose3d(self.botPoseRed.get())

    def getTID(self) -> float:
        return self.limelightTable.getNumber("tid", -1.0)

    def periodic(self) -> None:
        SmartDashboard.putNumber("BotPose TagID", self.getTID())

        botpose = self.getBotPose()
        SmartDashboard.putNumber("BotPose_X", metersToInches(botpose.x))
        SmartDashboard.putNumber("BotPose_Y", metersToInches(botpose.y))
        SmartDashboard.putNumber("BotPose_Z", metersToInches(botpose.z))

        botpose = self.getBotPoseBlue()
        SmartDashboard.putNumber("BlueBotPose_X", metersToInches(botpose.x))
        SmartDashboard.putNumber("BlueBotPose_Y", metersToInches(botpose.y))
        SmartDashboard.putNumber("BlueBotPose_Z", metersToInches(botpose.z))

        botpose = self.getBotPoseRed()
        SmartDashboard.putNumber("RedBotPose_X", metersToInches(botpose.x))
        SmartDashboard.putNumber("RedBotPose_Y", metersToInches(botpose.y))
        SmartDashboard.putNumber("RedBotPose_Z", metersToInches(botpose.z))

if __name__ == '__main__':
    pass
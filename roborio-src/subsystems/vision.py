import os
import typing
import wpilib
from commands2 import SubsystemBase
from photonvision import PhotonCamera, PoseStrategy, RobotPoseEstimator, SimPhotonCamera
from robotpy_apriltag import AprilTagFieldLayout
from wpimath.geometry import Pose3d, Quaternion, Rotation3d, Transform3d, Translation3d
from wpilib import SmartDashboard
from ntcore import NetworkTableInstance, NetworkTable
from wpimath.units import metersToInches, inchesToMeters, radiansToDegrees
from field import FROGFieldLayout

RED_ALLIANCE = wpilib.DriverStation.Alliance.kRed
BLUE_ALLIANCE = wpilib.DriverStation.Alliance.kBlue

def arrayToPose3d(poseArray) -> Pose3d:
    return Pose3d(
        Translation3d(poseArray[0], poseArray[1], poseArray[2]),
        Rotation3d(poseArray[3], poseArray[4], poseArray[5]),
    )

class FROGPhotonVision(SubsystemBase):
    def __init__(self):
        super().__init__()
        # TODO: add second camera
        self.camera = PhotonCamera("IMX219")
        self.fieldLayout = FROGFieldLayout()
        self.poseEstimator = RobotPoseEstimator(
            self.fieldLayout,
            PoseStrategy.CLOSEST_TO_CAMERA_HEIGHT,
            [
                (
                    self.camera,
                    Transform3d(
                        Translation3d(inchesToMeters(13.75), 0, inchesToMeters(5.5)),
                        Rotation3d(0, 0, 0),
                    ),
                ),
            ],
        )
        self.currentPose = Pose3d()
        self.previousEstimatedRobotPose = None
        self.time = None

    def getEstimatedRobotPose(self) -> typing.Tuple[Pose3d, float]:
        # if self.previousEstimatedRobotPose:
        #     self.poseEstimator.setReferencePose(self.previousEstimatedRobotPose)
        if not wpilib.RobotBase.isSimulation():
            self.currentPose, self.time = self.poseEstimator.update()
            # if self.currentPose:
            #     self.previousEstimatedRobotPose = self.currentPose
            return (self.currentPose, self.time)
        else:
            return (None, None)

    def periodic(self) -> None:
        SmartDashboard.putNumber(
            "PhotonVision_X_Inches", metersToInches(self.currentPose.X())
        )
        SmartDashboard.putNumber(
            "PhotonVision_Y_Inches", metersToInches(self.currentPose.Y())
        )
        SmartDashboard.putNumber(
            "PhotonVision_T_Degrees", self.currentPose.rotation().angle_degrees
        )


class FROGLimeLightVision(SubsystemBase):
    def __init__(self):
        super().__init__()
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
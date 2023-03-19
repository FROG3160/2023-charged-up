import os
import typing
import wpilib, logging
from robotpy_apriltag import AprilTagFieldLayout
from wpimath.geometry import Pose3d, Quaternion, Rotation3d, Transform3d, Translation3d
from ntcore import NetworkTableInstance, NetworkTable
from wpimath.units import metersToInches, inchesToMeters, radiansToDegrees
from wpimath.filter import MedianFilter
from components.field import FROGFieldLayout
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
            self.drive_vRotate = self.drive_vX = self.drive_vY = 0

    def getVelocities(self):
        """Get calculated velocities from vision target data

        Returns:
            Tuple(vX, vY, vT): X, Y, and rotation velocities as a tuple.
        """
        return (self.drive_vX, self.drive_vY, self.drive_vRotate)

    def hasGrabberTarget(self):
        return self.tv

    def execute(self) -> None:
        self.getTarget()

    def setGrabberPipeline(self, objectInt: int):
        self.ll_grabberTable.putNumber("pipeline", objectInt)

    def setUpperPipeline(self, pipeNum: int):
        self.ll_upperTable.putNumber("pipeline", pipeNum)


if __name__ == "__main__":
    pass

import os
import typing

from commands2 import SubsystemBase
from photonvision import PhotonCamera, PoseStrategy, RobotPoseEstimator
from robotpy_apriltag import AprilTagFieldLayout
from wpimath.geometry import (Pose3d, Quaternion, Rotation3d, Transform3d,
                              Translation3d)

apriltagsFilename = r"apriltags_layout.json"
# get the dir of THIS file (vision.py), go up one level (..), and use the specified filename
apriltagsLayoutPath = os.path.join(os.path.dirname(__file__), r"..", apriltagsFilename)


class FROGPhotonVision(SubsystemBase):
    def __init__(self):
        super().__init__()
        self.camera = PhotonCamera("IMX219")
        self.fieldLayout = AprilTagFieldLayout(apriltagsLayoutPath)
        self.poseEstimator = RobotPoseEstimator(
            self.fieldLayout,
            PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
            [
                (
                    self.camera,
                    Transform3d(
                        Translation3d(10, 0, 6), Rotation3d(Quaternion(1, 0, 0, 0))
                    ),
                ),
            ],
        )
        self.previousEstimatedRobotPose = None

    def getEstimatedRobotPose(self) -> typing.Tuple[Pose3d, float]:
        if self.previousEstimatedRobotPose:
            self.poseEstimator.setReferencePose(self.previousEstimatedRobotPose)
        self.previousEstimatedRobotPose, self.time = self.poseEstimator.update()
        return (self.previousEstimatedRobotPose, self.time)

from wpimath.geometry import Pose3d, Rotation3d, Transform3d, Translation3d
from wpimath.units import inchesToMeters
from robotpy_apriltag import AprilTagFieldLayout
import os, wpilib, logging

apriltagsFilename = r"apriltags_layout.json"
# get the dir of THIS file (vision.py), go up one level (..), and use the specified filename
apriltagsLayoutPath = os.path.join(os.path.dirname(__file__), r"..", apriltagsFilename)

X_CHANGE = inchesToMeters(31)
Y_CHANGE = inchesToMeters(22.25)
Z_CHANGE = 0.462788

RED_ALLIANCE = wpilib.DriverStation.Alliance.kRed
BLUE_ALLIANCE = wpilib.DriverStation.Alliance.kBlue

botPositionsFromTag = (
    Transform3d(
        Translation3d(X_CHANGE, -Y_CHANGE, -Z_CHANGE),
        Rotation3d(0, 0, 0)
    ),
    Transform3d(
        Translation3d(X_CHANGE, 0, -Z_CHANGE),
        Rotation3d(0, 0, 0)
    ),
    Transform3d(
        Translation3d(X_CHANGE, Y_CHANGE, -Z_CHANGE),
        Rotation3d(0, 0, 0)
    )
)

# tag list ordered by view from driverstation of the alliance color
# ( rightTagID, MiddleTagID, LeftTagID)
blueTagList = (8, 7, 6)
redTagList = (3, 2, 1)

def getAlliance():
    return wpilib.DriverStation.getAlliance()



class FROGFieldLayout(AprilTagFieldLayout):
    def __init__(self):
        self.logger = logging.getLogger("FROGFieldLayout")
        super().__init__(apriltagsLayoutPath)
        self.alliance = wpilib.DriverStation.Alliance.kInvalid
        #set layout to be specific to the alliance end
   
    def getTagtoRobotTransform(self, fieldPose: Pose3d, tagID:int) -> Transform3d:
        return fieldPose - self.getTagPose(tagID)
    # get position
    def getTagRelativePosition(self, tagID: int, position: int) -> Pose3d:
        return self.getTagPose(tagID) + botPositionsFromTag[position-1]

    def getGridRelativePosition(self, gridNum: int, position: int) -> Pose3d:
        return self.getTagRelativePosition( self.tagList[gridNum-1], position )
    # set alliance/change origin
    def setAlliance(self, alliance = getAlliance()):
        self.logger.info(f"FROGFieldLayout.setAlliance() called with {alliance}")
        if alliance == RED_ALLIANCE:
            self.setOrigin(self.OriginPosition.kRedAllianceWallRightSide)
            self.tagList = redTagList
            self.alliance = RED_ALLIANCE
        elif alliance == BLUE_ALLIANCE:
            self.setOrigin(self.OriginPosition.kBlueAllianceWallRightSide)
            self.tagList = blueTagList
            self.alliance = BLUE_ALLIANCE
    


if __name__ == '__main__':
    pass
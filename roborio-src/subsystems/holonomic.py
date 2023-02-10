from wpimath.trajectory import TrajectoryGenerator, TrajectoryConfig, TrapezoidProfileRadians
from wpimath.geometry import Pose2d, Translation2d, Transform2d, Rotation2d
from wpimath.units import feetToMeters
from wpimath.controller import PIDController, ProfiledPIDControllerRadians, HolonomicDriveController
from subsystems.drivetrain import SwerveChassis
import math
from wpilib import Timer
from typing import List

MAX_TRAJECTORY_SPEED = feetToMeters(5)
MAX_TRAJECTORY_ACCEL = feetToMeters(5)


class FROGHolonomicController(HolonomicDriveController):
    def __init__(self, kinematics):
        # the holonomic controller
        self.kinematics = kinematics
        self.xController = PIDController(1, 0, 0)
        self.yController = PIDController(1, 0, 0)
        self.angleController = ProfiledPIDControllerRadians(1, 0, 0, TrapezoidProfileRadians.Constraints(math.pi, math.pi))
        self.angleController.enableContinuousInput(-1*math.pi, math.pi)
        super().__init__(self.xController, self.yController, self.angleController)
        self.timer = Timer()



    def initTrajectory(self, startPose: Pose2d, wayPoints: list[Translation2d], endPose: Pose2d ):
        # the trajectory setup
        trajectoryConfig = TrajectoryConfig(MAX_TRAJECTORY_SPEED, MAX_TRAJECTORY_ACCEL)
        trajectoryConfig.setKinematics(self.kinematics)
        self.trajectory = TrajectoryGenerator.generateTrajectory(
			Pose2d(0, 0, Rotation2d.fromDegrees(0)), # Starting position
			[Translation2d(1,1), Translation2d(2,-1)], # Pass through these points
			Pose2d(3, 0, Rotation2d.fromDegrees(0)), # Ending position
			trajectoryConfig
        )

    def getChassisSpeeds(self, chassisPose: Pose2d, finalRotation2d: Rotation2d):
        # get the pose of the trajectory at the current time
        goalPose = self.trajectory.sample(self.timer.get())
        return self.calculate(chassisPose, goalPose, finalRotation2d)



    
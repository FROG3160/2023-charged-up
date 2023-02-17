from magicbot import AutonomousStateMachine, timed_state, state
from logging import Logger
from components.drivetrain import SwerveChassis

class moveToGridRight(AutonomousStateMachine):
    MODE_NAME = 'Move to Grid Right'

    swerveChassis: SwerveChassis

    @state(first=True)
    def loadTrajectory(self, initial_call):
        if initial_call:
            self.swerveChassis.holonomicController.loadPathPlanner('mid-grid_charging-right')
        self.next_state("runTrajectory")

    @state
    def runTrajectory(self):
        self.swerveChassis.autoDrive()
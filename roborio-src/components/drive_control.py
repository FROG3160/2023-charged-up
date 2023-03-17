from magicbot import state, timed_state, feedback, tunable, default_state
from magicbot.state_machine import StateMachine
from components.drivetrain import SwerveChassis


class driveControl(StateMachine):
    
    drivetrain: SwerveChassis

    def __init__(self) -> None:
        self.vX = 0
        self.vY = 0
        self.vT = 0
        self.throttle = 0
        pass

    # State fieldOriented (as the default state) This will be the first state.
    @default_state()
    def fieldOriented(self):
        self.drivetrain.fieldOrientedDrive(self.vX, self.vY, self.vT, self.throttle)

    # State robotOriented (for driving to cones, cubes and posts).
    @state()
    def robotOriented(self):
        self.drivetrain.robotOrientedDrive(self.vX, self.vY, self.vT, self.throttle)

    # State locked (for turning the wheels in to keep it from moving).
    @state()
    def locked(self):
        self.drivetrain.lockChassis(self.vX, self.vY, self.vT, self.throttle)

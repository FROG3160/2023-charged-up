from magicbot.state_machine import StateMachine
from magicbot import state, timed_state, feedback, tunable, default_state
from components.arm import Arm
from components.grabber import FROGGrabber
from components.vision import FROGLimeLightVision
import config


class ArmControl(StateMachine):
    # need arm
    arm: Arm

    def __init__(self):
        self.last_state = None
        pass

    def setNextState(self, state):
        self.next_state(state)

    def stop(self):
        self.arm.manual(0, 0)

    @state(must_finish=True)
    def leaveZero(self):
        self.arm.leaveZero()
        self.next_state('atHome')

    # first state at Home
    @state()
    def atHome(self):
        pass
        # if self.commandedState:
        #     self.arm.leaveZero()
        # if self.arm.boom.getPosition() > 1000 and self.arm.stick.getPosition() > 1000:
        #     self.next_state(self.commandedState)

    # state moving to Home
    # need to clear mid post move boom until X
    # then allow stick to move down
    @state(first=True)
    def moveToHome(self, initial_call):
        if self.arm.boom.getPosition() > config.BOOM_FLOOR_PICKUP:
            self.arm.boom.toPosition(config.BOOM_GRID_MID)
            block = True
        else:
            self.arm.runToPosition(config.BOOM_HOME, config.STICK_HOME)
            block = False
        if self.arm.atPosition and not block:
            self.next_state("atHome")

    @state()
    def zeroHome(self, initial_call):
        if initial_call:
            self.arm.runToZero()
        if self.arm.atReverseLimit():
            self.next_state('atHome')


    @state()
    def moveToFloor(self):
        # If the boom is too far forward, the stick might hit the floor,
        # so make sure it's back to the "mid" position before completing.
        if self.arm.boom.getPosition() > config.BOOM_FLOOR_PICKUP:
            self.arm.boom.toPosition(config.BOOM_GRID_MID)
            block = True
        else:
            self.arm.runToPosition(config.BOOM_FLOOR_PICKUP, config.STICK_FLOOR_PICKUP)
            block = False
        if self.arm.atPosition and not block:
            self.next_state("atFloor")

    # state at floor
    @state()
    def atFloor():
        pass

    @state()
    def moveToManipulate(self):
        if self.arm.boom.getPosition() > config.BOOM_FLOOR_PICKUP:
            self.arm.boom.toPosition(config.BOOM_GRID_MID)
            block = True
        else:
            self.arm.runToPosition(
                config.BOOM_FLOOR_MANIPULATE, config.STICK_FLOOR_MANIPULATE
            )
            block = False
        if self.arm.atPosition and not block:
            self.next_state("atManipulate")

    @state()
    def atManipulate(self):
        pass

    @state()
    def moveToMid(self, initial_call):
        self.arm.runToPosition(config.BOOM_GRID_MID, config.STICK_GRID_MID)
        if self.arm.atPosition():
            self.next_state("atMid")

    @state()
    def atMid(self):
        pass

    @state()
    def moveToShelf(self, initial_call):
        self.arm.runToPosition(config.BOOM_SHELF, config.STICK_SHELF)
        if self.arm.atPosition():
            self.next_state("atShelf")

    @state()
    def atShelf(self):
        pass

    @state()
    def moveToUpper(self):
        if self.arm.stick.getPosition() < config.STICK_GRID_MID - 20480:
            self.arm.runToPosition(config.BOOM_GRID_MID, config.STICK_GRID_MID)
            block = True
        else:
            self.arm.runToPosition(config.BOOM_GRID_UPPER, config.STICK_GRID_UPPER)
            block = False
        if self.arm.atPosition and not block:
            self.next_state("atUpper")

    @state()
    def atUpper(self):
        pass


class GrabberControl(StateMachine):
    grabber: FROGGrabber
    limelight: FROGLimeLightVision
    armControl: ArmControl

    def __init__(self):
        pass

    @state(first=True)
    def holding(self):
        pass

    @state()
    def dropping(self, initial_call):
        if initial_call:
            self.grabber.open()
            if self.grabber.sensor.isCube():
                self.grabber.wheelsOn(-0.5)
        if self.grabber.getProximity() < 220:
            self.next_state("intakeWait")

    @timed_state(duration = 1, next_state = "stopEject")
    def intakeWait(self):
        pass

    @state()
    def stopEject(self):
        self.grabber.wheelsOff()
        self.next_state("empty")

    @state()
    def empty(self):
        pass

    @state()
    def looking(self, initial_call):
        if initial_call:
            self.grabber.open()
        if self.limelight.hasTarget():
            self.next_state("intaking")

    @state()
    def intaking(self):
        self.logger.info(f"Intaking, area = {self.limelight.ta}")
        if self.limelight.ta is not None:
            if self.limelight.ta >= 30:
                if self.grabber.getProximity() < 1000:
                    self.logger.info("Turning intake wheels on")
                    self.grabber.wheelsOn(1)
                self.next_state("grabbing")
        else:
            self.next_state("looking")

    @state()
    def grabbing(self):
        if self.grabber.getProximity() > 230:
            self.logger.info("Closing grabber")
            self.grabber.close()
            self.next_state("lifting")

    @state()
    def lifting(self, initial_call):
        if initial_call and not self.armControl.arm.stick.getPosition() > config.STICK_FLOOR_PICKUP+ 20480:
            self.armControl.next_state('moveToHome')
        else:
            self.next_state("stoppingIntake")
        self.armControl.engage()
        if self.armControl.arm.atPosition:
            self.next_state("stoppingIntake")

    @state()
    def stoppingIntake(self):
        if self.grabber.getProximity() > 1000:
            self.logger.info("Shutting down intake wheels")
            self.grabber.wheelsOff()
            self.next_state("holding")

from magicbot.state_machine import StateMachine
from magicbot import state, timed_state, feedback, tunable, default_state
from components.arm import Arm
from components.grabber import FROGGrabber
from components.vision import FROGLimeLightVision, LL_CONE, LL_CUBE
from components.drivetrain import SwerveChassis
from wpimath.geometry import Rotation2d, Transform2d, Pose2d
from components.led import FROGLED
# from components.led_control import LedControl
import config
from wpilib.shuffleboard import Shuffleboard
from enum import StrEnum

class ArmStates(StrEnum):
    MOVING_TO_HOME = 'movingToHome'
    MOVING_TO_SHELF = 'movingToShelf'
    MOVING_TO_UPPER = 'movingToUpper'
    MOVING_TO_FLOOR = 'movingToFloor'
    MOVING_TO_MANIPULATE = 'movingToManipulate'



class ArmControl(StateMachine):
    # need arm
    arm: Arm
    grabber: FROGGrabber

    def __init__(self):
        self.last_state = None
        pass

    def setup(self):
        armTab = Shuffleboard.getTab("Arm")
        armTab.add(title="Boom Motor", defaultValue=self.arm.boom.motor)
        armTab.add(title="Stick Motor", defaultValue=self.arm.stick.motor)

    def setNextState(self, state):
        self.next_state(state)

    def stop(self):
        self.arm.manual(0, 0)

    def moveToHome(self):
        self.engage(initial_state=ArmStates.MOVING_TO_HOME)
    
    def moveToShelf(self):
        self.engage(initial_state=ArmStates.MOVING_TO_SHELF)

    def moveToUpper(self):
        self.engage(initial_state=ArmStates.MOVING_TO_UPPER)

    def moveToFloor(self):
        self.engage(initial_state=ArmStates.MOVING_TO_FLOOR)

    def moveToManipulate(self):
        self.engage(initial_state=ArmStates.MOVING_TO_MANIPULATE)

    # @state(must_finish=True)
    # def leaveZero(self):
    #     self.arm.leaveZero()
    #     self.next_state('atHome')

    # first state at Home
    @state()
    def atHome(self):
        self.last_state = 'atHome'
        # if self.commandedState:
        #     self.arm.leaveZero()
        # if self.arm.boom.getPosition() > 1000 and self.arm.stick.getPosition() > 1000:
        #     self.next_state(self.commandedState)

    @feedback
    def isAtHome(self) -> bool:
        return self.arm.getArmPosition() == 'home'

    # state moving to Home
    # need to clear mid post move boom until X
    # then allow stick to move down
    @state(first=True)
    def movingToHome(self, initial_call):
        if self.arm.boomExtended():
            self.arm.retractBoom()
            block = True
        else:
            self.arm.runToPosition('home')
            block = False
        if self.arm.getArmPosition() == 'home' and not block:
            self.next_state("atHome")

    @state()
    def zeroHome(self, initial_call):
        if initial_call:
            self.arm.runToZero()
        if self.arm.atReverseLimit():
            self.next_state('atHome')


    @state()
    def movingToFloor(self):
        # If the boom is too far forward, the stick might hit the floor,
        # so make sure it's back to the "mid" position before completing.
        self.dropPlateIfHome()
        if self.arm.boomExtended():
            self.arm.retractBoom()
            block = True
        else:
            self.arm.runToPosition('floor')
            block = False
        if self.arm.getArmPosition == 'floor' and not block:
            self.next_state("atFloor")

    # state at floor
    @state()
    def atFloor(self):
        self.last_state = 'atFloor'
        pass

    @state()
    def movingToManipulate(self):
        self.dropPlateIfHome()
        if self.arm.boomExtended():
            self.arm.retractBoom()
            block = True
        else:
            self.arm.runToPosition(
                'manipulate'
            )
            block = False
        if self.arm.getArmPosition() == 'manipulate' and not block:
            self.next_state("atManipulate")

    @state()
    def atManipulate(self):
        self.last_state='atManipulate'
        pass

    @state()
    def movingToMid(self, initial_call):
        self.dropPlateIfHome()
        self.arm.runToPosition('shelf')
        if self.arm.getArmPosition() == 'shelf':
            self.next_state("atMid")

    @state()
    def atMid(self):
        self.last_state = 'atMid'
        pass

    @state()
    def movingToShelf(self, initial_call):
        self.dropPlateIfHome()
        self.arm.runToPosition('shelf')
        if self.arm.getArmPosition() == 'shelf':
            self.next_state("atShelf")

    @state()
    def atShelf(self):
        self.last_state = 'atShelf'
        pass

    @state()
    def movingToUpper(self):
        self.dropPlateIfHome()
        if not self.arm.stickRaised():
            self.arm.runToPosition('shelf')
            block = True
        else:
            self.arm.runToPosition('upper')
            block = False
        if self.arm.getArmPosition() == 'upper' and not block:
            self.next_state("atUpper")

    @state()
    def atUpper(self):
        self.last_state = 'atUpper'
        pass

    def dropPlateIfHome(self):
        if self.arm.getArmPosition() == 'home':
            self.grabber.plateDown()

    def getArmPosition(self):
        return self.arm.getArmPosition()

    


class GrabberControl(StateMachine):
    grabber: FROGGrabber
    limelight: FROGLimeLightVision
    armControl: ArmControl
    swerveChassis: SwerveChassis
    leds: FROGLED
    last_state = None
    targetPresent = False
    hasObject = False
    deploySize = tunable(20.0)

    VERBOSE_LOGGING = True

    def __init__(self):
        self.coneSupport = True

    def disableConeSupport(self):
        self.coneSupport = False

    def enableConeSupport(self):
        self.coneSupport = True

    @state(first=True)
    def holding(self):
        self.grabber.closeJaws()
        self.last_state = 'holding'

    @state()
    def dropping(self, initial_call):
        if initial_call:
            self.grabber.openJaws()
            if self.grabber.sensor.isCube():
                self.grabber.wheelsOn(-0.5)
        #waiting until the object is nearly out
        if self.grabber.getProximity() < 220:
            self.last_state = 'dropping'
            self.next_state("intakeWait")

    

    @timed_state(duration = 0.5, next_state = "stopEject")
    def intakeWait(self):
        self.last_state = 'intakeWait'

    @state()
    def stopEject(self):
        self.grabber.wheelsOff()
        self.hasObject = False
        self.last_state = 'stopEject'
        self.next_state("looking")

    @state()
    def empty(self):
        self.last_state = 'empty'
    
    @state()
    def reset(self):
        self.grabber.wheelsOff()
        self.grabber.openJaws()
        self.last_state = 'reset'
        self.next_state_now('looking')

    @state()
    def looking(self, initial_call):
        if initial_call:
            self.grabber.openJaws()
        if self.limelight.getGrabberPipeline() == LL_CONE:
            self.leds.yellowPocketSlow()
        else:
            self.leds.purplePocketSlow()
        if self.limelight.hasGrabberTarget():
            self.next_state("intaking")
        elif self.grabber.getProximity() > 230:
            self.grabber.wheelsOn(1)
            self.last_state = 'looking'
            self.next_state("grabbing")

    @state()
    def intaking(self):

        # self.logger.info(f"Intaking, area = {self.limelight.ta}")
        if self.limelight.ta is not None:
            if self.limelight.getGrabberPipeline() == LL_CONE:
                self.leds.yellowPocketFast()
            else:
                self.leds.purplePocketFast()
            self.targetPresent = True
            if self.limelight.ta >= 30:
                if self.grabber.getProximity() < 1000:
                    self.grabber.wheelsOn(1)
                self.last_state = 'intaking'
                self.next_state("grabbing")
        else:
            self.targetPresent = False
            self.last_state = 'intaking'
            self.next_state("looking")

    @state()
    def grabbing(self):
        if self.grabber.getProximity() < 1000:
            self.grabber.wheelsOn(1)
        if self.grabber.getProximity() > 230:
            self.grabber.closeJaws()
            self.last_state = 'grabbing'
            self.hasObject = True
            self.next_state("retracting")

    @state()
    def retracting(self):
        self.setLedsToHeldObject()
        if self.armControl.getArmPosition() == 'floor':
            self.armControl.done()# resetting state machine mostly for autonomous 
            self.next_state("lifting")
        else:
            self.next_state('waitForPullback')

    @state()
    def waitForPullback(self, initial_call):
        if initial_call:
            self.xPosition = self.swerveChassis.estimator.getEstimatedPosition().X()
        if self.grabber.getProximity() > 1000:
            self.grabber.wheelsOff()
        if self.xPosition - self.swerveChassis.estimator.getEstimatedPosition().X() > 0.25:
            self.armControl.moveToHome()
        if self.armControl.getArmPosition() == 'home':
            self.next_state('stoppingIntake')


    @state()
    def lifting(self, initial_call):
        if self.grabber.getProximity() > 1000:
            self.grabber.wheelsOff()
        self.armControl.moveToHome()
        if self.armControl.getArmPosition() == 'home':
            self.armControl.done()
            self.last_state = 'lifting'
            self.next_state("stoppingIntake")

    @state()
    def stoppingIntake(self):
        self.setLedsToHeldObject()
        if self.grabber.getProximity() > 1000:
            self.grabber.wheelsOff()
        self.last_state = 'stoppingIntake'
        self.targetPresent = False
        # self.next_state("holding")
        if self.grabber.sensor.isCube():
            self.next_state("holding")
        elif self.armControl.isAtHome():
            self.next_state('supportCone')
    
    @state()
    def supportCone(self):
        if self.coneSupport:
            self.grabber.plateUp()
            self.next_state('releaseCone')
        else:
            self.next_state('holding')

    @timed_state(duration=0.12, next_state = 'holding')
    def releaseCone(self):
        self.grabber.openJaws()


    @state()
    def waitForMove(self, initial_call):
        #TODO: Figure out the calculation WRT which way the robot is facing
        # transformBy(Transform2d(2, 0, 3.14159) does it, with 3.14.159 being
        # 180 degrees from current gyro heading, and says move 2 meters in
        # the opposite direction of the heading
        if initial_call:
            startPose = self.swerveChassis.estimator.getEstimatedPosition()
            endPose = startPose.transformBy(Transform2d(0.5, 0, Rotation2d.fromDegrees(180)))

        #now we need to keep getting the change from the startPose and once it's good enough
        # drop arm
        
    def setLedsToHeldObject(self):
        if self.grabber.sensor.isCube():
            self.leds.purple()
        else:
            self.leds.yellow()

    def look(self):
        self.engage(initial_state='looking')
        
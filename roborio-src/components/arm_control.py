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
        return self.arm.getArmPosition == 'home' and self.arm.atPosition == True

    # state moving to Home
    # need to clear mid post move boom until X
    # then allow stick to move down
    @state(first=True)
    def moveToHome(self, initial_call):
        if self.arm.boomExtended():
            self.arm.retractBoom()
            block = True
        else:
            self.arm.runToPosition('home')
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
        self.checkFromHome()
        if self.arm.boomExtended():
            self.arm.retractBoom()
            block = True
        else:
            self.arm.runToPosition('floor')
            block = False
        if self.arm.atPosition and not block:
            self.next_state("atFloor")

    # state at floor
    @state()
    def atFloor(self):
        self.last_state = 'atFloor'
        pass

    @state()
    def moveToManipulate(self):
        self.checkFromHome()
        if self.arm.boomExtended():
            self.arm.retractBoom()
            block = True
        else:
            self.arm.runToPosition(
                'manipulate'
            )
            block = False
        if self.arm.atPosition and not block:
            self.next_state("atManipulate")

    @state()
    def atManipulate(self):
        self.last_state='atManipulate'
        pass

    @state()
    def moveToMid(self, initial_call):
        self.checkFromHome()
        self.arm.runToPosition('shelf')
        if self.arm.atPosition():
            self.next_state("atMid")

    @state()
    def atMid(self):
        self.last_state = 'atMid'
        pass

    @state()
    def moveToShelf(self, initial_call):
        self.checkFromHome()
        self.arm.runToPosition('shelf')
        if self.arm.atPosition():
            self.next_state("atShelf")

    @state()
    def atShelf(self):
        self.last_state = 'atShelf'
        pass

    @state()
    def moveToUpper(self):
        self.checkFromHome()
        if not self.arm.stickRaised():
            self.arm.runToPosition('shelf')
            block = True
        else:
            self.arm.runToPosition('upper')
            block = False
        if self.arm.atPosition and not block:
            self.next_state("atUpper")

    @state()
    def atUpper(self):
        self.last_state = 'atUpper'
        pass

    def checkFromHome(self):
        if self.last_state == 'atHome':
            self.grabber.plateDown()


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

    def __init__(self):
        pass

    @state(first=True)
    def holding(self):
        self.grabber.closeJaws()
        self.checkHeldObject()
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
        self.logger.info(f'Looking -- pipeline is: {self.limelight.getGrabberPipeline()}')
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
            self.logger.info(f'Intaking -- pipeline is: {self.limelight.getGrabberPipeline()}')
       
            if self.limelight.getGrabberPipeline() == LL_CONE:
                self.leds.yellowPocketFast()
            else:
                self.leds.purplePocketFast()
            self.targetPresent = True
            if self.limelight.ta >= 30:
                if self.grabber.getProximity() < 1000:
                    self.logger.info("Turning intake wheels on")
                    self.grabber.wheelsOn(1)
                self.last_state = 'intaking'
                self.next_state("grabbing")
        else:
            self.targetPresent = False
            self.last_state = 'intaking'
            self.next_state("looking")

    @state()
    def grabbing(self):
        if self.grabber.getProximity() > 230:
            self.logger.info("Closing grabber")
            self.grabber.closeJaws()
            self.last_state = 'grabbing'
            self.hasObject = True
            self.next_state("lifting")

    @state()
    def lifting(self, initial_call):
        # TODO:  set a block when moving the arm to shelf position
        # remove block when setting to floor position
        # maybe have block occur UNLESS it's at floor for pickup
        # have a pickup boolean?
        self.checkHeldObject()
        if initial_call and self.armControl.last_state == 'atFloor':
        #if initial_call and not self.armControl.arm.stick.getPosition() > config.STICK_FLOOR_PICKUP+ 20480:
            self.last_state = 'lifting'
            self.armControl.next_state('moveToHome')
            block = False
        else:
            self.last_state = 'lifting'
            self.next_state("stoppingIntake")
            block = True
        if not block:
            self.armControl.engage()
            if self.armControl.last_state == 'atHome':
                self.last_state = 'lifting'
                self.next_state("stoppingIntake")

    @state()
    def stoppingIntake(self):
        self.checkHeldObject()
        if self.grabber.getProximity() > 1000:
            self.logger.info(f'stoppingIntake: Checking object: isCube = {self.grabber.sensor.isCube()}')
            self.logger.info("stoppingIntake: Shutting down intake wheels")
            self.grabber.wheelsOff()
            self.last_state = 'stoppingIntake'
            self.targetPresent = False
            # self.next_state("holding")
            if self.grabber.sensor.isCube():
                self.next_state("holding")
                self.logger.info('stoppingIntake: Cube found')
            elif self.armControl.isAtHome():
                self.next_state('supportCone')
                self.logger.info('stoppingIntake: Cone found, arm at home')
    
    @state()
    def supportCone(self):
        self.grabber.plateUp()
        self.next_state('releaseCone')

    @timed_state(duration=0.5, next_state = 'holding')
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
        
    def checkHeldObject(self):
        if self.grabber.sensor.isCube():
            self.leds.purple()
        else:
            self.leds.yellow()
        

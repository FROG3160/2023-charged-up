from ctre import WPI_TalonFX, ControlMode, NeutralMode, TalonFXConfiguration
import config
import logging
from magicbot import tunable, feedback


class SubArm():
    def __init__(
        self,
        name: str,
        motorID: int,
        motorConfig: TalonFXConfiguration,
        neutralMode: NeutralMode,
        logger,
        positions,
        motorInverted=False,
        sensorPhase = False,
    ):
        self.name = name
        self.motor = WPI_TalonFX(motorID)
        self.motor.configAllSettings(motorConfig)
        self.motorConfig = motorConfig
        self.motor.setNeutralMode(neutralMode)
        self.motor.setInverted(motorInverted)
        self.motor.setSensorPhase(sensorPhase)
        
        self.positions = positions

        self.targetPosition = 'home'
        self.currentPosition = 'home'
        self.logger = logger


    def run(self, speed: float):
        self.motor.set(speed)

    def setPosition(self, positionName):
        self.targetPosition = positionName
        self.currentPosition = 'invalid'
        self.motor.set(ControlMode.MotionMagic, self.getTargetPositionValue())

    def stop(self):
        self.motor.set(0)

    def getTrajectoryPosition(self):
        return self.motor.getActiveTrajectoryPosition()

    def getEncoderPosition(self):
        return self.motor.getSelectedSensorPosition()

    def atRevLimit(self):
        return self.motor.isRevLimitSwitchClosed()
    
    def getTargetPositionValue(self):
        return self.positions[self.targetPosition]
    
    def atTarget(self):
        if abs(self.motor.getActiveTrajectoryPosition() - self.getTargetPositionValue()) < 200:
            return True
        elif self.targetPosition == 'home' and self.atRevLimit():
            return True
        else:
            return False

    
    def execute(self):
        #Check to see if we are running Motion Magic
        if self.targetPosition is not None:
            if self.atTarget():
                # self.logger.info(f"SubArm: {self.name} - Stopped at {self.motor.getActiveTrajectoryPosition()} with commanded Position: {self.commandedPosition}")
                self.currentPosition = self.targetPosition
                self.stop()
           
        #shut down motor if we are moving back to home with PercentOutput control and we hit the limit switch
        if self.atRevLimit() and self.motor.get() < 0 and self.motor.getControlMode() == ControlMode.PercentOutput:
            self.stop()
            

        
            


class Arm():

    stickRaisedOffset = tunable(40000)

    def __init__(self) -> None:

        self.logger = logging.getLogger("FROGArm")
        self.boom = SubArm("Boom", config.BOOM_MOTOR_ID, config.cfgBoomMotor, NeutralMode.Brake, logger = self.logger, positions = config.boomPositions)
        self.stick = SubArm("Stick", config.STICK_MOTOR_ID, config.cfgStickMotor, NeutralMode.Brake, logger = self.logger, positions = config.stickPositions)

    def manual(self, boomSpeed, stickSpeed):
        self.boom.run(boomSpeed)
        self.stick.run(stickSpeed)
    
    def runToZero(self):
        self.manual(-0.15, -0.15)

    # def leaveZero(self):
    #     self.boom.motor.set(ControlMode.Position, 256)
    #     self.stick.motor.set(ControlMode.Position, 256)

    def runToPosition(self, position: str):
        self.boom.setPosition(position)
        self.stick.setPosition(position)

    def retractBoom(self):
        self.boom.setPosition('shelf')

    @feedback
    def getArmPosition(self):
        if self.boom.currentPosition == self.stick.currentPosition:
            return self.boom.currentPosition
        else:
            return 'unknown'

    @feedback
    def isAtHome(self):
        return self.boom.atRevLimit() and self.stick.atRevLimit()
    
    @feedback
    def atReverseLimit(self):
        return self.boom.atRevLimit() and self.stick.atRevLimit()
    
    @feedback
    def boomExtended(self):
        # check to see if the boom is extended too far to bring
        # the stick down
        return self.boom.getEncoderPosition() > config.BOOM_FLOOR_PICKUP
    
    @feedback
    def stickRaised(self):
        # check to see if the stick is up high enough to extend
        # boom without hitting the grid
        return self.stick.getEncoderPosition() > config.STICK_SHELF - self.stickRaisedOffset

    def execute(self):
        self.boom.execute()
        self.stick.execute()

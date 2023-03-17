from ctre import WPI_TalonFX, ControlMode, NeutralMode, TalonFXConfiguration
import config
import logging


class SubArm():
    def __init__(
        self,
        name: str,
        motorID: int,
        motorConfig: TalonFXConfiguration,
        neutralMode: NeutralMode,
        logger,
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
        self.commandedPosition = None
        self.logger = logger
        self.atPosition = False

    def getEncoderPosition(self) -> float:
        return self.motor.getSelectedSensorPosition()

    def run(self, speed: float):
        self.motor.set(speed)

    def toPosition(self, position: float):
        self.commandedPosition = position
        self.atPosition = False
        self.motor.set(ControlMode.MotionMagic, position)

    def stop(self):
        self.motor.set(0)

    def getTrajectoryPosition(self):
        return self.motor.getActiveTrajectoryPosition()

    def getPosition(self):
        return self.motor.getSelectedSensorPosition()

    def atRevLimit(self):
        return self.motor.isRevLimitSwitchClosed()
    
    def execute(self):
        #Check to see if we are running Motion Magic
        if self.commandedPosition is not None:
            if abs(self.motor.getActiveTrajectoryPosition() - self.commandedPosition) < 200:
                # self.logger.info(f"SubArm: {self.name} - Stopped at {self.motor.getActiveTrajectoryPosition()} with commanded Position: {self.commandedPosition}")
                self.commandedPosition = None
                self.atPosition = True
                self.stop()
        if self.atRevLimit() and self.motor.get() < 0 and self.motor.getControlMode() == ControlMode.PercentOutput:
            self.stop()
            

        
            


class Arm():
    def __init__(self) -> None:

        self.logger = logging.getLogger("FROGArm")
        self.boom = SubArm("Boom", config.BOOM_MOTOR_ID, config.cfgBoomMotor, NeutralMode.Brake, logger = self.logger)
        self.stick = SubArm("Stick", config.STICK_MOTOR_ID, config.cfgStickMotor, NeutralMode.Brake, logger = self.logger)

    def manual(self, boomSpeed, stickSpeed):
        self.boom.run(boomSpeed)
        self.stick.run(stickSpeed)
    
    def runToZero(self):
        self.manual(-0.15, -0.15)

    def leaveZero(self):
        self.boom.motor.set(ControlMode.Position, 256)
        self.stick.motor.set(ControlMode.Position, 256)

    def runToPosition(self, boomPosition, stickPosition):
        self.boom.toPosition(boomPosition)
        self.stick.toPosition(stickPosition)

    def atPosition(self):
        return self.boom.atPosition and self.stick.atPosition
    
    def atReverseLimit(self):
        return self.boom.atRevLimit() and self.stick.atRevLimit()

    def execute(self):
        self.boom.execute()
        self.stick.execute()

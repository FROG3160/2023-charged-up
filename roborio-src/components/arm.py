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

    def getEncoderPosition(self) -> float:
        return self.motor.getSelectedSensorPosition()

    def run(self, speed: float):
        # TODO: Determine the actual position the arm is moving in based on
        # running the motor forward and backward.
        self.motor.set(speed)

    def toPosition(self, position: float):
        self.commandedPosition = position
        self.motor.set(ControlMode.MotionMagic, position)

    def stop(self):
        self.motor.set(0)

    def atPosition(self):
        self.motor.getActiveTrajectoryPosition()
    
    def execute(self):
        #Check to see if we are running Motion Magic

        if self.commandedPosition is not None:
            if abs(self.motor.getActiveTrajectoryPosition() - self.commandedPosition) < 200:
                self.logger.info(f"SubArm: {self.name} - Stopped at {self.motor.getActiveTrajectoryPosition()} with commanded Position: {self.commandedPosition}")
                self.commandedPosition = None
                self.stop()


class Arm():
    def __init__(self) -> None:

        self.logger = logging.getLogger("FROGArm")
        self.boom = SubArm("Boom", config.BOOM_MOTOR_ID, config.cfgBoomMotor, NeutralMode.Brake, logger = self.logger)
        self.stick = SubArm("Stick", config.STICK_MOTOR_ID, config.cfgStickMotor, NeutralMode.Brake, logger = self.logger)

    def manual(self, boomSpeed, stickSpeed):
        self.boom.run(boomSpeed)
        self.stick.run(stickSpeed)

    def execute(self):
        self.boom.execute()
        self.stick.execute()
        pass

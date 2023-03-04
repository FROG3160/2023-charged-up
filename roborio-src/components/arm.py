from ctre import WPI_TalonFX, ControlMode, NeutralMode
import config


class SubArm():
    def __init__(
        self,
        motorID=int,
    ):
        self.motor = WPI_TalonFX(motorID)

    def getEncoderPosition(self) -> float:
        return self.motor.getSelectedSensorPosition()

    def run(self, speed: float):
        # TODO: Determine the actual position the arm is moving in based on
        # running the motor forward and backward.
        self.motor.set(speed)

    def toPosition(self, position: float):
        self.motor.set(ControlMode.MotionMagic, position)

    def stop(self):
        self.motor.set(0)


class Arm():
    def __init__(self, boomMotorID, stickMotorID) -> None:

        self.boom = SubArm(boomMotorID)
        self.boom.motor.configAllSettings(config.cfgBoomMotor)
        self.boom.motor.setNeutralMode(NeutralMode.Brake)
        self.boom.motor.setInverted(False)
        self.boom.motor.setSensorPhase(False)

        self.stick = SubArm(stickMotorID)
        self.stick.motor.configAllSettings(config.cfgStickMotor)
        self.stick.motor.setNeutralMode(NeutralMode.Brake)
        self.stick.motor.setInverted(False)
        self.stick.motor.setSensorPhase(False)

    def manual(self, boomSpeed, stickSpeed):
        self.boom.run(boomSpeed)
        self.stick.run(stickSpeed)

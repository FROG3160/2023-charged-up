from commands2 import SubsystemBase
from ctre import WPI_TalonFX
import config


class SubArm(SubsystemBase):
    def __init__(
        self,
        motorID=int,
    ):
        super().__init__()
        self.motor = WPI_TalonFX(motorID)

    def getEncoderPosition(self) -> float:
        return self.motor.getSelectedSensorPosition()

    def run(self, speed: float):
        # TODO: Determine the actual position the arm is moving in based on
        # running the motor forward and backward.
        self.motor.set(speed)

    def stop(self):
        self.motor.set(0)


class Arm(SubsystemBase):
    def __init__(self, boomMotorID, stickMotorID) -> None:
        super().__init__()

        self.boom = SubArm(boomMotorID)
        self.boom.motor.configAllSettings(config.cfgBoomMotor)
        self.boom.motor.setInverted(False)
        self.boom.motor.setSensorPhase(False)

        self.stick = SubArm(stickMotorID)
        self.stick.motor.configAllSettings(config.cfgStickMotor)
        self.stick.motor.setInverted(False)
        self.stick.motor.setSensorPhase(False)

    def manual(self, boomSpeed, stickSpeed):
        self.boom.run(boomSpeed)
        self.stick.run(stickSpeed)

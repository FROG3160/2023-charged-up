from commands2 import SubsystemBase
from ctre import WPI_TalonFX


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
        self.stick = SubArm(stickMotorID)

    def manual(self, boomSpeed, stickSpeed):
        self.boom.run(boomSpeed)
        self.stick.run(stickSpeed)

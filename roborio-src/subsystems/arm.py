
from commands2 import SubsystemBase
from ctre import WPI_TalonFX


class arm(SubsystemBase):
    

    def __init__(
        self,
        armMotorID = int,
    ):
        super().__init__()
        self.FROGArm = WPI_TalonFX(armMotorID)

    def getEncoderPosition(self) -> float:
        return self.FROGArm.getSelectedSensorPosition()


    def run(self, speed: float):
    # TODO: Determine the actual position the arm is moving in based on 
    # running the motor forward and backward.
        self.FROGArm.set(speed)

    def stop(self):
        self.FROGArm.set(0)

    
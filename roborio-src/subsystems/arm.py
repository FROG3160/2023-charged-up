
from commands2 import SubsystemBase
from ctre import WPI_TalonFX


class Arm(SubsystemBase):
    

    def __init__(
        self,
        motorID = int,
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

    

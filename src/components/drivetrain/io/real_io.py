from drivetrain.io.base_io import DrivetrainIOBase

from phoenix6.hardware import TalonFX
from phoenix6.canbus import CANBus

from wpimath import Pose2d, DCMotor

DC_MOTOR = DCMotor.cim(1)

class RealDrivetrainIO(DrivetrainIOBase):
    def __init__(self) -> None:
        self._left_drive = TalonFX(11, CANBus(""))
        self._right_drive = TalonFX(12, CANBus(""))

    def get_pose(self) -> Pose2d | None:
        return None

    def set_left_voltage(self, voltage: float) -> None:
        self._left_drive.setVoltage(voltage)

    def set_right_voltage(self, voltage: float) -> None:
        self._right_drive.setVoltage(voltage)

from abc import ABC, abstractmethod

from wpimath import Pose2d, DCMotor

DC_MOTOR = DCMotor.cim(1)

class DrivetrainIOBase(ABC):
    @abstractmethod
    def get_pose(self) -> Pose2d | None:
        """
        Returns the pose of the drivetrain if possible to measure, else `None`.
        """
        ...

    @abstractmethod
    def set_left_voltage(self, voltage: float) -> None:
        pass

    @abstractmethod
    def set_right_voltage(self, voltage: float) -> None:
        pass

    def update(self) -> None:
        pass

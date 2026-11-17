from abc import ABC, abstractmethod

from wpilib.simulation import DifferentialDrivetrainSim

from phoenix6.hardware import TalonFX
from phoenix6.canbus import CANBus

from wpimath import Pose2d, DifferentialDriveWheelPositions, DCMotor


TRACK_WIDTH = 0.6   # meters
DC_MOTOR = DCMotor.cim(1)
MOI = 1.0   # moment of inertia
MASS = 50   # kg
WHEEL_RADIUS = 0.06 # meters
GEAR_RATIO = 3.0
CLOCK_SPEED = 0.02


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


class SimDrivetrainIO(DrivetrainIOBase):
    def __init__(self) -> None:
        self._diff_drive = DifferentialDrivetrainSim(
            drive_motor=DC_MOTOR, 
            gearing=GEAR_RATIO,
            trackwidth=TRACK_WIDTH, 
            j=MOI, 
            mass=MASS,
            wheel_radius=WHEEL_RADIUS
        )
        self._left_voltage = 0
        self._right_voltage = 0

    def get_pose(self) -> Pose2d | None:
        return self._diff_drive.get_pose()

    def set_left_voltage(self, voltage: float) -> None:
        self._left_voltage = voltage

    def set_right_voltage(self, voltage: float) -> None:
        self._right_voltage = voltage

    def update(self) -> None:
        self._diff_drive.set_inputs(
            self._left_voltage,
            self._right_voltage
        )
        self._diff_drive.update(CLOCK_SPEED)

from abc import ABC, abstractmethod

from phoenix6.hardware import TalonFX
from phoenix6.canbus import CANBus

from components.intake.constants import IntakeConstants as const


class IntakeIOBase(ABC):
    @abstractmethod
    def set_voltage(self, voltage: float) -> None:
        ...

    @abstractmethod
    def get_voltage(self) -> float:
        ...

    def update(self) -> None:
        pass

class RealIntakeIO(IntakeIOBase):
    def __init__(self) -> None:
        self._motor = TalonFX(const.CAN_ID, CANBus(""))

    def set_voltage(self, voltage: float) -> None:
        self._motor.setVoltage(voltage)

    def get_voltage(self) -> float:
        return self._motor.get_motor_voltage().value

class SimIntakeIO(IntakeIOBase):
    def __init__(self) -> None:
        self._voltage = 0

    def set_voltage(self, voltage: float) -> None:
        self._voltage = voltage

    def get_voltage(self) -> float:
        return self._voltage
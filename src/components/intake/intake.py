from magicbot import will_reset_to
from magicbot.magiccomponent import MagicComponent
from wpilib import RobotController
import telemetry

from components.intake.io import IntakeIOBase

class Intake(MagicComponent):
    intake_io: IntakeIOBase

    _percent = will_reset_to(0.0)

    def setup(self):
        telemetry.log("Intake Voltage", self.intake_io.get_voltage())

    def set_percent(self, percent: float) -> None:
        self._percent = percent

    def execute(self) -> None:
        self.intake_io.set_voltage(self._percent * RobotController.get_battery_voltage())
        telemetry.log("Intake Voltage", self.intake_io.get_voltage())
from wpilib import XboxController
from magicbot import MagicRobot

from components.drivetrain.drivetrain import Drivetrain
from components.drivetrain.io import RealDrivetrainIO, SimDrivetrainIO, DrivetrainIOBase

from components.intake.intake import Intake
from components.intake.io import RealIntakeIO, SimIntakeIO, IntakeIOBase

class Robot(MagicRobot):
    drivetrain: Drivetrain
    drivetrain_io: DrivetrainIOBase
    intake_io: IntakeIOBase
    intake: Intake

    controller: XboxController

    def create_objects(self):
        self.controller = XboxController(0)

        if self.is_simulation():
            self.drivetrain_io = SimDrivetrainIO()
            self.intake_io = SimIntakeIO()
        else:
            self.drivetrain_io = RealDrivetrainIO()
            self.intake_io = RealIntakeIO()

    def teleop_periodic(self):
        self.drivetrain.drive(
            -self.controller.get_left_y(),
            -self.controller.get_right_x()
        )
        if self.controller.get_b_button() and not self.controller.get_a_button():
            self.intake.set_percent(-1.0)
        elif self.controller.get_a_button() and not self.controller.get_b_button():
            self.intake.set_percent(1.0)
        
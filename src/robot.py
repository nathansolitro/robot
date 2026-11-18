from wpilib import XboxController
from magicbot import MagicRobot
from components.drivetrain.drivetrain import Drivetrain
from components.drivetrain.io import RealDrivetrainIO, SimDrivetrainIO, DrivetrainIOBase

class Robot(MagicRobot):
    drivetrain: Drivetrain
    drivetrain_io: DrivetrainIOBase
    controller: XboxController

    def create_objects(self):
        self.controller = XboxController(0)

        if self.is_simulation():
            self.drivetrain_io = SimDrivetrainIO()
        else:
            self.drivetrain_io = RealDrivetrainIO()

    def teleop_periodic(self):
        self.drivetrain.drive(
            -self.controller.get_left_y(),
            -self.controller.get_right_x()
        )
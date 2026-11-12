from wpilib import XboxController
from magicbot import MagicRobot
from components.drivetrain import Drivetrain

class Robot(MagicRobot):
    drivetrain: Drivetrain
    controller: XboxController

    def create_objects(self):
        self.controller = XboxController(0)

    def teleop_periodic(self):
        pass
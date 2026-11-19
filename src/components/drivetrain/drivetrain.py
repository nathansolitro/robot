from magicbot import will_reset_to
from magicbot.magiccomponent import MagicComponent
from wpilib import RobotController, DifferentialDrive, Field2d
import telemetry
from lemonlib.smart.profile import SmartProfile

from components.drivetrain.io import DrivetrainIOBase

class Drivetrain(MagicComponent):
    drivetrain_io: DrivetrainIOBase

    left_controller = SmartProfile(
        "Left Controller",
        {
            "kP": 1.0,
            "kI": 0.0,
            "kD": 1.0,
        },
        tuning_enabled=True,
    )
    right_controller = SmartProfile(
        "Right Controller",
        {
            "kP": 1.0,
            "kI": 0.0,
            "kD": 1.0,
        },
        tuning_enabled=True,
    )
    forward_percent = will_reset_to(0.0)
    angular_percent = will_reset_to(0.0)

    def setup(self) -> None:
        self._field = Field2d()
        telemetry.log("Field", self._field)

    def drive(self, forward_percent: float, angular_percent: float) -> None:
        self.forward_percent = forward_percent
        self.angular_percent = angular_percent

    def execute(self) -> None:
        velocities = DifferentialDrive.arcade_drive_ik(
            self.forward_percent,
            self.angular_percent
        )
        self.drivetrain_io.set_left_voltage(velocities.left * RobotController.get_battery_voltage())
        self.drivetrain_io.set_right_voltage(velocities.right * RobotController.get_battery_voltage())
        self.drivetrain_io.update()

        pose = self.drivetrain_io.get_pose()
        if pose is not None:
            self._field.set_robot_pose(pose)
            telemetry.log("Field", self._field)

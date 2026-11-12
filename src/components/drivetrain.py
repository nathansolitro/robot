from magicbot import will_reset_to
from magicbot.magiccomponent import MagicComponent
from wpilib import RobotController, DifferentialDrive
from lemonlib.smart.profile import SmartProfile
from phoenix6.hardware import TalonFX
from phoenix6.canbus import CANBus

MAX_FORWARD_SPEED = 4.2  # m/s
MAX_ROTATION_SPEED = 8.0  # rad/s
TRACK_WIDTH = 0.6  # m

class Drivetrain(MagicComponent):
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

    def setup(self):
        self.left_drive = TalonFX(11, CANBus(""))
        self.right_drive = TalonFX(12, CANBus(""))

    def drive(self, forward_percent: float, angular_percent: float) -> None:
        self.forward_percent = forward_percent
        self.angular_percent = angular_percent

    def execute(self):
        velocities = DifferentialDrive.arcade_drive_ik(
            self.forward_percent * MAX_FORWARD_SPEED,
            self.angular_percent * MAX_ROTATION_SPEED
        )
        self.left_drive.setVoltage(velocities.left / MAX_FORWARD_SPEED * RobotController.get_battery_voltage())
        self.left_drive.setVoltage(velocities.right / MAX_FORWARD_SPEED * RobotController.get_battery_voltage())
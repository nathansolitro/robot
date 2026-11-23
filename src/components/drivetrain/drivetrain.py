from magicbot import will_reset_to
from magicbot.magiccomponent import MagicComponent
from wpilib import RobotController, Field2d
from wpimath import DifferentialDriveKinematics, ChassisVelocities


import telemetry
from lemonlib.smart.profile import SmartProfile

from components.drivetrain.io import DrivetrainIOBase

from components.drivetrain.constants import DrivetrainConstants as const


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
        self._kinematics = DifferentialDriveKinematics(const.TRACK_WIDTH)
        telemetry.log("Field", self._field)

    def drive(self, forward_percent: float, angular_percent: float) -> None:
        self.forward_percent = forward_percent
        self.angular_percent = angular_percent

    def on_enable(self) -> None:
        self.left_pid = self.left_controller.create_wpi_pid_controller()
        self.right_pid = self.right_controller.create_wpi_pid_controller()

    def execute(self) -> None:
        chassis = ChassisVelocities(
            self.forward_percent * const.MAX_FORWARD_VELOCITY,
            0.0, 
            self.angular_percent * const.MAX_TURN_VELOCITY
        )
        velocities = self._kinematics.to_wheel_velocities(chassis)

        left_velocity = self.left_pid.calculate(velocities.left)
        self.left_pid.set_setpoint(velocities.left)

        right_velocity = self.right_pid.calculate(velocities.right)
        self.right_pid.set_setpoint(velocities.right)

        self.drivetrain_io.set_left_voltage(left_velocity)
        self.drivetrain_io.set_right_voltage(right_velocity)
        self.drivetrain_io.update()

        pose = self.drivetrain_io.get_pose()
        if pose is not None:
            self._field.set_robot_pose(pose)
            telemetry.log("Field", self._field)
            telemetry.log("X", round(pose.x, 2))
            telemetry.log("Y", round(pose.y, 2))
            telemetry.log("Rot", round(pose.rotation().degrees(), 2))

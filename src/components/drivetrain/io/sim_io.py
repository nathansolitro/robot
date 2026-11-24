from drivetrain.io.base_io import DrivetrainIOBase

from wpilib.simulation import DifferentialDrivetrainSim

from wpimath import Pose2d, DCMotor

from components.drivetrain.constants import DrivetrainConstants as const

DC_MOTOR = DCMotor.cim(1)

class SimDrivetrainIO(DrivetrainIOBase):
    def __init__(self) -> None:
        self._diff_drive = DifferentialDrivetrainSim(
            drive_motor=DC_MOTOR, 
            gearing=const.GEAR_RATIO,
            trackwidth=const.TRACK_WIDTH, 
            j=const.MOI, 
            mass=const.MASS,
            wheel_radius=const.WHEEL_RADIUS
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
        self._diff_drive.update(const.CLOCK_SPEED)

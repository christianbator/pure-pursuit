#
# simulation_data.py
# pure-pursuit
#
# Created by Christian Bator on 01/02/2025
#

from typing import Optional
from math import degrees, radians, fabs

from pure_pursuit.model.robot import Robot, RobotPose, RobotCommand
from pure_pursuit.controller.pure_pursuit_controller import PurePursuitController
from pure_pursuit.utilities.math_extensions import is_float_less_or_equal, average
from pure_pursuit.utilities.geometry import TargetPoint, ReferencePoint, Path, distance_between
from pure_pursuit.utilities.print_colors import cyan, green, bright_red

#
# SimulationState
#
class SimulationState:

    def __init__(
        self,
        pose: RobotPose,
        reference_point: ReferencePoint,
        cross_track_error: float,
        checkpoint_index: int,
        look_ahead_distance: float,
        target_point: Optional[TargetPoint],
        command: RobotCommand
    ):
        self._pose = pose
        self._reference_point = reference_point
        self._cross_track_error = cross_track_error
        self._checkpoint_index = checkpoint_index
        self._look_ahead_distance = look_ahead_distance
        self._target_point = target_point
        self._command = command
    
    @property
    def pose(self) -> RobotPose:
        return self._pose
    
    @property
    def reference_point(self) -> ReferencePoint:
        return self._reference_point
    
    @property
    def cross_track_error(self) -> float:
        return self._cross_track_error
    
    @property
    def checkpoint_index(self) -> float:
        return self._checkpoint_index
    
    @property
    def look_ahead_distance(self) -> float:
        return self._look_ahead_distance
    
    @property
    def target_point(self) -> Optional[TargetPoint]:
        return self._target_point

    @property
    def command(self) -> RobotCommand:
        return self._command

#
# SimulationData
#
class SimulationData:

    def __init__(
        self,
        robot: Robot,
        controller: PurePursuitController,
        dt: float,
        path: Path,
        states: list[SimulationState],
        step_calculation_times_ns: list[float]
    ):
        self._robot = robot
        self._controller = controller
        self._path = path
        self._states = states
        self._step_calculation_times_ns = step_calculation_times_ns
        self._dt = dt

    @property
    def path(self) -> Path:
        return self._path

    @property
    def states(self) -> list[SimulationState]:
        return self._states

    @property
    def dt(self) -> float:
        return self._dt

    @property
    def path_length(self) -> float:
        return self._path.end_point.distance_along_path

    @property
    def average_waypoint_angle(self) -> float:
        return average([degrees(fabs(waypoint.angle)) for waypoint in self._path])

    @property
    def average_step_calculation_time_ms(self) -> float:
        return average(self._step_calculation_times_ns) / 1_000_000.0

    @property
    def cross_track_errors(self) -> list[float]:
        return [state.cross_track_error for state in self._states]

    @property
    def average_absolute_cross_track_error(self) -> float:
        return average([fabs(cross_track_error) for cross_track_error in self.cross_track_errors])

    @property
    def average_velocity(self) -> float:
        return average([state.pose.velocity for state in self._states])

    @property
    def max_angular_velocity(self) -> float:
        return max([fabs(state.pose.angular_velocity) for state in self._states])

    @property
    def end_point_distance(self) -> float:
        return distance_between(self._states[-1].pose.position, self._path.end_point)

    def result_text(self, avg_abs_cross_track_error_threshold: float) -> str:
        result = "> Results:\n"
        result += "  > ---\n"

        result += f"  > Path length: {cyan(f'{self.path_length:0,.3f} m')}\n"
        result += f"  > Average waypoint angle: {cyan(f'{self.average_waypoint_angle:0,.1f}\u00B0 ({radians(self.average_waypoint_angle):.3f} rad)')}\n"
        result += "  > ---\n"

        result += f"  > Steps: {cyan(f'{len(self._states):,}')}\n"
        result += f"  > Runtime: {cyan(f'{(len(self._states) * self._dt):0,.3f} s')}\n"
        result += f"  > Average step calculation time: {cyan(f'{self.average_step_calculation_time_ms:0,.3f} ms')}\n"
        result += "  > ---\n"

        if is_float_less_or_equal(self.average_absolute_cross_track_error, avg_abs_cross_track_error_threshold):
            cross_track_error_text = green(f"{self.average_absolute_cross_track_error:0,.3f} m")
        else:
            cross_track_error_text = bright_red(f"{self.average_absolute_cross_track_error:0,.3f} m")

        result += f"  > Average absolute cross track error: {cross_track_error_text}\n"
        result += f"  > Average velocity: {cyan(f'{self.average_velocity:0,.3f} m/s')}\n"

        if is_float_less_or_equal(self.max_angular_velocity, self._robot.max_angular_velocity):
            angular_speed_text = green(f"{self.max_angular_velocity:0,.3f} rad/s")
        else:
            angular_speed_text = bright_red(f"{self.max_angular_velocity:0,.3f} rad/s")

        result += f"  > Max angular speed: {angular_speed_text}\n"

        if is_float_less_or_equal(self.end_point_distance, self._controller.end_condition_distance):
            ending_distance_text = green(f"{self.end_point_distance:0,.3f} m")
        else:
            ending_distance_text = bright_red(f"{self.end_point_distance:0,.3f} m")

        result += f"  > Ending distance: {ending_distance_text}\n"
        result += "  > ---"

        return result

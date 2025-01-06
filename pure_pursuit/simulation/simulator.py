#
# simulator.py
# pure-pursuit
#
# Created by Christian Bator on 01/02/2025
#

from time import time_ns
from pathlib import Path

from pure_pursuit.controller.pure_pursuit_controller import PurePursuitController
from pure_pursuit.model.robot import Robot, RobotPose, RobotCommand
from pure_pursuit.model.diff_drive_kinematics import propagate_pose
from pure_pursuit.simulation.simulation_data import SimulationState, SimulationData
from pure_pursuit.utilities.geometry import Path as GeoPath

#
# Simulator
#
class Simulator:

    #
    # Initialization
    #
    def __init__(
        self,
        robot: Robot,
        initial_pose: RobotPose,
        controller: PurePursuitController,
        dt: float,
        path: GeoPath
    ):
        self._robot = robot
        self._initial_pose = initial_pose
        self._controller = controller
        self._dt = dt
        self._path = path

    #
    # Simulation
    #
    def simulate(self) -> SimulationData:
        previous_pose = self._initial_pose
        previous_command = None

        states: list[SimulationState] = []
        step_calculation_times_ns: list[float] = []

        while not(self._controller.is_path_complete):
            # Start timing step calculation
            step_start_time_ns = time_ns()

            # Propagate pose from previous command
            pose = propagate_pose(robot = self._robot, previous_pose = previous_pose, previous_command = previous_command, dt = self._dt)

            # Receive a new command from the controller
            command = self._controller.update(pose = pose, dt = self._dt)

            # Stop timing step calculation
            step_calculation_times_ns.append(time_ns() - step_start_time_ns)

            states.append(
                SimulationState(
                    pose = pose,
                    reference_point = self._controller.current_reference_point,
                    cross_track_error = self._controller.current_cross_track_error,
                    checkpoint_index = self._controller.current_checkpoint_index,
                    look_ahead_distance = self._controller.current_look_ahead_distance,
                    target_point = self._controller.current_target_point,
                    command = command
                )
            )

            previous_pose = pose
            previous_command = command

        # Stop and append final state with velocity = 0.0
        previous_command = RobotCommand(right_wheel_angular_velocity = 0.0, left_wheel_angular_velocity = 0.0)
        final_pose = propagate_pose(robot = self._robot, previous_pose = previous_pose, previous_command = previous_command, dt = self._dt)

        states.append(
            SimulationState(
                pose = final_pose,
                reference_point = self._controller.current_reference_point,
                cross_track_error = self._controller.current_cross_track_error,
                checkpoint_index = self._controller.current_checkpoint_index,
                look_ahead_distance = self._controller.current_look_ahead_distance,
                target_point = self._controller.current_target_point,
                command = previous_command
            )
        )

        simulation_data = SimulationData(
            robot = self._robot,
            controller = self._controller,
            dt = self._dt,
            path = self._path,
            states = states,
            step_calculation_times_ns = step_calculation_times_ns
        )

        return simulation_data

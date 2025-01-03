#
# pure_pursuit.py
# pure-pursuit
#
# Created by Christian Bator on 01/03/2025
#

import json
from typing import Optional
from argparse import ArgumentParser
from pathlib import Path

from pure_pursuit.config.robomower_config import RobomowerConfig
from pure_pursuit.config.pure_pursuit_config import PurePursuitConfig
from pure_pursuit.config.simulation_config import SimulationConfig
from pure_pursuit.controller.pure_pursuit_controller import PurePursuitController
from pure_pursuit.controller.path_adapter import PathAdapter
from pure_pursuit.model.robomower import Robomower, RobomowerPose
from pure_pursuit.simulation.simulator import Simulator
from pure_pursuit.animation.animator import Animator
from pure_pursuit.plotting import plot_cross_track_errors
from pure_pursuit.utilities.geometry import Point, line_segment_angle
from pure_pursuit.utilities.print_colors import cyan

from matplotlib import pyplot as plot
from matplotlib.animation import FuncAnimation

#
# Constants
#
OUTPUT_DIR = Path("output")
RESULT_DIR = OUTPUT_DIR / "results"
ANIMATION_DIR = OUTPUT_DIR / "animations"

is_paused = False
animation: Optional[FuncAnimation] = None

#
# Main
#
def main():
    #
    # Parse Arguments
    #
    parser = ArgumentParser()
    required_named_arguments = parser.add_argument_group("required named arguments")
    required_named_arguments.add_argument("-p", "--path-filepath", required = True)
    required_named_arguments.add_argument("-s", "--simulation-config-filepath", required = True)
    required_named_arguments.add_argument("-r", "--robomower-config-filepath", required = True)
    required_named_arguments.add_argument("-c", "--pure-pursuit-config-filepath", required = True)
    parser.add_argument("--animate", action = "store_true")
    parser.add_argument("--follow", action = "store_true")
    parser.add_argument("--save-animation", action = "store_true")
    parser.add_argument("--quiet", action = "store_true")
    parser.add_argument("--graphs", action = "store_true")
    args = parser.parse_args()

    simulation_config: Optional[SimulationConfig] = None
    with open(args.simulation_config_filepath) as simulation_config_file:
        simulation_config_json = json.load(simulation_config_file)
        simulation_config = SimulationConfig.decode(simulation_config_json)

    if not simulation_config:
        print("> Error: Invalid simulation config")
        exit(2)

    robomower_config: Optional[RobomowerConfig] = None
    with open(args.robomower_config_filepath) as robomower_config_file:
        robomower_config_json = json.load(robomower_config_file)
        robomower_config = RobomowerConfig.decode(robomower_config_json)

    if not robomower_config:
        print("> Error: Invalid robomower config")
        exit(2)

    pure_pursuit_config: Optional[PurePursuitConfig] = None
    with open(args.pure_pursuit_config_filepath) as pure_pursuit_config_file:
        pure_pursuit_config_json = json.load(pure_pursuit_config_file)
        pure_pursuit_config = PurePursuitConfig.decode(pure_pursuit_config_json)

    if not pure_pursuit_config:
        print("> Error: Invalid pure pursuit config")
        exit(2)

    #
    # Initialization
    #
    path_filepath = Path(args.path_filepath)
    
    with open(path_filepath) as path_file:
        path_data = json.load(path_file)

    raw_points = [Point(x = point["x"], y = point["y"]) for point in path_data]

    # Pre-process path by calculating target velocities based on configuration parameters and acceleration limits
    path = PathAdapter(raw_points = raw_points).adapt_path(
        max_velocity = robomower_config.max_velocity,
        max_acceleration = robomower_config.max_acceleration,
        angle_velocity_parameter = pure_pursuit_config.angle_velocity_parameter
    )

    # Create robot
    robomower = Robomower(config = robomower_config)
    
    # Set initial pose to the first point on the path
    initial_pose = RobomowerPose(
        position = raw_points[0],
        heading = line_segment_angle([raw_points[0], raw_points[1]]),
        velocity = 0.0,
        angular_velocity = 0.0
    )

    # Create controller
    controller = PurePursuitController(config = pure_pursuit_config, robomower = robomower, path = path)
    
    #
    # Simulate
    #
    dt = 1.0 / simulation_config.control_frequency

    simulator = Simulator(
        robomower = robomower,
        initial_pose = initial_pose,
        controller = controller,
        dt = dt,
        path = path
    )

    path_name = path_filepath.stem
    print(f"> Simulating {cyan(path_name)} ...")

    simulation_data = simulator.simulate()

    print("  > Done")
    print(simulation_data.result_text(avg_abs_cross_track_error_threshold = simulation_config.avg_abs_cross_track_error_threshold))

    result_filepath = RESULT_DIR / f"{path_filepath.stem}.json"

    print(f"> Saving output to {result_filepath} ...")
    
    RESULT_DIR.mkdir(parents = True, exist_ok = True)

    with open(result_filepath, "w") as result_file:
        json.dump(simulation_data.cross_track_errors, result_file, indent = 4, sort_keys = True)

    print("  > Done")

    #
    # Plot Cross Track Errors
    #
    graph_figure = None

    if args.graphs:
        graph_figure = plot_cross_track_errors(simulation_data = simulation_data)

    #
    # Animate
    #
    animation_figure = None

    if args.animate:
        animator = Animator(
            robomower = robomower,
            controller = controller,
            simulation_data = simulation_data,
            path_name = path_name,
            follow = args.follow,
            print_frame_number = args.save_animation
        )

        animation_figure, animation = animator.animate()

        if args.save_animation:
            ANIMATION_DIR.mkdir(parents = True, exist_ok = True)

            filepath = ANIMATION_DIR / f"{path_name}.mp4"
            print(f"> Saving animation to '{filepath}'...")

            animation.save(filename = filepath, writer = "ffmpeg")
            plot.close(animation_figure)
            animation_figure = None

            print("  > Done")
        else:
            is_paused = False

            def toggle_pause(_):
                global is_paused

                if animation is None or animation.event_source is None:
                    return

                if is_paused:
                    animation.resume()
                else:
                    animation.pause()

                is_paused = not(is_paused)

            animation_figure.canvas.mpl_connect("button_press_event", toggle_pause)
    
    # Show plots
    if graph_figure is not None or animation_figure is not None:
        plot.show()

    return 0

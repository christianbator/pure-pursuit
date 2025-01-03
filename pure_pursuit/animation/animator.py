#
# animator.py
# pure-pursuit
#
# Created by Christian Bator on 01/02/2025
#

from typing import Tuple
from string import capwords
from math import floor, ceil, cos, sin

from pure_pursuit.simulation.simulation_data import SimulationData
from pure_pursuit.animation.matplotlib_extensions import draw_point, draw_points, draw_line, draw_circle, calculate_circle_points
from pure_pursuit.utilities.geometry import bounding_box, Point
from pure_pursuit.model.robomower import Robomower
from pure_pursuit.controller.pure_pursuit_controller import PurePursuitController

import matplotlib.pyplot as plot
import matplotlib.ticker as ticker
import matplotlib.patches as patches
import matplotlib.transforms as transforms
from matplotlib.animation import FuncAnimation
from matplotlib.figure import Figure

#
# Animator
#
class Animator:

    #
    # Initialization
    #
    def __init__(
            self,
            robomower: Robomower,
            controller: PurePursuitController,
            simulation_data: SimulationData,
            path_name: str,
            follow: bool,
            print_frame_number: bool
    ):
        initial_state = simulation_data.states[0]
        max_look_ahead_diameter = 2.0 * controller.max_look_ahead_distance
        path_bounds = bounding_box(simulation_data.path.waypoints)

        if follow:
            buffer = max(max_look_ahead_diameter, max(robomower.length, robomower.track_width))
            xlim = (initial_state.pose.position.x - buffer, initial_state.pose.position.x + buffer)
            ylim = (initial_state.pose.position.y - buffer, initial_state.pose.position.y + buffer)
        else:
            buffer = 0.3 * max(path_bounds[1] - path_bounds[0], path_bounds[3] - path_bounds[2])
            xlim = (path_bounds[0] - buffer, path_bounds[1] + buffer)
            ylim = (path_bounds[2] - buffer, path_bounds[3] + buffer)

        sanitized_path_name = path_name.replace("-", " ")
        capitalized_title = capwords(sanitized_path_name)

        figure = plot.figure(capitalized_title, figsize = (7.0, 7.0))

        axes = figure.add_subplot(
            111,
            aspect = "equal",
            autoscale_on = False,
            xlim = xlim, 
            ylim = ylim
        )

        axes.set_title(capitalized_title, fontweight = "bold", pad = 20)
        axes.set_xlabel("x (m)", fontweight = "bold", labelpad = 10)
        axes.set_ylabel("y (m)", fontweight = "bold", labelpad = 10)
        
        if follow:
            min_xy = floor(min(path_bounds[0], path_bounds[2]))
            max_xy = ceil(max(path_bounds[1], path_bounds[3]))
            difference = max_xy - min_xy
            locator = ticker.FixedLocator(range(min_xy - difference, max_xy + difference))
            axes.xaxis.set_major_locator(locator)
            axes.yaxis.set_major_locator(locator)

        axes.grid(which = "major", alpha = 0.3)

        self._robomower = robomower
        self._simulation_data = simulation_data
        self._follow = follow
        self._print_frame_number = print_frame_number
        self._initial_state = initial_state
        self._figure = figure
        self._axes = axes
        self._buffer = buffer

    #
    # Animation
    #
    def animate(self) -> Tuple[Figure, FuncAnimation]:
        path_line = draw_line(
            axes = self._axes,
            path = self._simulation_data.path.waypoints,
            show_points = True,
            line_style = "-",
            line_color = "silver"
        )

        checkpoint_dot = draw_point( 
            axes = self._axes,
            point = self._simulation_data.path[self._initial_state.checkpoint_index],  
            color = "tab:purple"
        )

        reference_point_dot = draw_point(
            axes = self._axes,
            point = self._initial_state.pose.position,  
            color = "tab:red"
        )

        draw_points(
            axes = self._axes,
            points = [self._simulation_data.path.start_point, self._simulation_data.path.end_point],
            color = "tab:red"
        )

        traveled_path_line = draw_line(
            axes = self._axes,
            path = [],
            line_color = "tab:red",
            line_label = "Traveled Path"
        )

        robot_center_dot = draw_point(
            axes = self._axes,
            point = self._initial_state.pose.position,
            label = "Pose",
            color = "tab:green"
        )
        
        robot_circle = draw_circle(
            axes = self._axes,
            center = self._initial_state.pose.position,
            radius = self._initial_state.look_ahead_distance,
            color = "tab:green"
        )
        
        robot_heading_scalar = 0.6
        robot_heading_line = draw_line(
            axes = self._axes,
            path = [
                self._simulation_data.states[0].pose.position,
                Point(
                    x = self._initial_state.pose.position.x + robot_heading_scalar * self._initial_state.look_ahead_distance * cos(self._initial_state.pose.heading),
                    y = self._initial_state.pose.position.y + robot_heading_scalar * self._initial_state.look_ahead_distance * sin(self._initial_state.pose.heading)
                )
            ],
            line_width = 1,
            line_color = "tab:green"
        )

        robot_frame = patches.Rectangle(
            (self._initial_state.pose.position.x - 0.25 * self._robomower.length, self._initial_state.pose.position.y - 0.5 * self._robomower.track_width),
            self._robomower.length,
            self._robomower.track_width,
            linewidth = 1.5,
            edgecolor = ("tab:red", 0.3),
            facecolor = ("tab:red", 0.1),
            zorder = path_line.get_zorder() - 1
        )

        robot_frame_rotation = transforms.Affine2D().rotate_around(
            self._initial_state.pose.position.x,
            self._initial_state.pose.position.y,
            self._initial_state.pose.heading
        )

        robot_frame.set_transform(robot_frame_rotation + self._axes.transData)

        self._axes.add_patch(robot_frame)

        target_point_dot = draw_point(
            axes = self._axes,
            point = self._initial_state.target_point,
            label = "Target Point",
            color = "tab:blue"
        )

        time_text = self._axes.text(x = 0.03, y = 0.95, s = "t: ", fontweight = "bold", transform = self._axes.transAxes)
        velocity_text = self._axes.text(x = 0.03, y = 0.91, s = "v:", fontweight = "bold", transform = self._axes.transAxes)
        angular_velocity_text = self._axes.text(x = 0.03, y = 0.87, s = "\u03C9:", fontweight = "bold", transform = self._axes.transAxes)

        left_wheel_line_x = 0.04
        right_wheel_line_x = 0.08

        left_wheel_text = self._axes.text(x = left_wheel_line_x, y = 0.02, s = "L", ha = "center", fontweight = "bold", transform = self._axes.transAxes)
        right_wheel_text = self._axes.text(x = right_wheel_line_x, y = 0.02, s = "R", ha = "center", fontweight = "bold", transform = self._axes.transAxes)

        wheel_line_ymin = 0.05
        wheel_line_max_length = 0.15
        wheel_line_line_width = 4.0

        left_wheel_line = self._axes.vlines(
            x = left_wheel_line_x,
            ymin = wheel_line_ymin,
            ymax = wheel_line_ymin,
            linewidth = wheel_line_line_width,
            color = "tab:green",
            transform = self._axes.transAxes
        )

        right_wheel_line = self._axes.vlines(
            x = right_wheel_line_x,
            ymin = wheel_line_ymin,
            ymax = wheel_line_ymin, 
            linewidth = wheel_line_line_width,
            color = "tab:green",
            transform = self._axes.transAxes
        )

        num_animation_frames = len(self._simulation_data.states)

        traveled_xdata = [state.pose.position.x for state in self._simulation_data.states]
        traveled_ydata = [state.pose.position.y for state in self._simulation_data.states]

        handles, labels = self._axes.get_legend_handles_labels()

        legend_order = [1, 2, 0]
        self._axes.legend([handles[index] for index in legend_order], [labels[index] for index in legend_order], loc = "lower right")

        def initialize_animation() -> tuple:
            return (
                checkpoint_dot,
                reference_point_dot,
                robot_center_dot,
                robot_circle,
                robot_heading_line,
                robot_frame,
                target_point_dot,
                time_text,
                velocity_text,
                angular_velocity_text,
                left_wheel_line,
                right_wheel_line
            )

        def animate_step(frame_index: int) -> tuple:
            if self._print_frame_number:
                print(f"  > Frame: {frame_index} / {num_animation_frames - 1}", end = "\n" if frame_index == (num_animation_frames - 1) else "\r")

            state = self._simulation_data.states[frame_index]

            time_text.set_text(f"t: {frame_index * self._simulation_data.dt:0,.1f} s")
            velocity_text.set_text(f"v: {state.pose.velocity:0,.2f} m/s")

            angular_velocity = 0.0 if -0.001 < state.pose.angular_velocity < 0.001 else state.pose.angular_velocity
            angular_velocity_text.set_text(f"\u03C9: {angular_velocity:0,.3f} rad/s")

            left_wheel_angular_velocity = state.command.left_wheel_angular_velocity
            right_wheel_angular_velocity = state.command.right_wheel_angular_velocity

            left_wheel_line.set_color("tab:red" if left_wheel_angular_velocity < 0.0 else "tab:green")
            right_wheel_line.set_color("tab:red" if right_wheel_angular_velocity < 0.0 else "tab:green")

            left_wheel_line.set_paths([[
                (left_wheel_line_x, wheel_line_ymin),
                (left_wheel_line_x, wheel_line_ymin + self._robomower.wheel_radius * left_wheel_angular_velocity / self._robomower.max_velocity * wheel_line_max_length)
            ]])

            right_wheel_line.set_paths([[
                (right_wheel_line_x, wheel_line_ymin),
                (right_wheel_line_x, wheel_line_ymin + self._robomower.wheel_radius * right_wheel_angular_velocity / self._robomower.max_velocity * wheel_line_max_length)
            ]])

            checkpoint_dot.set_xdata([self._simulation_data.path[state.checkpoint_index].x])
            checkpoint_dot.set_ydata([self._simulation_data.path[state.checkpoint_index].y])

            reference_point_dot.set_xdata([state.reference_point.x])
            reference_point_dot.set_ydata([state.reference_point.y])

            traveled_path_line.set_xdata(traveled_xdata[:frame_index])
            traveled_path_line.set_ydata(traveled_ydata[:frame_index])

            robot_center_dot.set_xdata([state.pose.position.x])
            robot_center_dot.set_ydata([state.pose.position.y])

            robot_circle_point_xdata, robot_circle_point_ydata = calculate_circle_points(state.pose.position, state.look_ahead_distance)
            robot_circle.set_xdata(robot_circle_point_xdata)
            robot_circle.set_ydata(robot_circle_point_ydata)

            robot_heading_line.set_xdata([
                state.pose.position.x,
                state.pose.position.x + robot_heading_scalar * state.look_ahead_distance * cos(state.pose.heading)
            ])

            robot_heading_line.set_ydata([
                state.pose.position.y, 
                state.pose.position.y + robot_heading_scalar * state.look_ahead_distance * sin(state.pose.heading)
            ])

            robot_frame.set_transform(None)

            robot_frame.set_xy((
                state.pose.position.x - 0.25 * self._robomower.length,
                state.pose.position.y - 0.5 * self._robomower.track_width)
            )

            robot_frame_rotation = transforms.Affine2D().rotate_around(
                state.pose.position.x,
                state.pose.position.y,
                state.pose.heading
            )

            robot_frame.set_transform(robot_frame_rotation + self._axes.transData)

            if state.target_point is not None:
                target_point_dot.set_xdata([state.target_point.x])
                target_point_dot.set_ydata([state.target_point.y])

            if self._follow:
                self._axes.set_xlim(state.pose.position.x - self._buffer, state.pose.position.x + self._buffer)
                self._axes.set_ylim(state.pose.position.y - self._buffer, state.pose.position.y + self._buffer)

            return (
                checkpoint_dot,
                reference_point_dot,
                robot_center_dot,
                robot_circle,
                robot_heading_line,
                robot_frame,
                target_point_dot,
                time_text,
                velocity_text,
                angular_velocity_text,
                left_wheel_line,
                right_wheel_line
            )

        animation = FuncAnimation(
            self._figure,
            animate_step, 
            init_func = initialize_animation,
            frames = num_animation_frames,
            interval = self._simulation_data.dt * 1000.0,
            repeat = False
        )

        return (self._figure, animation)

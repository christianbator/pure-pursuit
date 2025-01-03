#
# diff_drive_kinematics.py
# pure-pursuit
#
# Created by Christian Bator on 01/03/2025
#

from typing import Optional
from math import cos, sin

from pure_pursuit.utilities.geometry import Point
from pure_pursuit.model.robomower import Robomower, RobomowerPose, RobomowerCommand

#
# Pose Propagation
#
def propagate_pose(robomower: Robomower, previous_pose: RobomowerPose, previous_command: Optional[RobomowerCommand], dt: float) -> RobomowerPose:
    if previous_command is None:
        return previous_pose

    velocity = robomower.wheel_radius * (previous_command.right_wheel_angular_velocity + previous_command.left_wheel_angular_velocity) / 2.0

    angular_velocity = robomower.wheel_radius * (previous_command.right_wheel_angular_velocity - previous_command.left_wheel_angular_velocity) / robomower.track_width

    return RobomowerPose(
        position = Point(
            x = previous_pose.position.x + cos(previous_pose.heading) * velocity * dt,
            y = previous_pose.position.y + sin(previous_pose.heading) * velocity * dt
        ),
        heading = previous_pose.heading + angular_velocity * dt,
        velocity = velocity,
        angular_velocity = angular_velocity
    )

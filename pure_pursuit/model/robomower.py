#
# robomower.py
# pure-pursuit
#
# Created by Christian Bator on 01/02/2025
#

from pure_pursuit.config.robomower_config import RobomowerConfig
from pure_pursuit.utilities.geometry import Point

#
# Robomower
#
class Robomower:

    def __init__(self, config: RobomowerConfig):
        self._wheel_radius = config.wheel_radius
        self._track_width = config.track_width
        self._length = config.length
        self._max_velocity = config.max_velocity
        self._max_acceleration = config.max_acceleration
        self._max_angular_velocity = config.max_angular_velocity
        self._max_angular_acceleration = config.max_angular_acceleration

    @property
    def wheel_radius(self) -> float:
        return self._wheel_radius
    
    @property
    def track_width(self) -> float:
        return self._track_width
    
    @property
    def length(self) -> float:
        return self._length
    
    @property
    def max_velocity(self) -> float:
        return self._max_velocity
    
    @property
    def max_acceleration(self) -> float:
        return self._max_acceleration

    @property
    def max_angular_velocity(self) -> float:
        return self._max_angular_velocity
    
    @property
    def max_angular_acceleration(self) -> float:
        return self._max_angular_acceleration

#
# RobomowerPose
#
class RobomowerPose:

    def __init__(self, position: Point, heading: float, velocity: float, angular_velocity: float):
        self._position = position
        self._heading = heading
        self._velocity = velocity
        self._angular_velocity = angular_velocity

    @property 
    def position(self) -> Point:
        return self._position

    @property
    def heading(self) -> float:
        return self._heading

    @property
    def velocity(self) -> float:
        return self._velocity

    @property
    def angular_velocity(self) -> float:
        return self._angular_velocity

    def right_wheel_angular_velocity(self, robomower: Robomower) -> float:
        return (self.velocity + 0.5 * self.angular_velocity * robomower.track_width) / robomower.wheel_radius

    def left_wheel_angular_velocity(self, robomower: Robomower) -> float:
        return (self.velocity - 0.5 * self.angular_velocity * robomower.track_width) / robomower.wheel_radius

    def __str__(self) -> str:
        return f"(x: {self.position.x:0,.3f}, y: {self.position.y:0,.3f}, theta: {self.heading:0,.3f}, v: {self.velocity:0,.3f}, \u03C9: {self.angular_velocity:0,.3f})"

#
# RobomowerCommand
#
class RobomowerCommand:

    def __init__(self, right_wheel_angular_velocity: float, left_wheel_angular_velocity: float):
        self._right_wheel_angular_velocity = right_wheel_angular_velocity
        self._left_wheel_angular_velocity = left_wheel_angular_velocity

    @property
    def right_wheel_angular_velocity(self) -> float:
        return self._right_wheel_angular_velocity

    @property
    def left_wheel_angular_velocity(self) -> float:
        return self._left_wheel_angular_velocity

    def __str__(self) -> str:
        return f"(r: {self.right_wheel_angular_velocity:0,.3f}, l: {self.left_wheel_angular_velocity:0,.3f})"

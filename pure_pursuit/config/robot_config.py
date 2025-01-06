#
# robot_config.py
# pure-pursuit
#
# Created by Christian Bator on 01/02/2025
#

from typing import Self

from pure_pursuit.utilities.json_codable import JSON, JSONDecodable

#
# RobotConfig
#
class RobotConfig(JSONDecodable):

    def __init__(
        self,
        wheel_radius: float,
        track_width: float,
        length: float,
        max_velocity: float,
        max_acceleration: float,
        max_angular_velocity: float,
        max_angular_acceleration: float
    ):
        self._wheel_radius = wheel_radius
        self._track_width = track_width
        self._length = length
        self._max_velocity = max_velocity
        self._max_acceleration = max_acceleration
        self._max_angular_velocity = max_angular_velocity
        self._max_angular_acceleration = max_angular_acceleration

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
 
    @classmethod
    def decode(cls, json_data: JSON) -> Self:
        return cls(
            wheel_radius = float(json_data["wheel_radius"]),
            track_width = float(json_data["track_width"]),
            length = float(json_data["length"]),
            max_velocity = float(json_data["max_velocity"]),
            max_acceleration = float(json_data["max_acceleration"]),
            max_angular_velocity = float(json_data["max_angular_velocity"]),
            max_angular_acceleration = float(json_data["max_angular_acceleration"])
        )

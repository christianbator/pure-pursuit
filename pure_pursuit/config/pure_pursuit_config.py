#
# pure_pursuit_config.py
# pure-pursuit
#
# Created by Christian Bator on 01/02/2025
#

from typing import Self

from pure_pursuit.utilities.json_codable import JSON, JSONDecodable

#
# PurePursuitConfig
#
class PurePursuitConfig(JSONDecodable):

    def __init__(
        self,
        min_look_ahead_distance: float,
        max_look_ahead_distance: float,
        angle_velocity_parameter: float,
        final_approach_velocity: float,
        end_condition_distance: float
    ):
        self._min_look_ahead_distance = min_look_ahead_distance
        self._max_look_ahead_distance = max_look_ahead_distance
        self._angle_velocity_parameter = angle_velocity_parameter
        self._final_approach_velocity = final_approach_velocity
        self._end_condition_distance = end_condition_distance

    @property
    def min_look_ahead_distance(self) -> float:
        return self._min_look_ahead_distance
    
    @property
    def max_look_ahead_distance(self) -> float:
        return self._max_look_ahead_distance
    
    @property
    def angle_velocity_parameter(self) -> float:
        return self._angle_velocity_parameter
    
    @property
    def final_approach_velocity(self) -> float:
        return self._final_approach_velocity
    
    @property
    def end_condition_distance(self) -> float:
        return self._end_condition_distance
 
    @classmethod
    def decode(cls, json_data: JSON) -> Self:
        return cls(
            min_look_ahead_distance = float(json_data["min_look_ahead_distance"]),
            max_look_ahead_distance = float(json_data["max_look_ahead_distance"]),
            angle_velocity_parameter = float(json_data["angle_velocity_parameter"]),
            final_approach_velocity = float(json_data["final_approach_velocity"]),
            end_condition_distance = float(json_data["end_condition_distance"])
        )

#
# simulation_config.py
# pure-pursuit
#
# Created by Christian Bator on 01/02/2025
#

from typing import Self

from pure_pursuit.utilities.json_codable import JSON, JSONDecodable

#
# SimulationConfig
#
class SimulationConfig(JSONDecodable):

    def __init__(
        self,
        control_frequency: float,
        avg_abs_cross_track_error_threshold: float
    ):
        self._control_frequency = control_frequency
        self._avg_abs_cross_track_error_threshold = avg_abs_cross_track_error_threshold

    @property
    def control_frequency(self) -> float:
        return self._control_frequency

    @property
    def avg_abs_cross_track_error_threshold(self) -> float:
        return self._avg_abs_cross_track_error_threshold
 
    @classmethod
    def decode(cls, json_data: JSON) -> Self:
        return cls(
            control_frequency = float(json_data["control_frequency"]),
            avg_abs_cross_track_error_threshold = float(json_data["avg_abs_cross_track_error_threshold"])
        )

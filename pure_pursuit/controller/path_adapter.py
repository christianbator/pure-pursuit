#
# path_adapter.py
# pure-pursuit
#
# Created by Christian Bator on 01/02/2025
#

from math import fabs, sqrt

from pure_pursuit.utilities.math_extensions import is_float_equal, end_index, inclusive_range
from pure_pursuit.utilities.geometry import Point, Waypoint, Path, distance_between, angle_between_segments

#
# PathAdapter
#
class PathAdapter:

	#
	# Initialization
	#
	def __init__(self, raw_points: list[Point]):
		self.raw_points = raw_points

	#
	# Path Adaptation
	#
	def adapt_path(self, max_velocity: float, max_acceleration: float, angle_velocity_parameter: float) -> Path:
		angles: list[float] = []
		distances: list[float] = []
		distance_sum = 0.0
		target_velocities: list[float] = []

		for index, point in enumerate(self.raw_points):
			if index == 0:
				angle = 0.0
				target_velocity = 0.0
			elif index == end_index(self.raw_points):
				angle = 0.0
				distance_sum += distance_between(self.raw_points[index - 1], point)
				target_velocity = 0.0
			else:
				angle = angle_between_segments(
					[self.raw_points[index - 1], point],
					[point, self.raw_points[index + 1]]
				)

				segment_length = distance_between(self.raw_points[index - 1], self.raw_points[index])
				distance_sum += segment_length

				parameter_constrained_target_velocity = min(max_velocity, angle_velocity_parameter / fabs(angle)) if not is_float_equal(angle, 0.0) else max_velocity

				acceleration_constrained_target_velocity = sqrt(target_velocities[index - 1]**2 + 2.0 * max_acceleration * segment_length)

				target_velocity = min(
					parameter_constrained_target_velocity,
					acceleration_constrained_target_velocity
				)

			angles.append(angle)
			distances.append(distance_sum)
			target_velocities.append(target_velocity)

		for index in inclusive_range(end_index(self.raw_points) - 1, 1, -1):
			segment_length = distances[index + 1] - distances[index]
			deceleration_constrained_target_velocity = sqrt(target_velocities[index + 1]**2 + 2.0 * max_acceleration * segment_length)

			target_velocities[index] = min(target_velocities[index], deceleration_constrained_target_velocity)

		waypoints: list[Waypoint] = [
			Waypoint(position = item[0], angle = item[1], distance_along_path = item[2], target_velocity = item[3])
			for item in zip(self.raw_points, angles, distances, target_velocities)
		]

		path = Path(waypoints = waypoints)

		return path

#
# pure_pursuit_controller.py
# pure-pursuit
#
# Created by Christian Bator on 01/02/2025
#

from typing import Optional
from operator import itemgetter
from math import inf, sqrt, fabs

from pure_pursuit.config.pure_pursuit_config import PurePursuitConfig
from pure_pursuit.model.robomower import Robomower, RobomowerPose, RobomowerCommand
from pure_pursuit.utilities.math_extensions import end_index, is_float_within, inclusive_range, constrain, sgn
from pure_pursuit.utilities.geometry import (
    PointProtocol,
    Point,
    Waypoint,
    TargetPoint,
    ReferencePoint,
    Path,
    point_line_orientation,
    distance_between,
    orthogonal_point_on_segment,
    signed_point_vector_distance,
	are_points_equal
)

class PurePursuitController:

	def __init__(self, config: PurePursuitConfig, robomower: Robomower, path: Path):
		# Configuration
		self._min_look_ahead_distance = config.min_look_ahead_distance
		self._max_look_ahead_distance = config.max_look_ahead_distance
		self._final_approach_velocity = config.final_approach_velocity
		self._end_condition_distance = config.end_condition_distance

		self._max_velocity = robomower.max_velocity
		self._max_acceleration = robomower.max_acceleration
		self._max_angular_velocity = robomower.max_angular_velocity
		self._max_angular_acceleration = robomower.max_angular_acceleration
		self._track_width = robomower.track_width
		self._wheel_radius = robomower.wheel_radius
		self._path = path

		# Reference point state
		self._current_reference_point_index = 0
		self._current_reference_point: ReferencePoint = path.start_point
		self._current_cross_track_error = 0.0

		# Look-ahead state
		self._current_checkpoint_index = 0
		self._current_look_ahead_distance = config.min_look_ahead_distance
		self._current_target_point: Optional[TargetPoint] = None

		# End condition state
		self._is_path_complete = False
		self._previous_distance_remaining = inf

	@property
	def max_look_ahead_distance(self) -> float:
		return self._max_look_ahead_distance

	@property
	def end_condition_distance(self) -> float:
		return self._end_condition_distance

	@property
	def current_reference_point(self) -> ReferencePoint:
		return self._current_reference_point

	@property
	def current_cross_track_error(self) -> float:
		return self._current_cross_track_error

	@property
	def current_checkpoint_index(self) -> int:
		return self._current_checkpoint_index

	@property
	def current_look_ahead_distance(self) -> float:
		return self._current_look_ahead_distance

	@property
	def current_target_point(self) -> Optional[TargetPoint]:
		return self._current_target_point
	
	@property
	def is_path_complete(self) -> bool:
		return self._is_path_complete

	#
	# Update
	#
	def update(self, pose: RobomowerPose, dt: float) -> RobomowerCommand:
		# Calculate look-ahead distance based on velocity
		look_ahead_distance = self.calculate_look_ahead_distance(
			path = self._path,
			pose = pose,
			current_checkpoint_index = self.current_checkpoint_index,
			max_velocity = self._max_velocity,
			min_look_ahead_distance = self._min_look_ahead_distance,
			max_look_ahead_distance = self._max_look_ahead_distance
		)

		# Search for next target point
		checkpoint_index, target_point = self.next_target_point(
			path = self._path,
			pose = pose,
			current_checkpoint_index = self.current_checkpoint_index,
			current_target_point = self.current_target_point,
			look_ahead_distance = look_ahead_distance
		)

		# Calculate reference point and cross track error
		reference_point_index, reference_point = self.next_reference_point(
			path = self._path,
			pose = pose,
			current_reference_point_index = self._current_reference_point_index,
			current_reference_point = self.current_reference_point,
			checkpoint_index = checkpoint_index
		)
		
		reference_segment = [self._path[reference_point_index], self._path[reference_point_index + 1]]
		cross_track_error = point_line_orientation(pose.position, reference_segment) * distance_between(pose.position, reference_point)

		# Calculate target velocities and handle end conditions
		target_velocity = target_point.target_velocity

		if checkpoint_index >= end_index(self._path) - 1:
			target_velocity, is_path_complete, distance_remaining = self.handle_end_conditions(
				path = self._path,
				pose = pose,
				target_point = target_point,
				final_approach_velocity = self._final_approach_velocity,
				end_condition_distance = self._end_condition_distance,
				previous_distance_remaining = self._previous_distance_remaining,
				dt = dt
			)

			self._is_path_complete = is_path_complete
			self._previous_distance_remaining = distance_remaining

		# Create command to drive along curvature
		signed_curvature = self.calculate_signed_curvature(
			pose = pose,
			target_point = target_point
		)

		command = self.create_command(
			pose = pose,
			signed_curvature = signed_curvature,
			target_velocity = target_velocity,
			max_velocity = self._max_velocity,
			max_acceleration = self._max_acceleration,
			max_angular_velocity = self._max_angular_velocity,
			max_angular_acceleration = self._max_angular_acceleration,
			track_width = self._track_width,
			wheel_radius = self._wheel_radius,
			dt = dt
		)

		# Store current state
		self._current_reference_point_index = reference_point_index
		self._current_reference_point = reference_point
		self._current_cross_track_error = cross_track_error
		self._current_checkpoint_index = checkpoint_index
		self._current_look_ahead_distance = look_ahead_distance
		self._current_target_point = target_point

		return command

	#
	# Look-ahead Distance
	#
	def calculate_look_ahead_distance(
		self,
		path: Path,
		pose: RobomowerPose,
		current_checkpoint_index: int,
		max_velocity: float,
		min_look_ahead_distance: float,
		max_look_ahead_distance: float
	) -> float:

		if current_checkpoint_index < end_index(path) - 1:
			velocity_term = pose.velocity
		else:
			velocity_term = max(pose.velocity, path[end_index(path) - 1].target_velocity)

		velocity_percentage = velocity_term / max_velocity

		look_ahead_distance = min_look_ahead_distance + velocity_percentage * (max_look_ahead_distance - min_look_ahead_distance)

		return look_ahead_distance

	#
	# Target Point
	#
	def next_target_point(
		self,
		path: Path,
		pose: RobomowerPose,
		current_checkpoint_index: int,
		current_target_point: Optional[TargetPoint],
		look_ahead_distance
	) -> tuple[int, TargetPoint]:

		# If we've seen the end, return the end point as the target point
		if current_checkpoint_index == end_index(path):
			return (
				end_index(path),
				TargetPoint(
					position = path.end_point.position,
					distance_along_path = path.end_point.distance_along_path,
					target_velocity = path.end_point.target_velocity
				)
			)

		# Store current target point distance along path for comparison, so we never move backwards
		if current_target_point is None:
			current_target_point_distance_along_path = 0.0
		else:
			current_target_point_distance_along_path = current_target_point.distance_along_path

		# Check for intersections in current segment:
		#   - Take the furthest intersection along segment
		#   - If intersection is further along path than the current target point, choose it to be the next target point
		current_checkpoint = path[current_checkpoint_index]
		next_waypoint = path[current_checkpoint_index + 1]

		intersections = self.line_segment_circle_intersections(
			pose = pose,
			point_1 = path[current_checkpoint_index],
			point_2 = path[current_checkpoint_index + 1],
			look_ahead_distance = look_ahead_distance
		)

		for intersection in intersections:
			intersection_distance_along_path = current_checkpoint.distance_along_path + distance_between(current_checkpoint, intersection)

			if intersection_distance_along_path > current_target_point_distance_along_path:
				return self.create_intersection_target_point(
					checkpoint_index = current_checkpoint_index,
					intersection = intersection,
					waypoint_1 = current_checkpoint,
					waypoint_2 = next_waypoint
				)

		# If no intersections in current segment:
		#   - Find first segment whose end point is at least `look_ahead_distance` away
		#   - Either a further along intersection exists on this segment,
		#     or we have an intersection that moves the target point backwards (maintain current target point in this case),
		#     - or we must choose the segment end point
		for index in range(current_checkpoint_index + 1, len(path)):
			waypoint = path[index]
			previous_waypoint = path[index - 1]

			if distance_between(pose.position, waypoint) > look_ahead_distance:
				intersections = self.line_segment_circle_intersections(
					pose = pose,
					point_1 = previous_waypoint,
					point_2 = waypoint,
					look_ahead_distance = look_ahead_distance
				)

				if len(intersections) > 0:
					for intersection in intersections:
						intersection_distance_along_path = previous_waypoint.distance_along_path + distance_between(previous_waypoint, intersection)

						if intersection_distance_along_path > current_target_point_distance_along_path:
							return self.create_intersection_target_point(
								checkpoint_index = index - 1,
								intersection = intersection,
								waypoint_1 = previous_waypoint,
								waypoint_2 = waypoint
							)

					# No further along intersections exist in the segment, so maintain current target point
					assert(current_target_point is not None)
					return (current_checkpoint_index, current_target_point)
				else:
					if current_checkpoint_index == index - 1:
						# We previously had a target point in this segment, but no longer intersect, so maintain that target point.
						# This happens when the look-ahead distance shrinks due to angular velocity limitng immediately after
						# crossing a new checkpoint. This results in zero intersections, but we have a valid target point.
						assert(current_target_point is not None)
						return (current_checkpoint_index, current_target_point)
					else:
						# We're looking beyond the current target segment, and no intersections exist,
						# so we can choose the segment end point as the target point
						return (
							index - 1,
							TargetPoint(
								position = waypoint.position,
								distance_along_path = waypoint.distance_along_path,
								target_velocity = waypoint.target_velocity
							)
						)

		# All remaining segment end points are within `look_ahead_distance`,
		# so return the end point as the target point
		return (
			end_index(path),
			TargetPoint(
				position = path.end_point.position,
				distance_along_path = path.end_point.distance_along_path,
				target_velocity = path.end_point.target_velocity
			)
		)

	def create_intersection_target_point(
		self,
		checkpoint_index: int,
		intersection: Point,
		waypoint_1: Waypoint,
		waypoint_2: Waypoint
	) -> tuple[int, TargetPoint]:
		"""
		Returns a target point from an intersection with a segment
		"""
		distance_along_path = waypoint_1.distance_along_path + distance_between(waypoint_1, intersection)

		proportion_of_segment = distance_between(waypoint_1, intersection) / (waypoint_2.distance_along_path - waypoint_1.distance_along_path)

		if checkpoint_index == 0:
			# Scale first segment velocity by v = x / (a * x + (1 - a)) and to prevent extremely slow start up
			a = 0.9
			velocity_scalar = proportion_of_segment / (a * proportion_of_segment + (1.0 - a))
		else:
			velocity_scalar = proportion_of_segment

		target_velocity = waypoint_1.target_velocity + velocity_scalar * (waypoint_2.target_velocity - waypoint_1.target_velocity)

		return (checkpoint_index, TargetPoint(
			position = intersection,
			distance_along_path = distance_along_path,
			target_velocity = target_velocity
		))

	def line_segment_circle_intersections(self, pose: RobomowerPose, point_1: PointProtocol, point_2: PointProtocol, look_ahead_distance: float) -> list[Point]:
		"""
		Returns a list of intersections between the line segment and circle defined by (center: pose.position, radius: look_ahead_distance)
		"""
		valid_intersections = []

		# Extract point values
		current_x = pose.position.x
		current_y = pose.position.y

		x1 = point_1.x
		y1 = point_1.y
		x2 = point_2.x
		y2 = point_2.y

		# Translate points to place current position at the origin:
		# subtract current_x and current_y from [x1, y1] and [x2, y2] to offset the system
		offset_x1 = x1 - current_x
		offset_y1 = y1 - current_y
		offset_x2 = x2 - current_x
		offset_y2 = y2 - current_y

		# Calculate discriminant
		dx = offset_x2 - offset_x1
		dy = offset_y2 - offset_y1
		dr_squared = dx**2 + dy**2
		D = offset_x1 * offset_y2 - offset_x2 * offset_y1
		discriminant = look_ahead_distance**2 * dr_squared - D**2

		# If discriminant is >= 0, at least one intersection exists
		if discriminant >= 0.0:
			# Solve for intersection(s)
			intersection_x1 = (D * dy + sgn(dy) * dx * sqrt(discriminant)) / dr_squared
			intersection_y1 = (-D * dx + abs(dy) * sqrt(discriminant)) / dr_squared
			intersection_x2 = (D * dy - sgn(dy) * dx * sqrt(discriminant)) / dr_squared
			intersection_y2 = (-D * dx - abs(dy) * sqrt(discriminant)) / dr_squared

			# Offset the system back to its original position
			intersection_1 = Point(intersection_x1 + current_x, intersection_y1 + current_y)
			intersection_2 = Point(intersection_x2 + current_x, intersection_y2 + current_y)

			min_x = min(x1, x2)
			min_y = min(y1, y2)
			max_x = max(x1, x2)
			max_y = max(y1, y2)

			# Check for valid solutions within line segment boundaries
			for intersection in [intersection_1, intersection_2]:
				if is_float_within(intersection.x, min_x, max_x) and is_float_within(intersection.y, min_y, max_y):
					valid_intersections.append(intersection)

		return valid_intersections

	#
	# Reference Point
	#
	def next_reference_point(
		self,
		path: Path,
		pose: RobomowerPose,
		current_reference_point_index: int,
		current_reference_point: ReferencePoint,
		checkpoint_index: int
	) -> tuple[int, ReferencePoint]:

		potential_reference_points = []

		min_index = current_reference_point_index
		max_index = min(checkpoint_index, end_index(path) - 1)

		for index in inclusive_range(min_index, max_index):
			segment = [path[index], path[index + 1]]
			orthogonal_point = orthogonal_point_on_segment(pose.position, segment)

			if orthogonal_point is not None:
				distance_along_path = path[index].distance_along_path + distance_between(path[index], orthogonal_point)

				if distance_along_path > current_reference_point.distance_along_path:
					reference_point = ReferencePoint(
						position = orthogonal_point,
						distance_along_path = distance_along_path
					)

					pose_distance = distance_between(pose.position, orthogonal_point)

					# Score reference point estimates by how close the pose is and how near they are down the path length.
					# This hopefully prevents problems with intersecting paths and being closer to a later, overlapping segment.
					inverse_score = (0.75 * pose_distance + 0.6 * distance_along_path / path.total_distance)

					potential_reference_points.append((index, reference_point, inverse_score))

		if len(potential_reference_points) > 0:
			index, reference_point, _ = min(potential_reference_points, key = itemgetter(2))
			return (index, reference_point)
		else:
			return (current_reference_point_index, current_reference_point)

	#
	# Command Creation
	#
	def calculate_signed_curvature(self, pose: RobomowerPose, target_point: TargetPoint) -> float:
		"""
		Radius of rotation: R = l^2 / 2·d
		where l is distance to target point and d is orthogonal distance from target point to robot heading vector
		
		Curvature: C = 1 / R

		Result is positive if target point is to the right of robot heading vector
		"""

		distance_to_target_point = distance_between(pose.position, target_point)

		signed_orthogonal_distance_to_target_point = signed_point_vector_distance(
			point = target_point,
			vector_start = pose.position,
			vector_angle = pose.heading
		)
		
		return (2.0 * signed_orthogonal_distance_to_target_point) / distance_to_target_point**2

	def handle_end_conditions(
		self,
		path: Path,
		pose: RobomowerPose,
		target_point: TargetPoint,
		final_approach_velocity: float,
		end_condition_distance: float,
		previous_distance_remaining: float, 
		dt: float
	) -> tuple[float, bool, float]:

		distance_remaining = distance_between(pose.position, path.end_point)

		if distance_remaining < end_condition_distance:
			is_path_complete = True
			target_velocity = 0.0
		elif are_points_equal(target_point, path.end_point) and (distance_remaining > previous_distance_remaining):
			is_path_complete = True
			target_velocity = 0.0
		else:
			is_path_complete = False

			final_deceleration = -pose.velocity**2 / (2.0 * distance_remaining)
			velocity_delta = final_deceleration * dt

			target_velocity = max(pose.velocity + velocity_delta, final_approach_velocity)

		return (target_velocity, is_path_complete, distance_remaining)

	def limit_velocity(self, pose: RobomowerPose, target_velocity: float, max_velocity: float, max_acceleration: float, dt: float) -> float:
		limited_velocity = constrain(target_velocity, 0.0, max_velocity)

		max_velocity_delta = max_acceleration * dt
		velocity_delta = constrain(limited_velocity - pose.velocity, -max_velocity_delta, max_velocity_delta)

		return pose.velocity + velocity_delta

	def limit_angular_velocity(
		self,
		pose: RobomowerPose,
		target_angular_velocity: float,
		max_angular_velocity: float,
		max_angular_acceleration: float,
		dt: float
	) -> float:

		limited_angular_velocity = constrain(target_angular_velocity, -max_angular_velocity, max_angular_velocity)
		
		max_angular_velocity_delta = max_angular_acceleration * dt
		angular_velocity_delta = constrain(limited_angular_velocity - pose.angular_velocity, -max_angular_velocity_delta, max_angular_velocity_delta)

		return pose.angular_velocity + angular_velocity_delta

	def create_command(
		self,
		pose: RobomowerPose,
		signed_curvature: float,
		target_velocity: float,
		max_velocity: float,
		max_acceleration: float,
		max_angular_velocity: float,
		max_angular_acceleration: float,
		track_width: float,
		wheel_radius: float,
		dt: float
	) -> RobomowerCommand:

		if self.is_path_complete:
			return RobomowerCommand(
				right_wheel_angular_velocity = 0.0,
				left_wheel_angular_velocity = 0.0
			)

		limited_target_velocity = max_angular_velocity / fabs(signed_curvature) if signed_curvature != 0.0 else max_velocity

		limited_velocity = self.limit_velocity(
			pose = pose,
			target_velocity = min(target_velocity, limited_target_velocity),
			max_velocity = max_velocity,
			max_acceleration = max_acceleration,
			dt = dt
		)

		limited_right_wheel_velocity = 0.5 * limited_velocity * (2.0 - track_width * signed_curvature)
		limited_left_wheel_velocity = 0.5 * limited_velocity * (2.0 + track_width * signed_curvature)

		target_angular_velocity = (limited_right_wheel_velocity - limited_left_wheel_velocity) / track_width

		limited_angular_velocity = self.limit_angular_velocity(
			pose = pose,
			target_angular_velocity = target_angular_velocity,
			max_angular_velocity = max_angular_velocity,
			max_angular_acceleration = max_angular_acceleration,
			dt = dt
		)

		right_wheel_velocity = limited_velocity + 0.5 * limited_angular_velocity * track_width
		left_wheel_velocity = limited_velocity - 0.5 * limited_angular_velocity * track_width

		return RobomowerCommand(
			right_wheel_angular_velocity = right_wheel_velocity / wheel_radius,
			left_wheel_angular_velocity = left_wheel_velocity / wheel_radius
		)

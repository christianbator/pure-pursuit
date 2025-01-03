#
# geometry.py
# pure-pursuit
#
# Created by Christian Bator on 01/02/2025
#

from typing import Any, Protocol, Iterator, Sequence, Optional
from math import degrees, dist, pi, atan2, fabs, sqrt, cos, sin

from pure_pursuit.utilities.math_extensions import is_float_equal, is_float_within, sgn

#
# Point
#
class PointProtocol(Protocol):

    @property
    def x(self) -> float:
        ...

    @property
    def y(self) -> float:
        ...

class Point:

    def __init__(self, x: float, y: float):
        self._x = x
        self._y = y

    @property
    def x(self) -> float:
        return self._x

    @property
    def y(self) -> float:
        return self._y

    def __str__(self) -> str:
        return f"({self.x:0,.3f}, {self.y:0,.3f})"

    def to_json(self) -> dict[str, Any]:
        return {
            "x": self.x,
            "y": self.y
        }

#
# ReferencePoint
#
class ReferencePoint:

    def __init__(self, position: Point, distance_along_path: float):
        self._position = position
        self._distance_along_path = distance_along_path

    @property 
    def position(self) -> Point:
        return self._position

    @property
    def x(self) -> float:
        return self._position.x

    @property
    def y(self) -> float:
        return self._position.y

    @property
    def distance_along_path(self) -> float:
        return self._distance_along_path

    def __str__(self) -> str:
        return f"(x: {self.x:0,.3f}, y: {self.y:0,.3f}, d: {self.distance_along_path:0,.3f})"

#
# TargetPoint
#
class TargetPoint(ReferencePoint):

    def __init__(self, position: Point, distance_along_path: float, target_velocity: float):
        self._position = position
        self._distance_along_path = distance_along_path
        self._target_velocity = target_velocity

    @property 
    def position(self) -> Point:
        return self._position

    @property
    def x(self) -> float:
        return self._position.x

    @property
    def y(self) -> float:
        return self._position.y

    @property
    def distance_along_path(self)-> float:
        return self._distance_along_path

    @property
    def target_velocity(self) -> float:
        return self._target_velocity

    def __str__(self) -> str:
        return f"(x: {self.x:0,.3f}, y: {self.y:0,.3f}, d: {self.distance_along_path:0,.3f}, v: {self.target_velocity:0,.2f})"

#
# Waypoint
#
class Waypoint(TargetPoint):

    def __init__(self, position: Point, angle: float, distance_along_path: float, target_velocity: float):
        self._position = position
        self._angle = angle
        self._distance_along_path = distance_along_path
        self._target_velocity = target_velocity

    @property 
    def position(self) -> Point:
        return self._position

    @property
    def x(self) -> float:
        return self._position.x

    @property
    def y(self) -> float:
        return self._position.y

    @property
    def angle(self) -> float:
        return self._angle

    @property
    def distance_along_path(self) -> float:
        return self._distance_along_path

    @property
    def target_velocity(self) -> float:
        return self._target_velocity

    def __str__(self) -> str:
        return f"(x: {self.x:0,.3f}, y: {self.y:0,.3f}, a: {degrees(self.angle):0,.1f}, d: {self.distance_along_path:0,.3f}, v: {self.target_velocity:0,.3f})"

#
# Path
#
class Path:

    def __init__(self, waypoints: list[Waypoint]):
        self._waypoints = waypoints

    @property
    def waypoints(self) -> list[Waypoint]:
        return self._waypoints
    
    @property
    def start_point(self) -> Waypoint:
        return self.waypoints[0]

    @property
    def end_point(self) -> Waypoint:
        return self.waypoints[-1]

    @property
    def total_distance(self) -> float:
        return self.end_point.distance_along_path

    def __len__(self) -> int:
        return len(self.waypoints)

    def __getitem__(self, index) -> Waypoint:
        return self.waypoints[index]

    def __iter__(self) -> Iterator[Waypoint]:
        return iter(self.waypoints)

    def __str__(self) -> str:
        result = "[\n"

        for waypoint in self.waypoints:
            result += "  "
            result += waypoint.__str__()
            result += ",\n"

        result += "]"

        return result

#
# Utility Methods
#
def distance_between(point_1: PointProtocol, point_2: PointProtocol) -> float:
    return dist([point_1.x, point_1.y], [point_2.x, point_2.y])

def bounding_box(points: Sequence[PointProtocol]) -> tuple[float, float, float, float]:
    xs = [point.x for point in points]
    ys = [point.y for point in points]

    return (min(xs), max(xs), min(ys), max(ys))

def are_points_equal(point_1: PointProtocol, point_2: PointProtocol) -> bool:
    return is_float_equal(point_1.x, point_2.x) and is_float_equal(point_1.y, point_2.y)

def circumference(radius: float) -> float:
    return 2.0 * pi * radius

def add_angles(angle_1: float, angle_2: float) -> float:
    return normalize_angle(angle_1 + angle_2)

def subtract_angles(angle_1: float, angle_2: float) -> float:
    return normalize_angle(angle_2 - angle_1)

def normalize_angle(angle: float) -> float:
    result = angle

    if result > pi:
        result -= 2.0 * pi
    elif result <= -pi:
        result += 2.0 * pi

    return result

def line_segment_angle(segment: list[PointProtocol]) -> float:
    a = segment[0]
    b = segment[1]

    return atan2(b.y - a.y, b.x - a.x)

def angle_between_segments(segment_1: list[PointProtocol], segment_2: list[PointProtocol]) -> float:
    segment_1_angle = line_segment_angle([segment_1[0], segment_1[1]])
    segment_2_angle = line_segment_angle([segment_2[0], segment_2[1]])

    return subtract_angles(segment_1_angle, segment_2_angle)
 
def point_vector_orientation(point: PointProtocol, vector_start: PointProtocol, vector_angle: float) -> int:
    return point_line_orientation(point, [vector_start, add_point_on_vector(vector_start, vector_angle)])

def point_line_orientation(point: PointProtocol, line: Sequence[PointProtocol]) -> int:
    """
    Parameters: point and line as [a, b] where a and b are points on line 
    Returns: orientation of [a, point] x [a, b], 1.0 if point is collinear with or right of [a, b], -1.0 if point is left of [a, b]
    """
    a = line[0]
    b = line[1]

    cross_product = (point.x - a.x) * (b.y - a.y) - (point.y - a.y) * (b.x - a.x)

    return sgn(cross_product)

def point_vector_distance(point: PointProtocol, vector_start: PointProtocol, vector_angle: float) -> float:
    return point_line_distance(point, [vector_start, add_point_on_vector(vector_start, vector_angle)])

def point_line_distance(point: PointProtocol, line: list[PointProtocol]):
    x1 = line[0].x
    y1 = line[0].y
    x2 = line[1].x 
    y2 = line[1].y

    # Standard ax + by + c = 0 form, but distributed (x2 - x1) to avoide divide by zero
    a = -(y2 - y1)
    b = x2 - x1
    c = x1 * y2 - x2 * y1

    return fabs(a * point.x + b * point.y + c) / sqrt(a**2 + b**2)

def signed_point_vector_distance(point: PointProtocol, vector_start: PointProtocol, vector_angle: float) -> float:
    return signed_point_line_distance(point, [vector_start, add_point_on_vector(vector_start, vector_angle)])

def signed_point_line_distance(point: PointProtocol, line: list[PointProtocol]) -> float:
    orientation = point_line_orientation(point = point, line = line)
    distance = point_line_distance(point = point, line = line)
    
    return orientation * distance

def orthogonal_point_on_segment(point: PointProtocol, segment: Sequence[PointProtocol]) -> Optional[Point]:
    a = segment[0]
    b = segment[1]

    proportion_of_segment = ((point.x - a.x) * (b.x - a.x) + (point.y - a.y) * (b.y - a.y)) / ((b.x - a.x)**2 + (b.y - a.y)**2)

    if is_float_within(proportion_of_segment, 0.0, 1.0):
        return Point(a.x + proportion_of_segment * (b.x - a.x), a.y + proportion_of_segment * (b.y - a.y))
    else:
        return None

def add_point_on_vector(vector_start: PointProtocol, vector_angle: float) -> Point:
    return Point(vector_start.x + cos(vector_angle), vector_start.y + sin(vector_angle))

#
# matplotlib_extensions.py
# pure-pursuit
#
# Created by Christian Bator on 01/02/2025
#

from typing import Sequence, Optional
from math import radians, cos, sin

from pure_pursuit.utilities.math_extensions import inclusive_range
from pure_pursuit.utilities.geometry import PointProtocol

from matplotlib.pyplot import Axes
from matplotlib.lines import Line2D

#
# Draw Points
#
def draw_point(axes: Axes, point: Optional[PointProtocol], size: float = 4, color: Optional[str] = None, alpha: float = 1.0, label: Optional[str] = None) -> Line2D:
	elements: list[Line2D] = axes.plot(
		[point.x] if point is not None else [],
		[point.y] if point is not None else [],
		"o",
		markersize = size,
		color = color,
		alpha = alpha,
		label = label
	)

	return elements[0]

def draw_points(axes: Axes, points: Sequence[PointProtocol], size: float = 4.0, color: Optional[str] = None, alpha: float = 1.0, label: Optional[str] = None) -> Line2D:
	elements: list[Line2D] = axes.plot(
		[point.x for point in points],
		[point.y for point in points],
		"o",
		markersize = size,
		color = color,
		alpha = alpha,
		label = label
	)

	return elements[0]

#
# Draw Lines
#
def draw_line(axes: Axes, path: Sequence[PointProtocol], show_points: bool = False, line_style: str = "-", line_width: float = 1.5, line_color: Optional[str] = None, line_label: Optional[str] = None) -> Line2D:
	elements: list[Line2D] = axes.plot(
		[point.x for point in path], 
		[point.y for point in path],
		marker = "." if show_points else None,
		linestyle = line_style,
		linewidth = line_width,
		color = line_color,
		label = line_label
	)

	return elements[0]

#
# Draw Circles
#
def calculate_circle_points(center: PointProtocol, radius: float) -> tuple[list[float], list[float]]:
	angles = []
	for i in inclusive_range(0, 360, 10):
		angle_in_radians = radians(float(i))
		angles.append(angle_in_radians)
	
	xs = []
	ys = []
	for angle in angles:
		xs.append(center.x + radius * cos(angle))
		ys.append(center.y + radius * sin(angle))

	return (xs, ys)

def draw_circle(axes: Axes, center: PointProtocol, radius: float, line_width: float = 1.0, color: Optional[str] = None) -> Line2D:
	circle_points = calculate_circle_points(center, radius)
	elements: list[Line2D] = axes.plot(*circle_points, linestyle = "-", linewidth = line_width, color = color)

	return elements[0]

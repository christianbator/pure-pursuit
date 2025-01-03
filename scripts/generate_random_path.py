#
# generate_random_path.py
# pure-pursuit
#
# Created by Christian Bator on 01/02/2025
#

from typing import Optional
import json
import argparse
from pathlib import Path
from random import seed, random
from math import radians, pi, cos, sin

from pure_pursuit.utilities.geometry import Point, line_segment_angle

#
# Constants
#
RANDOM_SEED = 1
NUM_POINTS = 16
MIN_DISTANCE = 0.3 # m (12 inches)
MAX_DISTANCE = 2.0 # m (6 ft 6 inches)
MAX_ANGLE_DELTA = radians(108.0)

#
# Utility Methods
#
def random_in_range(min_value: float, max_value: float) -> float:
	return min_value + (random() * (max_value - min_value))

def generate_point_after(point_1: Optional[Point], point_2: Point, min_distance: float, max_distance: float, max_angle_delta: float) -> Point:
	if point_1 is None:
		theta = random_in_range(0.0, 2.0 * pi)
	else:
		previous_theta = line_segment_angle([point_1, point_2])
		delta_theta = random_in_range(-max_angle_delta, max_angle_delta)
		theta = previous_theta + delta_theta

	radius = random_in_range(min_distance, max_distance)

	x = point_2.x + radius * cos(theta)
	y = point_2.y + radius * sin(theta)

	return Point(x, y)

#
# Main
#
def main(num_paths: int, output_dir: Path):
	seed(RANDOM_SEED)

	path_text = "path" if num_paths == 1 else "paths"
	print(f"> Generating {num_paths} {path_text} ...")
	print(f"> Saving to '{output_dir}'...")

	for path_num in range(0, num_paths):
		path_name = f"random-path-{path_num + 1}"
		points = [Point(0.0, 0.0)]

		for index in range(1, NUM_POINTS):
			point_1 = None if (index - 2) < 0 else points[index - 2]
			point_2 = points[index - 1]

			point = generate_point_after(
				point_1,
				point_2,
				min_distance = MIN_DISTANCE,
				max_distance = MAX_DISTANCE,
				max_angle_delta = MAX_ANGLE_DELTA
			)	

			points.append(point)
			
		filepath = output_dir / f"{path_name}.json"

		with open(filepath, "w") as file:
			points_json = [point.to_json() for point in points]
			file.write(json.dumps(points_json, indent = 4, sort_keys = True))
		
	print("> Done")

if __name__ == "__main__":
	parser = argparse.ArgumentParser()
	parser.add_argument("-n", "--num_paths", type = int, required = True)
	parser.add_argument("-o", "--output_dir", required = True)
	args = parser.parse_args()

	main(num_paths = args.num_paths, output_dir = Path(args.output_dir))

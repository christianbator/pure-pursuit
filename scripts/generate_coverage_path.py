#
# generate_coverage_path.py
# pure-pursuit
#
# Created by Christian Bator on 01/02/2025
#

import json
import argparse
from pathlib import Path

from pure_pursuit.utilities.geometry import Point

#
# Constants
#
STRIPE_WIDTH = 0.5
WAYPOINT_DISTANCE = 2.0

#
# Main
#
def main(max_x: int, max_y: int, output_dir: Path):
	print(f"> Generating coverage path ({max_x} x {max_y} m\u00b2) ...")

	x = 0.0
	y = 0.0

	points = [Point(x, y)]
	is_direction_up = True

	while True:
		if is_direction_up:
			while y < max_y:
				y += WAYPOINT_DISTANCE
				points.append(Point(x, y))
		else:
			while y > 0:
				y -= WAYPOINT_DISTANCE
				points.append(Point(x, y))

		x += STRIPE_WIDTH

		if x < max_x:
			points.append(Point(x, y))
			is_direction_up = not is_direction_up
		else:
			break

	path_name = f"coverage-path-{max_x}x{max_y}"
	filepath = output_dir / f"{path_name}.json"
	print(f"> Saving to '{filepath}'...")

	output_dir.mkdir(parents = True, exist_ok = True)

	with open(filepath, "w") as file:
		points_json = [point.to_json() for point in points]
		file.write(json.dumps(points_json, indent = 4, sort_keys = True))

	print("> Done")

if __name__ == "__main__":
	parser = argparse.ArgumentParser()
	parser.add_argument("-x", "--max_x", type = int, required = True)
	parser.add_argument("-y", "--max_y", type = int, required = True)
	parser.add_argument("-o", "--output_dir", required = True)
	args = parser.parse_args()

	main(max_x = args.max_x, max_y = args.max_y, output_dir = Path(args.output_dir))

#
# math_extensions.py
# pure-pursuit
#
# Created by Christian Bator on 01/02/2025
#

from typing import Sized
from math import isclose

#
# Types
#
Number = int | float
Vector = list[float]
Vector2D = list[Vector] 

#
# Float Comparison
#
def is_float_equal(value: float, test_value: float) -> bool:
	return isclose(value, test_value, abs_tol = 1e-9)

def is_float_less_or_equal(value: float, test_value: float) -> bool:
	if value < test_value:
		return True
	elif is_float_equal(value, test_value):
		return True
	else:
		return False

def is_float_greater_or_equal(value: float, test_value: float) -> bool:
	if value > test_value:
		return True
	elif is_float_equal(value, test_value):
		return True
	else:
		return False

def is_float_within(value: float, min_value: float, max_value: float) -> bool:
	if min_value < value < max_value:
		return True
	elif is_float_equal(value, min_value):
		return True
	elif is_float_equal(value, max_value):
		return True
	else:
		return False

#
# Utility Methods
#
def sgn(number: Number) -> int:
	if number > 0.0:
		return 1
	elif is_float_equal(number, 0.0):
		return 1
	else:
		return -1

def average(list_input: list[Number]) -> float:
	return float(sum(list_input)) / float(len(list_input))

def end_index(list_input: Sized) -> int:
	return len(list_input) - 1

def inclusive_range(start: int, end: int, increment: int = 1) -> range:
	return range(start, end + increment, increment)

def constrain(value: Number, min_value: Number, max_value: Number) -> Number:
	if value < min_value:
		return min_value
	elif value > max_value:
		return max_value
	else:
		return value

def is_increasing(list_input: list[Number]) -> bool:
    return all(x <= y for x, y in zip(list_input, list_input[1:]))

def is_decreasing(list_input: list[Number]) -> bool:
    return all(x >= y for x, y in zip(list_input, list_input[1:]))

def is_strictly_increasing(list_input: list[Number]) -> bool:
    return all(x < y for x, y in zip(list_input, list_input[1:]))

def is_strictly_decreasing(list_input: list[Number]) -> bool:
    return all(x > y for x, y in zip(list_input, list_input[1:]))

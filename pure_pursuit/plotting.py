#
# plotting.py
# pure-pursuit
#
# Created by Christian Bator on 01/03/2025
#

from math import ceil, floor

from pure_pursuit.simulation.simulation_data import SimulationData

from matplotlib import pyplot as plot
from matplotlib.figure import Figure

#
# Plotting
#
def plot_cross_track_errors(simulation_data: SimulationData) -> Figure:
    figure = plot.figure("Cross Track Errors", figsize = (10, 6))
    axes = figure.add_subplot(111)

    x_data = range(0, len(simulation_data.cross_track_errors))

    axes.plot(
        x_data, 
        [0] * len(simulation_data.cross_track_errors)
    )

    # Convert meters to centimeters
    error_scalar = 100.0 

    axes.plot(
        x_data,
        [error * error_scalar for error in simulation_data.cross_track_errors]
    )

    y_min = min(simulation_data.cross_track_errors) * error_scalar
    y_max = max(simulation_data.cross_track_errors) * error_scalar

    buffer = 1.2
    axes.set_ylim((buffer * y_min, buffer * y_max))
    axes.set_yticks(range(floor(buffer * y_min), ceil(buffer * y_max), 1))

    axes.set_title("Cross Track Errors", fontweight = "bold", pad = 20)
    axes.set_xlabel("Step", fontweight = "bold", labelpad = 10)
    axes.set_ylabel("Error (cm)", fontweight = "bold", labelpad = 10)

    return figure

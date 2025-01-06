# Pure Pursuit
A pure pursuit implementation in python.  
  
Specifically, this is an implementation of an adaptive pure pursuit controller for diff drive path tracking, including animated simulation and cross track error reporting. The code is type-annotated for easy porting to other languages.  
<br>
<p align="center">
  <img src="docs/random-path-1.gif" width="480">
</p>

## Quickstart
Requires `python >= 3.12`
```
git clone git@github.com:christianbator/pure-pursuit.git
cd pure-pursuit

python3 -m venv .venv
source .venv/bin/activate

pip install .
pure-pursuit -p paths/random-path-1.json \
             -s config/simulation-config.json \
             -r config/robomower-config.json \
             -c config/pure-pursuit-config.json \
             --animate --follow --graphs
```

## Usage
You can run pure pursuit simulations with or without animations and the cross track error plot. Follow the instructions below to get started:

### Simulation
```
pure-pursuit -p paths/random-path-1.json \
             -s config/simulation-config.json \
             -r config/robomower-config.json \
             -c config/pure-pursuit-config.json
```

**Configuration**  
The simulation requires 3 configuration arguments and associated json files (defaults are provided):
1. `-s`: simulation config file
```
{
  "control_frequency": 50, # Hertz, so time between steps is (1 / control_frequency) seconds
  "avg_abs_cross_track_error_threshold": 0.01 # meters, this threshold is for reporting if the controller did a good job tracking the path on average
}
```

2. `-r`: robot config file
```
{
  "wheel_radius": 0.16, # meters, the radius of the drive wheel
  "track_width": 0.60, # meters, the distance between the two drive wheels
  "length": 1.22, # meters, the overall length of the robot
  "max_velocity": 1.75, # meters/second, the max allowed linear velocity
  "max_acceleration": 0.2, # meters/second^2, the max allowed linear acceleration / deceleration
  "max_angular_velocity": 0.785, # radians/second, the max allowed angular velocity
  "max_angular_acceleration": 1.571 # radians/second^2, the max allowed angular acceleration / deceleration
}
```

3. `-c`: pure pursuit config file
```
{
  "min_look_ahead_distance": 0.10, # meters, the minimum distance from the robot's center to look for a path intersection
  "max_look_ahead_distance": 1.00, # meters, the maximum distance from the robot's center to look for a path intersection
  "angle_velocity_parameter": 0.3, # constant, tunable parameter, see Background > Path Analysis below
  "final_approach_velocity": 0.10, # meters/second, constant velocity in the last path segment to appraoch the goal
  "end_condition_distance": 0.05, # meters, how close to the goal we consider the path "complete"
}
```
<br>
After running the simulation it will output some statistics as computed in `pure_pursuit/simulation/simulation_data.py`:
<br>
<br>
<p align="left">
  <img src="docs/simulation-output.png" height="320">
</p>

**Cross Track Error**  
The cross track error is defined as the absolute orthogonal distance from the path to the robot's point of rotation (mid-point of the axle). The red dot in the animation is called the "reference point", and it represents where the robot would be if the path tracking was perfect. The distance from the robot's point of rotation (mid-point of the axle) to the reference point is the _cross track error_.

You can view the cross track error by passing the `--graphs` argument:
```
pure-pursuit -p paths/random-path-1.json \
             -s config/simulation-config.json \
             -r config/robomower-config.json \
             -c config/pure-pursuit-config.json \
             --graphs
```

Example cross track error graph:
<p align="center">
  <img src="docs/cross-track-error.png" height="320">
</p>

### Animation
You can view an animation of the simulation by passing the `--animate` argument. If you specify the optional `--follow` argument, the animation will center the robot's position and move along with it:
```
pure-pursuit -p paths/random-path-1.json \
             -s config/simulation-config.json \
             -r config/robomower-config.json \
             -c config/pure-pursuit-config.json \
             --animate --follow
```

To pause the animation, click anywhere in the animation window.

<p align="center">
  <img src="docs/random-path-1.gif" width="480">
</p>

The animation shows a few things:
1. Grey line: Path to follow
1. Red line: Actual path traveled
1. 🟢: Robot's current pose, green line is the heading
1. 🔵: Pure pursuit controller's current target point (intersection with path)
1. 🔴: Reference point on the path (orthogonal distance to robot's pose)
1. 🟣: Pure pursuit controller's current checkpoint (last "seen" waypoint of the path)
1. Green ring: Look-ahead distance from current pose
1. L, R: Bottom left, the left and right wheel angular velocities
1. Red rectangle: Robot's frame
1. t: Top left, the current simulation time 
1. v, ω: Top left, the robot's linear velocity and angular velocity 
  
**Saving Animations**  
Prerequisites:
1. [ffmpeg](https://ffmpeg.org) (`brew install ffmpeg` or `sudo apt install ffmpeg`)

You can save the animation as an mp4 by passing the `--save-animation` argument (only works if `--animate` is also passed):
```
pure-pursuit -p paths/random-path-1.json \
             -s config/simulation-config.json \
             -r config/robomower-config.json \
             -c config/pure-pursuit-config.json \
             --animate --follow --save-animation
```  

This will use ffmpeg to write the animation to the `output/animations/` directory, and you'll see output like this:
```
> Saving animation to 'output/animations/random-path-1.mp4'...
  > Frame: 3118 / 3118
  > Done
```
Note: This won't display the animation, only save it to a file. This is useful after you've already run an interesting simulation and want to save it for later.

### Generating Paths
There are two helper scripts in `scripts/` to generate paths:
1. `generate_coverage_path.py`:
```
python3 scripts/generate_coverage_path.py [-h] -x MAX_X -y MAX_Y -o OUTPUT_DIR
```
This will generate a coverage path of a rectangular area defined by MAX_X and MAX_Y (think robotic lawn mower). Specify the max coordinates and output directory like so:
```
python3 scripts/generate_coverage_path.py -x 3 -y 4 -o paths
```

2. `generate_random_path.py`:
```
python3 scripts/generate_random_path.py [-h] -n NUM_PATHS -o OUTPUT_DIR
```
This will generate a random walk with varying distances and angles between waypoints. Specify the number of random paths and output directory like so:
```
python3 scripts/generate_random_path.py -n 2 -o paths
```

### Building
If you make any changes, you can run the mypy analyzer to verify the type annotations with the build script in `build/`:
```
./build/build.sh
```

## Background
The original [pure pursuit](https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf) controller was introduced by R. Craig Coulter in 1992, and many adaptive variants of it have been proposed throughout the years.  

The core idea is to look ahead along the path and control the robot in an arc to the specified "look-ahead" point. Improvements to this basic idea are to vary how far along the path to look based on the path's geometry and the robot's current pose.

This implementation does the following:
<br>

### Path Analysis
File:
- `pure_pursuit/controller/path_adapter.py`

Parameters:
- `max_velocity`: Max linear velocity
- `max_acceleration`: Max absolute acceleration (also deceleration)
- `angle_velocity_parameter`: Tunable parameter for how fast to take sharp angles in the path (bigger value -> faster)

The first step is to transform the given path into a set of WayPoints:
```python
WayPoint
  x: float
  y: float
  distance_along_path: float
  target_velocity: float
```

To calculate the target velocity at a particular waypoint, we first do a forward pass and estimate the target velocity based on the parameters defined above. Then we do a backwards pass to verify the target deceleration between waypoints doesn't violate the `max_acceleration` parameter.   

### Control
File:
- `pure_pursuit/controller/pure_pursuit_controller.py`

In each update step, we ask the controller to provide a new command for the robot to execute. The command is a simple tuple:
```python
RobotCommand
  right_wheel_angular_velocity: float
  left_wheel_angular_velocity: float
```
The controller computes the command based on the path geometry, the robot's current pose, and the constraints defined by the configuration parameters. The high-lvel steps are:
1. Calculate look-ahead distance based on current velocity
1. Search for next target point
1. Calculate target velocity at next target point
1. Caclulate the signed curvature of the arc from the current pose to the target point
1. Create a command to drive along the arc to the target point
1. Handle end conditions

### Pose Propagation
File:
- `pure_pursuit/model/diff_drive_kinematics.py`

After executing the calculated command, we update the robot's pose by propagating it forward using basic kinematics equations.

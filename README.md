# TAS Racer

This project was part of the Autonomous Systems course at the Technical University of Munich. The goal was to improve the racing stack of an autonomous car in ROS-2 by contributing one key feature. I focused on generating a fast trajectory by combining the shortest distance and minimum curvature. This contribution helped me earn the top grade of A* for the project. The final presentation is available [here](./Presentation.pptx).

Within the work, I implemented and evaluated 4 different types of path planners:

- Shortest Path using A-Star
- Shortest Path by solving an optimization problem
- Minimum Curvature path by solving an optimization problem
- Experiments have shown that the best planner is neither of the both, so I implemented a final planner which interpolates between minimum distance and minimum curvature. With this one I achieved the best lap times

One issue with all these methods was that the path generation had to be done offline using an image of the map. To overcome this issue I added a real time component. I.e., this component does dynamic replanning around the offline calculated paths to be able to adapt to objects.

**Note** I will be using the words trajectory and path equivalently throughout this work. Both mean a series of points (x,y) + pose (yaw).

# Packages

## Global Helpers:

A python directory which is a collection of commonly used functions and classes

## Global Racetrajectory Optimization

This module is concerned with doing two things

1. Generating optimal trajectories
2. Preparing racetracks from images. I.e. just using an image of a racetrack, there are helper functions to 1) generate a 3D-world for simulation in GAZEBO and 2) extracting the center line and track widths as this is required for the path planners.

The module is no real ROS module and thus does not have to be build using colcon, just `cd` into the repo and execute the scripts.

## Localization

Typically `NAV-2` uses `AMCL` for localization. Unfortunately this doesn't work that well in a racetrack, where the information for the laser scanner is basically the same everywhere (no landmarks, similar track width everywhere ...). Thus I implemented my own localization package. It is an imitation of localization via GPS and uses the coordinates that one can read out from GAZEBO.

## Offline Planner

Given a precomputed trajectory, this module takes over the Planner task in `NAV-2`. Given a current location it returns the precomputed path until the goal.

## Online Planner

Improvement on online planner that allows for dynamic recalculation of the optimal route. The offline computed paths are provided as references. When dynamic objects appear, the route is calculated around these objects and back onto the ref-trajectory using `A*`.

## Play Simulation:

This package contains just a script to start a simulation. It reads in the trajectory, spawns a car at the starting position and starts navigation to the goal. 

It is also no proper ros-module, but instead just a python script.

## Tas-2 Simulator:

The package containing the `tas2-simulator`

Adaptions have been made to:

1. The launch files. There are now two new files: `bringup_simulation` to launch GAzebo and RVIZ, `bringup_navstack` to start nav2
2. The configuration files for `nav-2`
3. The folder containing maps and gazebo files to use our own files
4. The TAS car. The car had one major implementation issue which has led to the reference joint used by gazebo for localization to always stay fixed in `x` and `y` but to fall in `z`. It took me a very long time to realize what was up and that this issue was due to the `car` and not my own generated worlds. Unfortunately I needed to read the location for two files `play simulation` and `localization`. Thus, I spent a long time debugging the car and finally fixed the issue by adding a forgotten link between base footprint and base.

# The out Directory

... is the directory where one can find the generated trajectories, maps, and worlds. The name `out` is maybe a bit confusing since it is also the directory where one inserts the image of the track first. Then when running the scripts all output is also stored in this folder.

# How to run

**Note: A more detailed tutorial for the demos is provided later**

Step 1: Generate a black and white image of the racetrack and save it under the name `track.jpeg` in `out/some_folder_name/track.jpeg`.

Step 2: From inside `global_racetrajectory_optimization` run first: `generate_gazebo_world.py` and then `extract_centerline.py`. Both scripts have a command line parser where one can override the path to the `track.jpeg` file.

Step 3:

- For minimum distance path generation using A-Star run: `a_star_shortest_distance_path.py`
- For minimum distance path using optimization or minimum curvature path run: `main_globaltraj.py`. Inside the file you can config which of the three modes to run (i.e. `shortest_path`, `mincurv`, `interp`)

Step 4: Run the Simulation by

- Run `ros2 launch tas2-simulator bringup_simulation.py`(you can choose the gazebo-world via command line arguments)
- Run `ros2 launch tas2-simulator bringup_navstack.py` (you can chosse the map using command line arguments)
- Start simulation by running `python play_simulation.py`. Again, this script has a command line parser which lets one choose the trajectory to follow


## Notebooks

There is also an extra folder `notebooks`. This folder is not really part of my submission. Instead, it has two notebooks concerning the preprocessing steps. The one shows the new preprocessing steps based on image processing, while the other one shows the first version which was based on simple python. It might be interesting to have a look at it to see how both work and why I decided to completely reimplement everything halfway through the project.


# Packages that are no longer part of this repo because they did not work as expected

This should just be a small overview for you about the things that I have done which are no longer part of this repo

## Path Preprocessing V1

Before arriving at the current version of generating a world, and the centerline from an image I already had a working version. This version was not based on image processing but instead only python programming and numpy. Basically all tasks where solved by recursively spawning out from one point in the numpy grid and marking track borders, valid regions and invalid regions. Unfortunately this implementation came with two drawbacks

1. Time: Processing of a map could take upwards of 1 minute. This was not necessarily that bad since generation was done offline anyhow. Still, in the future it would be desirable to move the trajectory generation to be online in this case this would be unacceptable
2. The bigger problem was that the centerline estimation was not good enough. To calculate the centerline I was basically taking one point of one of the track borders and then iterated over the other track border until I found the minimum distance pair. The center point was then exactly half ot the vector connecting these points. The problem of this approach was that it produced centerlines which did not behave well since
    1. Calculating the centerline like this is only an approximation 
    2. Points calculated like this do not follow a clear path but instead the order can be inconsistent

Especially the second part was an issue. I tried to overcome it by manually filtering out points that were inconsistent, then interpolating the rest and then once again sampling from this. Unfortunately, also this did not always produce good results, The problem was now that it was hard to say which point to throw out and when throwing out the good points instead of the outliers the trajectory quickly stated to become uneven. I tried to overcome this by applying Gaussian Blur when filtering out points but still results where not good enough and led to poor performance when calculating the minimum curvature trajectory.

## Online Planner V1

The current version of the online planner is completely implemented in `c++`. Originally, this was not the plan. Implementing the planner in `c++` naturally was more difficult than in python due to for example no numpy support, harder integration of open cv, and also because I already implemented a simulation of the replanning in python (see `src/global_racetrajectory_optimization/adaptive_replanning.py`). Thus, the initial approach was a service/client architecture. The planner for `nav-2` would be very simple. Whenever it is tasked to calculate a new path it would ask a path planning service (written) in python for the new path and then only return it. I spent a long time implementing this architecture (had to learn how to write services, custom interfaces ...). Unfortunately when trying out my approach I realized that it is not doable in ROS2.

The issue was that when `nav-2` calls the path-planner plugin, this call is already done in a callback way in its own process. Then the planner would again call the python service which would open a second callback in the first callback which always led for the nodes to crash. I spent multiple days trying to fix this issue, for example by providing extra context to the callbacks. The issue was just that there was not a lot of literature to find, especially since these kinds of problems were not a problem with `ROS-1`. Ultimately I had to give up and resort to writing everything in C++.

# Demo

Prerequisites:
- All packages built using colcon
- Current workspace sourced in every terminal (`. install setup.bash`)
- Installed all python packages as stated in `requirements.txt` (`pip install -r requirements.txt`)
- in base (not virtunalenv) python distribution also install  `pip3 install transforms3d`. (Note unfortunately it is not possible to install it via colcon see [here](https://index.ros.org/p/tf_transformations/) for more information.)
- For the online planner, a working installation of opencv (see https://docs.opencv.org/4.x/d2/de6/tutorial_py_setup_in_ubuntu.html)

## Test out path generation
I already created a demo folder in the out directory containing one track.

To try out the demo `cd` into the `global_racetrajectory_optimization` folder and

**Step 1: Generate World**
`python generate_gazebo_world.py`

**Step2: Prepare Track**
`python extract_centerline.py`

**Step 3: Create Path**
- A*: `python a_star_shortest_distance_path.py`
- Minimum distance (optimization):
    1. In `main_globaltraj.py` update line `opt_type=?` to `opt_type=shortest_path`
    2. Run `python main_globaltraj.py`
- Minimum curvature
    1. In `main_globaltraj.py` update line `opt_type=?` to `opt_type=mincurv`
    2. Run `python main_globaltraj.py`
- Interpolated:
    1. In `main_globaltraj.py` update line `opt_type=?` to `opt_type=interp`
    2. In the `params/racecar.ini` you can choose the interpolation constant `alpha` (see `optim_opts_interp`). Setting it to 1 means only minimum curvature, to 0 only shortest distance, and everything in between interpolates.
    2. Run `python main_globaltraj.py`

**Important** Finally, all trajectories are closed, so to start a valid simulation which does not directly abort due to reaching the goal uncomment the last 5 or so points in each trajectory file.

**Attention** On the first run `main_globaltraj.py` will likely throw an error `ValueError: Input vector should be 1-D.`. I don't know where this error comes from since the input is valid. It seems a bit like a bug to me. Anyhow, the problem can easily be fixed by navigating to the file that throws the error (it is part of scipy and thus something along the lines of `/home/tim/.pyenv/versions/tas-demo/lib/python3.10/site-packages/scipy/spatial/distance.py`). In this file replace the block 
```
def _validate_vector(u, dtype=None):
    # XXX Is order='c' really necessary?
    u = np.asarray(u, dtype=dtype, order='c')
    if u.ndim == 1:
        return u
    raise ValueError("Input vector should be 1-D.")
```

with 
```
def _validate_vector(u, dtype=None):
    # XXX Is order='c' really necessary?
    u = np.asarray(u, dtype=dtype, order='c')
    if u.ndim == 1:
        return u
    elif u.ndim == 2:
        #warnings.warn(f"Array of wrong size. Array is of dimensions 2 with content {u}. We try to flatten the array to help")
        return u.flatten()

    raise ValueError("Input vector should be 1-D.")
```

Since of course this might lead to unexpected behavior, I recommend doing this **only** in a virtual environment.


## Test out ros package

**Note** Typically it would be required to move the map and world files generated in the previous step into the corresponding `maps` and `worlds` folder in the `tas2-simulator`. I already did this for you. The only thing you have to do is to **update all absolute paths** such that they correspond to your file system.

### Offline Planner:

Before everything start the simulation by running `ros2 launch tas2-simulator bringup_simulation.py world:=/home/tim/tas2-racer/src/tas2-simulator/models/worlds/world_track1_no_obstacles.world` (Note please change absolute path)


Now you can start the different planning modes by following:

1. Offline Planner, using A*
    1. in `navigation.yaml` make sure the lines for the offline planner are commented out and set the path of `trajectory_file` to point to the right trajectory (absolute path)
    2. Start `navstack` by running `ros2 launch tas2-simulator bringup_navstack.py map:=/home/tim/tas2-racer/src/tas2-simulator/maps/map_track1_no_obstacles.yaml`
    3. Start simulation by running (from inside `src/play_simulation`) `python3 play_simulation.py -t ../../out/tracks/demo/trajectory_astar.csv`

2. Offline Planner, using shortest path (optimization)
    1. in `navigation.yaml` make sure the lines for the offline planner are commented out and set the path of `trajectory_file` to point to the right trajectory (absolute path)
    2. Start `navstack` by running `ros2 launch tas2-simulator bringup_navstack.py map:=/home/tim/tas2-racer/src/tas2-simulator/maps/map_track1_no_obstacles.yaml`
    3. Start simulation by running (from inside `src/play_simulation`) `python3 play_simulation.py -t ../../out/tracks/demo/trajectory_shortest_path.csv`

3. Offline Planner, using Minimum Curvature
    1. in `navigation.yaml` make sure the lines for the offline planner are commented out and set the path of the `trajectory_file` to point to the right trajectory (absolute path)
    2. Start `navstack` by running `ros2 launch tas2-simulator bringup_navstack.py map:=/home/tim/tas2-racer/src/tas2-simulator/maps/map_track1_no_obstacles.yaml`
    3. Start simulation by running (from inside `src/play_simulation`) `python3 play_simulation.py -t ../../out/tracks/demo/trajectory_mincurv.csv`

3. Offline Planner, using Interpolated Approach
    1. in `navigation.yaml` make sure the lines for the offline planner are commented out and set the path of the `trajectory_file` to point to the right trajectory (absolute path)
    2. Start `navstack` by running `ros2 launch tas2-simulator bringup_navstack.py map:=/home/tim/tas2-racer/src/tas2-simulator/maps/map_track1_no_obstacles.yaml`
    3. Start simulation by running (from inside `src/play_simulation`) `python3 play_simulation.py -t ../../out/tracks/demo/trajectory_interp.csv`

### Online Planner

Before everything start the simulation by `ros2 launch tas2-simulator bringup_simulation.py world:=/home/tim/tas2-racer/src/tas2-simulator/models/worlds/world_track1_obstacles.world`

I prepared a shorter trajectory to demonstrate the online planner, it can be found under `out/tracks/demo_dynamic_planning`using this file you can start the simulation via

1. in `navigation.yaml` make sure the lines for the online planner are commented out and set the path of `trajectory_file` to point to the right trajectory, i.e. make it point to the absolute path of `out/tracks/demo_dynamic_planning`
2. Start `navstack` by running `ros2 launch tas2-simulator bringup_navstack.py map:=/home/tim/tas2-racer/src/tas2-simulator/maps/map_track1_obstacles.yaml`
3. Start simulation by running (from inside `src/play_simulation`) `python3 play_simulation.py -t ../../out/tracks/demo_dynamic_planning/trajectory_mincurv.csv`

## Test out dynamic replanning simulation
Since the dynamic replanning does not perfectly work in ROS (as explained in my presentation), I made a simulation to show how it works in theory. From inside `src/global_racetrajectory_optimization`, run `python adaptive_replanning.py`.

A map will be rendered. Using the keyboard you can either move along the trajectory by typing `Enter` into the terminal, you can also spawn a new object right below the current one by typing `s` followed by `Enter`. Please play around with different times of spawning the object and see how the path planning behaves


# Contributions
All the code was written by myself Tim Lindenau. We started as a group of three but both others have not done anything and then midways through left the group.

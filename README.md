# Project Idea
The idea was the implementation of multiple path planners targetet towards racing. More exact three differnt types of planners have been implemented
- Shortest Path using A-Star
- Shortest Path by solving an optimization problem
- Minimum Curvature Path

One issue with all these methods was, that the path generation had to be done offline using an image of the map. To overcome this issue I added a real time component. I.e., this component does dynamic replanning around the offline calculated paths to be ablte to adapt to objects.


# Packages

## GLobal Helpers:
A python directory which is a collection of common functions and classes
## Global Racetrajectory Optimization
This module is all concernec with doing two things
1) Generating optimal trajectories 
2) Preparsing of images of racetracks. I.e. just using an image of racetrackst there are helper functions to 1) generate a 3D-world for simulation in GAZEBO and 2) extracting the center line and track widths as it is required for the path planners

The module is no real ROS module and thus does not have to be build using colcon, just `cd` into the repo and execute the scripts

## Localization
Typically `NAV-2` uses `ACML` for localization. Unfoerntunately this doesnt work that much in a racetrack, where the information for the laser scanner is basically the same everywhere (no landmarks, similar track width everywhere ...). Thus I implemented my own localization package, it is an immitation of Localization via GPS and uses the coordinates that one can read out from GAZEBO 

## Offline Planner
Given a precomputed trajectory, this module takkes over the Planner taks in nav2. Given a current location it returns the path until the goal

## Online Planner
Improvement on online planner which allows for dynamic recalculation of th eoptimal route. The offline computet trajectories are provided as references and when dynamic objects path this way, the route is calculated around these objects and back onto the ref-trajetroy using A star

## Play Simulation:
Scripts to start a simulation. It reads in the trajectory, spwans a car at the starting position and starts navigation to the goal. It is also no proper ros-module, but instead just a python script

## Tas2 simulator:
The package containing the `Tas2 Simulator`

Adaptions have been made to:
1) The launch files. There are now two new files: `bringup_simulation` to launch GAzebo and RVIZ, `bringup_navstack` to start nav2
2) The configuration files for `nav-2`
3) The folder containing maps and gazebo files to use our own files
4) One big difference is that I am no longer using the `tas-car`. The tas-car has some big implementation issue which leads to the reference joint used by gazebo for localization to always stay fixed in `x` and `y` but to fall in `z`. It took me a very long time to realize what was up and that this issue was due to the `car` and not my own generated worlds. Unfortunately I need the location for two files
 `play simulation` and `localization`. I also tried to fix the issue but did not succeed. As a last resort, I am thus, using the turtlebot at the moment.

 # The out Directory
 ... is the directory where one can find the genertated trajectories, maps, and worlds. The name out is maybe a bit confusing since it is also the directory where one inserts the image of the track first, form which the rest is then generated.

 # How to run
 Step 1: Generated a black white image of the racetrack and save it under the name `track.jpeg` in `out/some_folder_name`.
 Step 2: From inside `global_racetrajectory_optimization` run first: `generate_gazebo_world.py` and then `extract_centerline.py`. Both scripts have a command line parser where one must provide the path to the `track.jpeg` file.
Step 3:
- For  minimum distance path generation using A-Star: run `a_star_shortest_distance_path.py`
- For minimum distance path using optimization or minimum curvature path run: `main_globaltraj.py`. Inside the file you can config which of the two modes to run

Step 4: Run the Simulation by
- Run `ros2 launch tas2-simulator bringup_simulation.py`(you can choose the gazebo-world via command line arguments)
- Run `ros2 launch tas2-simulator bringup_navstack.py` (Different configs can be found in the `navigation.yaml` file, deactivate and activate parts by commenting them out)
- Start simulation by running `python play_simulation.py`. Again this model has a command line parser which lets one choose the trajectory to follow

## Packages that are no longer part of this repo because they did not work

## Path Preprocessing V1
Before arriving at the current version of generating a world, and the centerline from an image I already had a working version. This version was not based on image prcessing but instead only python programming and numpy. Basically all tasks where solved by recursiverly spawning out from one point in the numpy grid and marking track borders, valid regions and invalid regions. Onfortunately this implementation came with two drawbacks
1. Time: Parsing of a map could take upwards of 1 minute. This was not necessarily that bad since generation was done offline anyhow. Still in the future it would be desirable to move the trajectory generation to be online in this case this would be unacceptable
2. The bigger problem was that the centerline estimation was not good enough. To calculate the centerline I was basically taking one point of one of ther borders and then iterated over the other track border until I found the minimum distance pair. The center point was then exactly half ot the vector connecting these points. The problem of this approach was that it produced centerlines which did not behave well
    1. Calculating the centerline like this is only an approximation
    2. Points calculated like this do not follow a clear path but istead the order can be inconsistent
Especially the secon part was an issue. I tried to overcome it by manually filtering out points that were incosinstend, than interppolating the rest and then once again sampling from this, but also this did not always produce good results, Th eproblem was now that it is hard to say which point to throw out and when throwing out the outliers instead of the good points the trajectory starts to become uneven. I tried to overcome this by allpying Gaussian Blur when filtering out points but still results where not good enough and led to poor performance when calculating the minimum curvature trajectory

## Online Planner V1
The current version of the online planner is compeletely implemented in `c++`. Originally, this was not the plan. Implementing the planner in `c++` naturally was more difficult than in python due to for example no numpy support, harder integration of open cv, and also because I already implemented a simulation of the replanning in python (see `src/global_racetrajectory_optimization/adaptive_replanning.py`). Thus, the initial approach was a service architecture. The planner intergrated in `nav-2` would be very simple. Whenever it is tasked to calculate a new path it would as a path planning service (written) in python for the new path and then only return it. I spent a long time implementing this architetcure (has to learn how to write services, custom interfaces ...). Unfortunately when trying out my approach I realized that it is not doable in ROS2. 

The issue was that when `nav-2` calls the path-finder this call is already done in a callback way in its own process. Then the planner would again call the python service which would open a second callback in the first callback which always led to the nodes to crash. I spent multiple days trying to fix this issue, for example byp providing extract context to the callbacks. The issue was just that there was not a lot of literature to find, especially since these kinds of problems were not a problem with `ROS-1`. Ultimately I had to give up and resort to writing everything in C++.
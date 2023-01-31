# What is this Package about

This package covers everything that has to do with calculating paths and preparing tracks. It also contains a simulator where I used a dynamic model which suits an rc car well. Any trajectory can be imported into his simulator and it calculate the approximate track time. This package is based on the work of Alexander Heilmeier. However, I significantly extended it to include all the preprocessing, implement a new planner that interpolates between minimum distance and minimum curvature, also implement an A* planner, adapt the simulator to our use case and allow it to process any trajectory given as an input ... 

## Description of scripts
- `a_star_shortest_distance_path.py`: Calculate shortest path using A-Star
- `adaptive_replanning.py`: Simulation to show the replanning methods used in the `online_planner`. Only used to create animations for better understanding
-`extract_centerline.py` Given an image extract track borders, centerline and track widths. Even though it sounds like a common task it is tricky since trivial python approaches typically run into edge cases (See repo history).The newest version is based on advanced image processing algorithms like euclidean distance transforms and the watershed algorithm
- `generate_gazebo_world.py`: Given an image of a track, build a 3D world of the track for simulation in Gazebo
- `main_globaltraj.py`: Calculate minimum distance path, minimum curvature path and a interpolation between both + do simulations for lap time based on a model of our car. It also includes the possibility to do an interpolation between the shortest path and minimum curvature path. This file is based on the works of Alexander Heilmeier and has been customized to better fit our use-case. For example I added the code for the interpolation.
- `python simulate_external_trajectory.py` Simulator: Given reference trajectory and centerline, calculate lap time following custom dynamic model

Everything was written by Tim Lindenau

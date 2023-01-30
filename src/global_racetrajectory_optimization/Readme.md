# What is this REPO about

Everything that has to do with calculating paths and preparing tracks. It also contains simulator where I used a dynamic model which suits an rc car well. Any trajectory can be imported into his simulator and it claculates the apprixmate track time.

## Desctiption of scripts
- `a_star_shortest_distance_path.py`: Calculate shortest path using A-Star
- `adaptive_replanning.py`: Simulation to show the replanning methods usied in the `online_planner`. Only used to create animations for better undersating
-`extract_centerline.py` Given an image extract track borders, centerline and track widths. Even though it souds like a common taks it is tricky since trivial python approaches typically run into edge cases (See repo history).The newest version is based on advanced image processing algorithms like euclidean distance transforms and the watershed algorithm
- `generate_gazebo_world.py`: Given an image of a track, build a 3D world of the track for simulation in Gazebo
- `main_globaltraj.py`: Calculate minimum distance path and minimum curvature path + do simulations for lap time based on a model of our car. It also includes the possibility to do an interpolation between the shortest path and minimum curvature path. This file is based on the works of Alexander Heilmeier and has been customized to better fit our use-case. For example I added the code for the interpolation.
- `python simulate_external_trajectory.py` Simulator: Given referece trajectory and centerline, calculate lap time following custom dynamic model

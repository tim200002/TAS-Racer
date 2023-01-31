# What is this REPO about
A GPS-like localization approach using the true position of the car.

## Implementation Details
The typical `AMCL` localization module did not perform well in the race-track environment. This is due to the environment being very homogeneous without any landmarks for the laser. To overcome this issue I built my own localization module which uses the (true) position of the car as published by GAZEBO. I then take this information to manually publish the map to odom transform. 

Written by Tim Lindenau

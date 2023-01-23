# What is this REPO about
A GPS-like localization approach using the true position of the car

## IMplementation Details
The typcial `ACML` localization module did not perform well in the race-track environment. This is due to the environment being very homogeneours without any landmarks for the laser. To ovecome this issue I built my own localization module which uses the (true) position of the car as published by GAZEBO. I then take this information to manyually publish the map to odom transform.

# What is this module about
The offline planner cannot react to dynamically changing environments. To overcome this issue I crated this planner which given an offline calcualted reference trajectory finds a good way around this trajectory while avoiding any objects.

## Implementation Details
There are bascically two modes
1) If the reference trajectory is clear, follow this trajectory
2) If the ference trajectory is blocked, claculate trajectory around objects back onto reference trajectory. For the second step we are using an efficient implementation of A-Star. Furthermore, to increase efficiency with every step we check if the old path is still valid and only recalculate those parts which are now colliding.

## Dependencies
This package is dependent on openCV, follow their website for a guide how to install
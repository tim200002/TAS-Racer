# What is this module about
The offline planner cannot react to dynamically changing environments. To overcome this issue I crated this planner which given an offline calculated reference trajectory finds a good way around this trajectory while avoiding any objects. 

## Implementation Details
There are basically two states the planner can be in
1) If the reference trajectory is clear, follow this trajectory
2) If the reference trajectory is blocked, calculate trajectory around objects back onto reference trajectory. For the second step we are using an efficient implementation of A-Star. Furthermore, to increase efficiency with every step we check if the old path is still valid and only recalculate those parts which are now colliding. Replanning also has the issue that the vehicle is typically longer than wide. Thus the safety margin must be increased. We achieve this in two ways 1) for merging out we increase the margin, however it might be that the start point does not fulfill the margin so we do a soft approach, where if the margin cannot be fulfilled the next best option is taking. When then 2) merging back we try to generate a smooth path such that the object is already parallel to the track borders and thus the width becomes the important margin again. To achieve this, we do not calculate A* completely until the merge back point but only safely around the object. Then we use a Bezier curve to generate a smooth path from the last path of A* to the merge back point.

## Dependencies
This package is dependent on openCV, follow their website for a guide how to install

Package written by Tim Lindenau
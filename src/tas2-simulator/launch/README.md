## Explanation of different scripts
- `bringup_simulation.py` Start simulation means starting RVIZ and gazebo. Via the flag `headless` one can decide whether to start the gazebo client gui (default true)
- `spawn_car.py` Spawns car at position set in the file. Also starts the state publisher of the car
- `delete_car.py` Deletes car from gazebo. Should be used before spawning new car
- `bringup_navstack.py` Once simulation has been started and cars have been spawned this file should be started to bringup the whole navstack

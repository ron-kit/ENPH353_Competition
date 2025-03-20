# ENPH353 2025W Competition

Shared repo for the Fizz Detective competition

## Launch instructions

1. Clone into `~/ros_ws/`
2. Run this in a terminal:
```
source ~/ros_ws/devel/setup.bash
cd ~/ros_ws/src/2024_competition/enph353/enph353_utils/scripts
./run_sim.sh -vpgw
```
3. Close the new controller terminal that opens
4. Open a new tab and run this:
```
./score_tracker.py
```
5. Open another tab and run this:
```
roslaunch controller_pkg my_launch.launch 
```

## Behavior

18/03/25: The environment loads and Robbie the robot is controllable through cmd.

19/03/25: Robbie drives forward for one second and stops. This is sufficient to pass time trials.
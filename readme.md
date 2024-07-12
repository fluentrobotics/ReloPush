## Test data
Set of instances with random deviation from nominal test data.

## Log
- RAW: raw data acquired as the program runs
- xlsx: re-organized raw data

## Script
- testdata_gen_from_nominal: testdata instance generator. Nominal poses are typed in the script.
- log_handler: re-organized raw data to xlsx.
- repeat: automatically repeats simulations.

## reloPush arguments
1. Testdata file path
2. Instance index
3. Leave log? (1=yes, 0=no) - if yes, visualization will only occur once.

## Run Simulation on RViz
```
roscore
roslaunch mushr_sim teleop.launch use_keyboard:=false
roslaunch mushr_rhc sim.launch
rostopic pub /initialpose geometry_msgs/PoseWithCovarianceStamped '{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''}, pose: {pose: {position: {x: 2, y: 2.5, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.7071068, w: 0.7071068}}, covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0]}}'
rosrun reloPush reloPush
```

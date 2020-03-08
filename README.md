## Robot Project 1
NUS, EE4308

### Setup
Clone this repo inside of the folder `catkin_ws/src`. After that, run `catkin_make` in the
root directory of the workspace.

### Install
First build the workspace:
```
catkin_make install -DCMAKE_BUILD_TYPE=Release
```

### Launch
To launch the robot, run the following command:
```
roslaunch bringup robot_bringup.launch
```
or
```
roslaunch bringup proj1.sh
```
to start the mission.

### TODO
#### Part 1
- [x] Implement the Inverse Sensor Model
- [x] Implement Odometry Motion Model with wheel encoders
- [x] Fuse simply with other sensor information for Odometry Motion Model
- [x] Implement Binary Log Odds
- [x] Implement inflation zones or potential fields to provide guarantees against collisions
- [x] Implement General Line Algorithm **=> see `global_planner/src/thetastar.cpp`, already implemented**

#### Part 2
- [x] Implement either ~~ANYA~~ or **Theta\*** (any-angle) path planner
- [x] Post-process to get a series of turning points

#### Part 3
- [x] Project global turning points to local space
- [x] Implement trajectory generation using splines between global turning points. Path must be collision free.
- [x] Generate via-points on the spline and use them for pure pursuit

### Topics
- `/robot_pose`: geometry_msgs/Pose2D
- `/map`: nav_msgs/OccupancyGrid
- `/navigation/goal`: geometry_msgs/Pose2D
- `/navigation/path`: nav_msgs/Path, global coordinates

## Robot Project 1
NUS, EE4308

### Setup
Clone this repo inside of the folder `catkin_ws/src`. After that, run `catkin_make` in the
root directory of the workspace.

### TODO
#### Part 1
- [ ] Implement the Inverse Sensor Model
- [ ] Implement Odometry Motion Model with wheel encoders
- [ ] Fuse simply with other sensor information for Odometry Motion Model
- [ ] Implement Binary Log Odds
- [ ] Implement inflation zones or potential fields to provide guarantees against collisions
- [ ] Implement General Line Algorithm **=> see `global_planner/src/thetastar.cpp`, already implemented**

#### Part 2
- [x] Implement either ~~ANYA~~ or **Theta\*** (any-angle) path planner
- [ ] Post-process to get a series of turning points

#### Part 3
- [ ] Project global turning points to local space
- [ ] Implement Potential Field to guide the robot while avoiding obstacles
- [ ] Move continuously from start to end

### Topics
- `/robot_pose`: geometry_msgs/Pose2D
- `/map`: nav_msgs/OccupancyGrid
- `/navigation/path`: nav_msgs/Path, global coordinates
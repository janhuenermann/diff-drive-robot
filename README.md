## Robot Project
NUS, EE4308

### Install
Make sure you have the following requirements met:
- ROS Melodic
- Eigen 3 for math routines
- OpenCV 3 or higher
- Gazebo 9 with TurtleBot
- [hector_quadrotor](https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor)

To install, clone this repository inside a folder called `catkin_ws/src`. Make sure you have all your other workspaces sourced, including the workspace containing the hector_quadrotor packages. Then build the workspace by running the following command from the directory `catkin_ws`.
```bash
catkin_make install -DCMAKE_BUILD_TYPE=Release
```

### Launch
To launch the robot, run the following command:
```bash
roslaunch bringup robot_bringup.launch
```
or
```bash
roslaunch bringup proj1.sh
```
to start the mission.

### TODO
#### Part 1
- [x] Mapping and motion model
- [x] Global planning
- [x] Trajectory generation and following

#### Part 2
- [ ] Sensor fusion
- [ ] Extend mission planner
- [ ] Motion control (easy)
- [ ] Modify launch structure
- [ ] Fine-tune ground robot

### Topics
- `/robot_pose`: geometry\_msgs/Pose2D
- `/map`: nav\_msgs/OccupancyGrid
- `/navigation/goal`: geometry\_msgs/Pose2D
- `/navigation/path`: nav\_msgs/Path, global coordinates

<img src="architecture.png" alt="Architecture overview" width="660px" />
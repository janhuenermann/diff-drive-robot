## Robot Project
NUS, EE4308

### Setup
Clone this repo inside of the folder `catkin_ws/src`. After that, run `catkin_make` in the
root directory of the workspace.

### Install
First build the workspace:
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
- [x] Localization (part 1)
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
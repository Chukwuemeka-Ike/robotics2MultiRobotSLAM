# robotics2MultiRobotSLAM
The code in this repository was developed for the ROSMaster X3 robot built by Yahboom. All instructions refer to the packages that the company provides as part of its codebase.

## Multi-Robot SLAM
### Usage
The instructions and code in this repository assume you are working with two robots.

On the first robot, run the following command.
```bash
roslaunch multi_robot_slam multi_robot_slam.launch
```
This launch file starts the robot's sensors and mapping for robot 1, along with the *multirobot_map_merge* package's merge node. Lastly, it uses TF static transform broadcasters to publish the relationship between the merged map and the individual maps.

On the second robot, run the following command.
```bash
roslaunch multi_robot_slam robot_2.launch
```
This file starts the sensors and mapping for robot 2.

In separate terminals on both robots, run the following commands.
```bash
# Robot 1.
ROS_NAMESPACE=robot1 rosrun teleop_twist_keyboard teleop_twist_keyboard.py

# Robot 2.
ROS_NAMESPACE=robot2 rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
These nodes allow the user move the robots around the workspace while the SLAM algorithms continuously build the map. The *multirobot_map_merge* node merges the individual maps as they are updated.

Once the SLAM has been performed for a sufficient amount of time, save the map using the following command.
```bash
rosrun map_server map_saver -f <path_to_desired_file>
```
This map file currently has to be placed in the */<yahboomcar_nav pkg dir>/maps/* directory for the Yahboom navigation stack to function as expected.

## Landmark Finder
### Usage
Start the robot's sensors and navigation stack.
```bash
roslaunch yahboomcar_nav laser_usb_bringup.launch
roslaunch yahboomcar_nav yahboomcar_navigation.launch use_rviz:=false map:=<map_name>
```
Start the AR tag tracker.
```bash
roslaunch yahboomcar_visual ar_track.launch open_rviz:=false
```
Start the *landmark_finder* node.
```bash

```

## Landmark Navigation
### Usage
First, ensure the pre-built map <map_name> is located in the */<yahboomcar_nav pkg dir>/maps/* directory. On the robot, run the following commands in separate terminals.
```bash
roslaunch yahboomcar_nav laser_bringup.launch
roslaunch yahboomcar_nav yahboomcar_navigation.launch use_rviz:=false map:=<map_name>
```
These commands ensure the LIDAR and the robot's navigation stack are up and running. After successfully setting up the stack, navigate to the *landmark_navigation* package folder and start the navigation node.
```bash
cd <landmark_navigation pkg dir>
rosrun landmark_navigation landmark_nav.py
```
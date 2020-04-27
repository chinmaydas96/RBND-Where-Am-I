# Robotic Software Engineer ND 
## Project 3: Where Am I

[image1]: ./img/RobotND-Project3-Gazebo-World.png  " "
[image2]: ./img/RobotND-Project3-AMCL-Gazebo.gif  " "


Simple ROS/Gazebo project for localization and path planning with the ROS packages AMCL (Adaptive Monte Carlo Localization) and move_base.

Gazebo world with mobile robot:
![][image1]

Rviz with map, localization and path planning:
![][image2]

## How To Use

### Clone repo as catkin_ws, initialize workspace and build
```
$ git clone https://github.com/civcode/RobotND-P3-Where-Am-I.git catkin_ws
$ cd catkin_ws/src 
$ catkin_init_workspace
$ cd .. && catkin_make
```

### Launch ROS and Gazebo
```
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```

### Launch ROS AMCL and move_base
```
$ source devel/setup.bash
$ roslaunch my_robot amcl.launch
```

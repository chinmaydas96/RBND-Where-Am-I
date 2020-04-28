# Udacity : Where am I? project

## Setup

Clone the repo, and rename the top directory to catkin_ws, so that you have the top folder structure:

> workspace -  
&nbsp;&nbsp;  -catkin_ws  
&nbsp;&nbsp; &nbsp;&nbsp;     -build  
&nbsp;&nbsp; &nbsp;&nbsp;     -devel  
&nbsp;&nbsp; &nbsp;&nbsp;     -src  
&nbsp;&nbsp; &nbsp;&nbsp;     -screenshots

In order to run, open 3 terminals and source ros in each. (source devel/setup.bash)

### In terminal 1:

launch gazebo, Rviz and spawn a robot.

> roslaunch my_robot midworld.launch

This will run the launch file for a medium sized world.
Also included are launch files for a larger world ("biggerworld.launch"). Note that if this is used the amcl.launch file needs to be updated to reflect the biggerworld.map

### In terminal 2:

> roslaunch my_robot amcl.launch

to launch the map server and amcl

### In terminal 3:

> rosrun teleop_twist_keyboard teleop_twist_keyboard.py

## Results

The robot is able to locals quickly. The main parameters I adjusted were the min and max particles.

>"min_particles" value="5"  
"max_particles" value="500"  

#### Initial State
![Alt text](/screenshots/initial-not_localised.png?raw=true "Initial State")

#### Localised
![Alt text](/screenshots/localised.png?raw=true "Localised")

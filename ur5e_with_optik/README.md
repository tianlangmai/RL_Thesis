### Dependencies
You need following packages
- Ros Noetic 
- fmauch_universal_robot
- Universal_Robots_ROS_Driver

Follow [Official Documentation](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)

### UR5e With Optik
Clone the **ur5e_with_optik** repository
```
https://git-ce.rwth-aachen.de/llt_dpp/all/ur5e_with_optik.git
```
There are 2 packages under **ur5e_with_optik**
- ###### ur5e_with_optik
    - Includes robot description and gazebo simulation of the robot with optik
- ###### ur5e_with_optik_moveit_config
    - Includes moveit config of the robot with optik


### Simulation ###
###### DO NOT FORGET TO RUN FOLLOWINGS AS ROOT AND SOURCE BEFORE RUNNING!
Initialize gazebo world and spawn the robot
```sh
roslaunch ur5e_with_optik ur5e_gazebo.launch
```
Start moveit
```sh
 roslaunch ur5e_with_optik_moveit_config planning_execution.launch sim:=true
```
Start RViz
```sh
roslaunch ur5e_with_optik_moveit_config moveit_rviz.launch rviz_config:=$(rospack find ur5e_with_optik_moveit_config)/launch/moveit.rviz sim:=true
```

### Dealing With Real Robot
Simply get rid of 'sim' parameter while starting nodes.

# Notes
TODO In simulation link length is not yet calibrated.

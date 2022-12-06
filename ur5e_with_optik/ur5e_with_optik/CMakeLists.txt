cmake_minimum_required(VERSION 3.0.2)
project(ur5e_with_optik)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg moveit_core moveit_ros_planning moveit_ros_planning_interface) 

catkin_package(
#  LIBRARIES ur5e_with_optik
  CATKIN_DEPENDS 
    roscpp
    
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#add_executable(${PROJECT_NAME}_node scripts/cartesian_goal.py)

# add_executable(cartesian_planner src/cartesian_planner.cpp scripts/forward_kinematics.py)
# add_executable(jerk_optimization src/jerk_optimization.cpp)
# target_link_libraries(cartesian_planner ${catkin_LIBRARIES})
# target_link_libraries(jerk_optimization ${catkin_LIBRARIES})
#target_link_libraries(forward_kinematics ${catkin_LIBRARIES})


install(DIRECTORY config launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

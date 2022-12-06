// #include <ros/ros.h>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/robot_model/robot_model.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <moveit/robot_model_loader/robot_model_loader.h>
// #include <moveit/robot_state/robot_state.h>
// #include <moveit_msgs/GetPositionIK.h>
// #include <iostream>
// #include <fstream>
// #include <string>
// #include <tf/tf.h>
// #include <tf/transform_datatypes.h>
// #include <moveit/robot_state/robot_state.h>

// static const std::string PLANNING_GROUP = "manipulator";

// // Construct waypoints
// std::vector<geometry_msgs::Pose> doubleSquarePath(float center_x, float center_y, float center_z, float edgeLength, geometry_msgs::Quaternion orientation)
// {
//     std::vector<geometry_msgs::Pose> waypoints;
//     std::vector<std::pair<int,int>> a_list;
//     a_list.push_back(std::make_pair(1,1));
//     a_list.push_back(std::make_pair(1,-1));
//     a_list.push_back(std::make_pair(-1,-1));
//     a_list.push_back(std::make_pair(-1,0));

//     for(int i=0; i<4; i++)
//     {
//         geometry_msgs::Pose newPose;
//         newPose.position.x=center_x + a_list[i].first*edgeLength/2;
//         newPose.position.y=center_y + a_list[i].second*edgeLength/2;
//         newPose.position.z=center_z;
//         newPose.orientation = orientation;
//         waypoints.push_back(newPose);
//     }
//     for(int i=0; i<4; i++)
//     {
//         geometry_msgs::Pose newPose;
//         newPose.position.x=center_x + a_list[i].first*edgeLength/4;
//         newPose.position.y=center_y + a_list[i].second*edgeLength/4;
//         newPose.position.z=center_z;
//         newPose.orientation = orientation;
//         waypoints.push_back(newPose);
//     }
//     return waypoints;
// }

// // Set constant speed to a trajectory
// void setAvgCartesianSpeed(moveit_msgs::RobotTrajectory &traj, const std::string end_effector, const double speed)
// {
//     robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
//     robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
//     robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
//     kinematic_state->setToDefaultValues();
 
//     int num_waypoints = traj.joint_trajectory.points.size();                                //gets the number of waypoints in the trajectory
//     const std::vector<std::string> joint_names = traj.joint_trajectory.joint_names;    //gets the names of the joints being updated in the trajectory
 
//     //set joint positions of zeroth waypoint
//     kinematic_state->setVariablePositions(joint_names, traj.joint_trajectory.points.at(0).positions);
 
//     double euclidean_distance, new_timestamp, old_timestamp, q1, q2, q3, dt1, dt2, v1, v2, a;
//     trajectory_msgs::JointTrajectoryPoint *prev_waypoint, *curr_waypoint, *next_waypoint;
 
//     for(int i = 0; i < num_waypoints - 1; i++)      //loop through all waypoints
//     {
//         curr_waypoint = &traj.joint_trajectory.points.at(i);
//         next_waypoint = &traj.joint_trajectory.points.at(i+1);
         
//         //set joints for next waypoint
//         kinematic_state->setVariablePositions(joint_names, next_waypoint->positions);
 
//         //do forward kinematics to get cartesian positions of end effector for next waypoint
//         next_end_effector_state = kinematic_state->getGlobalLinkTransform(end_effector);
 
//         //get euclidean distance between the two waypoints
//         euclidean_distance = pow(pow(next_end_effector_state.translation()[0] - current_end_effector_state.translation()[0], 2) + 
//                             pow(next_end_effector_state.translation()[1] - current_end_effector_state.translation()[1], 2) + 
//                             pow(next_end_effector_state.translation()[2] - current_end_effector_state.translation()[2], 2), 0.5);
 
//         new_timestamp = curr_waypoint->time_from_start.toSec() + (euclidean_distance / speed);      //start by printing out all 3 of these!
//         old_timestamp = next_waypoint->time_from_start.toSec();
 
//         //update next waypoint timestamp & joint velocities/accelerations if joint velocity/acceleration constraints allow
//         if(new_timestamp > old_timestamp)
//         {
//             next_waypoint->time_from_start.fromSec(new_timestamp);
//             //ROS_INFO("Waypoint speed set. Waypoint number: %d/%d",i,num_waypoints);
//         }
//         else
//         {
//             ROS_WARN_NAMED("setAvgCartesianSpeed", "Average speed is too fast. Moving as fast as joint constraints allow. Waypoint number: %d/%d",i, num_waypoints);
//         }
         
//         //update current_end_effector_state for next iteration
//         current_end_effector_state = next_end_effector_state;
//     }
     
//     //now that timestamps are updated, update joint velocities/accelerations (used updateTrajectory from iterative_time_parameterization as a reference)
//     for(int i = 0; i < num_waypoints; i++)
//     {
//         curr_waypoint = &traj.joint_trajectory.points.at(i);            //set current, previous & next waypoints
//         if(i > 0)
//             prev_waypoint = &traj.joint_trajectory.points.at(i-1);
//         if(i < num_waypoints-1)
//             next_waypoint = &traj.joint_trajectory.points.at(i+1);
 
//         if(i == 0)          //update dt's based on waypoint (do this outside of loop to save time)
//             dt1 = dt2 = next_waypoint->time_from_start.toSec() - curr_waypoint->time_from_start.toSec();
//         else if(i < num_waypoints-1)
//         {
//             dt1 = curr_waypoint->time_from_start.toSec() - prev_waypoint->time_from_start.toSec();
//             dt2 = next_waypoint->time_from_start.toSec() - curr_waypoint->time_from_start.toSec();
//         }
//         else
//             dt1 = dt2 = curr_waypoint->time_from_start.toSec() - prev_waypoint->time_from_start.toSec();
 
//         for(int j = 0; j < joint_names.size(); j++)     //loop through all joints in waypoint
//         {
//             if(i == 0)                      //first point
//             {
//                 q1 = next_waypoint->positions.at(j);
//                 q2 = curr_waypoint->positions.at(j);
//                 q3 = q1;
//             }
//             else if(i < num_waypoints-1)    //middle points
//             {
//                 q1 = prev_waypoint->positions.at(j);
//                 q2 = curr_waypoint->positions.at(j);
//                 q3 = next_waypoint->positions.at(j);
//             }
//             else                            //last point
//             {
//                 q1 = prev_waypoint->positions.at(j);
//                 q2 = curr_waypoint->positions.at(j);
//                 q3 = q1;
//             }
 
//             if(dt1 == 0.0 || dt2 == 0.0)
//                 v1 = v2 = a = 0.0;
//             else
//             {
//                 v1 = (q2 - q1)/dt1;
//                 v2 = (q3 - q2)/dt2;
//                 a = 2.0*(v2 - v1)/(dt1+dt2);
//             }
 
//             //actually set the velocity and acceleration
//             curr_waypoint->velocities.at(j) = (v1+v2)/2;
//             curr_waypoint->accelerations.at(j) = a;
//         }
//     }
// }

// // Set constant speed to cartesian points.
// void setAvgCartesianSpeedWP(std::vector<std::array<double, 4>> &cartesianTraj, const double speed)
// {
//     double euclidean_distance, time_stamp;
    
//     cartesianTraj[0][3] = 0;
//     for(int i=0;i<cartesianTraj.size()-1;i++)
//     {
//         euclidean_distance = pow(pow(cartesianTraj[i][0] - cartesianTraj[i+1][0], 2) + 
//                             pow(cartesianTraj[i][1] - cartesianTraj[i+1][1], 2) + 
//                             pow(cartesianTraj[i][2] - cartesianTraj[i+1][2], 2), 0.5);
//         time_stamp = euclidean_distance/speed;
//         cartesianTraj[i][3] = time_stamp;
//     }
// }

// // Interpolate waypoints
// //

// int main(int argc, char **argv)
// {
//     // Initialize node
//     ros::init(argc, argv, "jerk_optimization_node");
//     ros::NodeHandle node_handle;
//     ros::AsyncSpinner spinner(1);
// 	spinner.start();
//     //

//     // Initialize MoveGroup
//     moveit::planning_interface::MoveGroupInterface	move_group(PLANNING_GROUP);
//     move_group.setMaxAccelerationScalingFactor(1);
//     move_group.setMaxVelocityScalingFactor(1);
//     // Set start state
//     move_group.setStartStateToCurrentState();

//     // Get waypoints
//     std::vector<geometry_msgs::Pose> waypoints = doubleSquarePath(-0.4,0.4,0.07,0.3,move_group.getCurrentPose().pose.orientation);

//     // Compute trajectory
//     // moveit_msgs::RobotTrajectory traj;
//     // move_group.computeCartesianPath(waypoints, 0.01, 0.0, traj);
//     //

//     // Execute trajectory
//     //move_group.execute(traj);


//     // Construct compute_ik service
//     ros::ServiceClient service_client = node_handle.serviceClient<moveit_msgs::GetPositionIK> ("compute_ik");
//     moveit_msgs::GetPositionIK::Request service_request;
//     moveit_msgs::GetPositionIK::Response service_response;
//     service_request.ik_request.group_name = move_group.getName();
//     service_request.ik_request.timeout.sec = 5;
//     service_request.ik_request.robot_state.joint_state.name = move_group.getJointNames();
//     service_request.ik_request.avoid_collisions = true;

//     // Construction of solution graph
//     std::vector<std::vector<std::vector<double>>> solution_graph; 

//     // Allocate memory
//     solution_graph.resize(waypoints.size());

//     // Obtain solutions
//     for(int i=0;i<waypoints.size();i++)
//     {
//         service_request.ik_request.robot_state.joint_state.position = move_group.getCurrentJointValues();
//         service_request.ik_request.pose_stamped.pose = waypoints[i];
//         service_client.call(service_request, service_response);
//         ROS_INFO("Response: %d", service_response.error_code);
//         solution_graph[i].push_back(service_response.solution.joint_state.position);
//     }
    
//     //Publish Response
//     //moveit_msgs::RobotState solution = service_response.solution;
//     for(int i=0;i<solution_graph.size();i++)
//     {
//         ROS_INFO("XXXXXX   WAYPOINT %d XXXXXX\n",i+1);
//         for(int j=0;j<solution_graph[i].size();j++)
//             for(int k=0;k<solution_graph[i][j].size();k++)
//                 ROS_INFO("Joint %d: %f",k+1,solution_graph[i][j][k]);
//         ROS_INFO("XXXXXXXXXXXXXXXXXXXXXXXXXXX\n\n");
//     }
//     ROS_INFO(" ### EXAMPLE FOR A SINGLE WAYPOINT ### \n");
//     for(int i=0;i<10;i++)
//     {
//         service_request.ik_request.robot_state.joint_state.position = move_group.getCurrentJointValues();
//         service_request.ik_request.pose_stamped.pose = waypoints[i%2];
//         service_client.call(service_request, service_response);
//         ROS_INFO("Response: %d", service_response.error_code);
        
//         ROS_INFO("XXXXXX   WAYPOINT %d XXXXXX\n",1+i%2);
//         for(int j=0;j<solution_graph[0].size();j++)
//             for(int k=0;k<solution_graph[0][j].size();k++)
//                 ROS_INFO("Joint %d: %f",k+1,service_response.solution.joint_state.position[k]);
//         ROS_INFO("XXXXXXXXXXXXXXXXXXXXXXXXXXX\n\n");
//         move_group.setStartStateToCurrentState();
//         move_group.setJointValueTarget(service_response.solution.joint_state);
//         ROS_INFO("AAAAA %d",move_group.move());
//     }
            
//     return 0;
// }



// /*  1st Way

// +   1. Set waypoints.
//     2. Interpolate them.
// +   3. Write another constant speed setter.
//     4. Use IK to find solutions and set their timestamps using (3).
//     5. Jerk optimization.
// */

// /*  2nd WAY

// +   1. Set waypoints.
// +   2. computeCartesianPath using moveit and get the joint values of interpolated path.
// +   3. Conduct set average cartesian speed.
//     4. Use FK to obtain interpolated waypoints and match their time stamps.
//     5. Find IK solutions of them.
//     6. Jerk optimization.
// */




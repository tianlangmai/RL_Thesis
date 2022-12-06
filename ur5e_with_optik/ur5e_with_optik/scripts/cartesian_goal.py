#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_msgs.msg
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler




def square_path(center_x : float, center_y : float, center_z : float, edgeLength : float):
    """A basic function to simplify square path.

    Args:
        center_x (float): x value of center of the square
        center_y (float): y value of center of the square
        center_z (float): Z value of center of the square
        edgeLength (float): Length of edges

    Returns:
        tuple: 4 Waypoints in a tuple
    """
    
    a_list = [(-1,-1),(-1,1),(1,1),(1,-1)]
    squareWaypoints = list()
    
    for i,j in a_list:
        newPoint = PoseStamped().pose
        newPoint.position.x=center_x + i*edgeLength/2
        newPoint.position.y=center_y + j*edgeLength/2
        newPoint.position.z=center_z
        newPoint.orientation = arm.get_current_pose().pose.orientation # TAKES CURRENT POSITION - CHANGE TO INTENDED ORIENTATION IF NEEDED
        squareWaypoints.append(copy.deepcopy(newPoint))
        
    return tuple(squareWaypoints)




if __name__ == "__main__":
    rospy.init_node("cartesian_move", anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    arm = MoveGroupCommander('manipulator')
    traj_publisher = rospy.Publisher('move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)
    
    arm.set_start_state_to_current_state() # Set current state
    waypoints = square_path(0.50,0.50,0.1,0.25)
    rospy.loginfo("### WAYPOINTS ###")
    rospy.loginfo(waypoints)
    rospy.loginfo("### END OF WAYPOINTS ###")
    (plan, fraction) = arm.compute_cartesian_path(
                                  waypoints,   # waypoints to follow
                                  0.01,        # eef_step
                                  0.0)         # jump_threshold
    
    rospy.loginfo("PLANNED TRAJECTORY BEING EXECUTED")

    arm.execute(plan_msg=plan)
    rospy.loginfo(plan)
    rospy.loginfo("Executed and exitting")
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)
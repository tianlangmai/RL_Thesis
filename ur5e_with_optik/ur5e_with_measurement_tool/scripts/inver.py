#!/usr/bin/env python3
import csv
import math
import sys
from time import sleep
import moveit_commander
import rospy
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
from numpy import array
import random
from ur_msgs.srv import SetIO
import numpy as np
from spatialmath import SE3
from math import pi
import socket
from numpy.linalg import norm
import pathlib
from robotics_tool_box_peter_corke_ur5e import UR5e
import copy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler



joint_values = []
pose_values = []
data_pos_LT = []

HIGH_VOLTAGE = 1
LOW_VOLTAGE = 0


def sampling_from_observable_task_space(
    x_low: float = 0.25,
    x_high: float = 0.55,
    y_low: float = -0.5,
    y_high: float = 0.40-0.1,
    z_low: float = 0.5,
    z_high: float = 0.90-0.1,
    rx_low: float = -1 * math.pi,
    rx_high: float = 1 * math.pi,
    ry_low: float = 0.9 ,
    ry_high: float = 2.318 ,
    rz_low: float = -0.8,
    rz_high: float = 0.3,
) -> array:
    """Sample an observable pose in task space unifromly between low and high.

    Args:
        x_low (int, optional): Defaults to 0.250.
        x_high (int, optional): Defaults to 0.500.
        y_low (int, optional): Defaults to -0.500.
        y_high (int, optional): Defaults to 0.500.
        z_low (int, optional): Defaults to 0.5.
        z_high (int, optional): Defaults to 0.800.
        rx_low (float, optional): Defaults to 0.
        rx_high (float, optional): Defaults to 0.5math.pi.
        ry_low (float, optional): Defaults to 0.1*math.pi.
        ry_high (float, optional): Defaults to 0.8*math.pi.
        rz_low (float, optional): Defaults to 0.1*math.pi.
        rz_high (float, optional): Defaults to 0.8*math.pi.
        base (bool, optional): If True, the high and low values are defined in robot base frame.Defaults to True.


    Returns:
        array: ndarray with pose per row.
    """
    x = random.uniform(x_low, x_high)
    y = random.uniform(y_low, y_high)
    z = random.uniform(z_low, z_high)
    rx = random.uniform(rx_low, rx_high)
    ry = random.uniform(ry_low, ry_high)
    rz = random.uniform(rz_low, rz_high)
    pose = array([x, y, z, rx, ry, rz])

    return pose


def set_ur5e_controller_io(fun: int = 1, pin: int = 4, state: int = 0) -> None:
    """Set io values on UR5e controller.
    pose
            fun (int, optional): Function used to set io. See message definition for details. Defaults to 1 (set digital io).
            pin (int, optional): Pin to set. Defaults to 4 (digital io pin 4).
            state (int, optional): State to set pin to. Defaults to 0 (digital off/ low voltage).
    """
    rospy.wait_for_service("/ur_hardware_interface/set_io")
    try:
        IOService = rospy.ServiceProxy("/ur_hardware_interface/set_io", SetIO)
        IOService(1, 4, state)
    except rospy.ServiceException as e:
        print("Service unavailable: %s" % e)


def move_to_next_joint_values(joints: list = []) -> None:
    """
        move to the next joint values, than set pin to 1 (digital on/ high voltage) to meassure the pose.
        Finally set pin to 0 (digital off/ low voltage) again.
    Args:
        joints (list): List of joint values. Defaults to [].
    """
    arm.set_start_state_to_current_state()
    arm.go(joints, wait=True)
    sleep(3)
    # set pin to 0
    # set_ur5e_controller_io(state=HIGH_VOLTAGE)
    sleep(3)
    # set pint to 1
    # set_ur5e_controller_io(state=LOW_VOLTAGE)


def save_joints(
    joint_values: list, file_name: csv = pathlib.Path.cwd() / "joint_values.csv"
):
    """save joint values in a csv file

    Args:
        joint_values (list): list of joint values
        file_name (csv, optional):the file csv where the joints values are goint to be saved.
        Defaults to "joint_values.csv".
    """

    with open(file_name, "w", encoding="UTF8", newline="") as f:
        writer = csv.writer(f)
        # header
        row = ["", "q0", "q1", "q2", "q3", "q4", "q5"]
        writer.writerow(row)
        for joints in joint_values:
            writer.writerow(joints)
    print("Result joint values file successfully created. ")


def save_robot_pose(
    pose_values: list, file_name: csv = pathlib.Path.cwd() / "robot_pose_values.csv"
):
    """Save poses in a csv file

    Args:
        pose_values (list): list with the position and orientation of the poses given to the robot
        file_name (csv, optional): path where the pose values are going to be saved.
         Defaults to pathlib.Path.cwd()/"robot_pose_values.csv".
    """

    with open(file_name, "w", encoding="UTF8", newline="") as f:
        writer = csv.writer(f)
        row = ["", "x", "y", "z", "rx", "ry", "rz"]
        writer.writerow(row)
        for pose in pose_values:
            writer.writerow(pose)
    print("Result robot pose values file successfully created. ")


def save_LT_pose(
    pose_values: list, file_name: csv = pathlib.Path.cwd() / "LT_pose.csv"
):
    """save position meassured with the Laser Tracker(LT) in a csv file

    Args:
        pose_values (list): list of the position(x,y,z)of poses meassured from laser tracker
        file_name (csv, optional): file path where the pose values are going to be saved.
        Defaults to pathlib.Path.cwd()/"LT_pose.csv".
    """

    with open(file_name, "w", encoding="UTF8", newline="") as f:
        writer = csv.writer(f)

        for pose in pose_values:
            writer.writerow(pose)
    print("Result LT pose values file successfully created. ")


def process_xyz(
    y_raw: csv = pathlib.Path.cwd() / "LT_pose.csv",
    file_name: csv = pathlib.Path.cwd() / "LT_poses_processed.csv",
) -> None:
    """transform the raw data in the format X, Y, Z
                                            x1,y1,z1
                                            ...
        and save it into csv file

    Args:
        y_raw (csv, optional): _description_. Defaults to pathlib.Path.cwd()/"LT_pose.csv".
        file_name (csv, optional): _description_. Defaults to pathlib.Path.cwd()/"LT_poses_processed.csv".
    """
    rows = []
    # eliminating what is not position X,Y,Z
    with open(y_raw, "r", encoding="UTF8", newline="") as f:
        for line in f:
            row = (
                line.replace("b'X,", "")
                .replace("Y,", "")
                .replace("Z,", "")
                .replace("Time(sec),", "")
                .split(",")
            )
            rows.append(row[0:3])

    # Write the rows in file
    with open(file_name, "w", encoding="UTF8", newline="") as f:
        writer = csv.writer(f)
        row = ["", "x", "y", "z"]
        writer.writerow(row)
        for pose in rows:
            # convert values into meters
            pos = list(np.float_(pose) * 1e-3)
            writer.writerow((pos))


def add_scene_objects(scene, robot):
       
    rospy.sleep(2)
      
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0
    p.pose.position.y = 0
    p.pose.position.z = -0.945
    p.pose.orientation.w = 1.0
    scene.add_box("table_1", p, size=(1.2, 2.4, 0.9))

    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0
    p.pose.position.y = -0.5875
    p.pose.position.z = -0.045
    p.pose.orientation.w = 1.0
    scene.add_box("experimental_setup_for_cutting", p, size=(1.21, 0.875, 0.1))

    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0
    p.pose.position.y = 0
    p.pose.position.z = -0.045
    p.pose.position.y = 0.1875
    p.pose.position.z = -0.045
    p.pose.orientation.w = 1.0
    scene.add_box("item_cutting_setup", p, size=(0.3 ,0.295 ,0.045))
   
def get_random_waypoints_new(arm, offset=0.05):
    waypoints = []
    pos_start =  sampling_from_observable_task_space()
    pos_target =  sampling_from_observable_task_space()
    # convert euler to quarternion
    quat_start = quaternion_from_euler ( pos_start[3],  pos_start[4],  pos_start[5])
    quat_target = quaternion_from_euler ( pos_target[3],  pos_target[4],  pos_target[5])
   
    wpose = arm.get_current_pose().pose
    wpose.position.x = pos_start[0]
    wpose.position.y = pos_start[1]
    wpose.position.z = pos_start[2]
    wpose.orientation.x = quat_start[0]
    wpose.orientation.y = quat_start[1]
    wpose.orientation.z = quat_start[2]
    wpose.orientation.w = quat_start[3]
   
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.x = pos_start[0]
    wpose.position.y = pos_start[1]+offset
    wpose.position.z = pos_start[2]
    wpose.orientation.x = quat_start[0]
    wpose.orientation.y = quat_start[1]
    wpose.orientation.z = quat_start[2]
    wpose.orientation.w = quat_start[3]
   
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.x = pos_target[0]
    wpose.position.y = pos_target[1]+offset
    wpose.position.z = pos_target[2]
    wpose.orientation.x = quat_target[0]
    wpose.orientation.y = quat_target[1]
    wpose.orientation.z = quat_target[2]
    wpose.orientation.w = quat_target[3]
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.x = pos_target[0]
    wpose.position.y = pos_target[1]
    wpose.position.z = pos_target[2]
    wpose.orientation.x = quat_target[0]
    wpose.orientation.y = quat_target[1]
    wpose.orientation.z = quat_target[2]
    wpose.orientation.w = quat_target[3]
    waypoints.append(copy.deepcopy(wpose))
    return waypoints

def get_plan(arm,  waypoints,  eef_step = 0.01):     
    (plan_cart, fraction) = arm.compute_cartesian_path(
            waypoints,  # waypoints to follow
            eef_step,        # eef_step
            0.0        # jump_threshold
        )  
    plan_joint = plan_cart
    while len(plan_joint.joint_trajectory.points) > len(waypoints)+1:
        (plan_joint, fraction_2) = arm.compute_cartesian_path(
            waypoints,  # waypoints to follow
            eef_step,        # eef_step
            0.0        # jump_threshold
        )
        eef_step += 0.05
   
    return plan_cart, plan_joint, fraction 
def move_to_home_position(arm):
    joints = np.radians([93.7,-81.10,128.41,-137.33,90.11,-3.83])
    arm.set_start_state_to_current_state()
    rospy.loginfo("moving to home position.")
    arm.go(joints, wait=True)

if __name__ == "__main__":

    UDP_IP = "255.255.255.255"
    UDP_PORT = 10000

    # Connect Laser tracker via UDP
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Internet  # UDP
    sock.bind((UDP_IP, UDP_PORT))
    poses = []
    # Initiate a node in ros
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("forward_kinematics", anonymous=True)
    robot_moveit = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
  
    add_scene_objects(scene, robot_moveit)
    rospy.sleep(2)

    arm = MoveGroupCommander("manipulator")
    rospy.loginfo("Forward Kinematics node has been started.")
    move_to_home_position(arm)    
    for it in range(1):
        success = 0
      
        while success < 1:
            waypoints = get_random_waypoints_new(arm, offset=0.1)
            plan_cart, plan_joint, fraction = get_plan(arm, waypoints)
            if fraction == 1:
                rospy.logwarn("============ Robot moves to start position. ============")
                arm.go(waypoints[0], wait = True)
                arm.stop()
                start_joint = arm.get_current_joint_values()
                plan_cart, plan_joint, fraction = get_plan(arm, waypoints)
        plans = [plan_cart, plan_joint]

        rospy.loginfo("---------move to start------")
        arm.go(start_joint)
        rospy.loginfo("---------Executing planing------")
        resp = arm.execute(plan_joint, wait=True) # if resp = True execution is successful
        arm.stop()

        if resp:
            rospy.loginfo("---------Executing ist succesful------")
            final_joint = arm.get_current_joint_values()
            rospy.loginfo("---------Printing actual position------")
            c_pose = arm.get_current_pose().pose
            print(c_pose.position)
            poses.append([c_pose.position.x, c_pose.position.y, c_pose.position.z])
            rospy.loginfo("--------saving data from LT------- ")
            """ try:
                data, addr = sock.recvfrom(512)
                data_pos = str(data).split(",")
                rospy.loginfo("--------printing data from LT-------")
                print(data_pos)
            except:
                rospy.loginfo("Connection with LT failed. Try again.")
                # saving positions of poses from robot and LT into arrays
                robot_xyz = waypoints[-1]
                LT_xyz = np.array([data_pos[1], data_pos[3], data_pos[5]], dtype=float)
                # convert values into meters
                LT_xyz = LT_xyz * 1e-3
                # sanity check with euclidian norm (<= 10mm)
            if norm(LT_xyz - robot_xyz) <= 10 * 1e-3:
                # data_pos_LT.append(data_pos)
                pose_values.append(robot_xyz)
                # convert to degrees
                joint_values.append(np.float_(final_joint) * (180 / pi))
            else:
                resp = False
                rospy.loginfo(
                    "Distance between LT pose and robot pose is large!!. Point will not be saved"
                )
            """

        else:
            arm.stop()
            rospy.sleep(0.5)
            move_to_home_position(arm)
            it = it -1

#save_LT_pose(data_pos)
save_joints(joint_values)
save_robot_pose(pose_values)
save_robot_pose(poses,"poses_final.csv")

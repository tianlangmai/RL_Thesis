#!/usr/bin/env python3

import csv
import math
import sys
from datetime import datetime
from time import sleep

import moveit_commander
import rospy
from moveit_commander import MoveGroupCommander
from numpy import size, random, array
from ur_msgs.srv import SetIO

item_number = 0
K = 3
N = K**6
tcp_values = []
joint_values = []
dataset_file = "/ur5e_ws/src/ur5e_with_optik/ur5e_with_optik/scripts/dataset.csv"
result_file = "tcp_positions_" + str(datetime.now(tz=None)) + ".csv"
HIGH_VOLTAGE = 1
LOW_VOLTAGE = 0


def sampling_from_observable_task_space(
    x_low: int = 250,
    x_high: int = 500,
    y_low: int = -500,
    y_high: int = 500,
    z_low: int = 275,
    z_high: int = 700,
    rx_low: float = -math.pi,
    rx_high: float = math.pi,
    ry_low: float = 0.1 * math.pi,
    ry_high: float = 0.9 * math.pi,
    rz_low: float = 0.1 * math.pi,
    rz_high: float = 0.9 * math.pi,
    base: bool = True,
    size: int = 1,
) -> array:
    """Sample an observable pose in task space unifromly between low and high.

    Args:
        x_low (int, optional): Defaults to 250.
        x_high (int, optional): Defaults to 500.
        y_low (int, optional): Defaults to -500.
        y_high (int, optional): Defaults to 500.
        z_low (int, optional): Defaults to 275.
        z_high (int, optional): Defaults to 700.
        rx_low (float, optional): Defaults to -math.pi.
        rx_high (float, optional): Defaults to math.pi.
        ry_low (float, optional): Defaults to 0.1*math.pi.
        ry_high (float, optional): Defaults to 0.9*math.pi.
        rz_low (float, optional): Defaults to 0.1*math.pi.
        rz_high (float, optional): Defaults to 0.9*math.pi.
        base (bool, optional): If True, the high and low values are defined in robot base frame.Defaults to True.
        size (int, optional): Number of poses to sample. Defaults to 1.

    Returns:
        array: ndarray with pose per row.
    """
    x = random.uniform(x_low, x_high, size=size).item[0]
    y = random.uniform(y_low, y_high, size=size).item[0]
    z = random.uniform(z_low, z_high, size=size).item[0]
    rx = random.uniform(rx_low, rx_high, size=size).item[0]
    ry = random.uniform(ry_low, ry_high, size=size).item[0]
    rz = random.uniform(rz_low, rz_high, size=size).item[0]
    pose = array((x, y, z, rx, ry, rz))
    return pose


def set_ur5e_controller_io(fun: int = 1, pin: int = 4, state: int = 0) -> None:
    """Set io values on UR5e controller.

    The message definition can be found at <http://docs.ros.org/en/noetic/api/ur_msgs/html/srv/SetIO.html>.
    The pin definition on the controller can be fount in UR5e_pins_diagram.png
    The pin to int reference can be found at <https://github.com/ros-industrial/ur_msgs/pull/16/files/085c1ca7b73e5c6b9dab8e44381b058140599e8f>.

    Args:
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


def baser(n, k):
    for i in range(k):
        n = n // K
    return int(n % K)


def write_to_file(file_name):
    with open(file_name, "w", encoding="UTF8", newline="") as f:
        writer = csv.writer(f)
        row = ["", "x", "y", "z"]
        writer.writerow(row)
        for tcps in tcp_values:
            writer.writerow(tcps)
    print("Result file successfully created. " + file_name)


def print_curr_joint_values():
    print("\n")
    if item_number == 0:
        print("Beginning joint values: ")
    else:
        print("Current item: ", item_number - 1)
        print("Current joint values: ")
    print(arm.get_current_joint_values())


def print_next_joint_values():
    print("\n")
    print("Next item: ", item_number)
    print("Next joint values: ")
    try:
        print(joint_values[item_number])
        return True
    except:
        print("End of items")
        return False


def move_to_next_joint_values(iteration: int) -> None:
    """Move UR5e to next joint value.

    Moves arm to joint value of iteration. Waits for 3s.
    Sets controller io. Waits for 3s. Sets controller io.
    Appends robot configuration file.

    Args:
        iteration (int): Current iteration number.
    """
    arm.set_start_state_to_current_state()
    arm.go(joint_values[iteration])
    sleep(3)
    set_ur5e_controller_io(state=HIGH_VOLTAGE)
    sleep(3)
    set_ur5e_controller_io(state=LOW_VOLTAGE)
    tcp_values_appender(iteration, True)


def tcp_values_appender(iteration, status):
    if status:
        curr_pose = arm.get_current_pose().pose.position
        tmp_tcp = [iteration, curr_pose.x, curr_pose.y, curr_pose.z]
        tcp_values.append(tmp_tcp)
    else:
        tmp_tcp = [iteration, "ERR", "ERR", "ERR"]
        tcp_values.append(tmp_tcp)


def construct_values_from_file(file_name, tmp_cnt):
    joint_values.clear()
    tmp_tmp_cnt = 0
    with open(file_name, newline="") as csvfile:
        csv_reader = csv.reader(csvfile, delimiter=",", quotechar="|")
        for row in csv_reader:
            if row[0] == "q1" or tmp_tmp_cnt < tmp_cnt:
                tmp_tmp_cnt += 1
                continue
            # elif float(row[1])>0:
            #    row[1]=str(float(row[1])-2*math.pi)
            j = []
            try:
                for i in range(6):
                    j.append(math.radians(float(row[i])))
            except:
                rospy.WARN(
                    "READ ERROR. CHECK IF YOUR DATASET IS CORRECTLY CONSTRUCTED!"
                )
                break
            joint_values.append(j)
    rospy.loginfo(
        "Joint angles are constructed from file! Number of items: {}".format(
            size(joint_values) // 6
        ),
    )


def construct_values(tmp_cnt):
    joint_values.clear()
    while tmp_cnt <= N:
        j = []
        for t in range(6):
            j.append((float(4 * math.pi / 3) * baser(tmp_cnt, t) - 2 * math.pi) / 4)
        joint_values.append(j)
        tmp_cnt += 1
    print(
        "Joint values are constructed. Number of items left:", size(joint_values) // 6
    )


if __name__ == "__main__":
    rospy.init_node("forward_kinematics", anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    arm = MoveGroupCommander("manipulator")
    rospy.loginfo("Forward Kinematics node has been started.")
    arm.set_max_velocity_scaling_factor(0.2)
    construct_values_from_file(dataset_file, 0)

    user_response = ""
    while not rospy.is_shutdown():
        print_curr_joint_values()
        if not print_next_joint_values():
            print("Exitting.")
            exit()
        if user_response == "":
            user_response = input("Give input: start 's', exit 'e', jump 'jump': ")
        if user_response == "s":
            rospy.loginfo("Executing next joint values.")
            try:
                move_to_next_joint_values(item_number)
                if item_number % 10 == 5:
                    write_to_file(result_file)
                item_number += 1
            except:
                rospy.logwarn("Execution failed! Item number: %d", item_number)
                tcp_values_appender(item_number, False)
                if item_number % 10 == 5:
                    write_to_file(result_file)
                item_number += 1
                continue
        elif user_response == "e":
            rospy.loginfo("Exitting")
            moveit_commander.roscpp_shutdown()
            moveit_commander.os._exit(0)
            exit()
        elif user_response == "jump":
            jump_to_str = input("Enter the item number (row): ")
            try:
                item_number = int(jump_to_str)
            except:
                rospy.WARN("Invalid item number.")
                continue
            user_response = ""
        else:
            rospy.loginfo("Unexpected input. Try again.")
            user_response = ""
            continue

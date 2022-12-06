from unittest import result
import numpy as np
import rospy
from gazebo_msgs.srv import GetWorldProperties, GetModelState
from sensor_msgs.msg import JointState
import robot_gazebo_env
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
from openai_ros_common import ROSLauncher

class UR5eEnv(robot_gazebo_env.RobotGazeboEnv):

    def __init__(self, ros_ws_abspath):
        rospy.logdebug("========= In UR5e Env")

        self.controllers_list = []

        self.robot_name_space = ""
        self.reset_controls = False

        super(UR5eEnv, self).__init__(controllers_list=self.controllers_list,
                                      robot_name_space=self.robot_name_space,
                                      reset_controls=False,
                                      start_init_physics_parameters=False,
                                      reset_world_or_sim="WORLD")
        
        self.JOINT_STATES_SUBSCRIBER = '/joint_states'
        self.joint_names = ["shoulder_pan_joint",
                            "shoulder_lift_joint",
                            "elbow_joint",
                            "wrist_1_joint",
                            "wrist_2_joint",
                            "wrist_3_joint"]
        
        self.gazebo.unpauseSim()
        self._check_all_systems_ready()

        self.joint_states_sub = rospy.Subscriber(
            self.JOINT_STATES_SUBSCRIBER, JointState, self.joints_callback)
        self.joints = JointState()

        self.move_ur5e = MoveUR5e()

        self.wait_ur5e_ready()

        self.gazebo.pauseSim()

        rospy.logdebug("========= Out UR5e Env")

    
    def _check_all_systems_ready(self):
        self._check_all_sensor_ready()
        return True
    
    def _check_all_sensor_ready(self):
        self._check_all_joint_states_ready()
        
        rospy.logdebug("ALL SENSOR READY")
    
    def _check_all_joint_states_ready(self):
        self.joints = None
        while self.joints is None and not rospy.is_shutdown():
            try:
                self.joints = rospy.wait_for_message(
                    self.JOINT_STATES_SUBSCRIBER, JointState, timeout=1.0)
                rospy.logdebug(
                    "Current " + str(self.JOINT_STATES_SUBSCRIBER) + " READY=>" + str(self.joints))
            
            except:
                rospy.logerr(
                    "Current " + str(self.JOINT_STATES_SUBSCRIBER) + " not ready yet, retrying...")
        
        return self.joints

    def joints_callback(self, data):
        self.joints = data
    
    def get_joints(self):
        return self.joints
    
    def get_joints_name(self):
        return self.joint_names
    
    def get_joints_angle(self):
        #self.gazebo.unpauseSim()
        angles = self.move_ur5e.joints_angle()
        #self.gazebo.pauseSim()
        return angles

    def set_trajectory_ee(self, action):

        ee_target = geometry_msgs.msg.Pose()

        ee_target.position.x = action[0]
        ee_target.position.y = action[1]
        ee_target.position.z = 0.1

        ee_target.orientation.x = 0.795703
        ee_target.orientation.y = 0.605583
        ee_target.orientation.z = -0.011191
        ee_target.orientation.w = 0.000

        result = self.move_ur5e.ee_traj(ee_target)
        return result
    
    def set_trajectory_init_joints(self, initial_pose):
    
        positions_array = [None] * 6
        positions_array[0] = initial_pose["shoulder_pan_joint"]
        positions_array[1] = initial_pose["shoulder_lift_joint"]
        positions_array[2] = initial_pose["elbow_joint"]
        positions_array[3] = initial_pose["wrist_1_joint"]
        positions_array[4] = initial_pose["wrist_2_joint"]
        positions_array[5] = initial_pose["wrist_3_joint"]

        self.move_ur5e.joint_traj(positions_array)

        return True
    
    def set_trajectory_joints(self, target_joints):

        positions_array = [None] * 6
        for i in range(3):
            positions_array[i] = target_joints[i]
        positions_array[3] = self.get_joints_angle()[3]
        positions_array[4] = self.get_joints_angle()[4]
        positions_array[5] = self.get_joints_angle()[5]

        self.move_ur5e.joint_traj(positions_array)

        return True
    
    def create_action_ee(self, position):
        optik_target = np.array(position)

        return optik_target
    
    def create_action_joints(self, joints_angle):
        joints_action = np.array(joints_angle)

        return joints_action

    def create_joints_dict(self, joints_positions):
        pass

    def get_ee_pose(self):
        self.gazebo.unpauseSim()
        optik_pose = self.move_ur5e.ee_pose()
        self.gazebo.pauseSim()
        return optik_pose
    
    def get_ee_rpy(self):
        pass

    def wait_ur5e_ready(self):
        pass

    def _init_env_variables(self):
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.
        """
        raise NotImplementedError()

    def _compute_reward(self, observations, done):
        """Calculates the reward to give based on the observations given.
        """
        raise NotImplementedError()

    def _set_action(self, action):
        """Applies the given action to the simulation.
        """
        raise NotImplementedError()

    def _get_obs(self):
        raise NotImplementedError()

    def _is_done(self, observations):
        """Checks if episode done based on observations given.
        """
        raise NotImplementedError()
    
class MoveUR5e():
    def __init__(self):
        rospy.logdebug("======== In MoveUR5e")
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        rospy.logdebug("======== Out MoveUR5e")

    def ee_traj(self, pose):
        self.group.set_pose_target(pose)
        result = self.execute_trajectory()
        return result

    def joint_traj(self, positions_array):

        self.group_variable_values = self.group.get_current_joint_values()
        self.group_variable_values[0] = positions_array[0]
        self.group_variable_values[1] = positions_array[1]
        self.group_variable_values[2] = positions_array[2]
        self.group_variable_values[3] = positions_array[3]
        self.group_variable_values[4] = positions_array[4]
        self.group_variable_values[5] = positions_array[5]
        self.group.set_joint_value_target(self.group_variable_values)
        result = self.execute_trajectory()

        return result
    
    def execute_trajectory(self):
        self.plan = self.group.plan()
        result = self.group.go(wait=True)

        return result

    def ee_pose(self):
        optik_pose = self.group.get_current_pose()
        return optik_pose
    
    def ee_rpy(self):
        optik_rpy = self.group.get_current_rpy()
        return optik_rpy
    
    def joints_angle(self):
        joints_angle = self.group.get_current_joint_values()
        return joints_angle
    





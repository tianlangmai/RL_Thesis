from hashlib import new
from gym import utils
import copy
import rospy
from gym import spaces
import robot_env
from gym.envs.registration import register
import numpy as np
from sensor_msgs.msg import JointState
from openai_ros_common import ROSLauncher
from task_commons import LoadYamlFileParamsTest
import os

CONTIUOUS = False

class TaskEnv(robot_env.UR5eEnv, utils.EzPickle):
    def __init__(self):
        """Reinforcement learning environment for specific task for training the robot
            Superior: Robot environment including the main control methods and rules

            In this task environment, I define the state of observation as the position xyz value
            of optic tool and the distance between current position and desire position. Besides,
            the action space involves the movement of optic tool in x and y direction in delta distance
            within each time step. The reward function considers three factors. Firstly, the optic
            tool moves closer to or far away from the goal will be taken into account. Second, 
            we want the optic tool follow a straight line as much as accurately, in other words,
            the optic tool should be as much as close to the baseline in x level which is -0.6 in 
            this case. Moreover, we also want the optic tool reaches the goal directly without any
            redundant movement, so the y level distance still needs to be considered. When the optic
            tool reaches the goal or it is too far away from the goal, the environment will stop and
            restart again.   
        """
        ros_ws_abspath = "/home/tianlang/REL_project_ws"

        # Run the launch file to start the gazabo world and moveit
        ROSLauncher(rospackage_name="ur5e_with_optik",
                    launch_file_name="ur5e_gazebo.launch",
                    ros_ws_abspath=ros_ws_abspath)
        
        # Load the necessary parameters for RL environment into ROS parameter server
        '''LoadYamlFileParamsTest(rospackage_name="rl_env",
                               rel_path_from_package_to_file="config",
                               yaml_file_name="ur5e_test.yaml")'''
        
        rospy.logdebug("Enter TaskEnv")

        # The necessary parameters which will be included in specific yaml file in the future
        self.n_actions = 4
        self.n_observations = 4
        self.position_ee_max = 1.0
        self.position_ee_min = -1.0
        self.init_pos = {"shoulder_pan_joint":0.5332868, "shoulder_lift_joint":-1.66074, "elbow_joint":-2.006057,
                         "wrist_1_joint":-1.022607, "wrist_2_joint":1.573836, "wrist_3_joint":0.802461}
        self.position_delta = 0.01
        self.desired_goal = [-0.6, 0.1, 0.1]
        self.step_punishment = -1
        self.closer_reward = 10
        self.impossible_movement_punishment = -100
        self.reached_goal_reward = 100 
        self.w_xaxis = -7
        self.w_yaxis = -3
        self.ee_max_distance = 0.28

        self.action_space = spaces.Discrete(self.n_actions)

        observations_high_range = np.array(
            [self.position_ee_max]*self.n_observations)
        observations_low_range = np.array(
            [self.position_ee_min]*self.n_observations)

        high = np.array([observations_high_range])
        low = np.array([observations_low_range])

        self.observation_space = spaces.Box(low, high)

        super(TaskEnv, self).__init__(ros_ws_abspath)

    def get_params(self):
        """get parameters from the ros parameter server
        """        
        pass

    def set_reference_array(self, startpoint, endpoint, step):

        reference_array = [list(startpoint)]
        interval = (endpoint[1] - startpoint[1])/step
        for i in step:
            startpoint[1] += interval
            reference_array.append(list(startpoint))
        
        return reference_array

    def _set_init_pose(self):
        """reset the robot to inital pose when we call the reset function within the environment

        Returns:
            boolean : whether moveit find the planning solution based on our required goal or not
        """        
        rospy.logdebug("Init Pos:")
        rospy.logdebug(self.init_pos)
        rospy.logdebug("Moving to SETUP Joints")

        self.movement_result = self.set_trajectory_init_joints(self.init_pos)
        self.last_optik_target = [self.get_ee_pose().pose.position.x, self.get_ee_pose().pose.position.y,
                                    self.get_ee_pose().pose.position.z]
        
        # calculate the initial distance from the desire goal
        self.current_dist_from_des_pos_ee = self.calculate_distance_between(self.desired_goal, 
                                self.last_optik_target)
        

        return self.movement_result

    def _init_env_variables(self):
        rospy.logdebug("Init Env Variable...")
        rospy.logdebug("Init Env Variable...END")

    def _set_action(self, action):
        """Based on the sampled action to make the robot move

        Args:
            action (integer): the label of the action as a control 
            command which is applied into the robot
        """        
        delta_optik_target = [0.0]*2

        if action == 0:  # X+
            delta_optik_target[0] += self.position_delta
            self.last_action = "X+"
        elif action == 1:  # X-
            delta_optik_target[0] -= self.position_delta
            self.last_action = "X-"
        elif action == 2:  # Y+
            delta_optik_target[1] += self.position_delta
            self.last_action = "Y+"
        elif action == 3:  # Y-
            delta_optik_target[1] -= self.position_delta
            self.last_action = "Y-"
        
        optik_target = copy.deepcopy(self.last_optik_target)
        optik_target[0] += delta_optik_target[0]
        optik_target[1] += delta_optik_target[1]

        action_end_effector = self.create_action_ee(optik_target)
        self.movement_result = self.set_trajectory_ee(action_end_effector)
        if self.movement_result:
            self.last_optik_target = copy.deepcopy(optik_target)
        else:
            rospy.logerr("Impossible End Effector Position......" + str(optik_target))
        
        rospy.logwarn("END Set Action ===>" + str(action) + ", NAME="+str(self.last_action))

    
    def _get_obs(self):
        """After the robot performed an action, we read the observation

        Returns:
            list: observation list involving the position of optic tool and the distance from

        """        
        
        optik_pose = self.get_ee_pose()
        optik_pose_array = [optik_pose.pose.position.x, optik_pose.pose.position.y, 
                            optik_pose.pose.position.z]

        obs = optik_pose_array

        new_dist_from_des_pos_ee = self.calculate_distance_between(self.desired_goal, optik_pose_array)

        obs.append(new_dist_from_des_pos_ee)

        rospy.logdebug("OBSERVATIONS====>>>>>>>"+str(obs))

        return obs

    def _is_done(self, observations):
        """Determine whether the environment needs to stop or not 

        Args:
            observations (list): observation list

        Returns:
            boolean: whether the environment should stop or not
        """        
        current_pose = observations[:3]

        done = self.calculate_if_done(
            self.movement_result, self.desired_goal, current_pose)
        
        return done
    
    def _compute_reward(self, observations, done):
        """Calculate the reward value from the observations 

        Args:
            observations (list): observations list
            done (function): _description_

        Returns:
            float: reward value
        """        
        
        current_pos = observations[:3]
        new_dis_from_des_pos_ee = observations[-1]

        reward = self.calculate_reward(
            self.movement_result, self.desired_goal, current_pos, new_dis_from_des_pos_ee)
        
        rospy.logwarn(">>>REWARD>>>"+str(reward))

        return reward

    def calculate_if_done(self, movement_result, desired_goal, current_pos):
        """Include all conditions of stopping the environment

        Args:
            movement_result (boolean): whether moveit planner can find the planning solution or not 
            desired_goal (list): desired position in xyz
            current_pos (list): current position in xyz

        Returns:
            boolean: whether the environment should stop or not
        """        

        done = False

        if movement_result:
            position_similar = np.all(np.isclose(
                desired_goal, current_pos, atol=0.11))
            
            if position_similar:
                done = True
                rospy.logdebug("Reach a Desired Position")
            elif self.calculate_distance_between(desired_goal, current_pos) > self.ee_max_distance:
                done = True
                rospy.logdebug("Far away from desired position")
        else:
            done = True
            rospy.logdebug("Reached a TCP position not reachable")
    
        return done


    def calculate_distance_between(self, v1, v2):
        """Calculate the distance between two points in 3D world

        Args:
            v1 (list): generally desired goal position
            v2 (list): generally current point position

        Returns:
            np.array: distance between two points
        """        
        dist = np.linalg.norm(np.array(v1)-np.array(v2))

        return dist

    def calculate_reward(self, movement_result, desired_goal, current_pos, new_dist_from_des_pos_ee):
        """Considering all reward function terms and combining them all with each other

        Args:
            movement_result (boolean): whether moveit planner can find the planning solution or not
            desired_goal (list): desired position in xyz
            current_pos (list): current position in xyz
            new_dist_from_des_pos_ee (np.array): after performing the action the distance between current
                                                 and desired position

        Returns:
            float: reward value
        """              
        if movement_result:
            position_similar = np.all(np.isclose(desired_goal, current_pos, atol=0.11))

            rospy.logwarn("desired_goal="+str(desired_goal))
            rospy.logwarn("current_pos="+str(current_pos))
            rospy.logwarn("current_dist_from_des_pos_ee="+str(self.current_dist_from_des_pos_ee))
            rospy.logwarn("new_dist_from_des_pos_ee="+str(new_dist_from_des_pos_ee))

            delta_dist = new_dist_from_des_pos_ee - self.current_dist_from_des_pos_ee
            reward = self.w_xaxis * (current_pos[0] - self.desired_goal[0])**2 + self.w_yaxis * \
                        (current_pos[1]-self.desired_goal[1])**2
            if position_similar:
                reward = self.reached_goal_reward
                rospy.logwarn("Reached a desired position")
            else:
                if delta_dist < 0:
                    reward += self.closer_reward
                    rospy.logwarn("CLOSER to desired position="+str(delta_dist))
                else:
                    reward += self.step_punishment
                    rospy.logwarn("FURTHER from desired position="+str(delta_dist))
        else:
            reward = self.impossible_movement_punishment
            rospy.logwarn("Reach TCP no reachable")

        self.current_dist_from_des_pos_ee = new_dist_from_des_pos_ee
        rospy.logdebug("Updated distance from Goal=="+str(new_dist_from_des_pos_ee))

        return reward






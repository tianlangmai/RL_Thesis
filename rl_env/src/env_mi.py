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
import time

class TaskEnv(robot_env.UR5eEnv, utils.EzPickle):
    def __init__(self):
        """Reinforcement learning environment for specific task for training the robot
            Superior: Robot environment including the main control methods and rules

            In this task environment, I define the state of observation as the position xyz value
            of optic tool and the distance between current position and desire position. Besides,
            the action space only involves the rotation of shoulder_pan_joint, shoulder_lift_joint and 
            elbow_joint in delta angle within each time step, so that we can keep the orientation of optic 
            tool the same as the initial orientation. The reward function considers four factors. 
            Firstly, the optic tool moves closer to or far away from the goal will be taken 
            into account. Secondly, we want the optic tool follow a straight line as much as accurately, 
            in other words, the optic tool should be as much as close to the baseline in x and z level which 
            are -0.6 and 0.1 respectively in this case. Moreover, we also want the optic tool reaches the 
            goal directly without any redundant movement, so the y level distance still needs to be considered. 
            When the optic tool reaches the goal or it is too far away from the goal, the environment will stop and
            restart again.
        """    
        ros_ws_abspath = "/home/tianlang/RL_Thesis"

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
        self.n_actions = 12
        self.n_observations = 4
        self.position_ee_max = 1.0
        self.position_ee_min = -1.0
        self.init_pos = {"shoulder_pan_joint":0.5332868, "shoulder_lift_joint":-1.66074, "elbow_joint":-2.006057,
                         "wrist_1_joint":-1.022607, "wrist_2_joint":1.573836, "wrist_3_joint":0.802461}
        self.position_delta = 0.01
        self.startpoint = [-0.6, -0.1, 0.1]
        self.desired_goal = [-0.6, 0.1, 0.1]
        self.step_punishment = -50
        self.closer_reward = 50
        self.impossible_movement_punishment = -100
        self.reached_goal_reward = 100 
        #self.w_xaxis = -7
        #self.w_yaxis = -3
        self.ee_max_distance = 0.28
        self.weight_distance = 10000
        #self.w_zaxis = -5
        self.action_bound = 0.0001
        action_upper = np.array([self.action_bound] * self.n_actions)
        self.action_space = spaces.Box(-action_upper, action_upper)
        self.reference_trajectory = self.set_reference_array(self.startpoint, self.desired_goal, 100)
        #print(self.reference_trajectory)
        observations_high_range = np.array(
            [self.position_ee_max]*self.n_observations)
        observations_low_range = np.array(
            [self.position_ee_min]*self.n_observations)

        self.a1 = 0.2449
        self.b1 = -0.4595
        self.c1 = -0.1396
        self.d1 = 0.5448
        self.a2 = -0.0168
        self.b2 = 0.0315
        self.c2 = 0.0094
        self.d2 = -1.6615
        self.a3 = 0.0179
        self.b3 = -0.0336
        self.c3 = -0.0102
        self.d3 = -2.0051
        #high = np.array([observations_high_range])
        #low = np.array([observations_low_range])

        self.observation_space = spaces.Box(observations_low_range, observations_high_range)


        super(TaskEnv, self).__init__(ros_ws_abspath)

    def get_params(self):
        """get parameters from the ros parameter server
        """        
        pass

    def set_reference_array(self, startpoint, endpoint, step):

        reference_array = []
        start_point = copy.deepcopy(startpoint)
        reference_array.append(start_point)
        interval = (endpoint[1] - startpoint[1])/step
        for i in range(step):
            startpoint[1] = startpoint[1] + interval
            point = copy.deepcopy(startpoint)
            reference_array.append(point)
        
        return np.array(reference_array, dtype=np.float32)

    def _set_init_pose(self, count):
        """reset the robot to inital pose when we call the reset function within the environment

        Returns:
            boolean : whether moveit find the planning solution based on our required goal or not
        """ 
        rospy.logdebug("Init Pos:")
        rospy.logdebug(self.init_pos)
        rospy.logdebug("Moving to SETUP Joints")

        self.movement_result = self.set_trajectory_init_joints(self.init_pos)
        self.last_joints_angle = [self.get_joints_angle()[0], self.get_joints_angle()[1], self.get_joints_angle()[2]]
        self.last_optik_target = [self.get_ee_pose().pose.position.x, self.get_ee_pose().pose.position.y,
                                    self.get_ee_pose().pose.position.z]

        
        self.current_dist_from_des_pos_ee = self.calculate_distance_between(self.reference_trajectory[count], 
                                self.last_optik_target)
        

        return self.movement_result

    def _init_env_variables(self):
        rospy.logdebug("Init Env Variable...")
        rospy.logdebug("Init Env Variable...END")

    def _set_action(self, action, dt):
        """Based on the sampled action to make the robot move

        Args:
            action (integer): the label of the action as a control 
            command which is applied into the robot
        """

        poly_dev = [0.0]*12

        for i in range(12):
            poly_dev[i] = action[i]
        
        joints_angle = copy.deepcopy(self.last_joints_angle)
        #for i in range(3):
            #joints_angle[i] += delta_joint_angle[i]
        #current_time = time.time()
        #dt = current_time - begin_time
        #print(dt)
        #self.dt = t
        joints_angle[0] = (self.a1+poly_dev[0])*(dt**3) + (self.b1+poly_dev[1])*(dt**2) + (self.c1+poly_dev[2])*dt + self.d1+poly_dev[3]
        joints_angle[1] = (self.a2+poly_dev[4])*(dt**3) + (self.b2+poly_dev[5])*(dt**2) + (self.c2+poly_dev[6])*dt + self.d2+poly_dev[7]
        joints_angle[2] = (self.a3+poly_dev[8])*(dt**3) + (self.b3+poly_dev[9])*(dt**2) + (self.c3+poly_dev[10])*dt + self.d3+poly_dev[11]

        joints_action = self.create_action_joints(joints_angle)
        self.movement_result = self.set_trajectory_joints(joints_action)
        if self.movement_result:
            self.last_joints_angle = copy.deepcopy(joints_angle)
        else:
            rospy.logerr("Impossible Joints Position......" + str(joints_action))
        
        #rospy.logwarn("END Set Action ===>" + str(action) + ", NAME="+str(self.last_action))

    
    def _get_obs(self, count):
        """After the robot performed an action, we read the observation

        Returns:
            list: observation list involving the position of optic tool and the distance from

        """   
        
        optik_pose = self.get_ee_pose()
        optik_pose_array = [optik_pose.pose.position.x, optik_pose.pose.position.y, 
                            optik_pose.pose.position.z]

        obs = optik_pose_array

        new_dist_from_des_pos_ee = self.calculate_distance_between(self.reference_trajectory[count], optik_pose_array)

        obs.append(new_dist_from_des_pos_ee)
        #print(obs)

        #rospy.logdebug("OBSERVATIONS====>>>>>>>"+str(obs))

        return obs

    def _is_done(self, observations, count):
        """Determine whether the environment needs to stop or not 

        Args:
            observations (list): observation list

        Returns:
            boolean: whether the environment should stop or not
        """ 
        
        current_pose = observations[:3]

        done = self.calculate_if_done(
            self.movement_result, self.desired_goal, current_pose, count)
        
        return done
    
    def _compute_reward(self, observations, done, count):
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
            self.movement_result, self.desired_goal, current_pos, new_dis_from_des_pos_ee, count)
        
        #rospy.logwarn(">>>REWARD>>>"+str(reward))

        return reward

    def calculate_if_done(self, movement_result, desired_goal, current_pos, count):
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
                desired_goal, current_pos, atol=0.01))
            
            if position_similar:
                done = True
                rospy.logdebug("Reach a Desired Position")
            elif self.calculate_distance_between(self.reference_trajectory[count], current_pos) > self.ee_max_distance:
                done = True
                rospy.logdebug("Far away from desired position")
            elif current_pos[2] < 0.01:
                done = True
                rospy.logdebug("Position is too low")
            elif count == 100:
                done = True
        else:
            done = True
            rospy.logdebug("Reached a TCP position not reachable")
    
        return done


    def calculate_distance_between(self, v1:list, v2:list) -> float: 
        """Calculate the distance between two points in 3D world

        Args:
            v1 (list): generally desired goal position
            v2 (list): generally current point position

        Returns:
            np.array: distance between two points
        """      
        dist = np.linalg.norm(np.array(v1)-np.array(v2))

        return dist

    def calculate_reward(self, movement_result:bool, desired_goal:list, current_pos:list, new_dist_from_des_pos_ee, count) -> float:
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
            position_similar = np.all(np.isclose(self.reference_trajectory[count], current_pos, atol=0.005))
            #startpoint = copy.deepcopy(self.startpoint)
            #desired_goal = copy.deepcopy(self.desired_goal)
            #reference_trajectory = self.set_reference_array(startpoint, desired_goal, 100)
            #rospy.logwarn("desired_goal="+str(desired_goal))
            #rospy.logwarn("current_pos="+str(current_pos))
            #rospy.logwarn("current_dist_from_des_pos_ee="+str(self.current_dist_from_des_pos_ee))
            #rospy.logwarn("new_dist_from_des_pos_ee="+str(new_dist_from_des_pos_ee))
            #rospy.logwarn("current_reference_point="+str(reference_trajectory[count]))

            delta_dist = new_dist_from_des_pos_ee - self.current_dist_from_des_pos_ee
            reward = -self.weight_distance * self.calculate_distance_between(current_pos, self.reference_trajectory[count])
            '''reward = self.w_xaxis * (current_pos[0] + 0.6)**2 + self.w_yaxis * (current_pos[1]-0.1)**2 + self.w_zaxis \
                * (current_pos[2] - 0.1)**2'''
            #rospy.logwarn("dist from references = "+str(self.calculate_distance_between(current_pos, reference_trajectory[count])))
            if position_similar:
                reward += self.reached_goal_reward
                #rospy.logwarn("Reached a desired position")
            else:
                if delta_dist < 0:
                    reward += self.closer_reward
                    #rospy.logwarn("CLOSER to desired position="+str(delta_dist))
                else:
                    reward += self.step_punishment
                    #rospy.logwarn("FURTHER from desired position="+str(delta_dist))
        else:
            reward = self.impossible_movement_punishment
            #rospy.logwarn("Reach TCP no reachable")

        self.current_dist_from_des_pos_ee = new_dist_from_des_pos_ee
        rospy.logdebug("Updated distance from Goal=="+str(new_dist_from_des_pos_ee))

        return reward
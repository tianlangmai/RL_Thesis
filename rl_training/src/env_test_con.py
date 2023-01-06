#!/usr/bin/env python3
import sys
sys.path.append("/home/tianlang/RL_Thesis/src/rl_env/src/")
sys.path.append("/home/tianlang/RL_Thesis/src/rl_training/src/")
#from stable_baselines3 import TD3
#sys.path.append(r'../..')
from openai_ros_common import StartOpenAI_ROS_Environment
import rospy
import numpy as np
from stable_baselines3 import TD3
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise


if __name__ == '__main__':
    rospy.init_node('ur5e_test_con',
                    anonymous=True, log_level=rospy.WARN)
    env = StartOpenAI_ROS_Environment('UR5eTask-v3')
    for _ in range(50):
        state, done = env.reset(), False
        count = 0
        while not done:
            action= env.action_space.sample()
            state, reward, done, _ = env.step(action, count=count)
            count += 1


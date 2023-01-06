#!/usr/bin/env python3
import sys
sys.path.append("/home/tianlang/RL_Thesis/src/rl_env/src/")
#sys.path.append(r'../..')
from openai_ros_common import StartOpenAI_ROS_Environment
import rospy


if __name__ == '__main__':
    rospy.init_node('ur5e_test_v2',
                    anonymous=True, log_level=rospy.WARN)
    env = StartOpenAI_ROS_Environment('UR5eTask-v2')
    for _ in range(10):
        state, done = env.reset(), False
        count = 0
        while not done:
            count += 1
            action = env.action_space.sample()
            state, reward, done, _ = env.step(action, count)

    env.close()
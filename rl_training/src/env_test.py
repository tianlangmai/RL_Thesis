#!/usr/bin/env python3
import sys
sys.path.append("/home/tianlang/REL_project_ws/src/rl_env/src/")
#sys.path.append(r'../..')
from openai_ros_common import StartOpenAI_ROS_Environment
import rospy


if __name__ == '__main__':
    rospy.init_node('ur5e_test',
                    anonymous=True, log_level=rospy.WARN)
    env = StartOpenAI_ROS_Environment('UR5eTask-v0')
    for _ in range(10):
        state, done = env.reset(), False
        while not done:
            action = env.action_space.sample()
            state, reward, done, _ = env.step(action)

    env.close()

#!/usr/bin/env python3

import rospy
import random
import gym
import numpy as np
import math
from collections import deque
import sys
sys.path.append("/home/tianlang/REL_project_ws/src/rl_env/src/")
from openai_ros_common import StartOpenAI_ROS_Environment
import torch
from torch import nn
import torch.nn.functional as F
import rospkg
import os
import matplotlib.pyplot as plt

class DeepQNetwork(nn.Module):
    def __init__(self, n_obervations, n_actions):
        super(DeepQNetwork, self).__init__()
        self.policy = nn.Sequential(
            nn.Linear(in_features=n_obervations, out_features=24),
            nn.ReLU(),
            nn.Linear(in_features=24, out_features=48),
            nn.ReLU(),
            nn.Linear(in_features=48, out_features=n_actions),
            nn.Tanh()
        )

        '''for m in self.children():
            if isinstance(m, nn.Linear):
                nn.init.constant_(m.weight, 1)
                nn.init.constant_(m.bias, -100)'''
    
    def forward(self, state):
        Q = self.policy(state)

        return Q
    
class DQNSolver():
    def __init__(self, env, n_observations, n_actions, min_episodes, gamma, epsilon, 
                min_epsilon, epsilon_decay, alpha, alpha_decay, batch_size, network_update_frequency, 
                network_sync_frequency):
        self.memory = deque(maxlen=100000)
        self._env = env

        self.input_dim = n_observations
        self.output_dim = n_actions
        self.gamma = gamma
        self.epsilon = epsilon
        self.min_epsilon = min_epsilon
        self.epsilon_decay = epsilon_decay
        self.alpha = alpha
        #self.alpha_decay = alpha_decay
        self.min_episodes = min_episodes
        self.batch_size = batch_size

        self.q_network = DeepQNetwork(self.input_dim, self.output_dim)
        self.target_network = DeepQNetwork(self.input_dim, self.output_dim)
        self.target_network.load_state_dict(self.q_network.state_dict())
        self.optimizer = torch.optim.Adam(self.q_network.parameters(), lr=self.alpha)
        self.network_update_frequency = network_update_frequency
        self.network_sync_frequency = network_sync_frequency

    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))
    
    def choose_action(self, state, epsilon, do_train, iteration=0):

        state = torch.FloatTensor(state)

        if do_train and (np.random.random() <= epsilon):
            rospy.logfatal(">>>>>Chosen Random ACTION")
            action_chosen = self._env.action_space.sample()
        
        else:
            action_chosen = torch.argmax(self.q_network(state)).item()
        
        return action_chosen
    
    def get_epsilon(self, t):
        new_epsilon = max(self.min_epsilon, min(
            self.epsilon, 1.0 - math.log10((t + 1) * self.epsilon_decay)))

        return new_epsilon
    
    def preprocess_state(self, state):
        return np.reshape(state, [1, self.input_dim])
    
    def replay(self, batch_size):
        q_values = []
        q_targets = []
        mse = nn.MSELoss()
        minibatch = random.sample(
            self.memory, min(len(self.memory), batch_size))
        for state, action, reward, next_state, done in minibatch:
            state = torch.FloatTensor(state)
            next_state = torch.FloatTensor(next_state)
            #action = torch.IntTensor(action)
            #reward = torch.Tensor(reward)
            #done = torch.BoolTensor(done)

            q_value = self.q_network(state)[action]
            if done:
                q_target = reward
            else:
                q_target = reward + self.gamma * torch.max(self.target_network(next_state))
        
            q_values.append(q_value)
            q_targets.append(q_target)

        q_values = torch.tensor(q_values, requires_grad=True).view(-1,1)
        q_targets = torch.tensor(q_targets, requires_grad=True).view(-1,1)

        loss = mse(q_values, q_targets)
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()
    
    def run(self, num_episodes, do_train=False):

        rate = rospy.Rate(30)

        for e in range(num_episodes):
            state = self._env.reset()

            #state = self.preprocess_state(init_state)
            done = False

            step_count = 0
            while not done:

                action = self.choose_action(state, self.get_epsilon(e), do_train, step_count)

                next_state, reward, done, _ = self._env.step(action)
                #next_state = self.preprocess_state(next_state)
                
                if do_train:
                    
                    self.remember(state, action, reward, next_state, done)
                
                state = next_state
                step_count += 1

                if do_train:

                    if step_count % self.network_update_frequency == 0:
                        self.replay(self.batch_size)

                    if step_count % self.network_update_frequency == 0:
                        self.target_network.load_state_dict(self.q_network.state_dict())

        return e
    
    def save(self, model_dir_path):
        torch.save(self.q_network.state_dict(), model_dir_path)
    
        
if __name__== '__main__':
    rospy.init_node('ur5e_dqn_algorithm', 
                    anonymous=True, log_level=rospy.WARN)

    env = StartOpenAI_ROS_Environment('UR5eTask-v2')
    env_name = "ur5e_task_v2"
    rospackage_name = "rl_training"

    agent = DQNSolver(
        env,
        n_observations=4,
        n_actions=6,
        min_episodes=10,
        gamma=1.0,
        epsilon=0.2,
        min_epsilon=0.02,
        epsilon_decay=0.9,
        alpha=0.1,
        alpha_decay=0.01,
        batch_size=64,
        network_update_frequency=4,
        network_sync_frequency=200)
    
    agent.run(num_episodes=100, do_train=True)

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path(rospackage_name)
    outdir = pkg_path + '/models'
    if not os.path.exists(outdir):
        os.makedirs(outdir)
        rospy.logfatal("Created folder="+str(outdir))

    agent.save(outdir)



    
                






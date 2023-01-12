#!/usr/bin/env python3
import sys
sys.path.append("/home/tianlang/RL_Thesis/src/rl_env/src/")
sys.path.append("/home/tianlang/RL_Thesis/src/rl_training/src/TD3/")
from openai_ros_common import StartOpenAI_ROS_Environment
import numpy as np
import torch
import gym
import argparse
import os
import time

import utils
import TD3
import rospy
import numpy as np
import wandb

# Runs policy for X episodes and returns average reward
# A fixed seed is used for the eval environment
def eval_policy(policy, eval_env, seed, eval_episodes=10):
    eval_env.seed(seed + 100)
    total_reward = 0
    for _ in range(eval_episodes):
        eval_episodes_timestep = 0
        begin_time = time.time()
        state, done = eval_env.reset(eval_episodes_timestep), False
        while not done:
            eval_episodes_timestep += 1
            action = policy.select_action(np.array(state))
            state, reward, done, _ = eval_env.step(action, eval_episodes_timestep, begin_time)
            total_reward += reward
    avg_reward = total_reward / eval_episodes_timestep

    print("------------------------------------")
    print(f"Evaluation over {eval_episodes} episodes: {avg_reward:.3f}")
    print("------------------------------------")
    return avg_reward

def td3_training(env, load_model, save_model, seed, discount, tau, batch_size, policy_noise, 
                    noise_clip, policy_freq, start_timesteps, max_timesteps, eval_freq, 
                    expl_noise, lr_actor, lr_critic):
    


    file_name = "TD3_UR5eTask-v3_0"
    print("---------------------------------------")
    print(f"Policy: TD3, Env: UR5eTask-v3_0, Seed: {seed}")
    print("---------------------------------------")

    if not os.path.exists("/home/tianlang/trained_models/results"):
        os.makedirs("/home/tianlang/trained_models/results")

    if save_model and not os.path.exists("/home/tianlang/trained_models/models"):
        os.makedirs("/home/tianlang/trained_models/models")

    #env = gym.make(args.env)

    # Set seeds
    env.seed(seed)
    env.action_space.seed(seed)
    torch.manual_seed(seed)
    np.random.seed(seed)
    
    state_dim = env.observation_space.shape[0]
    action_dim = env.action_space.shape[0] 
    max_action = float(env.action_space.high[0])

    kwargs = {
        "state_dim": state_dim,
        "action_dim": action_dim,
        "max_action": max_action,
        "discount": discount,
        "tau": tau,
    }

    # Initialize policy
    #if args.policy == "TD3":
        # Target policy smoothing is scaled wrt the action scale
    kwargs["policy_noise"] = policy_noise * max_action
    kwargs["noise_clip"] = noise_clip * max_action
    kwargs["policy_freq"] = policy_freq
    kwargs["lr_actor"] = lr_actor
    kwargs["lr_critic"] = lr_critic
    policy = TD3.TD3(**kwargs)
    #elif args.policy == "OurDDPG":
        #policy = OurDDPG.DDPG(**kwargs)
    #elif args.policy == "DDPG":
        #policy = DDPG.DDPG(**kwargs)

    if load_model:
        policy_file = file_name #if args.load_model == "default" else args.load_model
        policy.load(f"/home/tianlang/trained_models/models/{policy_file}")

    replay_buffer = utils.ReplayBuffer(state_dim, action_dim)
    
    # Evaluate untrained policy
    #evaluations = [eval_policy(policy, env, seed)]
    evaluations = []

    episode_timesteps = 0
    state, done = env.reset(episode_timesteps), False
    episode_reward = 0
    episode_num = 0
    dt = 0
    #begin_time = time.time()

    for t in range(int(max_timesteps)):
        
        episode_timesteps += 1

        #begin_time = time.time()
        # Select action randomly or according to policy
        if t < start_timesteps:
            action = env.action_space.sample()
        else:
            action = (
                policy.select_action(np.array(state))
                + np.random.normal(0, max_action * expl_noise, size=action_dim)
            ).clip(-max_action, max_action)

        # Perform action
        next_state, reward, done, _ = env.step(action, episode_timesteps, dt) 
        done_bool = float(done) if episode_timesteps < env._max_episode_steps else 0
        
        # Store data in replay buffer
        replay_buffer.add(state, action, next_state, reward, done_bool)

        state = next_state
        episode_reward += reward
        dt += 0.002

        # Train agent after collecting sufficient data
        if t >= start_timesteps:
            policy.train(replay_buffer, batch_size)

        if done: 
            # +1 to account for 0 indexing. +0 on ep_timesteps since it will increment +1 even if done=True
            print(f"Total T: {t+1} Episode Num: {episode_num+1} Episode T: {episode_timesteps} Reward: {episode_reward:.3f}")
            # Reset environment
            dt = 0
            episode_timesteps = 0
            state, done = env.reset(episode_timesteps), False
            episode_reward = 0
            #episode_timesteps = 0
            episode_num += 1 

        # Evaluate episode
        if (t + 1) % eval_freq == 0:
            eval_values = eval_policy(policy, env, seed)
            evaluations.append(eval_values)
            #wandb.log({"avg_reward":eval_values})
            np.save(f"/home/tianlang/trained_models/results/{file_name}", evaluations)
            if save_model: policy.save(f"/home/tianlang/trained_models/models/{file_name}")

if __name__ == '__main__':
    rospy.init_node('ur5e_td3_v2',
                    anonymous=True)
    env = StartOpenAI_ROS_Environment('UR5eTask-v4')
    td3_training(env, 
                load_model=False, 
                save_model=False,
                start_timesteps=50, 
                expl_noise=0.01,
                tau=0.005,
                seed=0,
                discount=0.99,
                batch_size=64,
                policy_noise=0.002,
                policy_freq=30,
                noise_clip=0.005,
                eval_freq=5e3,
                max_timesteps=1e6,
                lr_actor=3e-4,
                lr_critic=3e-4)      
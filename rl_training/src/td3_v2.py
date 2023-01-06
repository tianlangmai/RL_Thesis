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

import utils
import TD3
import rospy
import numpy as np
import wandb

# Runs policy for X episodes and returns average reward
# A fixed seed is used for the eval environment
def eval_policy(policy, eval_env, seed, eval_episodes=10):
	#eval_env = StartOpenAI_ROS_Environment(env_name)
	eval_env.seed(seed + 100)

	avg_reward = 0.
	for _ in range(eval_episodes):
		state, done = eval_env.reset(), False
		eval_episodes_timestep = 0
		while not done:
			eval_episodes_timestep += 1
			action = policy.select_action(np.array(state))
			state, reward, done, _ = eval_env.step(action, eval_episodes_timestep)
			avg_reward += reward
	
	avg_reward /= reward

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
    evaluations = [eval_policy(policy, env, seed)]

    state, done = env.reset(), False
    episode_reward = 0
    episode_timesteps = 0
    episode_num = 0

    for t in range(int(max_timesteps)):
        
        episode_timesteps += 1

        # Select action randomly or according to policy
        if t < start_timesteps:
            action = env.action_space.sample()
        else:
            action = (
                policy.select_action(np.array(state))
                + np.random.normal(0, max_action * expl_noise, size=action_dim)
            ).clip(-max_action, max_action)

        # Perform action
        next_state, reward, done, _ = env.step(action, episode_timesteps) 
        done_bool = float(done) if episode_timesteps < env._max_episode_steps else 0
        
        # Store data in replay buffer
        replay_buffer.add(state, action, next_state, reward, done_bool)

        state = next_state
        episode_reward += reward

        # Train agent after collecting sufficient data
        if t >= start_timesteps:
            policy.train(replay_buffer, batch_size)

        if done: 
            # +1 to account for 0 indexing. +0 on ep_timesteps since it will increment +1 even if done=True
            print(f"Total T: {t+1} Episode Num: {episode_num+1} Episode T: {episode_timesteps} Reward: {episode_reward:.3f}")
            wandb.log({"reward":episode_reward})
            # Reset environment
            state, done = env.reset(), False
            episode_reward = 0
            episode_timesteps = 0
            episode_num += 1 

        # Evaluate episode
        if (t + 1) % eval_freq == 0:
            eval_values = eval_policy(policy, env, seed)
            evaluations.append(eval_values)
            wandb.log({"avg_reward":eval_values})
            np.save(f"/home/tianlang/trained_models/results/{file_name}", evaluations)
            if save_model: policy.save(f"/home/tianlang/trained_models/models/{file_name}")

if __name__ == '__main__':
    rospy.init_node('ur5e_td3_v2',
                    anonymous=True, log_level=rospy.WARN)
    env = StartOpenAI_ROS_Environment('UR5eTask-v3')
    wandb.login()

    sweep_config = {'method':'random'}
    metric = {'name':'loss', 'goal':'minimize'}
    sweep_config['metric'] = metric
    parameters_dict = {
        'seed':{'value': 0},
        'batch_size':{'distribution':'q_log_uniform_values', 'q':8, 'min':32, 'max':256},
        'start_timesteps':{'value':50},
        'tau':{'distribution':'uniform', 'min':0.0, 'max':1.0},
        'discount':{'distribution':'uniform', 'min':0.90, 'max':0.99},
        'policy_freq':{'distribution':'int_uniform', 'min':10.0, 'max':30.0},
        'policy_noise':{'distribution':'uniform', 'min':0.1, 'max':1.0},
        'noise_clip':{'distribution':'uniform', 'min':0.5, 'max':3.0},
        'expl_noise':{'distribution':'uniform', 'min':0.01, 'max':0.05},
        'eval_freq':{'value': 100},
        'max_timesteps':{'value':1000000},
        'lr_actor':{'value':3e-4},
        'lr_critic':{'value':3e-4},
        'w_xterm':{'distribution':'uniform', 'min':-10, 'max':-1},
        'w_yterm':{'distribution':'uniform', 'min':-10, 'max':-1},
        'w_zterm':{'distribution':'uniform', 'min':-10, 'max':-1}
    }

    sweep_config['parameters'] = parameters_dict

    sweep_id = wandb.sweep(sweep_config, project='reinforcement learning square path')

    def train(config=None):
        with wandb.init(config=config):
            config = wandb.config
            env.w_xaxis = config.w_xterm
            env.w_yaxis = config.w_yterm
            env.w_zaxis = config.w_zterm
            td3_training(env, load_model=False, save_model=True, 
                            seed=config.seed,
                            discount=config.discount,
                            tau=config.tau,
                            batch_size=config.batch_size,
                            policy_noise=config.policy_noise,
                            noise_clip=config.noise_clip,
                            policy_freq=config.policy_freq,
                            start_timesteps=config.start_timesteps,
                            max_timesteps=config.max_timesteps,
                            eval_freq=config.eval_freq,
                            expl_noise=config.expl_noise,
                            lr_actor=config.lr_actor,
                            lr_critic=config.lr_critic)
            
            #wandb.log({"rewards":env.cumulated_episode_reward})
    wandb.agent(sweep_id, train, count=1)



    
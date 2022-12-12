#!/usr/bin/env python3
import sys
sys.path.append("/root/catkin_ws/src/RL_Thesis/rl_env/src/")
from stable_baselines3 import DQN
from stable_baselines3.common.utils import safe_mean
from openai_ros_common import StartOpenAI_ROS_Environment
import rospy
import numpy as np
import wandb

if __name__ == '__main__':
    rospy.init_node('ur5e_test_dqnsb',
                    anonymous=True, log_level=rospy.WARN)
    env = StartOpenAI_ROS_Environment('UR5eTask-v2')

    wandb.login()

    sweep_config = {'method':'random'}
    metric = {'name':'loss', 'goal':'minimize'}
    sweep_config['metric'] = metric
    parameters_dict = {
        'learning_rate':{'distribution':'uniform', 'min':0.0, 'max':0.1},
        'batch_size':{'distribution':'q_log_uniform_values', 'q':8, 'min':32, 'max':256},
        'buffer_size':{'value':1000000},
        'learning_starts':{'value':50000},
        'tau':{'distribution':'uniform', 'min':0.0, 'max':1.0},
        'gamma':{'distribution':'uniform', 'min':0.90, 'max':0.99},
        'train_freq':{'distribution':'int_uniform', 'min':4.0, 'max':10.0},
        'gradient_steps':{'value':1},
        'target_update_interval': {'value':10000},
        'exploration_fraction': {'value':0.1},
        'exploration_initial_eps':{'value':1.0},
        'exploration_final_eps':{'value':0.05},
        'max_grad_norm':{'value':10},
        'w_xterm':{'distribution':'uniform', 'min':-10, 'max':-1},
        'w_yterm':{'distribution':'uniform', 'min':-10, 'max':-1},
        'w_zterm':{'distribution':'uniform', 'min':-10, 'max':-1}
    }
    sweep_config['parameters'] = parameters_dict

    sweep_id = wandb.sweep(sweep_config, project='reinforcement learning square path')

    def train(config=None):
        with wandb.init(config=config):
            config = wandb.config
            env.w_xterm = config.w_xterm
            env.w_yterm = config.w_yterm
            env.w_zterm = config.w_zterm
            model = DQN("MlpPolicy", env, 
                            learning_rate=config.learning_rate, 
                            buffer_size=config.buffer_size, 
                            learning_starts=config.learning_starts, 
                            batch_size=config.batch_size,
                            tau=config.tau,
                            gamma=config.gamma,
                            train_freq=config.train_freq,
                            gradient_steps=config.gradient_steps,
                            target_update_interval=config.target_update_interval,
                            exploration_fraction=config.exploration_fraction,
                            exploration_initial_eps=config.exploration_initial_eps,
                            exploration_final_eps=config.exploration_final_eps,
                            max_grad_norm=config.max_grad_norm,
                            wandb_tuning=True,
                            verbose=1)
            model.learn(total_timesteps=20, log_interval=1)
            wandb.log({"rewards":safe_mean([ep_info["r"] for ep_info in model.ep_info_buffer])})
            model.save("/home/tianlang/trained_models/dqn2_ur5e")
    
    wandb.agent(sweep_id, train, count=5)

    
    #del model

    #model = DQN.load("/home/tianlang/trained_models/dqn2_ur5e")

    #obs = env.reset()

    '''while True:
        action, _states = model.predict(obs, deterministic=True)
        obs, reward, done, info = env.step(action)
        #env.render()
        if done:
            obs = env.reset()'''
    
#!/usr/bin/env python3
import sys
sys.path.append("/home/tianlang/RL_Thesis/src/rl_env/src/")
from stable_baselines3 import TD3
from stable_baselines3.common.noise import NormalActionNoise
from stable_baselines3.common.utils import safe_mean
from openai_ros_common import StartOpenAI_ROS_Environment
import rospy
import numpy as np
import wandb

if __name__ == '__main__':
    rospy.init_node('ur5e_test_td3',
                    anonymous=True, log_level=rospy.WARN)
    env = StartOpenAI_ROS_Environment('UR5eTask-v3')

    wandb.login()

    # The noise objects for TD3
    n_actions = env.action_space.shape[-1]
    action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))

    sweep_config = {'method':'random'}
    metric = {'name':'loss', 'goal':'minimize'}
    sweep_config['metric'] = metric
    parameters_dict = {
        'learning_rate':{'distribution':'uniform', 'min':0.0, 'max':0.1},
        'batch_size':{'distribution':'q_log_uniform_values', 'q':8, 'min':32, 'max':256},
        'buffer_size':{'value':1000000},
        'learning_starts':{'value':500},
        'tau':{'distribution':'uniform', 'min':0.0, 'max':1.0},
        'gamma':{'distribution':'uniform', 'min':0.90, 'max':0.99},
        'train_freq':{'distribution':'int_uniform', 'min':4.0, 'max':10.0},
        'gradient_steps':{'value':-1},
        'policy_delay':{'distribution':'int_uniform', 'min':10.0, 'max':30.0},
        'target_policy_noise':{'distribution':'uniform', 'min':0.1, 'max':1.0},
        'target_noise_clip':{'distribution':'uniform', 'min':0.5, 'max':3.0},
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
            model = TD3("MlpPolicy", env, 
                            learning_rate=config.learning_rate, 
                            buffer_size=config.buffer_size, 
                            learning_starts=config.learning_starts, 
                            batch_size=config.batch_size,
                            tau=config.tau,
                            gamma=config.gamma,
                            train_freq=config.train_freq,
                            gradient_steps=config.gradient_steps,
                            action_noise=action_noise,
                            policy_delay=config.policy_delay,
                            target_policy_noise=config.target_policy_noise,
                            target_noise_clip=config.target_noise_clip,
                            wandb_tuning=True,
                            verbose=1)
            model.learn(total_timesteps=10, log_interval=4)
            wandb.log({"rewards":safe_mean([ep_info["r"] for ep_info in model.ep_info_buffer])})
            model.save("/home/tianlang/trained_models/td3_ur5e2")
    
    wandb.agent(sweep_id, train, count=5)

    print("predict")
    #del model

    model = TD3.load("/home/tianlang/trained_models/td3_ur5e2")
    for i in range (50):
        obs, done = env.reset(), False
        count = 0
        while not done:
            action, _states = model.predict(obs)
            obs, reward, done, info = env.step(action, count)
            count += 1
            

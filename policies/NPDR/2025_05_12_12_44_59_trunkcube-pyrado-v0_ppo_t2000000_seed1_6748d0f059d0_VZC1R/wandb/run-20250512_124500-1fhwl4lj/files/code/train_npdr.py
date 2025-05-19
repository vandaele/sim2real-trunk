"""NPDR on Random_envs

  Example:

    python test_dropo.py --env RandomHopper-v0 --test_env RandomHopper-v0 -n 1 --budget 50 --now 1 --data custom --data_path tmp/hopper10000 --wandb_mode disabled

"""

from pprint import pprint
import argparse
import pdb
import sys
import socket
import os
import time

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import gym
import torch
import wandb
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.evaluation import evaluate_policy

from pyrado.algorithms.meta.sbi_base import SBIBase


import re

import shutil


# import random_envs
from envs.RandomVecEnv import RandomSubprocVecEnv
from utils.utils import *
from policy.policy import Policy

from sofagym import *

import math

import sbi.utils as sbiutils
from sbi.inference import SNPE_C

import pyrado
from pyrado.algorithms.meta.npdr import NPDR
from pyrado.logger.experiment import save_dicts_to_yaml, setup_experiment
from pyrado.sampling.sbi_embeddings import (
    AllStepsEmbedding,
    BayesSimEmbedding,
    DeltaStepsEmbedding,
    DynamicTimeWarpingEmbedding,
    LastStepEmbedding,
    RNNEmbedding,
)
from pyrado.utils.argparser import get_argparser
from pyrado.utils.sbi import create_embedding
from pyrado.utils.data_types import EnvSpec
from pyrado.spaces.box import BoxSpace
from pyrado.spaces.discrete import DiscreteSpace
from pyrado.policies.feed_forward.playback import PlaybackPolicy


from copy import deepcopy

from pyrado.environments.pysim.pendulum import PendulumSim
from pyrado.policies.special.environment_specific import create_pend_excitation_policy


def is_vectorized(env):
		if env.reset().ndim == 2:
			return True
		else:
			return False

def main():

    assert args.env is not None
    assert not(args.resume ^ (args.resume_path is not None and args.resume_wandb is not None))
    if args.test_env is None:
        args.test_env = args.env
    if (args.unmodeled or args.reduced_sensing) and args.env != "trunkcube-pyrado-v0":
        raise ValueError(f"Unmodeled setting and Reduced sensing setting implemented only for trunkcube-pyrado-v0")
    
    torch.set_num_threads(args.now)   

    pprint(vars(args))

    # Set seed if desired
    set_seed(args.seed)

    resume_string = re.findall("_([^_]+)?$", args.resume_path)[0] if args.resume_path is not None else None #take the random string of the resume_path
    random_string = get_random_string(5) if not args.resume else resume_string 
    run_id = args.resume_wandb if (args.resume and args.wandb_mode == "online") else wandb.util.generate_id()

    pyrado.set_seed(args.seed, verbose=True)

    if args.run_path is not None:
        run_path = args.run_path+"/runs/"+str(args.env)+"/"+get_run_name(args)+"_"+random_string+"/"
    else:
        run_path = "runs/"+str(args.env)+"/"+get_run_name(args)+"_"+random_string+"/"
    
    if not args.no_output:
        create_dirs(run_path)

    # Experiment (set seed before creating the modules)
    ex_dir = setup_experiment(args.env, f"{NPDR.name}", "")

    
    print('\n ===== RUN NAME:', random_string, f' ({run_path}) ===== \n')


    infonly = 'InfOnly_' if args.inference_only else ''
    unmodeled = 'Unmodeled_' if args.unmodeled else ''

    wandb.init(config=vars(args),
             id = run_id,
             dir = run_path,
             project="SoRo-RL",
             group=('NPDR_'+unmodeled+args.env+'_train' if args.group is None else args.group),
             name= infonly+args.algo+'_seed'+str(args.seed)+'_'+random_string,
             save_code=True,
             tags=None,
             notes=args.notes,
             mode=(args.wandb_mode),
             resume="allow",
             )

    # Environments
    #env = make_vec_env(args.env, n_envs=args.now, seed=args.seed, vec_env_cls=RandomSubprocVecEnv, env_kwargs={'unmodeled': args.unmodeled})
    
    if args.env == "trunkcube-pyrado-v0":
        ext_dict = {'timer_limit': args.clipping } if args.clipping is not None else None
        # env = make_vec_env(args.env, n_envs=args.now, seed=args.seed, vec_env_cls=RandomSubprocVecEnv, env_kwargs={'unmodeled': args.unmodeled, 'reduced_sensing': args.reduced_sensing})
        env = gym.make(args.env, config_ext_dict=ext_dict, unmodeled = args.unmodeled, reduced_sensing = args.reduced_sensing)
        test_env = gym.make(args.test_env, config_ext_dict=ext_dict, reduced_sensing = args.reduced_sensing)
        env = env.unwrapped
        test_env = test_env.unwrapped
    elif args.env == "trunk-pyrado-v0":
        env = gym.make(args.env)
        test_env = gym.make(args.test_env)
        env = env.unwrapped
        test_env = test_env.unwrapped
    elif args.env == "hopper-pyrado":
        from pyrado.environments.mujoco.openai_hopper import HopperSim
        env_hparam = dict()
        env = HopperSim(**env_hparam)
        test_env = deepcopy(env)
    elif args.env == "pendulum-pyrado":
        env_hparam = dict(dt=1 / 50.0, max_steps=400)
        env = PendulumSim(**env_hparam)
        env.domain_param = dict(d_pole=0)
        env.domain_param = dict(tau_max=4.5)
        test_env = deepcopy(env)
    else:
        raise ValueError(f"NPDR implemented only for trunkcube-pyrado-v0 and trunk-pyrado-v0")


    if args.env == "trunkcube-pyrado-v0" or args.env == "trunk-pyrado-v0":
        target_task = env.get_task()
        print('Action space:', env.action_space)
        print('State space:', env.observation_space)
        print('Target dynamics:', target_task)
        
        obs_space = BoxSpace(env.observation_space.low, env.observation_space.high)
        act_space = DiscreteSpace(np.arange(env.action_space.n))
        state_space = BoxSpace(env.observation_space.low, env.observation_space.high)
        env_spec = EnvSpec(obs_space, act_space, state_space)

        # Define a mapping: index - domain parameter
        if is_vectorized(env):
            env_config = env.get_attr('config')[0]
        else:
            env_config = env.config
        
        dp_mapping = {index: value for index, value in enumerate(env_config["dynamic_params"])}

        # Prior
        prior_hparam = {
            'low': torch.tensor([env_config[param + "_min_search"] for param in env_config["dynamic_params"]]),
            'high': torch.tensor([env_config[param + "_max_search"] for param in env_config["dynamic_params"]])
        }
        prior = sbiutils.BoxUniform(**prior_hparam)
        max_steps = env_config["timer_limit"]
        
        print(f"Actual control frequency: {1/env.config['dt']/env.config['scale_factor']}Hz")
        print(f"Actual dim_trunk: {len(env.config['reduced_sensing_points'])*3}")

    elif args.env == "hopper-pyrado":
        target_task = env.get_nominal_domain_param()
        print('Action space:', env.act_space)
        print('Obs space:', env.obs_space)
        print('State space:', env.state_space)
        print('Target dynamics:', target_task)
        env_spec = EnvSpec(env.obs_space, env.act_space, env.state_space)
        
        # Define a mapping: index - domain parameter
        dp_mapping = {0: 'torsomass', 1: 'thighmass', 2: 'legmass', 3: 'footmass'}

        # Prior
        dp_nom = env.get_nominal_domain_param()

        prior_hparam = dict(
            low=torch.tensor([0.1, 0.1, 0.1, 0.1]),
            high=torch.tensor([10, 10, 10, 10]),
        )
        prior = sbiutils.BoxUniform(**prior_hparam)
        max_steps = env.max_steps
    elif args.env == "pendulum-pyrado":
        target_task = env.get_nominal_domain_param()
        print('Action space:', env.act_space)
        print('Obs space:', env.obs_space)
        print('State space:', env.state_space)
        print('Target dynamics:', target_task)

        # Define a mapping: index - domain parameter
        dp_mapping = {0: "pole_mass", 1: "pole_length"}

        # Prior
        dp_nom = env.get_nominal_domain_param()
        prior_hparam = dict(
            low=torch.tensor([dp_nom["pole_mass"] * 0.3, dp_nom["pole_length"] * 0.3]),
            high=torch.tensor([dp_nom["pole_mass"] * 1.7, dp_nom["pole_length"] * 1.7]),
        )
        prior = sbiutils.BoxUniform(**prior_hparam)
    if not args.training_only:

        if args.env == "pendulum-pyrado":
            # Time series embedding
            embedding_hparam = dict(downsampling_factor=1)
            embedding = create_embedding(BayesSimEmbedding.name, env.spec, **embedding_hparam)

            # Posterior (normalizing flow)
            posterior_hparam = dict(model="maf", hidden_features=20, num_transforms=2)

            num_real_rollouts = 1

            # Behavioral policy
            policy = create_pend_excitation_policy(env, 1)
        else:
            # Time series embedding
            embedding_hparam = dict(
                    downsampling_factor=2,
                    len_rollouts=max_steps,
                    # recurrent_network_type=nn.RNN,
                    # only_last_output=True,
                    # hidden_size=20,
                    # num_recurrent_layers=1,
                    # output_size=1,
                    # state_mask_labels=("Cube_X", "Cube_Y", "Cube_Z"), # select list or tuple of integers or stings to select specific states from their space.
                )
            embedding = create_embedding(AllStepsEmbedding.name, env_spec, **embedding_hparam) # or BayesSimEmbedding
            embedding_size = 100
            num_real_rollouts = 1 # number of real-world rollouts received by sbi, i.e. from every rollout exactly one data set is computed
            embedding_net = torch.nn.Linear(embedding.dim_output * num_real_rollouts, embedding_size)
        
            # Posterior (normalizing flow)
            posterior_hparam = dict(embedding_net=embedding_net, model="maf", hidden_features=100, num_transforms=len(target_task))

            # Behavioral policy

            unmodeled_str=""

            if unmodeled:
                for p in env.config["unmodeled_param"]:
                    unmodeled_str += f'{p}: {env.config[p+"_init"]}\n'
                wandb.config.unmodeled_param = unmodeled_str
                print("Unmodeled params:")
                print("\t"+unmodeled_str)
            if args.reduced_sensing:
                print("Reduced sensing setting")

            if not args.resume:
                wandb.config.path = run_path
                wandb.config.hostname = socket.gethostname()
                wandb.config.target_task = target_task

            """
                DATA COLLECTION
            """
            if args.data == 'random': # Collect data randomly
                # T = collect_offline_data(env=test_env, n=args.n_trajectories)
                test_env.evaluation = True
                T = collect_offline_data_clipping_single_env(env=test_env, n=args.n_trajectories, clipping=args.clipping, env_name=args.test_env, save_dataset=True)

            elif args.data == 'off_policy': # Collect data with external policy
                # policy = Policy(algo=args.algo, env=test_env, load_from_pathname=args.off_policy, device=args.device, seed=args.seed)
                policy = Policy(algo=args.algo, env=test_env, device=args.device, seed=args.seed)
                policy.load_state_dict(args.off_policy)
                # T = collect_offline_data(env=test_env, n=args.n_trajectories, policy=policy)
                T = collect_offline_data_clipping_single_env(env=test_env, n=args.n_trajectories, policy=policy, clipping=args.clipping, env_name=args.test_env)

            elif args.data == 'custom': # Custom offline dataset
                if os.path.isfile(args.data_path):
                    T = load_data_from_file(args.data_path)
                elif os.path.isdir(args.data_path):
                    T = load_data_from_dir(args.data_path)
                else:
                    raise ValueError(f"{args.data_path}: data path is not correct")
            else:
                raise ValueError(f"Unsupported args.data parameter: {args.data}")
            
            if args.clipping is not None:
                T = clip_trajectory(T, args.clipping)

            if T['actions'].ndim == 1:
                T['actions'] = T['actions'].reshape((len(T['actions']),1))
            wandb.config.n_transitions = T['observations'].shape[0]

            if is_vectorized(env):
                env.set_attr('_init_state', T['observations'][0])
            else:
                env._init_state = T['observations'][0]

            policy = PlaybackPolicy(
                        env_spec,
                        [T['actions']],
                        # no_reset=True,
                    )

            
        """
            INFERENCE
        """
        
        # Algorithm
        algo_hparam = dict(
            max_iter=args.max_iter,
            num_real_rollouts=num_real_rollouts,
            num_sim_per_round=args.num_sim_per_round,
            num_sbi_rounds=args.num_sbi_rounds,
            simulation_batch_size=args.simulation_batch_size,
            normalize_posterior=args.normalize_posterior,
            num_eval_samples=args.num_eval_samples,
            num_segments=1,
            stop_on_done=False,
            use_rec_act=True,
            posterior_hparam=posterior_hparam,
            subrtn_sbi_training_hparam=dict(
                num_atoms=10,  # default: 10
                training_batch_size=args.training_batch_size,  # default: 50
                learning_rate=3e-4,  # default: 5e-4
                validation_fraction=0.2,  # default: 0.1
                stop_after_epochs=20,  # default: 20
                discard_prior_samples=False,  # default: False
                use_combined_loss=False,  # default: False
                # retrain_from_scratch_each_round=False,  # default: False
                retrain_from_scratch=False,  # default: False --> `retrain_from_scratch_each_round` is now called `retrain_from_scratch` 
                                            # from https://github.com/mackelab/sbi/blob/6c4fa7a6fd254d48d0c18640c832f2d80ab2257a/CHANGELOG.md?plain=1#L96
                show_train_summary=False,  # default: False
                # max_num_epochs=5,  # only use for debugging
            ),
            # subrtn_sbi_sampling_hparam=dict(sample_with_mcmc=False),
            # New API for specifying sampling methods (#487). Old syntax:
            # ```python
            # posterior = inference.build_posterior(sample_with_mcmc=True)
            # ```

            # New syntax:

            # ```python
            # posterior = inference.build_posterior(sample_with="mcmc")  # or "rejection"
            # ```
            # from https://github.com/mackelab/sbi/blob/6c4fa7a6fd254d48d0c18640c832f2d80ab2257a/CHANGELOG.md?plain=1#L161
            subrtn_sbi_sampling_hparam=dict(sample_with="mcmc"),
            num_workers=args.now,
        )
        algo = NPDR(
            save_dir=ex_dir,
            env_sim=env,
            env_real=test_env,
            policy= policy,
            dp_mapping=dp_mapping,
            prior=prior,
            embedding=embedding,
            subrtn_sbi_class=SNPE_C,
            wandb=wandb,
            **algo_hparam,
        )

        # Save the hyper-parameters
        save_dicts_to_yaml(
            dict(env=env_config if args.env == "trunkcube-pyrado-v0" or args.env == "trunk-pyrado-v0" else env_hparam, seed=args.seed),
            dict(dp_mapping=dp_mapping),
            # dict(policy=policy_hparam, policy_name=policy.name),
            dict(prior=prior_hparam),
            dict(embedding=embedding_hparam, embedding_name=embedding.name),
            dict(posterior_nn=posterior_hparam),
            dict(algo=algo_hparam, algo_name=algo.name),
            save_dir=ex_dir,
        )

        shutil.copytree(ex_dir, run_path+'/pyrado_hyperparams')
        # Jeeeha
        start = time.time()
        wandb.config.update({"start_time":start}, allow_val_change=True)
        algo.train(seed=args.seed)
        elapsed = time.time()-start
        wandb.config.update({"elapsed_time":elapsed}, allow_val_change=True)

        shutil.copytree(ex_dir, run_path+'/pyrado_results')
        
        
        """
            OUTPUT RESULTS
        """

        iter_idx, round_idx = get_max_iter_and_round(ex_dir)
        posterior_npdr = SBIBase.load_posterior(ex_dir, idx_iter=iter_idx, idx_round=round_idx, obj=None, verbose=True)  # CHOICE
        index_to_name = env.dyn_ind_to_name
        map = posterior_npdr.map(show_progress_bars=True).tolist()
        wandb.run.summary["map"] = map
        output_npdr_results(args, target_task, ex_dir, elapsed, map, index_to_name)

        if not args.no_output:  # Output results to file
            # make_dir(args.output_dir)

            filename = 'NPDR_'+str(args.env)+'_n'+str(args.n_trajectories)+'_'+datetime.now().strftime("%Y%m%d_%H-%M-%S")+'.txt'
            with open(os.path.join(run_path, filename), 'a', encoding='utf-8') as file:
                output_npdr_results(args, target_task, ex_dir, elapsed, map, index_to_name, file=file)

    else:
        assert args.bounds_path is not None
        print("\nLoading posterior from ", args.bounds_path)
        iter_idx, round_idx = get_max_iter_and_round(args.bounds_path)
        posterior_npdr = SBIBase.load_posterior(args.bounds_path, idx_iter=iter_idx, idx_round=round_idx, obj=None, verbose=True)  # CHOICE
        if args.print_map:
            print("MAP values from posterior distribution:\n----------------\n")
            index_to_name = env.dyn_ind_to_name
            map = posterior_npdr.map().tolist()
            #print('\n'.join([str(index_to_name[i])+':\t'+str(round(map[i],5)) for i in range(len(map))]))
            print(map)
            wandb.run.summary["map"] = map

    if not args.inference_only:

        if args.env == "trunkcube-pyrado-v0":
            env = make_vec_env(args.env, n_envs=args.now, seed=args.seed, vec_env_cls=RandomSubprocVecEnv, env_kwargs={'unmodeled': args.unmodeled, 'reduced_sensing': args.reduced_sensing, 'noisy_setting': args.noisy_setting})
        elif args.env == "trunk-pyrado-v0":
            env = make_vec_env(args.env, n_envs=args.now, seed=args.seed, vec_env_cls=RandomSubprocVecEnv)
        else:
            raise ValueError(f"NPDR implemented only for trunkcube-pyrado-v0 and trunk-pyrado-v0")
        env.set_dr_distribution(dr_type='NPDR', distr=posterior_npdr)
        env.set_dr_training(True)
        size_layer=[]
        for _ in range(args.n_layers):
            size_layer.append(args.n_neurons)

        if args.resume:
            ckpt = os.listdir(os.path.join(args.resume_path, "logs"))[0]
            load_path  = os.path.join(args.resume_path, "logs", ckpt)
            assert os.path.exists(load_path), "model_ckpt_*_steps.zip hasn't been found"
            policy = Policy(algo=args.algo, 
                            env=env, 
                            lr=args.lr, 
                            batch_size=args.batch_size,
                            size_layer=size_layer,
                            device=args.device, 
                            seed=args.seed, 
                            load_from_pathname=load_path,
                            reset_num_timesteps=False)
            n_previous_steps = policy.model.num_timesteps
            policy.model.num_timesteps = n_previous_steps - args.eval_freq
            print(f"Checkpoint model loaded.\nResume training from step {n_previous_steps}.")
        else:

            policy = Policy(algo=args.algo,
                        env=env,
                        lr=args.lr,
                        batch_size=args.batch_size,
                        size_layer=size_layer,
                        device=args.device,
                        seed=args.seed)

        print('--- Policy training start ---')
        mean_reward, std_reward, best_policy, which_one = policy.train(timesteps=args.timesteps,
                                                                       stopAtRewardThreshold=args.reward_threshold,
                                                                       n_eval_episodes=args.eval_episodes,
                                                                       eval_freq=args.eval_freq,
                                                                       best_model_save_path=run_path,
                                                                       return_best_model=True)

        env.set_dr_training(False)

        policy.save_state_dict(run_path+"final_model.pth")
        policy.save_full_state(run_path+"final_full_state.zip")
        print('--- Policy training done ----')

        print('\n\nMean reward and stdev:', mean_reward, std_reward)

        wandb.run.summary["train_mean_reward"] = mean_reward
        wandb.run.summary["train_std_reward"] = std_reward
        wandb.run.summary["which_best_model"] = which_one

        torch.save(best_policy, run_path+"overall_best.pth")
        wandb.save(run_path+"overall_best.pth")


        """Evaluation on target domain"""
        print('\n\n--- TARGET DOMAIN EVALUATION ---')
        
        if args.env == "trunkcube-pyrado-v0":
            test_env = gym.make(args.test_env)
        else:
            test_env = gym.make(args.test_env)

        test_env.evaluation = True
        test_env.env.evaluation = True

        policy = Policy(algo=args.algo,
                    env=test_env,
                    device=args.device,
                    seed=args.seed,
                    lr=args.lr,
                    batch_size=args.batch_size,
                    size_layer=size_layer
                    )
        policy.load_state_dict(best_policy)

        mean_reward, std_reward = policy.eval(n_eval_episodes=args.test_episodes)
        print('Target reward and stdev:', mean_reward, std_reward)

        wandb.run.summary["target_mean_reward"] = mean_reward
        wandb.run.summary["target_std_reward"] = std_reward


    wandb.finish()

def output_npdr_results(args, gt, run_path, elapsed, map, index_to_name, file=None):
    print('\n-----------', file=file)
    print('RESULTS\n', file=file)
    print('ARGS:', vars(args), '\n\n', file=file)

    print('GROUND TRUTH dynamics parameters:', gt, '\n', file=file)

    print('Most likely domain parameters from posterior:\n---------------', file=file)
    ml_dp = pyrado.load(f"iter_{args.max_iter - 1}_ml_domain_param.pkl", run_path)
    for key, value in ml_dp.items():
        print(f"\t{key}: {value}", file=file)

    print("MAP values from posterior distribution:\n----------------\n")
    #print('\n'.join([str(index_to_name[i])+':\t'+str(round(map[i],5)) for i in range(len(map))]))
    print(map)
    print('Elapsed:', round(elapsed/60, 4), 'min', file=file)

def get_max_iter_and_round(ex_dir):
    max_iter = -1
    max_round = -1
    
    # Define the regex pattern to match the iteration and round in the filenames
    pattern = re.compile(r'iter_(\d+)_round_(\d+)_posterior\.pt')

    for filename in os.listdir(ex_dir):
        match = pattern.match(filename)
        if match:
            iter_num = int(match.group(1))
            round_num = int(match.group(2))

            if iter_num > max_iter:
                max_iter = iter_num
            if round_num > max_round:
                max_round = round_num

    return max_iter, max_round

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--env', default='trunkcube-pyrado-v0', type=str, help='Train gym env')
    parser.add_argument('--test_env', default=None, type=str, help='Test gym env')
    parser.add_argument('--group', default=None, type=str, help='Wandb run group')
    parser.add_argument('--algo', default='ppo', type=str, help='RL Algo (ppo, sac)')
    parser.add_argument('--lr', default=1e-4, type=float, help='Learning rate')
    parser.add_argument('--batch_size', default=144, type=int, help='Batch size')
    parser.add_argument('--n_layers', default=3, type=int, help='Network number of layers')
    parser.add_argument('--n_neurons', default=512, type=int, help='Network neurons in each layer')
    parser.add_argument('--now', default=1, type=int, help='Number of cpus for parallelization (Default: 1 => no parallelization)')
    parser.add_argument('--timesteps', '-t', default=2000000, type=int, help='Training timesteps')
    parser.add_argument('--reward_threshold', default=False, action='store_true', help='Stop at reward threshold')
    parser.add_argument('--eval_freq', default=10000, type=int, help='timesteps frequency for training evaluations')
    parser.add_argument('--eval_episodes', default=50, type=int, help='# episodes for training evaluations')
    parser.add_argument('--test_episodes', default=100, type=int, help='# episodes for test evaluations')
    parser.add_argument('--test_render', default=False, action='store_true', help='Render test episodes')
    parser.add_argument('--seed', default=0, type=int, help='Random seed')
    parser.add_argument('--device', default='cpu', type=str, help='<cpu,cuda>')
    parser.add_argument('--verbose', default=0, type=int, help='0,1,2')
    parser.add_argument('--notes', default=None, type=str, help='Wandb notes')
    parser.add_argument('--wandb_mode', default='online', type=str, help='Wandb mode: online (default), offline, disabled')
    parser.add_argument('--resume', default=False, action='store_true', help='Resume from previous training ckpt')
    parser.add_argument('--resume_path', default=None, type=str, help='Path for the ckpt training')
    parser.add_argument('--resume_wandb', default=None, type=str, help='Run ID of wandb previous run (e.g., \'wandb/run-date_time-ID\')')
    parser.add_argument('--run_path', default=None, type=str, help='Path for saving run results')

    parser.add_argument("--n_trajectories", "-n", type=int, default=None, help="Number of target trajectories in the target dataset to consider")
    parser.add_argument('--data', default='custom', type=str, help='Offline data collection method [random, off_policy, custom]')
    parser.add_argument('--data_path', default=None, type=str, help='Path to custom offline dataset')
    parser.add_argument('--off_policy', default=None, type=str, help='Path to model for data collection off-policy')
    parser.add_argument('--inference_only', default=False, action='store_true', help='Avoid policy training')
    parser.add_argument('--clipping', default=None, type=int, help='Clipping the real-world rollout at <clipping> state-transitions')
    parser.add_argument('--initial_info_only', default=True, action='store_true', help='Reset state only once while computing obj. function. - Always set to TRUE')
    parser.add_argument('--training_only', default=False, action='store_true', help='Avoid dynamics inference')
    parser.add_argument('--bounds_path', default=None, type=str, help='Path for best bounds to be loaded')

    parser.add_argument('--unmodeled', default=None, type=str, help='Unmodeled setting (implemented only for trunkcube env): <PARAMETER>-<VALUE>, e.g. YM-3600 or PR-0.449')
    parser.add_argument('--reduced_sensing', default=False, action='store_true', help='Reduced sensing setting (implemented only for trunkcube env)')
    parser.add_argument('--noisy_setting', default=False, action='store_true', help='Obs and actions are noised')

    parser.add_argument('--no_output', default=False, action='store_true', help='DO NOT save output results of the optimization')
    
    # NPDR
    parser.add_argument("--max_iter", type=int, default=2, help="maximum number of iterations (i.e. policy updates) that this algorithm runs")
    parser.add_argument("--num_sbi_rounds", type=int, default=7, help="set to an integer > 1 to use multi-round sbi. This way the posteriors (saved as `..._round_NUMBER...` will be tailored to the data of that round, where `NUMBER` counts up each round (modulo `num_real_rollouts`). If `num_sbi_rounds` = 1, the posterior is called amortized (it has never seen any target domain data).")
    parser.add_argument("--num_sim_per_round", type=int, default=4000, help="number of simulations done by sbi per round (i.e. iteration over the same target domain data set)")
    parser.add_argument("--simulation_batch_size", type=int, default=10, help="batch size forwarded to the sbi toolbox, requires batched simulator")
    parser.add_argument('--normalize_posterior', default=False, action='store_true', help="if `True` the normalization of the posterior density is enforced by sbi")
    parser.add_argument("--num_eval_samples", type=int, default=10, help="number of samples for evaluating the posterior in `eval_posterior()`")
    parser.add_argument("--training_batch_size", type=int, default=50, help="training arg of inference sbi methods")
    parser.add_argument('--print_map', default=False, action='store_true', help='Showing MAP from the loaded posterior')


    return parser.parse_args()

args = parse_args()

if __name__ == '__main__':
    main()


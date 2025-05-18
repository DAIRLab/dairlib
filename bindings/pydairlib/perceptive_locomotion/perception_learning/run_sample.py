"""
Train a policy for //bindings/pydairlib/perceptive_locomotion/perception_learning:DrakeCassieEnv
"""
import argparse
import os
from os import path

import gymnasium as gym

from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.env_checker import check_env
from pydairlib.perceptive_locomotion.perception_learning.sb3_contrib.ppo_recurrent.ppo_recurrent import RecurrentPPO

from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.PPO.ppo import PPO
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.callbacks import EvalCallback
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.env_util import make_vec_env
from pydairlib.perceptive_locomotion.perception_learning.stable_baselines3.common.vec_env import (
    DummyVecEnv,
    SubprocVecEnv,
    VecVideoRecorder,
)
import torch as th
import numpy as np

from pydrake.geometry import Meshcat
from pydrake.systems.all import (
    Diagram,
    Context,
    Simulator,
    InputPort,
    OutputPort,
    DiagramBuilder,
    InputPortIndex,
    OutputPortIndex,
    ConstantVectorSource
)

from pydairlib.perceptive_locomotion.systems.cassie_footstep_controller_gym_environment import (
    CassieFootstepControllerEnvironmentOptions,
    CassieFootstepControllerEnvironment,
)
import warnings
warnings.simplefilter(action='ignore', category=UserWarning)
import matplotlib.pyplot as plt

perception_learning_base_folder = "bindings/pydairlib/perceptive_locomotion/perception_learning"

def bazel_chdir():
    """When using `bazel run`, the current working directory ("cwd") of the
    program is set to a deeply-nested runfiles directory, not the actual cwd.
    In case relative paths are given on the command line, we need to restore
    the original cwd so that those paths resolve correctly.
    """
    if 'BUILD_WORKSPACE_DIRECTORY' in os.environ:
        os.chdir(os.environ['BUILD_WORKSPACE_DIRECTORY'])

def sample(sim_params):
    terrain = 'params/stair_curriculum.yaml'
    sim_params.terrain = os.path.join(perception_learning_base_folder, terrain)
    env = gym.make("DrakeCassie-v0",
        sim_params = sim_params,
    )
    rate = 1.0
    env.simulator.set_target_realtime_rate(rate)
    max_steps = 1000
    obs, _ = env.reset()
    input("Start..")
    for i in range(int(max_steps)):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        
        if terminated or truncated:
            input("The environment will reset. Press Enter to continue...")
            obs, _ = env.reset()

def run_play(sim_params, model_path=None):
    np.random.seed(222)
    th.manual_seed(222)
    # sim_params.visualize = True
    # sim_params.meshcat = Meshcat()

    env = gym.make("DrakeCassie-v0",
                    sim_params = sim_params,
                    )
    rate = 30.0
    env.simulator.set_target_realtime_rate(rate)
    max_steps = 999999
    
    lstm=True
    lstm_states = None
    episode_starts = np.ones((1,), dtype=bool)

    # model_path = 'jcon_for_friction_test.zip'
    # model_path = 'logs2/rl_model_18144000_steps.zip' #'joint1.zip'
    model_path = 'logs/rl_model_52920000_steps.zip' #'joint1.zip'
    # model_path = 'LSTM_94.zip'
    # model_path = 'ALIP_fc.zip'

    model = RecurrentPPO.load(model_path, env, verbose=0)
    # model.save('pbody')
    # th.save(model.policy.state_dict(), 'jpen')
    obs, _ = env.reset()

    # print("Parameter sizes:")
    # total_params = 0
    # for name, param in model.policy.named_parameters():
    #     num_params = param.numel()
    #     total_params += num_params
    #     print(f"{name}: {num_params} parameters")

    # print(f"Total number of parameters: {total_params}")
    input("Start..")
    total_reward = 0
    model.policy.eval()

    observ = []
    act = []
    data = []
    idx = 0
    save_counter = 1
    DES = []
    TRUE = []
    term_count = 0
    # print('stair down 0.4')
    # print('stair down 0.8')
    print('stair down 0.4')

    for i in range(1, max_steps):
        idx += 1
        if lstm:
            action, lstm_states = model.predict(obs, state=lstm_states, episode_start=episode_starts, deterministic=True)
        else:
            action, states = model.predict(obs, deterministic=True)
        # print(action)
        # print(obs[3*64*64:3*64*64+2])
        # print(obs[3*64*64+2:3*64*64+2+16])
        # print(obs[3*64*64+2+16:3*64*64+2+16+16])
        obs, reward, terminated, truncated, info = env.step(action)
        xvdes = obs[3*64*64+4:3*64*64+5]
        bf_vel = obs[3*64*64+6+16:3*64*64+6+16+2]
        
        DES.append(xvdes)
        TRUE.append(bf_vel[0])

        # obs_tensor = th.tensor(obs[:3*64*64])
        # vel = obs[3*64*64+4:3*64*64+6]
        # joint_pos = obs[3*64*64+6+16:3*64*64+6+16+20]
        # joint_vel = obs[3*64*64+6+16+20:3*64*64+6+16+20+22]
        # observation = np.concatenate((obs_tensor,vel,joint_pos,joint_vel))
        # observ.append(observation)
        # act.append(action)
        
        # obs_tensor = th.tensor(obs[:3*64*64], dtype=th.float32, device='cuda').view(1, 3, 64, 64)
        # latent = model.policy.mlp_extractor.actor_cnn(obs_tensor).cpu().detach().numpy().squeeze()
        # obs1 = obs[3*64*64:3*64*64+6+16]
        # observation = np.concatenate((latent, obs1))
        # data.append(observation)

        if lstm:
            episode_starts = terminated
        total_reward += reward

        # if idx == 400 and (terminated or truncated):
        #     data_array = np.array(observ)
        #     print(data_array.shape)
        #     np.save(f'./imitation/slope1/obs_{save_counter*400}', data_array)

        #     data_array = np.array(act)
        #     print(data_array.shape)
        #     np.save(f'./imitation/slope1/act_{save_counter*400}', data_array)
        #     observ = []
        #     act = []
        #     idx = 0  # reset step count
        #     save_counter += 1  # unique file name index

        if terminated or truncated:
            if lstm:
                lstm_states = None
                episode_starts = np.ones((1,), dtype=bool)
            obs, _ = env.reset()
            total_reward = 0
            observ = []
            act = []
            idx = 0
            term_count += 1

        if term_count == 100:
            DES = np.asarray(DES)
            TRUE = np.asarray(TRUE)
            MSE = []
            mse = np.mean((DES - TRUE)**2)
            
            print(mse)
            # MSE.append(mse)
            
            # MSE = np.asarray(MSE)
            # print(np.mean(MSE))
            break
    # data_array = np.array(data)
    # print(data_array.shape)
    # # filename = f"hidden_states/saliency/stair.npy"
    # np.save("stair.npy", data_array)
    
    # print(f"Saved data")

# def run_saliency(sim_params, model_path=None):
#     np.random.seed(111)
#     th.manual_seed(111)
#     # sim_params.visualize = True
#     # sim_params.meshcat = Meshcat()
#     env = gym.make("DrakeCassie-v0",
#                     sim_params = sim_params,
#                     )
#     rate = 30.0
#     env.simulator.set_target_realtime_rate(rate)

#     model_path = 'jcon_for_friction_test.zip'
#     model = RecurrentPPO.load(model_path, env, verbose=0)
#     input("Start..")
#     lstm_states = None
#     #obs_seq = np.load("./hidden_states/flat.npy")[95:110]
#     obs_seq = np.load("./hidden_states/slope.npy")[:110]

#     obs_seq = th.tensor(obs_seq, dtype=th.float32, requires_grad=True, device='cuda')  # (T, num_features)
#     lstm_output, lstm_states = model.policy.mlp_extractor.actor_combined_lstm(obs_seq, lstm_states)

#     action_output = model.policy.action_net(lstm_output.squeeze(0))
#     loss = action_output.norm()
#     loss.backward()
    
#     saliency = obs_seq.grad[95:110].abs().squeeze().cpu().detach().numpy()
#     # Get saliency (absolute gradients) of input
#     # saliency = obs_seq.grad.abs().squeeze().cpu().detach().numpy()
#     latent_saliency = saliency[:, :64].sum(axis=1)
#     other_saliency = saliency[:, 64:].mean(axis=0)
#     combined_saliency = np.concatenate(([latent_saliency.mean(axis=0)], other_saliency))
#     combined_saliency_normalized = combined_saliency / combined_saliency.sum() # Normalize
#     print("Sum of normalized combined saliency:", combined_saliency_normalized.sum())

#     import matplotlib.pyplot as plt
#     joint_names = ['hip_roll_left', 'hip_yaw_left', 'hip_pitch_left', 'knee_left', 'knee_joint_left',
#                    'ankle_joint_left', 'ankle_spring_joint_left', 'toe_left', 'hip_roll_right', 'hip_yaw_right',
#                    'hip_pitch_right', 'knee_right', 'knee_joint_right', 'ankle_joint_right',
#                    'ankle_spring_joint_right', 'toe_right']
#     #[f"Joint {i+1}" for i in range(16)]
#     feature_names = ["Latent", "vx", "vy", "Alip_x","Alip_y","Alip_Lx","Alip_Ly"]
#     feature_names = feature_names + joint_names

#     plt.figure(figsize=(10, 4), dpi=300)
#     plt.ylim(0.0, 0.3)  # Clip y values between 0.0 and 0.3
#     plt.yticks([i * 0.025 for i in range(13)], fontsize=12)  # Y-ticks from 0 to 0.3
#     plt.bar(range(len(combined_saliency_normalized)), combined_saliency_normalized, color='skyblue')
#     plt.xticks(range(len(feature_names)), feature_names, rotation=45, ha="right", fontsize=12)
#     plt.xlabel("Feature", fontsize=14)
#     plt.ylabel("Saliency Magnitude", fontsize=14)
#     plt.title("Feature Importance via Input Gradients\n(LSTM + Action Network, Slope)", fontsize=16, fontweight='bold')
#     plt.grid(True, axis='y', linestyle='--', linewidth=0.5, alpha=0.7)

#     plt.tight_layout()
#     # plt.savefig("saliency_bar_plot.png", dpi=300, bbox_inches="tight")
#     plt.show()

def run_saliency(sim_params, model_path=None):
    import matplotlib.pyplot as plt
    import numpy as np
    import torch as th

    def compute_saliency(model, obs_seq):
        obs_seq = th.tensor(obs_seq, dtype=th.float32, requires_grad=True, device='cuda')
        lstm_output, lstm_states = model.policy.mlp_extractor.actor_combined_lstm(obs_seq, None)
        action_output = model.policy.action_net(lstm_output.squeeze(0))
        loss = action_output.norm()
        loss.backward()

        saliency = obs_seq.grad[80:110].abs().squeeze().cpu().detach().numpy()
        latent_saliency = saliency[:, :64].sum(axis=1)
        other_saliency = saliency[:, 64:].mean(axis=0)
        combined = np.concatenate(([latent_saliency.mean(axis=0)], other_saliency))
        return combined / combined.sum()  # Normalize

    np.random.seed(111)
    th.manual_seed(111)

    env = gym.make("DrakeCassie-v0", sim_params=sim_params)
    env.simulator.set_target_realtime_rate(30.0)
    model = RecurrentPPO.load('jcon_for_friction_test.zip', env, verbose=0)

    obs_flat = np.load("./hidden_states/flat.npy")[:150]
    obs_slope = np.load("./hidden_states/slope.npy")[:150]
    obs_stair = np.load("./hidden_states/stair.npy")[:150]
    # obs_stair = np.load("./stair.npy")[:150]


    saliency_flat = compute_saliency(model, obs_flat)
    saliency_slope = compute_saliency(model, obs_slope)
    saliency_stair = compute_saliency(model, obs_stair)

    # Names
    joint_names = ['hip_roll_left', 'hip_yaw_left', 'hip_pitch_left', 'knee_left', 'knee_joint_left',
                   'ankle_joint_left', 'ankle_spring_joint_left', 'toe_left', 'hip_roll_right', 'hip_yaw_right',
                   'hip_pitch_right', 'knee_right', 'knee_joint_right', 'ankle_joint_right',
                   'ankle_spring_joint_right', 'toe_right']
    feature_names = ["Latent", "vx", "vy", "Alip_x", "Alip_y", "Alip_Lx", "Alip_Ly"] + joint_names
    
    # Grouped feature names
    feature_names_grouped = ["ALIP", "Joint Position"]

    # Indices for grouping
    vx_vy_indices = [feature_names.index("vx"), feature_names.index("vy")]
    alip_indices = [feature_names.index(name) for name in ["Alip_x", "Alip_y", "Alip_Lx", "Alip_Ly"]]
    joint_indices = [feature_names.index(name) for name in joint_names]

    def group_saliency(saliency):
        latent = saliency[0]
        velocity = saliency[vx_vy_indices].sum()
        alip = saliency[alip_indices].sum()
        joints = saliency[joint_indices].sum()
        return np.array([alip, joints])

    # Group all saliency values
    saliency_flat_grouped = group_saliency(saliency_flat)
    saliency_slope_grouped = group_saliency(saliency_slope)
    saliency_stair_grouped = group_saliency(saliency_stair)
    
    # Plotting
    x = np.arange(len(feature_names_grouped))
    width = 0.25

    plt.figure(figsize=(5, 5), dpi=300)
    plt.bar(x - width, saliency_flat_grouped, width=width, label='Flat', color='skyblue')
    plt.bar(x, saliency_slope_grouped, width=width, label='Slope', color='orange')
    plt.bar(x + width, saliency_stair_grouped, width=width, label='Stair', color='green')

    plt.ylim(0.0, 0.5)
    plt.yticks([i * 0.05 for i in range(11)], fontsize=12)
    plt.xticks(x, feature_names_grouped, rotation=0, fontsize=14, fontweight='bold')
    # plt.xlabel("Feature Group", fontsize=14, fontweight='bold')
    plt.ylabel("Attribution Magnitude", fontsize=14, fontweight='bold')
    plt.title("Saliency Map", fontsize=16, fontweight='bold')
    plt.grid(True, axis='y', linestyle='--', linewidth=0.5, alpha=0.7)
    plt.legend(fontsize=14)
    plt.tight_layout()
    # plt.savefig("grouped_saliency.png", dpi=300, bbox_inches="tight")
    plt.show()

    # Plot
    # x = np.arange(len(feature_names))
    # width = 0.25

    # plt.figure(figsize=(12, 5), dpi=300)
    # plt.bar(x - width, saliency_flat, width=width, label='Flat', color='skyblue')
    # plt.bar(x, saliency_slope, width=width, label='Slope', color='orange')
    # plt.bar(x + width, saliency_stair, width=width, label='Stair', color='green')

    # plt.ylim(0.0, 0.3)
    # plt.yticks([i * 0.025 for i in range(13)], fontsize=10)
    # plt.xticks(x, feature_names, rotation=45, ha="right", fontsize=10)
    # plt.xlabel("Feature", fontsize=12, fontweight='bold')
    # plt.ylabel("Saliency Magnitude", fontsize=12, fontweight='bold')
    # plt.title("Feature Importance via Saliency Map", fontsize=16, fontweight='bold')
    # plt.grid(True, axis='y', linestyle='--', linewidth=0.5, alpha=0.7)
    # plt.legend(fontsize=12)
    # plt.tight_layout()
    # # plt.savefig("combined_saliency.png", dpi=300, bbox_inches="tight")
    # plt.show()

def run_shap(sim_params, model_path=None):
    import shap
    np.random.seed(111)
    th.manual_seed(111)
    # sim_params.visualize = True
    # sim_params.meshcat = Meshcat()
    env = gym.make("DrakeCassie-v0",
                    sim_params = sim_params,
                    )
    rate = 30.0
    env.simulator.set_target_realtime_rate(rate)
    
    model_path = 'jcon_for_friction_test.zip'
    model = RecurrentPPO.load(model_path, env, verbose=0)
    input("Start..")

    def policy_wrapper(inputs):
        """Wrapper function to apply policy to batch inputs for SHAP."""
        inputs_tensor = th.tensor(inputs, dtype=th.float32).to('cuda')
        lstm_output, _ = model.policy.mlp_extractor.actor_combined_lstm(inputs_tensor)
        action_output = model.policy.action_net(lstm_output)
        return action_output.cpu().detach().numpy()
   
    obs_seq = np.load("./hidden_states/flat.npy")[:110]
    background_data = th.tensor(obs_seq, dtype=th.float32)  # Convert to tensor
    explain_data = th.tensor(obs_seq[95:96], dtype=th.float32)  # Convert to tensor
    explainer = shap.Explainer(policy_wrapper, background_data.numpy())  # SHAP expects NumPy
    shap_values = explainer.shap_values(explain_data.numpy())
    shap.summary_plot(shap_values, explain_data)

# def run_integrated_gradients(sim_params, model_path=None):
#     np.random.seed(111)
#     th.manual_seed(111)
    
#     env = gym.make("DrakeCassie-v0", sim_params=sim_params)
#     rate = 30.0
#     env.simulator.set_target_realtime_rate(rate)

#     model_path = 'jcon_for_friction_test.zip'
#     model = RecurrentPPO.load(model_path, env, verbose=0)
#     lstm_states = None
#     obs_seq = np.load("./hidden_states/flat.npy")[:300]
#     obs_tensor = th.tensor(obs_seq, dtype=th.float32, requires_grad=True, device='cuda').unsqueeze(0)
    
#     def integrated_gradients(model, input_tensor, baseline=None, steps=100):
#         """
#         Computes Integrated Gradients for an LSTM model.

#         Args:
#             model: The LSTM model.
#             input_tensor: The input sequence (batch_size, seq_len, input_dim).
#             baseline: The baseline input (default: zero tensor).
#             steps: Number of interpolation steps.

#         Returns:
#             Attribution scores for the input.
#         """
#         if baseline is None:
#             baseline = th.zeros_like(input_tensor)
    
#         alphas = th.linspace(0, 1, steps+1).to(input_tensor.device)
#         path_inputs = [baseline + alpha * (input_tensor - baseline) for alpha in alphas]
#         total_gradients = th.zeros_like(input_tensor)
#         lstm_states = None
#         for i, scaled_input in enumerate(path_inputs[1:]):
#             scaled_input = scaled_input.clone().detach().requires_grad_(True)
#             output, lstm_states = model.policy.mlp_extractor.actor_combined_lstm(scaled_input, lstm_states)
#             output = model.policy.action_net(output)
#             loss = output.sum()
#             model.policy.zero_grad()
#             loss.backward(retain_graph=(i < steps-1))
#             total_gradients += scaled_input.grad
#             scaled_input.grad = None
        
#         average_gradients = total_gradients / steps
#         attributions = (input_tensor - baseline) * average_gradients

#         return attributions.detach()

#     attributions = integrated_gradients(model, obs_tensor)
#     attributions = attributions.abs().squeeze().cpu().detach().numpy()

#     # Extract relevant feature importance
#     latent_attributions = attributions[:, :64].sum(axis=1)  # Sum across time dimension
#     other_attributions = attributions[:, 64:].mean(axis=0)  # Average over time for other features
#     combined_attributions = np.concatenate(([latent_attributions.mean(axis=0)], other_attributions))

#     # Normalize saliency values
#     combined_attributions_normalized = combined_attributions / combined_attributions.sum()
#     print("Sum of normalized combined attributions:", combined_attributions_normalized.sum())

#     # Define feature names
#     joint_names = [f"Joint {i+1}" for i in range(16)]
#     feature_names = ["Latent", "vx", "vy", "Alip1", "Alip2", "Alip3", "Alip4"] + joint_names

#     # Plot the results
#     plt.figure(figsize=(10, 4))
#     plt.ylim(0.0, 0.35)  # Clip y values between 0.0 and 0.3
#     plt.yticks([i * 0.025 for i in range(15)])  # Set y-tick intervals to 0.025
#     plt.bar(range(len(combined_attributions_normalized)), combined_attributions_normalized)
#     plt.xticks(range(len(feature_names)), feature_names, rotation=45, ha="right")
#     plt.xlabel("Feature Index")
#     plt.ylabel("Attributions Magnitude")
#     plt.title("Integrated Gradients: Flat")
#     plt.grid(True, axis='y', linestyle='--', alpha=0.7)
#     plt.show()

def run_integrated_gradients(sim_params, model_path=None):
    np.random.seed(111)
    th.manual_seed(111)
    
#     env = gym.make("DrakeCassie-v0", sim_params=sim_params)
#     rate = 30.0
#     env.simulator.set_target_realtime_rate(rate)

#     model_path = 'jcon_for_friction_test.zip'
#     model = RecurrentPPO.load(model_path, env, verbose=0)
#     lstm_states = None
#     obs_seq = np.load("./hidden_states/flat.npy")[:300]
#     obs_tensor = th.tensor(obs_seq, dtype=th.float32, requires_grad=True, device='cuda').unsqueeze(0)
    env = gym.make("DrakeCassie-v0", sim_params=sim_params)
    env.simulator.set_target_realtime_rate(30.0)
    model = RecurrentPPO.load('jcon_for_friction_test.zip', env, verbose=0)

    def integrated_gradients(model, input_tensor, baseline=None, steps=10):
        if baseline is None:
            baseline = th.zeros_like(input_tensor)

        alphas = th.linspace(0, 1, steps + 1).to(input_tensor.device)
        total_gradients = th.zeros_like(input_tensor)
        lstm_states = None

        for i, alpha in enumerate(alphas[1:]):
            scaled_input = (baseline + alpha * (input_tensor - baseline)).detach().requires_grad_(True)
            output, lstm_states = model.policy.mlp_extractor.actor_combined_lstm(scaled_input, lstm_states)
            output = model.policy.action_net(output)
            loss = output.sum()
            model.policy.zero_grad()
            loss.backward(retain_graph=(i < steps - 1))
            total_gradients += scaled_input.grad
            scaled_input.grad = None

        average_gradients = total_gradients / steps
        return (input_tensor - baseline) * average_gradients

    def compute_integrated_saliency(obs_seq):
        obs_tensor = th.tensor(obs_seq, dtype=th.float32, device='cuda').unsqueeze(0).requires_grad_(True)
        attributions = integrated_gradients(model, obs_tensor)
        attributions = attributions.abs().squeeze().cpu().detach().numpy()
        latent = attributions[:, :64].sum(axis=1)
        other = attributions[:, 64:].mean(axis=0)
        combined = np.concatenate(([latent.mean()], other))
        return combined / combined.sum()

    # Load observation sequences
    obs_flat = np.load("./hidden_states/flat.npy")[:200]
    obs_slope = np.load("./hidden_states/slope.npy")[:200]
    #obs_stair = np.load("./hidden_states/stair.npy")[:150]
    obs_stair = np.load("./stair.npy")[:200]

    saliency_flat = compute_integrated_saliency(obs_flat)
    saliency_slope = compute_integrated_saliency(obs_slope)
    saliency_stair = compute_integrated_saliency(obs_stair)

    # Feature grouping
    joint_names = ['hip_roll_left', 'hip_yaw_left', 'hip_pitch_left', 'knee_left', 'knee_joint_left',
                   'ankle_joint_left', 'ankle_spring_joint_left', 'toe_left', 'hip_roll_right', 'hip_yaw_right',
                   'hip_pitch_right', 'knee_right', 'knee_joint_right', 'ankle_joint_right',
                   'ankle_spring_joint_right', 'toe_right']
    feature_names = ["Latent", "vx", "vy", "Alip_x", "Alip_y", "Alip_Lx", "Alip_Ly"] + joint_names
    feature_names_grouped = ["ALIP", "Joint Position"]

    vx_vy_indices = [feature_names.index("vx"), feature_names.index("vy")]
    alip_indices = [feature_names.index(n) for n in ["Alip_x", "Alip_y", "Alip_Lx", "Alip_Ly"]]
    joint_indices = [feature_names.index(n) for n in joint_names]

    def group_saliency(saliency):
        latent = saliency[0]
        velocity = saliency[vx_vy_indices].sum()
        alip = saliency[alip_indices].sum()
        joints = saliency[joint_indices].sum()
        return np.array([alip, joints])

    # Group saliency values
    saliency_flat_grouped = group_saliency(saliency_flat)
    saliency_slope_grouped = group_saliency(saliency_slope)
    saliency_stair_grouped = group_saliency(saliency_stair)

    # Plotting
    x = np.arange(len(feature_names_grouped))
    width = 0.25

    plt.figure(figsize=(5, 5), dpi=300)
    plt.bar(x - width, saliency_flat_grouped, width=width, label='Flat', color='skyblue')
    plt.bar(x, saliency_slope_grouped, width=width, label='Slope', color='orange')
    plt.bar(x + width, saliency_stair_grouped, width=width, label='Stair', color='green')

    plt.ylim(0.0, 0.5)
    plt.yticks([i * 0.05 for i in range(11)], fontsize=12)
    plt.xticks(x, feature_names_grouped, rotation=0, fontsize=14, fontweight='bold')
    # plt.xlabel("Feature Group", fontsize=12, fontweight='bold')
    plt.ylabel("Attribution Magnitude", fontsize=14, fontweight='bold')
    plt.title("Integrated Gradients", fontsize=16, fontweight='bold')
    plt.grid(True, axis='y', linestyle='--', linewidth=0.5, alpha=0.7)
    plt.legend(fontsize=14)
    plt.tight_layout()
    # plt.savefig("integrated_gradients_grouped.png", dpi=300, bbox_inches="tight")
    plt.show()

def run_friction(sim_params, idx=0, model_path=None):
    np.random.seed(idx+900)
    th.manual_seed(idx+900)
    # sim_params.visualize = True
    # sim_params.meshcat = Meshcat()
    import warnings
    warnings.simplefilter(action='ignore', category=UserWarning)

    env = gym.make("DrakeCassie-v0",
                    sim_params = sim_params,
                    )
    rate = 30.0
    env.simulator.set_target_realtime_rate(rate)
    max_steps = 400
    
    lstm=True
    lstm_states = None
    episode_starts = np.ones((1,), dtype=bool)

    model_path = 'jcon_for_friction_test.zip' # domain2
    
    model = RecurrentPPO.load(model_path, env, verbose=0)
    obs, _ = env.reset()
    hidden_state = []
    total_reward = 0
    model.policy.eval()

    term = False
    for i in range(int(max_steps)):
        if lstm:
            action, lstm_states = model.predict(obs, state=lstm_states, episode_start=episode_starts, deterministic=True)
        else:
            action, states = model.predict(obs, deterministic=True)
        # print(action)
        ## lstm_states = (2[2,1,128])
        hidden_state.append(lstm_states[0][-1].copy())
        obs, reward, terminated, truncated, info = env.step(action)

        if lstm:
            episode_starts = terminated
        total_reward += reward

        if terminated or truncated:
            term = True
            print("Break")
            break

    if not term:
        hidden_states_array = np.array(hidden_state).squeeze(axis=1)
        filename = f"hidden_states/8/hidden_states_{idx}.npy"
        np.save(filename, hidden_states_array)
        
        print(f"Saved hidden states to {filename}")

def run_interact(sim_params, model_path=None):
    np.random.seed(2)
    th.manual_seed(2)
    sim_params.visualize = True
    sim_params.meshcat = Meshcat()
    env = gym.make("DrakeCassie-v0",
                    sim_params = sim_params,
                    )
    lstm=True
    lstm_states = None
    episode_starts = np.ones((1,), dtype=bool)
    model_path = 'joint.zip'
    model = RecurrentPPO.load(model_path, env, verbose=1)
    obs, _ = env.reset()
    input("Start..")
    total_reward = 0
    model.policy.eval()

    while True:
        if lstm:
            action, lstm_states = model.predict(obs, state=lstm_states, episode_start=episode_starts, deterministic=True)
        else:
            action, states = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(action)
        # print(action)
        # print(obs[3*64*64+4:3*64*64+6])
        if lstm:
            episode_starts = terminated
        total_reward += reward
        if terminated or truncated:
            print(total_reward)
            if lstm:
                lstm_states = None
                episode_starts = np.ones((1,), dtype=bool)
            obs, _ = env.reset()
            total_reward = 0


def run_eval(sim_params, num_env=3, model_path=None):
    # sim_params.visualize = True
    # sim_params.meshcat = Meshcat()
    
    env = make_vec_env(
                    "DrakeCassie-v0",
                    n_envs=num_env,
                    seed=42,
                    vec_env_cls=SubprocVecEnv,
                    env_kwargs={
                    'sim_params': sim_params,
                    },
                    )
    # env = gym.make("DrakeCassie-v0",
    #                 sim_params = sim_params,
    #                 )
    # rate = 1.0
    # env.simulator.set_target_realtime_rate(rate)
    max_steps = 3e4
    
    lstm=True
    lstm_states = None
    episode_starts = np.ones((num_env,), dtype=bool)

    model_path = 'logs/rl_model_5720000_steps.zip'
    
    model = RecurrentPPO.load(model_path, env, verbose=1)
    #model.save('pbody')
    #th.save(model.policy.state_dict(), 'atlas_new')
    obs = env.reset()
    input("Start..")
    total_reward = 0
    model.policy.eval()
    for i in range(int(max_steps)):
        if lstm:
            action, lstm_states = model.predict(obs, state=lstm_states, episode_start=episode_starts, deterministic=True)
        else:
            action, states = model.predict(obs, deterministic=True)
        scaling_factor = np.array([2, 2, 4])
        scale_action = action / scaling_factor

        obs, reward, dones, infos = env.step(action)
        if lstm:
            episode_starts = dones
        # print(reward)
        total_reward += reward
        print(infos)
        # if terminated or truncated:
        #     print(total_reward)
        #     if lstm:
        #         lstm_states = None
        #         episode_starts = np.ones((1,), dtype=bool)
        #     obs = env.reset()
        #     total_reward = 0

def _main(model_path=None):
    bazel_chdir()
    sim_params = CassieFootstepControllerEnvironmentOptions()
    gym.envs.register(
        id="DrakeCassie-v0",
        entry_point="pydairlib.perceptive_locomotion.perception_learning.utils.DrakeCassieEnv:DrakeCassieEnv")  # noqa

    #sample(sim_params)
    run_play(sim_params, model_path=None)
    # run_saliency(sim_params, model_path=None)
    # run_integrated_gradients(sim_params, model_path=None)
    # run_interact(sim_params, model_path=None)
    #run_eval(sim_params, model_path=None)
    
    # for i in range(200):
    #     run_friction(sim_params, idx=i, model_path=None)

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--model_path',
        type=str,
        default=None,
    )
    args = parser.parse_args()

    _main(args.model_path)

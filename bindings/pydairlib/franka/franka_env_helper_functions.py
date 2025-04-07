from pydairlib.franka.controllers import C3Options
import numpy as np
from pydrake.common.yaml import yaml_load
from pydrake.all import *

def compute_reward_sparse(times, sim_states):
  first_target = np.array([0.45, 0, 0.485])
  second_target = np.array([0.45, 0, 0.6])
  third_target = np.array([0.7, 0.0, 0.485])
  length = min(times.shape[0], times.shape[0])
  reached_first_target = np.any(np.all(np.isclose(sim_states[:, 7:10], second_target, atol=1e-3), axis=1))
  reached_second_target = np.any(np.all(np.isclose(sim_states[:, 7:10], third_target, atol=1e-3), axis=1))
  reached_third_target = reached_second_target and np.linalg.norm(sim_states[:length, 7:10] - sim_states[:length, 7:10], axis=1)[-1] < 0.1
  # return reached_first_target + reached_second_target + reached_third_target
  return reached_first_target + reached_second_target + reached_third_target

def compute_reward_mpc(times, sim_states):
  first_target = np.array([0.45, 0, 0.485])
  second_target = np.array([0.45, 0, 0.6])
  third_target = np.array([0.7, 0.0, 0.485])
  neutral_orientation = Quaternion()
  cumulative_cost = 0
  for i in range(times.shape[0]):
    tray_pos = sim_states[11:14, i]
    tray_orientation = sim_states[7:11, i]
    tray_quat = Quaternion(tray_orientation / np.linalg.norm(tray_orientation))
    cumulative_cost += np.linalg.norm(tray_pos - first_target)
    # tray_quat /= np.linalg.norm(tray_quat.wxyz())
    angle_difference = Quaternion.multiply(neutral_orientation.conjugate(), tray_quat)
    cumulative_cost += RotationMatrix(angle_difference).ToAngleAxis().angle()
  cumulative_cost /= times.shape[0]
  return cumulative_cost

def load_c3_options(c3_params_file):
  c3_params = yaml_load(filename=c3_params_file)
  c3_options_file = c3_params['c3_options_file'][c3_params['scene_index']]
  c3_options_dict = yaml_load(filename=c3_options_file)
  c3_options = C3Options()
  c3_options.admm_iter = c3_options_dict['admm_iter']
  c3_options.rho = c3_options_dict['rho']
  c3_options.rho_scale = c3_options_dict['rho_scale']
  c3_options.num_threads = c3_options_dict['num_threads']
  c3_options.delta_option = c3_options_dict['delta_option']
  c3_options.projection_type = c3_options_dict['projection_type']
  c3_options.contact_model = c3_options_dict['contact_model']
  c3_options.warm_start = c3_options_dict['warm_start']
  c3_options.use_predicted_x0 = c3_options_dict['use_predicted_x0']
  c3_options.solve_time_filter_alpha = c3_options_dict['solve_time_filter_alpha']
  c3_options.publish_frequency = c3_options_dict['publish_frequency']
  c3_options.world_x_limits = c3_options_dict['world_x_limits']
  c3_options.world_y_limits = c3_options_dict['world_y_limits']
  c3_options.world_z_limits = c3_options_dict['world_z_limits']
  c3_options.u_horizontal_limits = c3_options_dict['u_horizontal_limits']
  c3_options.u_vertical_limits = c3_options_dict['u_vertical_limits']
  c3_options.workspace_margins = c3_options_dict['workspace_margins']
  c3_options.N = c3_options_dict['N']
  c3_options.gamma = c3_options_dict['gamma']
  c3_options.gamma = c3_options_dict['gamma']
  c3_options.q_vector = c3_options_dict['q_vector']
  c3_options.r_vector = c3_options_dict['r_vector']
  c3_options.g_x = c3_options_dict['g_x']
  c3_options.g_gamma = c3_options_dict['g_gamma']
  c3_options.g_lambda_n = c3_options_dict['g_lambda_n']
  c3_options.g_lambda_t = c3_options_dict['g_lambda_t']
  c3_options.g_lambda = c3_options_dict['g_lambda']
  c3_options.g_u = c3_options_dict['g_u']
  c3_options.u_x = c3_options_dict['u_x']
  c3_options.u_gamma = c3_options_dict['u_gamma']
  c3_options.u_lambda_n = c3_options_dict['u_lambda_n']
  c3_options.u_lambda_t = c3_options_dict['u_lambda_t']
  c3_options.u_lambda = c3_options_dict['u_lambda']
  c3_options.u_u = c3_options_dict['u_u']
  c3_options.gamma = c3_options_dict['gamma']
  c3_options.mu = c3_options_dict['mu']
  c3_options.dt = c3_options_dict['dt']
  c3_options.solve_dt = c3_options_dict['solve_dt']
  c3_options.num_friction_directions = c3_options_dict['num_friction_directions']
  c3_options.num_contacts = c3_options_dict['num_contacts']
  c3_options.Q = c3_options_dict['w_Q'] * np.diag(np.array(c3_options_dict['q_vector']))
  c3_options.R = c3_options_dict['w_R'] * np.diag(np.array(c3_options_dict['r_vector']))
  g_vec = np.hstack((c3_options_dict['g_x'], c3_options_dict['g_lambda'], c3_options_dict['g_u']))
  u_vec = np.hstack((c3_options_dict['u_x'], c3_options_dict['u_lambda'], c3_options_dict['u_u']))
  c3_options.G = c3_options_dict['w_G'] * np.diag(g_vec)
  c3_options.U = c3_options_dict['w_U'] * np.diag(u_vec)
  return c3_options

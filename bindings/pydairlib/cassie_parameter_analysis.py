import sys

import lcm
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import cumtrapz
import process_lcm_log
from numpy.linalg import norm
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.systems.framework import DiagramBuilder
import pydairlib.lcm_trajectory
import pydairlib.multibody_utils
from loadLcmTrajs import loadLcmTrajs
from scipy import integrate


def get_index_at_time(times, t):
  return np.argwhere(times - t > 0)[0][0]

def print_osc_debug(t_idx, length, osc_debug):
  print('y = ' + str(osc_debug.y[t_idx:t_idx + length, :]))
  print('y_des = ' + str(osc_debug.y_des[t_idx:t_idx + length, :]))
  print('error_y = ' + str(osc_debug.error_y[t_idx:t_idx + length, :]))
  print('ydot = ' + str(osc_debug.ydot[t_idx:t_idx + length, :]))
  print('ydot_des = ' + str(osc_debug.ydot_des[t_idx:t_idx + length, :]))
  print('error_ydot = ' + str(osc_debug.error_ydot[t_idx:t_idx + length, :]))
  print('yddot_des = ' + str(osc_debug.yddot_des[t_idx:t_idx + length, :]))
  print('yddot_command = ' + str(
    osc_debug.yddot_command[t_idx:t_idx + length, :]))
  print('yddot_command_sol = ' + str(
    osc_debug.yddot_command_sol[t_idx:t_idx + length, :]))

def main():
  global filename
  global x_traj_nominal
  global u_traj_nominal
  global x_hybrid_trajs_nominal
  global u_hybrid_trajs_nominal
  global state_slice
  global time_slice
  global t0_, tf_
  global t_minus, t_plus, t_final, impact_idx
  global plant
  global world
  global nq, nv, nx, nu
  global data_folder

  # Set default directory for saving figures
  matplotlib.rcParams["savefig.directory"] = \
    "/home/yangwill/Documents/research/projects/cassie/jumping/analysis" \
    "/figures/"
  matplotlib.rcParams["savefig.directory"] = \
    "/home/yangwill/Documents/research/projects/five_link_biped" \
    "/hybrid_lqr/figures/"
  matplotlib.rcParams['text.latex.preamble'] = [r"\usepackage{amsmath}"]
  font = {'size'   : 20}
  matplotlib.rc('font', **font)
  matplotlib.rcParams['lines.linewidth'] = 4

  builder = DiagramBuilder()
  plant, _ = AddMultibodyPlantSceneGraph(builder, 0.0)
  Parser(plant).AddModelFromFile(
    "/home/yangwill/Documents/research/dairlib/examples/Cassie/urdf"
    "/cassie_v2.urdf")
  plant.mutable_gravity_field().set_gravity_vector(
    -9.81 * np.array([0, 0, 1]))
  plant.Finalize()

  # relevant MBP parameters
  nq = plant.num_positions()
  nv = plant.num_velocities()
  nx = plant.num_positions() + plant.num_velocities()
  nu = plant.num_actuators()


  pos_map = pydairlib.multibody_utils.makeNameToPositionsMap(plant)
  vel_map = pydairlib.multibody_utils.makeNameToVelocitiesMap(plant)
  act_map = pydairlib.multibody_utils.makeNameToActuatorsMap(plant)

  state_names_w_spr = [[] for i in range(len(pos_map) + len(vel_map))]
  for name in pos_map:
    state_names_w_spr[pos_map[name]] = name
    print(name)
  for name in vel_map:
    state_names_w_spr[nq + vel_map[name]] = name

  l_toe_frame = plant.GetBodyByName("toe_left").body_frame()
  r_toe_frame = plant.GetBodyByName("toe_right").body_frame()
  world = plant.world_frame()
  no_offset = np.zeros(3)
  context = plant.CreateDefaultContext()

  x_traj_nominal, x_hybrid_trajs_nominal, u_traj_nominal, \
  u_hybrid_trajs_nominal, decision_vars, datatypes \
    = loadLcmTrajs(37, nu, 3)

  nq_fb = 7
  nv_fb = 6

  state_names_wo_spr = datatypes

  switching_times = np.linspace(0.0, 0.05, 51)
  # x_perturbations = np.linspace(-0.05, 0.05, 11)
  x_perturbations = np.array([0])
  # stiffness = np.array([1e-4, 1e-3, 5e-3])
  stiffness = np.array([5e-3])
  data_folder = '../projects/cassie/jumping/analysis/processed_data/'
  suffix = '_new_sim_2'
  tracking_cost, u_cost = \
    load_parameters_from_log(context, switching_times, x_perturbations,
                             stiffness, suffix)
  save_parameters(tracking_cost, u_cost, suffix)
  # tracking_cost, u_cost = load_parameters(suffix)

  t_window = np.arange(0.6, 1.0, step=0.00025)
  plt.plot(1000 * t_window[:400], tracking_cost[:21, 0, :400].T[:, ::5])
  plt.xlabel('Time After Impact (ms)')
  plt.ylabel('OSC Tracking Cost')
  plt.ticklabel_format(axis="y", style="sci", scilimits=(0,0))
  plt.legend(['0 ms delay', '5 ms delay',
              '10 ms delay', '15 ms delay',
              '20 ms delay'])
  plt.figure('u_cost')
  plt.plot(1000 * t_window[:400], u_cost[:21, 0, :400].T[:, ::5])
  plt.xlabel('Time After Impact (ms)')
  plt.ylabel('Cumulative Controller Effort (Nm)')
  plt.legend(['0 ms delay', '5 ms delay',
              '10 ms delay', '15 ms delay',
              '20 ms delay'])
  plt.show()
  import pdb; pdb.set_trace()

def calc_costs(t_state, t_osc, osc_debug, control_inputs, fsm_state, contact_info):

  # Cost is norm of efforts applied over impact event + OSC tracking error
  t_impact_start_idx = np.argwhere(t_state >= 0.6)[0][0]
  # t_impact_start_idx = np.argwhere(contact_info[:, :, 2])[0,1]
  t_impact_end_idx = np.argwhere(t_state >= 1.0)[0][0]

  t_impact_start = t_state[t_impact_start_idx]
  t_impact_end = t_state[t_impact_end_idx]
  print(t_impact_start)

  n_timesteps = t_impact_end_idx - t_impact_start_idx
  u_cost = np.zeros(n_timesteps)
  tracking_cost = np.zeros(n_timesteps)

  W0 = 2000 * np.eye(3)
  W0[1, 1] = 200
  W1 = 20 * np.eye(3)
  W1[1, 1] = 10

  # print(t_osc[np.argwhere(fsm_state == 3)[0]])

  for i in range(t_impact_start_idx, t_impact_end_idx):
    u_i = np.reshape(control_inputs[get_index_at_time(t_osc, t_state[i])], (nu, 1))
    u_cost[i - t_impact_start_idx] = norm(u_i)
    osc_idx = get_index_at_time(t_osc, t_state[i])
    tracking_err_0 = osc_debug[0].yddot_command_sol[osc_idx, :] - \
                     osc_debug[0].yddot_des[osc_idx, :]
    tracking_err_1 = osc_debug[1].yddot_command_sol[osc_idx, :] - \
                     osc_debug[1].yddot_des[osc_idx, :]
    tracking_cost[i - t_impact_start_idx] = \
      tracking_err_0.T @ W0 @ tracking_err_0 + \
      tracking_err_1.T @ W1 @ tracking_err_1

  accumulated_u_cost = cumtrapz(u_cost, t_state[t_impact_start_idx:
                                                t_impact_end_idx])
  # tracking_cost = cumtrapz(tracking_cost, t_state[t_impact_start_idx:
  #                                               t_impact_end_idx])
  return tracking_cost[:400], accumulated_u_cost[:400]

def load_parameters(suffix):
  tracking_cost = np.load(data_folder + 'tracking_cost' + suffix + '.npy')
  u_cost = np.load(data_folder + 'u_cost' + suffix + '.npy')
  return tracking_cost, u_cost

def save_parameters(tracking_cost, u_cost, suffix):
  np.save(data_folder + 'tracking_cost' + suffix, tracking_cost)
  np.save(data_folder + 'u_cost' + suffix, u_cost)

def load_parameters_from_log(context, switching_times, x_pertubations,
                             stiffness, suffix):
  pos_map = pydairlib.multibody_utils.makeNameToPositionsMap(plant)
  vel_map = pydairlib.multibody_utils.makeNameToVelocitiesMap(plant)
  act_map = pydairlib.multibody_utils.makeNameToActuatorsMap(plant)

  n_ts = switching_times.shape[0]
  n_dx = x_pertubations.shape[0]
  n_del = stiffness.shape[0]

  tracking_cost = np.zeros((n_ts, n_dx, 400))
  u_cost = np.zeros((n_ts, n_dx, 400))

  # folder_path = \
  #   "../projects/cassie/jumping/logs/parameter_study/switching_time/"
  folder_path = \
    "../projects/cassie/jumping/logs/parameter_study/new_sim_2/"
  for k in range(stiffness.shape[0]):
    for i in range(switching_times.shape[0]):
      for j in range(x_pertubations.shape[0]):
        filename = folder_path + \
                   "lcmlog-delay_%.4f-pen_%.5f" \
                   % (switching_times[i], stiffness[k])
        print(filename)
        log = lcm.EventLog(filename, "r")
        contact_info, contact_info_locs, control_inputs, estop_signal, osc_debug, \
        q, switch_signal, t_contact_info, t_controller_switch, t_osc, t_osc_debug, \
        t_state, v, fsm = process_lcm_log.process_log(log, pos_map, vel_map)

        tracking_cost[i, j], u_cost[i, j] = \
          calc_costs(t_state, t_osc, osc_debug, control_inputs, fsm, contact_info)
  return tracking_cost, u_cost

if __name__ == "__main__":
  main()
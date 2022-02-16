import sys

import lcm
import numpy as np
import pathlib
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder
from pydairlib.lcm import process_lcm_log
import pydairlib.multibody
from pydairlib.cassie.cassie_utils import *
from pydairlib.common import plot_styler
from pydairlib.lcm import lcm_trajectory
from pydairlib.cassie.kinematics_helper import KinematicsHelper

import matplotlib.pyplot as plt


def convert_log(filename, controller_channel, save_est_contact_forces=False):
  figure_directory = '/home/yangwill/Documents/research/projects/impact_uncertainty/data/'
  # figure_directory = '/home/yangwill/workspace/dairlib/examples/contact_parameter_learning/cassie_sim_data'
  ps = plot_styler.PlotStyler()
  ps.set_default_styling(directory=figure_directory)

  builder = DiagramBuilder()
  plant_w_spr, scene_graph_w_spr = AddMultibodyPlantSceneGraph(builder, 0.0)
  plant_wo_spr, scene_graph_wo_spr = AddMultibodyPlantSceneGraph(builder, 0.0)
  pydairlib.cassie.cassie_utils.AddCassieMultibody(plant_w_spr, scene_graph_w_spr, True,
                                                   "examples/Cassie/urdf/cassie_v2.urdf", False, False)
  pydairlib.cassie.cassie_utils.AddCassieMultibody(plant_wo_spr, scene_graph_wo_spr, True,
                                                   "examples/Cassie/urdf/cassie_fixed_springs.urdf", False, False)
  plant_w_spr.Finalize()
  plant_wo_spr.Finalize()

  kinematics_calculator = KinematicsHelper()

  # relevant MBP parameters
  nq = plant_w_spr.num_positions()
  nv = plant_w_spr.num_velocities()
  nx = plant_w_spr.num_positions() + plant_w_spr.num_velocities()
  nu = plant_w_spr.num_actuators()

  pos_map = pydairlib.multibody.makeNameToPositionsMap(plant_w_spr)
  vel_map = pydairlib.multibody.makeNameToVelocitiesMap(plant_w_spr)
  act_map = pydairlib.multibody.makeNameToActuatorsMap(plant_w_spr)

  # joint/actuator names
  x_datatypes = pydairlib.multibody.createStateNameVectorFromMap(plant_w_spr)
  u_datatypes = pydairlib.multibody.createActuatorNameVectorFromMap(plant_w_spr)

  #
  # filename = sys.argv[1]
  # controller_channel = sys.argv[2]
  log = lcm.EventLog(filename, "r")
  path = pathlib.Path(filename).parent
  log_name = filename.split("/")[-1]
  log_date = filename.split('/')[-2]

  x, u_meas, t_x, u, t_u, contact_info, contact_info_locs, t_contact_info, \
  osc_debug, fsm, estop_signal, switch_signal, t_controller_switch, t_pd, kp, kd, cassie_out, u_pd, t_u_pd, \
  osc_output, full_log, t_lcmlog_u = process_lcm_log.process_log(log, pos_map, vel_map, act_map, controller_channel)

  # Default time window values, can override
  # t_start = t_u[10]
  # t_end = t_u[-10]
  # Override here #
  dataset_num = '04'
  impact_time = 30.650
  z_offset = 0.001

  t_start = impact_time - 0.005  # start window 5 ms before impact
  t_end = t_start + 0.050  # end window 50 ms after the start
  ### Convert times to indices
  t_slice = slice(np.argwhere(np.abs(t_x - t_start) < 1e-3)[0][0], np.argwhere(np.abs(t_x - t_end) < 1e-3)[0][0])
  # t_u_slice = slice(np.argwhere(np.abs(t_u - t_start) < 1e-3)[0][0], np.argwhere(np.abs(t_u - t_end) < 1e-3)[0][0])

  # log_file_num = log_name.split('-')[1]
  # print(log_file_num)

  x = x[t_slice]
  # t_slice_shift = slice(t_slice.start - 2, t_slice.stop - 2)
  # t_slice_shift = slice(t_slice.start - 2, t_slice.stop - 2)
  u_meas = u_meas[t_slice]
  t_x = np.reshape(t_x[t_slice], (t_x[t_slice].shape[0],))
  t_x = t_x - t_start + (t_start - t_x[0])
  x[:, pos_map['base_z']] -= z_offset
  print(t_x.shape[0])
  assert (t_x.shape[0] == 100)
  # t_x = np.arange(0, 0.05, 5e-4)

  # for visualizing hardware trajectory
  # plt.figure("efforts")
  # plt.plot(t_x, u_meas)
  plt.figure("state")
  plt.plot(t_x, x[:, -6:])

  np.save(ps.directory + 'curated_trajectories/' + 'x_' + dataset_num, x)
  np.save(ps.directory + 'curated_trajectories/' + 't_' + dataset_num, t_x)
  np.save(ps.directory + 'curated_trajectories/' + 'u_' + dataset_num, u_meas)

  foot_pos_hardware = np.empty((t_x.shape[0], 4))
  for i in range(t_x.shape[0]):
    x_i = x[i]
    foot_pos_hardware[i] = kinematics_calculator.compute_foot_z_position(x_i)
  plt.figure("foot_pos")
  ps.plot(t_x, foot_pos_hardware)
  plt.show()

  # np.save(ps.directory + log_date + '/' + 'x_' + dataset_num, x)
  # np.save(ps.directory + log_date + '/' + 't_' + dataset_num, t_x)
  # np.save(ps.directory + log_date + '/' + 'u_' + dataset_num, u_meas)

  # np.save(ps.directory + 'curated_trajectories/' + 'x_' + dataset_num, x)
  # np.save(ps.directory + 'curated_trajectories/' + 't_' + dataset_num, t_x)
  # np.save(ps.directory + 'curated_trajectories/' + 'u_' + dataset_num, u_meas)

  # for use with c++ simulator
  # controller_input_traj = lcm_trajectory.Trajectory()
  # controller_input_traj.traj_name = 'controller_inputs'
  # controller_input_traj.time_vector = t_u
  # controller_input_traj.datapoints = u_meas.transpose()
  # controller_input_traj.datatypes = [''] * u_meas.shape[1]
  # lcm_traj = lcm_trajectory.LcmTrajectory()
  # lcm_traj.AddTrajectory('controller_inputs', controller_input_traj)
  # lcm_traj.WriteToFile(ps.directory + log_date + '/' + 'u_traj_' + log_file_num)


  if save_est_contact_forces:
    # contact_forces = np.empty((len(full_log['CASSIE_GM_CONTACT_DISPATCHER']), 4, 3))
    contact_info_locs = [[], [], [], []]
    contact_forces = [[], [], [], []]  # Allocate space for all 4 point contacts
    for msg in full_log['CASSIE_GM_CONTACT_DISPATCHER']:
      num_left_contacts = 0
      num_right_contacts = 0
      # import pdb; pdb.set_trace()
      for i in range(msg.num_point_pair_contacts):
        if "toe_left" in msg.point_pair_contact_info[i].body1_name:
          if (num_left_contacts >= 2):
            continue
          contact_info_locs[num_left_contacts].append(msg.point_pair_contact_info[i].contact_point)
          contact_forces[num_left_contacts].append(msg.point_pair_contact_info[i].contact_force)
          num_left_contacts += 1
        elif "toe_right" in msg.point_pair_contact_info[i].body1_name:
          if (num_right_contacts >= 2):
            continue
          contact_info_locs[2 + num_right_contacts].append(msg.point_pair_contact_info[i].contact_point)
          contact_forces[2 + num_right_contacts].append(msg.point_pair_contact_info[i].contact_force)
          num_right_contacts += 1
      while num_left_contacts != 2:
        contact_forces[num_left_contacts].append((0.0, 0.0, 0.0))
        contact_info_locs[num_left_contacts].append((0.0, 0.0, 0.0))
        num_left_contacts += 1
      while num_right_contacts != 2:
        contact_forces[2 + num_right_contacts].append((0.0, 0.0, 0.0))
        contact_info_locs[2 + num_right_contacts].append((0.0, 0.0, 0.0))
        num_right_contacts += 1
    contact_forces = np.array(contact_forces)
    contact_info_locs = np.array(contact_info_locs)
    for i in range(contact_info_locs.shape[1]):
      # Swap front and rear contacts if necessary
      # Order will be rear contact then front contact in index 1
      if contact_info_locs[0, i, 0] > contact_info_locs[1, i, 0]:
        contact_forces[[0, 1], i, :] = contact_forces[[1, 0], i, :]
        contact_info_locs[[0, 1], i, :] = contact_info_locs[[1, 0], i, :]
      if contact_info_locs[2, i, 0] > contact_info_locs[3, i, 0]:
        contact_forces[[2, 3], i, :] = contact_forces[[3, 2], i, :]
        contact_info_locs[[2, 3], i, :] = contact_info_locs[[3, 2], i, :]
      # import pdb; pdb.set_trace()
      # contact_forces[i, 0, :] = np.array(contact_info.point_pair_contact_info[0].contact_force)
      # contact_forces[i, 1, :] = np.array(contact_info.point_pair_contact_info[1].contact_force)
      # contact_forces[i, 2, :] = np.array(contact_info.point_pair_contact_info[2].contact_force)
      # contact_forces[i, 3, :] = np.array(contact_info.point_pair_contact_info[3].contact_force)
    contact_forces = contact_forces[:, t_slice, :]
    np.save(ps.directory + 'lambda_' + log_file_num, contact_forces)

def convert_all_hardware_jumping_logs():
  controller_channel = 'OSC_JUMPING'
  root_log_dir = '/home/yangwill/Documents/research/projects/cassie/hardware/logs/2021/'
  jan_logs = np.arange(8, 18)
  feb_logs = np.arange(20, 34)
  # feb_26_logs = np.hstack((np.arange(0, 7), np.arange(11, 15), np.arange(16,18)))
  feb_26_logs = np.hstack((np.arange(0,18)))
  jan_logs = ['%0.2d' % i for i in jan_logs]
  feb_logs = ['%0.2d' % i for i in feb_logs]
  feb_26_logs = ['%0.2d' % i for i in feb_26_logs]

  # for log_num in jan_logs:
  #   print('log_num: ' + log_num)
  #   convert_log(root_log_dir + '01_27_21/lcmlog-' + log_num, controller_channel)
  # for log_num in feb_logs:
  #   print('log_num: ' + log_num)
  #   convert_log(root_log_dir + '02_12_21/lcmlog-' + log_num, controller_channel)
  for log_num in feb_26_logs:
    print('log_num: ' + log_num)
    convert_log(root_log_dir + '02_26_21/lcmlog-' + log_num, controller_channel)

  print('done')

def main():

  filename = sys.argv[1]
  controller_channel = sys.argv[2]
  convert_log(filename, controller_channel)

if __name__ == '__main__':
  # convert_all_hardware_jumping_logs()
  main()

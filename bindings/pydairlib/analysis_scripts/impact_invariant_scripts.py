import matplotlib
import numpy as np
import matplotlib.pyplot as plt
import scipy.linalg as linalg
import pydairlib.multibody
import scipy.io as io
from pydrake.multibody.tree import JacobianWrtVariable
from pydairlib.multibody.kinematic import DistanceEvaluator
from pydairlib.cassie.cassie_utils import LeftLoopClosureEvaluator
from pydairlib.cassie.cassie_utils import RightLoopClosureEvaluator
from pydairlib.common import FindResourceOrThrow
from pydairlib.lcm import lcm_trajectory


def save_data_for_matt(M, J, B, x, t_x, t_slice, nq, nv, u_meas, log_num):
  q_impact = x[t_slice, :nq]
  v_impact = x[t_slice, -nv:]
  t_impact = t_x[t_slice] - 30.645
  u_impact = u_meas[t_slice]
  output_file = 'impact_data_' + str(log_num) + '.mat'
  # output_file = 'nominal_configuration.mat'

  io.savemat(output_file, {'q': q_impact,
                           'v': v_impact,
                           't': t_impact,
                           'M': M,
                           'J': J,
                           'B': B,
                           'u': u_impact
                           })
  # io.savemat(output_file, {'M': M,
  #                          'J': J,
  #                          'B': B,
  #                          'x_init': x,
  #                          })

def plot_per_joint():

  return

def plot_all_joints():

  return

def generate_legend(joint_vel_datatypes):
  legend_elements_lines = []
  for i in range(len(joint_vel_datatypes)):
    legend_elements_lines.append(matplotlib.lines.Line2D([0], [0], color=ps.cmap(i), lw=3, label=joint_vel_datatypes[i]))

  plt.figure('blank')
  ax = plt.gca()
  ax.legend(handles = legend_elements_lines, loc='upper center',
            fancybox=True, shadow=True, ncol=3)
  ps.save_fig('projected_velocities_legend.png')

def load_nominal_trajectory():
  filename = FindResourceOrThrow("examples/Cassie/saved_trajectories/jumping_0.15h_0.3d")
  # filename = "/home/yangwill/Documents/research/projects/cassie/hardware/backup/dair/saved_trajectories/jumping_0.15h_0.3d"
  jumping_traj = lcm_trajectory.DirconTrajectory(filename)
  state_traj = jumping_traj.ReconstructStateTrajectory()
  input_traj = jumping_traj.ReconstructInputTrajectory()

  return state_traj


def plot_ii_projection(ps, t_x, x, plant, context, t_slice, pos_map_spr_to_wo_spr, vel_map_spr_to_wo_spr, linestyle, log_num, u_meas):
  # t_pre = 30.557
  # t_pre = 30.645
  t_pre = 30.595
  t_post = 30.795
  t_impact = 30.645
  t_idx = np.argwhere(np.abs(t_x - t_pre) < 1e-3)[0][0]
  t_impact_idx = np.argwhere(np.abs(t_x - 30.645) < 1e-3)[0][0]
  t_post_idx = np.argwhere(np.abs(t_x - t_post) < 1e-3)[0][0]

  t_slice = slice(t_idx, t_post_idx)

  l_toe_frame = plant.GetBodyByName("toe_left").body_frame()
  r_toe_frame = plant.GetBodyByName("toe_right").body_frame()
  front_contact_disp = np.array((-0.0457, 0.112, 0))
  rear_contact_disp = np.array((0.088, 0, 0))
  world = plant.world_frame()

  joint_vel_datatypes = [r'$Hip\ Roll_L$',
                         r'$Hip\ Roll_R$',
                         r'$Hip\ Yaw_L$',
                         r'$Hip\ Yaw_R$',
                         r'$Hip\ Pitch_L$',
                         r'$Hip\ Pitch_R$',
                         r'$Knee_L$',
                         r'$Knee_R$',
                         r'$Ankle_L$',
                         r'$Ankle_R$',
                         r'$Toe_L$',
                         r'$Toe_R$',
                         ]

  nq = plant.num_positions()
  nv = plant.num_velocities()

  nq_w_spr = pos_map_spr_to_wo_spr.shape[1]
  nv_w_spr = vel_map_spr_to_wo_spr.shape[1]

  x_datatypes = pydairlib.multibody.createStateNameVectorFromMap(plant)

  state_traj = load_nominal_trajectory()

  # Make adjustments if necessary to work with the model without springs
  if(plant.num_positions() == 23):
    x_pre = x[t_impact_idx]
  else:
    x = np.vstack((pos_map_spr_to_wo_spr @ x[:, :nq_w_spr].T,
                          vel_map_spr_to_wo_spr @ x[:, -nv_w_spr:].T)).T
    x_pre = x[t_impact_idx]
  plant.SetPositionsAndVelocities(context, x_pre)
  # plant.SetPositionsAndVelocities(context, state_traj.value(0.557715))
  M = plant.CalcMassMatrixViaInverseDynamics(context)
  M_inv = np.linalg.inv(M)
  J_l_f = plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable.kV, l_toe_frame, front_contact_disp, world, world)
  J_l_r = plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable.kV, l_toe_frame, rear_contact_disp, world, world)
  J_r_f = plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable.kV, r_toe_frame, front_contact_disp, world, world)
  J_r_r = plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable.kV, r_toe_frame, rear_contact_disp, world, world)
  J_l_loop = LeftLoopClosureEvaluator(plant).EvalFullJacobian(context)
  J_r_loop = RightLoopClosureEvaluator(plant).EvalFullJacobian(context)
  J = np.vstack((J_l_f, J_l_r, J_r_f, J_r_r, J_l_loop, J_r_loop))

  M_Jt = M_inv @ J.T
  P = linalg.null_space(M_Jt.T).T
  proj_vel = P.T @ P @ x[t_slice, -nv:].T

  # save_data_for_matt(M, J, plant.MakeActuationMatrix(), x, t_x, t_slice, nq, nv, u_meas, log_num)
  save_data_for_matt(M, J, plant.MakeActuationMatrix(), state_traj.value(0.595), t_x, t_slice, nq, nv, u_meas, log_num)

  # plt.figure("joint velocities")
  for i in range(0,6):
    plt.figure("joint velocities: " + str(i))
    ps.plot(1e3*(t_x[t_slice] - 30.645), x[t_slice, -12 + 2*i], xlabel='Time since Start of Impact (ms)', ylabel='Velocity (rad/s)', color=ps.cmap(2*i))
    ps.plot(1e3*(t_x[t_slice] - 30.645), x[t_slice, -11 + 2*i], xlabel='Time since Start of Impact (ms)', ylabel='Velocity (rad/s)', color=ps.cmap(1 + 2*i))
    # ps.plot(t_x[t_slice], x[t_slice, -12 + 2*i], xlabel='Time since Start of Impact (ms)', ylabel='Velocity (rad/s)', color=ps.cmap(2*i), linestyle = linestyle)
    # ps.plot(t_x[t_slice], x[t_slice, -11 + 2*i], xlabel='Time since Start of Impact (ms)', ylabel='Velocity (rad/s)', color=ps.cmap(1 + 2*i), linestyle = linestyle)
    ps.add_legend(['%s' % name for name in joint_vel_datatypes[2*i : 2*(i + 1)]])
    # ps.plot(t_x[t_slice], x[t_slice, -12 + 2*i], xlabel='Time since Start of Impact (ms)', ylabel='Velocity (rad/s)', color=ps.cmap(2*i))
    # ps.plot(t_x[t_slice], x[t_slice, -11 + 2*i], xlabel='Time since Start of Impact (ms)', ylabel='Velocity (rad/s)', color=ps.cmap(1 + 2*i))
  # ps.add_legend(['%s' % name for name in joint_vel_datatypes[-12:]])

  plt.ylim([-8, 8])
  plt.xlim([-50, 100])
  # plt.xticks(np.arange(-50, 30+0.1, 10))
  plt.yticks(np.arange(-8, 8.1, 2))
  # ps.save_fig('joint_velocities_hardware_for_video.png')

  # plt.figure("projected velocities")
  for i in range(0, 6):
    plt.figure("projected velocities: " + str(i))
    ps.plot(1e3*(t_x[t_slice] - 30.645), proj_vel.T[:, -12 + 2*i], xlabel='Time since Start of Impact (ms)', ylabel='Velocity (rad/s)', color=ps.cmap(2*i))
    ps.plot(1e3*(t_x[t_slice] - 30.645), proj_vel.T[:, -11 + 2*i], xlabel='Time since Start of Impact (ms)', ylabel='Velocity (rad/s)', color=ps.cmap(1 + 2*i))
    # ps.plot(t_x[t_slice], proj_vel.T[:, -12 + 2*i], xlabel='Time since Start of Impact (ms)', ylabel='Velocity (rad/s)', color=ps.cmap(2*i))
    # ps.plot(t_x[t_slice], proj_vel.T[:, -11 + 2*i], xlabel='Time since Start of Impact (ms)', ylabel='Velocity (rad/s)', color=ps.cmap(1 + 2*i))
    # ps.add_legend(['%s' % name for name in x_datatypes[-12 + 2*i: -10 + 2*i]])
    ps.add_legend(['%s' % name for name in joint_vel_datatypes[2*i : 2*(i + 1)]])
  # ps.add_legend(['%s' % name for name in x_datatypes[-12:]])

  plt.ylim([-3, 1])
  plt.xlim([-50, 100])
  # plt.xticks(np.arange(-50, 30+0.1, 10))
  plt.yticks(np.arange(-3, 1.1, 2))


  # Storing arrays for further analysis
  # log_file_num = '17'
  # np.save(ps.directory + 't_' + log_file_num, 1e3*(t_x[t_slice] - 30.645))
  # np.save(ps.directory + 'v_' + log_file_num, x[t_slice, -12:])
  # np.save(ps.directory + 'vproj_' + log_file_num, proj_vel.T[:, -12:])
  # ps.save_fig('projected_joint_velocities_hardware_for_video.png')


  # generate_legend(joint_vel_datatypes)
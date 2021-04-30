import matplotlib
import numpy as np
import matplotlib.pyplot as plt
import scipy.linalg as linalg
import pydairlib.multibody
from pydrake.multibody.tree import JacobianWrtVariable


def calc_loop_closure_jacobian(plant, context, x_pre):
  front_contact_disp = np.array((-0.0457, 0.112, 0))
  rear_contact_disp = np.array((0.088, 0, 0))
  heel_disp = np.array((.11877, -.01, 0.0))
  left_thigh_disp = np.array((0.0, 0.0, 0.045))
  right_thigh_disp = np.array((0.0, 0.0, -0.045))
  world = plant.world_frame()

  l_thigh_frame = plant.GetBodyByName("thigh_left").body_frame()
  r_thigh_frame = plant.GetBodyByName("thigh_right").body_frame()
  l_heel_frame = plant.GetBodyByName("heel_spring_left").body_frame()
  r_heel_frame = plant.GetBodyByName("heel_spring_right").body_frame()
  # plant.SetPositionsAndVelocities(context, x_pre)
  left_heel = plant.CalcPointsPositions(context, l_heel_frame, heel_disp, world)
  left_thigh = plant.CalcPointsPositions(context, l_thigh_frame, left_thigh_disp, world)
  right_heel = plant.CalcPointsPositions(context, r_heel_frame, heel_disp, world)
  right_thigh = plant.CalcPointsPositions(context, r_thigh_frame, right_thigh_disp, world)
  left_rel_pos = left_heel - left_thigh
  right_rel_pos = right_heel - right_thigh

  J_l_heel = plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable.kV, l_heel_frame, heel_disp, world, world)
  J_l_thigh = plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable.kV, l_thigh_frame, left_thigh_disp, world, world)
  J_r_heel = plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable.kV, r_heel_frame, heel_disp, world, world)
  J_r_thigh = plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable.kV, r_thigh_frame, right_thigh_disp, world, world)
  J_l_loop = (left_rel_pos.transpose() @ (J_l_heel - J_l_thigh)) / np.linalg.norm(left_rel_pos)
  J_r_loop = (right_rel_pos.transpose() @ (J_r_heel - J_r_thigh)) / np.linalg.norm(right_rel_pos)

  return J_l_loop, J_r_loop

def plot_ii_projection(ps, t_x, x, plant, context, t_slice, pos_map_spr_to_wo_spr, vel_map_spr_to_wo_spr):

  t_pre = 30.557
  t_idx = np.argwhere(np.abs(t_x - t_pre) < 1e-3)[0][0]
  x_pre = x[t_idx]

  l_toe_frame = plant.GetBodyByName("toe_left").body_frame()
  r_toe_frame = plant.GetBodyByName("toe_right").body_frame()
  front_contact_disp = np.array((-0.0457, 0.112, 0))
  rear_contact_disp = np.array((0.088, 0, 0))
  world = plant.world_frame()

  nq = pos_map_spr_to_wo_spr.shape[1]
  nv = vel_map_spr_to_wo_spr.shape[1]

  x_wo_spr_datatypes = pydairlib.multibody.createStateNameVectorFromMap(plant)
  x_wo_spr = np.vstack((pos_map_spr_to_wo_spr @ x[:, :nq].T, vel_map_spr_to_wo_spr @ x[:, -nv:].T)).T
  x_pre = x_wo_spr[t_idx]
  plant.SetPositionsAndVelocities(context, x_pre)
  M = plant.CalcMassMatrixViaInverseDynamics(context)
  M_inv = np.linalg.inv(M)
  J_l_f = plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable.kV, l_toe_frame, front_contact_disp, world, world)
  J_l_r = plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable.kV, l_toe_frame, rear_contact_disp, world, world)
  J_r_f = plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable.kV, r_toe_frame, front_contact_disp, world, world)
  J_r_r = plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable.kV, r_toe_frame, rear_contact_disp, world, world)
  J_l_loop, J_r_loop = calc_loop_closure_jacobian(plant, context, x_pre)
  J = np.vstack((J_l_f, J_l_r, J_r_f, J_r_r, J_l_loop, J_r_loop))
  # J = np.vstack((J_l_f, J_l_r, J_r_f, J_r_r))

  M_Jt = M_inv @ J.T
  # proj_ii = np.eye(18) - M_Jt @ np.linalg.inv(M_Jt.T @ M_Jt) @ M_Jt.T
  P = linalg.null_space(M_Jt.T).T
  # import pdb; pdb.set_trace()
  proj_vel = P.T @ P @ x_wo_spr[t_slice, -18:].T

  # proj_vel = P @ x[t_slice, -nv:].T
  # colors = ['B0', '']
  plt.figure("joint velocities")
  for i in range(6):
    # ps.plot(1e3*(t_x[t_slice] - 30.645), x_wo_spr[t_slice, -12:], xlabel='Time since Start of Impact (ms)', ylabel='Joint Velocities (rad/s)')
    ps.plot(1e3*(t_x[t_slice] - 30.645), x_wo_spr[t_slice, -12 + 2*i], xlabel='Time since Start of Impact (ms)', ylabel='Velocity (rad/s)', color=ps.cmap(2*i))
    ps.plot(1e3*(t_x[t_slice] - 30.645), x_wo_spr[t_slice, -11 + 2*i], xlabel='Time since Start of Impact (ms)', ylabel='Velocity (rad/s)', color=ps.cmap(1 + 2*i))

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
  # ps.add_legend(['%s' % name for name in x_wo_spr_datatypes[-12:]])

  # ps.add_legend(joint_vel_datatypes)

  plt.ylim([-8, 8])
  plt.xlim([-10, 30])
  plt.xticks(np.arange(-10, 30+0.1, 10))
  plt.yticks(np.arange(-8, 8.1, 2))
  # ps.save_fig('joint_velocities_hardware_for_video.png')

  plt.figure("projected velocities")
  # joint_indices = [3, 4, 5, 6, 7, 8, 9, 12, 13, 14, 15, 16, 17]
  for i in range(6):
    ps.plot(1e3*(t_x[t_slice] - 30.645), proj_vel.T[:, -12 + 2*i], xlabel='Time since Start of Impact (ms)', ylabel='Velocity (rad/s)', color=ps.cmap(2*i))
    ps.plot(1e3*(t_x[t_slice] - 30.645), proj_vel.T[:, -11 + 2*i], xlabel='Time since Start of Impact (ms)', ylabel='Velocity (rad/s)', color=ps.cmap(1 + 2*i))
  # ps.add_legend(['%.0f' % i for i in range(18)])
  plt.ylim([-3, 1])
  plt.xlim([-10, 30])
  plt.xticks(np.arange(-10, 30+0.1, 10))
  plt.yticks(np.arange(-3, 1.1, 2))
  legend_elements_lines = []
  for i in range(len(joint_vel_datatypes)):
    legend_elements_lines.append(matplotlib.lines.Line2D([0], [0], color=ps.cmap(i), lw=3, label=joint_vel_datatypes[i]))
  # ps.save_fig('projected_joint_velocities_hardware_for_video.png')

  # plt.figure('blank')
  # ax = plt.gca()
  # ax.legend(handles = legend_elements_lines, loc='upper center',
  #           fancybox=True, shadow=True, ncol=3)
  # ps.save_fig('projected_velocities_legend.png')
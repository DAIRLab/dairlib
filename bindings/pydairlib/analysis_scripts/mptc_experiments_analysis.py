import sys
import glob
import seaborn as sns
import lcm
import pandas as pd

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import process_lcm_log
import pathlib
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.systems.framework import DiagramBuilder
import pydairlib.lcm_trajectory
import pydairlib.multibody
from pydairlib.common import FindResourceOrThrow

class Log:
  def __init__(self, filename, controller_channel, plant_w_spr, plant_wo_spr):
    self.t_slice = slice(0,0)
    self.t_u_slice = slice(0,0)
    self.filename = filename
    self.controller_channel = controller_channel
    self.nq = plant_w_spr.num_positions()
    self.nv = plant_w_spr.num_velocities()
    self.nx = plant_w_spr.num_positions() + plant_w_spr.num_velocities()
    self.nu = plant_w_spr.num_actuators()
    self.pos_map = pydairlib.multibody.makeNameToPositionsMap(plant_w_spr)
    self.vel_map = pydairlib.multibody.makeNameToVelocitiesMap(plant_w_spr)
    self.act_map = pydairlib.multibody.makeNameToActuatorsMap(plant_w_spr)
    self.plant_w_spr = plant_w_spr
    self.plant_wo_spr = plant_wo_spr
    self.controller = filename.split("/")[-1].split("_")[0]
    self.load()

  def load(self):
    self.log = lcm.EventLog(self.filename, "r")

    self.x, self.u_meas, self.t_x, self.u, self.t_u, self.contact_info, self.contact_info_locs, self.t_contact_info, \
    self.osc_debug, self.fsm, self.estop_signal, self.switch_signal, self.t_controller_switch, self.t_pd, self.kp, \
    self.kd, self.cassie_out, self.u_pd, self.t_u_pd, self.osc_output, self.full_log, self.t_target_height, \
    self.target_height = \
      process_lcm_log.process_log(
      self.log,
      self.pos_map,
      self.vel_map,
      self.act_map,
      self.controller_channel)

  def slice_for_crouch(self):
    self.t_u_slice = get_crouch_slice(self.osc_debug, self.target_height, self.t_u, self.t_target_height)
    t_start_idx = np.argwhere(np.abs(self.t_x - self.t_u[self.t_u_slice.start]) < 1e-3)[0][0]
    t_end_idx = np.argwhere(np.abs(self.t_x - self.t_u[self.t_u_slice.stop]) < 1e-3)[0][0]
    self.t_slice = slice(t_start_idx, t_end_idx)

  def slice_for_walk(self):
    self.t_u_slice = get_walk_slice(self.osc_debug, self.fsm, self.t_u)
    try:
      t_start_idx = np.argwhere(np.abs(self.t_x - self.t_u[self.t_u_slice.start]) < 1e-3)[0][0]
    except:
      t_start_idx = np.argwhere(np.abs(self.t_x - self.t_u[self.t_u_slice.start]) < 2.5e-3)[0][0]
    try:
      t_end_idx = np.argwhere(np.abs(self.t_x - self.t_u[self.t_u_slice.stop]) < 1e-3)[0][0]
    except:
      t_end_idx = np.argwhere(np.abs(self.t_x - self.t_u[self.t_u_slice.stop]) < 2.5e-3)[0][0]
    self.t_slice = slice(t_start_idx, t_end_idx)

  def get_cv_force_swing_ft(self):
    n_samples = len(self.t_x[self.t_slice])
    context = self.plant_w_spr.CreateDefaultContext()

    Cv_force = np.zeros((n_samples, 3))

    for i in range(n_samples):
      idx = i + self.t_slice.start
      try:
        u_idx = np.argwhere(np.abs(self.t_u - self.t_x[idx]) < 1e-3)[0][0]
      except:
        u_idx = np.argwhere(np.abs(self.t_u - self.t_x[idx]) < 2.5e-3)[0][0]

      self.plant_w_spr.SetPositionsAndVelocities(context, self.x[idx, :])
      if self.fsm[u_idx] != 2:
        pt, frame = self.get_swing_foot_midpoint_frame(u_idx)
        Cv_force[i] = self.calc_task_space_cv_force_translational(context, frame, pt)
      else :
        Cv_force[i] = np.zeros((1,3))

    return Cv_force

  def get_commanded_torque_force_swing_ft(self):
    n_samples = len(self.t_u[self.t_u_slice])
    context = self.plant_w_spr.CreateDefaultContext()

    actuation_force = np.zeros((n_samples, 3))
    for i in range(n_samples):
      idx = i + self.t_u_slice.start
      try:
        x_idx = np.argwhere(np.abs(self.t_x - self.t_u[idx]) < 1e-3)[0][0]
      except:
        x_idx = np.argwhere(np.abs(self.t_x - self.t_u[idx]) < 2.5e-3)[0][0]

      self.plant_w_spr.SetPositionsAndVelocities(context, self.x[x_idx,:])
      pt, frame = self.get_swing_foot_midpoint_frame(idx)
      actuation_force[i] = self.calc_task_space_force_actuation_translational(
          context, frame, pt, idx)

    return actuation_force


  def get_swing_foot_midpoint_frame(self, ind):
    mid_pt = np.array((-0.0457, 0.112, 0)) + np.array((0.088, 0, 0)) / 2
    left_stance_state = 0
    right_stance_state = 1
    double_support_state = 2

    state = self.fsm[ind]
    i = 1
    while (state == double_support_state):
      state = self.fsm[ind-i]
      i = i+1

    if self.fsm[ind] == left_stance_state:
      return mid_pt, self.plant_w_spr.GetBodyByName("toe_right").body_frame()
    else:
      return mid_pt, self.plant_w_spr.GetBodyByName("toe_left").body_frame()


  def calc_task_space_cv_force_translational(self, context, frame, point):
    world = self.plant_w_spr.world_frame()
    Cv = self.plant_w_spr.CalcBiasTerm(context)
    M = self.plant_w_spr.CalcMassMatrixViaInverseDynamics(context)
    J = self.plant_w_spr.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable.kV,
                                                frame, point, world, world)
    M_inv = np.linalg.inv(M)
    M_k = np.linalg.inv(J @ M_inv @ J.T)
    return M_k @ J @ M_inv @ Cv

  def calc_task_space_force_actuation_translational(self, context, frame, point, u_idx):
    world = self.plant_w_spr.world_frame()
    M = self.plant_w_spr.CalcMassMatrixViaInverseDynamics(context)
    J = self.plant_w_spr.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable.kV,
                                                           frame, point, world, world)
    B = self.plant_w_spr.MakeActuationMatrix()
    M_inv = np.linalg.inv(M)
    M_k = np.linalg.inv(J @ M_inv @ J.T)

    return M_k @ J @ M_inv @ B @ self.osc_output[u_idx].qp_output.u_sol

  def get_total_input(self):
    n_samples = len(self.t_u[self.t_u_slice])
    u = np.zeros((n_samples, self.nu))

    for i,osc in enumerate(self.osc_output[self.t_u_slice]):
      u[i,:] = osc.qp_output.u_sol

    return np.sum(np.abs(u), axis=1)



def main():
  bgcol = 'white'
  textcol = '262626'
  labelsize = 36
  titlesize = 36
  legend_textsize = 26
  ticksize = 24
  linewidth = 2.5
  font_family = 'sans-serif'
  font_sans = 'Public Sans'


  c = sns.plotting_context("talk")
  sns.set_theme(context=c, style="ticks")
  matplotlib.rcParams["axes.facecolor"] = bgcol
  matplotlib.rcParams["axes.edgecolor"] = textcol
  matplotlib.rcParams["axes.labelcolor"] = textcol
  matplotlib.rcParams["axes.labelsize"] = labelsize
  matplotlib.rcParams["axes.titlesize"] = titlesize
  matplotlib.rcParams["xtick.color"] = textcol
  matplotlib.rcParams["ytick.color"] = textcol
  matplotlib.rcParams["xtick.labelsize"] = ticksize
  matplotlib.rcParams["ytick.labelsize"] = ticksize
  matplotlib.rcParams["xtick.direction"] = 'in'
  matplotlib.rcParams["ytick.direction"] = 'in'
  matplotlib.rcParams["figure.facecolor"] = bgcol
  matplotlib.rcParams["savefig.facecolor"] = bgcol
  matplotlib.rcParams["savefig.edgecolor"] = textcol
  matplotlib.rcParams["font.family"] = font_family
  matplotlib.rcParams["font.sans-serif"] = font_sans
  matplotlib.rcParams["text.color"] = textcol
  matplotlib.rcParams["lines.color"] = textcol
  matplotlib.rcParams["lines.linewidth"] = linewidth
  matplotlib.rcParams["legend.fontsize"] = legend_textsize
  matplotlib.rcParams["legend.framealpha"] = 0.25
  matplotlib.rcParams["legend.facecolor"] = 'd6cac9'
  matplotlib.rcParams["text.usetex"]='true'

  builder = DiagramBuilder()
  plant_w_spr, _ = AddMultibodyPlantSceneGraph(builder, 0.0)
  plant_wo_spr, _ = AddMultibodyPlantSceneGraph(builder, 0.0)

  CROUCH = 0
  WALK = 1

  mode = WALK
  Parser(plant_w_spr).AddModelFromFile(
    FindResourceOrThrow(
      "examples/Cassie/urdf/cassie_v2.urdf"))
  Parser(plant_wo_spr).AddModelFromFile(
    FindResourceOrThrow(
      "examples/Cassie/urdf/cassie_v2.urdf"))
  plant_w_spr.mutable_gravity_field().set_gravity_vector(
    -9.81 * np.array([0, 0, 1]))
  plant_w_spr.Finalize()

  controller_channel = sys.argv[2]
  logs = []


  for pattern in sys.argv[1:]:
    for file in glob.glob(pattern):
      print("Loading" + file)
      log = Log(file, controller_channel, plant_w_spr, plant_wo_spr)
      if (mode == CROUCH):
        log.slice_for_crouch()
      elif (mode == WALK):
        log.slice_for_walk()

      logs.append(log)


  if (mode == CROUCH):
    plot_y_comparison(logs, "com_traj", "COM Height", "Crouch Step Response")
  else:
    #plot_y_comparison(logs, "lipm_traj", 2, "Z_COM", "COM Height Tracking During Stumble Event")
    plot_y_comparison(logs, "swing_ft_traj", 2, "$Z_{foot}$", "Swing Foot Height Position Tracking")
    #plot_V_comparison(logs, "swing_ft_traj", "MPTC Lyapunov Function (walking)")
    #plot_cv_swing_foot_comparison(logs, "Swing Foot Coriolis Force")
    #plot_cv_contribution_task_space(logs, "Operational Space Force Contributions - Swing Foot", 2)
    #plot_total_input(logs, "Total Control Effort")
    # plot_cv_swing_foot_error_force(logs[0], "Task Space Coriolis Forces")

  plt.show()


def plot_y_com_collection(logs):
  table = pd.DataFrame( columns=["time", "y_com"])
  for i,log in enumerate(logs):
    data = {"y_com": log.osc_debug["com_traj"].y[log.t_u_slice, 2],
            "time": log.osc_debug["com_traj"].t[log.t_u_slice] - log.osc_debug["com_traj"].t[log.t_u_slice.start] }
    frame = pd.DataFrame(data, columns=["time", "y_com"])
    table = table.append(frame)

  sns.lineplot(x="time", y="y_com", data=table)

def plot_cv_swing_foot(log, title):
  table = pd.DataFrame(columns=["Time", "|Cv|"])
  cv = log.get_cv_force_swing_ft()
  cv_norm = np.linalg.norm(cv, axis=1)

  data = {"Time": log.t_x[log.t_slice],
          "|Cv|": cv_norm}
  table = table.append(pd.DataFrame(data, columns=["Time", "|Cv|"]))
  ax = sns.lineplot(x="Time", y="|Cv|", data=table).set_title(title)

def plot_cv_swing_foot_error_force(log, title):
  table = pd.DataFrame(columns=["Time", "Cv", "Force"])
  cv = log.get_cv_force_swing_ft()
  cv_norm = np.linalg.norm(cv, axis=1)



  shifted_time = log.osc_debug["swing_ft_traj"].t[log.t_u_slice] - log.osc_debug["swing_ft_traj"].t[log.t_u_slice.start]
  end_idx = np.argwhere(np.abs(log.osc_debug["swing_ft_traj"].t - log.t_x[log.t_slice.stop]) < 2.5e-3)[0][0]
  shift_slice = slice(log.t_u_slice.start, end_idx)

  error_force_tx = np.interp(log.t_x[log.t_slice], log.osc_debug["swing_ft_traj"].t[shift_slice],
                             log.osc_debug["swing_ft_traj"].CkYdot[shift_slice])
  diff = cv_norm - error_force_tx

  data2 = {"Time": log.t_x[log.t_slice] - log.t_x[log.t_slice.start],
          "Cv": cv_norm,
          "Force": "Bias Force"}

  data1 = {"Time": shifted_time[shift_slice],
           "Cv": log.osc_debug["swing_ft_traj"].CkYdot[shift_slice],
           "Force": "Velocity Error Bias Force"}

  data3 = {"Time": log.t_x[log.t_slice] - log.t_x[log.t_slice.start],
           "Cv": diff,
           "Force": "Difference"}
  table = table.append(pd.DataFrame(data1, columns=["Time", "Cv", "Force"]))
  table = table.append(pd.DataFrame(data2, columns=["Time", "Cv", "Force"]))
  table = table.append(pd.DataFrame(data3, columns=["Time", "Cv", "Force"]))


  ax = sns.lineplot(x="Time", y="Cv", hue="Force", data=table)
  ax.set_title(title)
  ax.set_xlabel("Time (s)")
  ax.set_ylabel("$C_{os}$ (N)")
  plt.legend(bbox_to_anchor=(0,1), loc="upper left", ncol=4)


def plot_cv_swing_foot_comparison(logs, title):
  plt.figure("Plot Cv Swing Foot Comparison")
  table = pd.DataFrame(columns=["Time", "|Cv|", "Controller"])
  for log in logs:
    cv = log.get_cv_force_swing_ft()
    cv_norm = np.linalg.norm(cv, axis=1)

    data = {"Time": log.t_x[log.t_slice],
            "|Cv|": cv_norm,
            "Controller": log.controller}
    table = table.append(pd.DataFrame(data, columns=["Time", "|Cv|", "Controller"]))
  ax = sns.lineplot(x="Time", y="|Cv|", hue="Controller", data=table).set_title(title)
  plt.legend(bbox_to_anchor=(0,1), loc="upper left", ncol=3)

def plot_total_input(logs, title):
  plt.figure("Total Input Torque")
  table = pd.DataFrame(columns=["Time", "u", "Controller"])
  for log in logs:
    u = log.get_total_input()
    data = {"Time": log.t_u[log.t_u_slice], # - log.t_u[log.t_u_slice.start],
            "u": u,
            "Controller": log.controller}
    table = table.append(pd.DataFrame(data, columns=["Time", "u", "Controller"]))
  ax = sns.lineplot(x="Time", y="u", hue= "Controller", data=table)
  ax.set_title(title)
  ax.set_ylabel("$u_{tot}$ (Nm)")
  ax.set_xlabel("Time (s)")


def plot_cv_contribution_task_space(logs, title, dim):
  plt.figure("Plot Bu Task Space Contrib")
  table = pd.DataFrame(columns=["Time", "Force", "Component", "Controller"])
  for log in logs:
    bu = log.get_commanded_torque_force_swing_ft()
    bu_norm =np.linalg.norm(bu, axis=1)
    cv = log.get_cv_force_swing_ft()
    cv_norm = np.linalg.norm(cv, axis=1)

    data1 = {"Time": log.t_u[log.t_u_slice], # - log.t_u[log.t_u_slice.start],
            "Force": bu_norm,
            "Component": "Bu",
            "Controller": log.controller}

    data2 = {"Time": log.t_x[log.t_slice], # - log.t_x[log.t_slice.start],
             "Force": cv_norm,
             "Component": "Cv",
             "Controller": log.controller}
    table = table.append(pd.DataFrame(data1, columns=["Time", "Force", "Component", "Controller"]))
    table = table.append(pd.DataFrame(data2, columns=["Time", "Force", "Component", "Controller"]))
  ax = sns.lineplot(x="Time", y="Force", hue="Controller", style="Component", data=table)
  ax.set_title(title)
  ax.set_ylabel("$\parallel F_{os} \parallel$ (N)")
  ax.set_xlabel("Time (s)")
  plt.legend(bbox_to_anchor=(1,1), loc="upper right", ncol=2)


def plot_y_comparison(logs, traj_name, dim, signal_name, plot_title):
  table = pd.DataFrame(columns=["Time", "Y", "Controller", "Signal"])
  for log in logs:
    # if log.controller == "osc":
    #   print("osc!")
    #   log.osc_debug[traj_name].t = log.osc_debug[traj_name].t - 1.75
    #   log.t_u = log.t_u - 1.75
    #   log.t_u_slice = slice(np.argwhere(np.abs(log.t_u - 0.25) < 1e-3)[0][0], log.t_u_slice.stop)

    data1 = {"Y": log.osc_debug[traj_name].y[log.t_u_slice, dim],
            "Time": log.osc_debug[traj_name].t[log.t_u_slice], #- log.osc_debug[traj_name].t[log.t_u_slice.start],
            "Controller": log.controller,
            "Signal" : signal_name}
    data2 = {"Y": log.osc_debug[traj_name].y_des[log.t_u_slice, dim],
             "Time": log.osc_debug[traj_name].t[log.t_u_slice], #- log.osc_debug[traj_name].t[log.t_u_slice.start],
             "Controller": log.controller,
             "Signal" : "Des. " + signal_name}
    frame1 = pd.DataFrame(data1, columns=["Time", "Y", "Controller", "Signal"])
    frame2 = pd.DataFrame(data2, columns=["Time", "Y", "Controller", "Signal"])
    table = table.append(frame1)
    table = table.append(frame2)

  ax = sns.lineplot(x="Time", y="Y", hue="Controller", style="Signal", data=table)
  ax.set_title(plot_title)
  ax.set_xlabel("Time (s)")
  ax.set_ylabel(signal_name + "(m)")
  plt.legend(bbox_to_anchor=(0, 1), loc="upper left", ncol=2)

def plot_V_comparison(logs, traj_name, plot_title):
  plt.figure("Lyapunov Function")
  table = pd.DataFrame(columns=["Time", "V", "Controller"])
  for log in logs:
    data1 = {"V": log.osc_debug[traj_name].V[log.t_u_slice], #/ np.max(log.osc_debug[traj_name].V[log.t_u_slice]),
             "Time": log.osc_debug[traj_name].t[log.t_u_slice], # - log.osc_debug[traj_name].t[log.t_u_slice.start],
             "Controller": log.controller}
    frame1 = pd.DataFrame(data1, columns=["Time", "V", "Controller"])
    table = table.append(frame1)

  ax = sns.lineplot(x="Time", y="V", hue="Controller",  data=table).set_title(plot_title)
  plt.legend(bbox_to_anchor=(0,1), loc="upper left", ncol=3)

def plot_cv_springs_(logs, traj_name, plot_title):
  plt.figure("Task Space Coriolis Forces")


  # controller_channel = sys.argv[2]
  # log = lcm.EventLog(filename, "r")
  # log1 = Log(filename, controller_channel, plant_w_spr, plant_wo_spr)
  #
  # path = pathlib.Path(filename).parent
  # filename = filename.split("/")[-1]
  #
  #
  #
  # matplotlib.rcParams["savefig.directory"] = path
  #
  # x, u_meas, t_x, u, t_u, contact_info, contact_info_locs, t_contact_info, \
  # osc_debug, fsm, estop_signal, switch_signal, t_controller_switch, t_pd, kp, kd, cassie_out, u_pd, t_u_pd, \
  # osc_output, full_log, t_target_height, target_height = \
  # process_lcm_log.process_log(log, pos_map, vel_map, act_map, controller_channel)
  #
  # n_msgs = len(cassie_out)
  # knee_pos = np.zeros(n_msgs)
  # t_cassie_out = np.zeros(n_msgs)
  # estop_signal = np.zeros(n_msgs)
  # motor_torques = np.zeros(n_msgs)
  #
  # for i in range(n_msgs):
  #   knee_pos[i] = cassie_out[i].leftLeg.kneeDrive.velocity
  #   t_cassie_out[i] = cassie_out[i].utime / 1e6
  #   motor_torques[i] = cassie_out[i].rightLeg.kneeDrive.torque
  #   estop_signal[i] = cassie_out[i].pelvis.radio.channel[8]
  #
  # # Default time window values, can override
  # t_start = t_u[10]
  # t_end = t_u[-10]
  #
  # # Override here #
  # # t_start = 20
  # # t_end = 23
  #
  # ### Convert times to indices
  #
  # t_u_slice = get_crouch_slice(osc_debug, target_height, t_u, t_target_height)
  # #t_u_slice = get_walking_slice(TODO @Brian-Acosta implement get_walking_slice)
  #
  # t_start_idx = np.argwhere(np.abs(t_x - t_u[t_u_slice.start]) < 1e-3)[0][0]
  # t_end_idx = np.argwhere(np.abs(t_x - t_u[t_u_slice.stop]) < 1e-3)[0][0]
  # t_slice = slice(t_start_idx, t_end_idx)
  #
  # ### All plotting scripts here
  # #plot_state(x, t_x, u, t_u, x_datatypes, u_datatypes)
  #
  # # plot_contact_est(full_log)
  # # plt.plot(t_contact_info, contact_info[0, :, 2], 'b-')
  # # plt.plot(t_contact_info, contact_info[2, :, 2], 'r-')
  # # plt.plot(t_u[t_u_slice], 100 * fsm[t_u_slice], 'k')
  #
  # plt.ylim([-100, 500])
  # # plt.plot(t_u[t_u_slice], fsm[t_u_slice])
  #
  # plot_osc_debug(t_u, fsm, osc_debug, t_cassie_out, estop_signal, osc_output)
  # plot_mptc_lyapunov_function(osc_debug, "com_traj")
  # plt.show()


def plot_osc_debug(t_u, fsm, osc_debug, t_cassie_out, estop_signal, osc_output):
  input_cost = np.zeros(t_u.shape[0])
  acceleration_cost = np.zeros(t_u.shape[0])
  soft_constraint_cost = np.zeros(t_u.shape[0])
  tracking_cost = np.zeros((t_u.shape[0], len(osc_debug)))
  tracking_cost_map = dict()
  num_tracking_cost = 0

  for i in range(t_u.shape[0] - 10):
    input_cost[i] = osc_output[i].input_cost
    acceleration_cost[i] = osc_output[i].acceleration_cost
    soft_constraint_cost[i] = osc_output[i].soft_constraint_cost
    for j in range(len(osc_output[i].tracking_data_names)):
      name = osc_output[i].tracking_data_names[j]
      if osc_output[i].tracking_data_names[j] not in tracking_cost_map:
        tracking_cost_map[name] = num_tracking_cost
        num_tracking_cost += 1
      tracking_cost[i, tracking_cost_map[name]] = osc_output[i].tracking_cost[j]

  for name in tracking_cost_map.keys():
    print(name)
    print(tracking_cost_map[name])

  plt.figure("costs")
  plt.plot(t_u[t_u_slice], input_cost[t_u_slice])
  plt.plot(t_u[t_u_slice], acceleration_cost[t_u_slice])
  plt.plot(t_u[t_u_slice], soft_constraint_cost[t_u_slice])
  plt.plot(t_u[t_u_slice], tracking_cost[t_u_slice])
  plt.legend(['input_cost', 'acceleration_cost', 'soft_constraint_cost'] +
             list(tracking_cost_map))

  osc_traj0 = "com_traj"
  osc_traj1 = "lipm_traj"
  osc_traj2 = "swing_ft_traj"

  #
  plot_osc(osc_debug, osc_traj0, 0, "pos")
  # plt.plot(osc_debug[osc_traj1].t[t_u_slice], fsm[t_u_slice])
  plot_osc(osc_debug, osc_traj0, 1, "pos")
  plot_osc(osc_debug, osc_traj0, 2, "pos")
  # plt.plot(osc_debug[osc_traj1].t[t_u_slice], fsm[t_u_slice])
  #
  plot_osc(osc_debug, osc_traj0, 0, "vel")
  plot_osc(osc_debug, osc_traj0, 1, "vel")
  plot_osc(osc_debug, osc_traj0, 2, "vel")
  #
  plot_osc(osc_debug, osc_traj0, 0, "accel")
  plot_osc(osc_debug, osc_traj0, 1, "accel")
  plot_osc(osc_debug, osc_traj0, 2, "accel")



def plot_osc(osc_debug, osc_traj, dim, derivative):
  fig = plt.figure(osc_traj + " " + derivative + " tracking " + str(dim))
  if (derivative == "pos"):
    plt.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].y_des[t_u_slice, dim])
    plt.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].y[t_u_slice, dim])
    plt.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].error_y[t_u_slice, dim])
    plt.legend(["y_des", "y", "error_y"])
    # plt.legend(["y_des", "y"])
  elif (derivative == "vel"):
    plt.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].ydot_des[t_u_slice, dim])
    plt.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].ydot[t_u_slice, dim])
    plt.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].error_ydot[t_u_slice, dim])
    plt.legend(["ydot_des", "ydot", "error_ydot"])
  elif (derivative == "accel"):
    plt.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].yddot_des[t_u_slice, dim])
    plt.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].yddot_command[t_u_slice, dim])
    plt.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].yddot_command_sol[t_u_slice, dim])
    plt.legend(["yddot_des", "yddot_command", "yddot_command_sol"])


def plot_feet_positions(plant, context, x, toe_frame, contact_point, world,
                        t_x, t_x_slice, foot_type, contact_type):
  foot_x = np.zeros((6, t_x.size))
  for i in range(t_x.size):
    plant.SetPositionsAndVelocities(context, x[i, :])
    foot_x[0:3, [i]] = plant.CalcPointsPositions(context, toe_frame,
                                                 contact_point, world)
    foot_x[3:6, i] = plant.CalcJacobianTranslationalVelocity(
      context, JacobianWrtVariable.kV, toe_frame, contact_point,
      world,
      world) @ x[i, -nv:]
  fig = plt.figure('foot pos: ' + filename)
  # state_indices = slice(4, 5)
  state_indices = slice(0, 1)
  # state_indices = slice(5, 6)
  # state_indices = slice(5, 6)
  state_names = ["x", "y", "z", "xdot", "ydot", "zdot"]
  state_names = [foot_type + name for name in state_names]
  state_names = [name + contact_type for name in state_names]
  plt.plot(t_x[t_x_slice], foot_x.T[t_x_slice, state_indices],
           label=state_names[state_indices])
  plt.legend()


def get_crouch_slice(osc_debug, target_height, t_u, t_target_height):
  t0_idx = np.argwhere(osc_debug["com_traj"].y_des[:,2] < 0.5)[0][0]
  t1_remote = np.argwhere(target_height < 0.5)[0][0]
  t1 = t_target_height[1]
  t2 = t_target_height[-1] + 1.0
  t0_shift = t_target_height[t1_remote] - t_u[t0_idx]

  t_start_idx = np.argwhere(np.abs(t_u - (t1 - t0_shift - 1)) < 1e-3)[0][0]
  t_stop_idx = np.argwhere(np.abs(t_u - (t2 - t0_shift)) < 1e-3)[0][0]

  return slice(t_start_idx, t_stop_idx)

def get_walk_slice(osc_debug, fsm, t_u):
  t_start = 1.555
  t_end = t_start + 0.3

  start_time_idx = np.argwhere(np.abs(t_u - t_start) < 1e-3)[0][0]
  end_time_idx = np.argwhere(np.abs(t_u - t_end) < 1e-3)[0][0]
  return slice(start_time_idx, end_time_idx)

def plot_mptc_lyapunov_function(osc_debug, osc_traj):
  fig = plt.figure(osc_traj + " lyapunov function ")
  plt.plot(osc_debug[osc_traj].t[t_u_slice], osc_debug[osc_traj].V[t_u_slice])

def plot_state(x, t_x, u, t_u, x_datatypes, u_datatypes):
  # pos_indices = slice(0 + 7, 23)
  vel_indices = slice(23 + 6, 45)
  pos_indices = slice(0,7)
  # vel_indices = slice(23, 23 + 6)
  u_indices = slice(0, 10)
  # overwrite
  # pos_indices = [pos_map["knee_joint_right"], pos_map["ankle_spring_joint_right"]]
  # pos_indices = tuple(slice(x) for x in pos_indices)

  plt.figure("positions: " + filename)
  # plt.plot(t_x[t_slice], x[t_slice, pos_map["knee_joint_right"]])
  # plt.plot(t_x[t_slice], x[t_slice, pos_map["ankle_spring_joint_right"]])
  plt.plot(t_x[t_slice], x[t_slice, pos_indices])
  plt.legend(x_datatypes[pos_indices])
  plt.figure("velocities: " + filename)
  plt.plot(t_x[t_slice], x[t_slice, vel_indices])
  plt.legend(x_datatypes[vel_indices])
  plt.figure("efforts: " + filename)
  plt.plot(t_u[t_u_slice], u[t_u_slice, u_indices])
  plt.legend(u_datatypes[u_indices])
  # plt.figure("efforts meas: " + filename)
  # plt.figure("Delay characterization")
  # plt.plot(t_x[t_slice], u_meas[t_slice, u_indices])
  # plt.legend(u_datatypes[u_indices])


if __name__ == "__main__":
  main()

import dairlib
import drake
import numpy as np


class lcmt_osc_tracking_data_t:
  def __init__(self):
    self.t = []
    self.y_dim = 0
    self.name = ""
    self.is_active = []
    self.y = []
    self.y_des = []
    self.error_y = []
    self.ydot = []
    self.ydot_des = []
    self.error_ydot = []
    self.yddot_des = []
    self.yddot_command = []
    self.yddot_command_sol = []

  def append(self, msg, t):
    self.t.append(t)
    self.is_active.append(msg.is_active)
    self.y.append(msg.y)
    self.y_des.append(msg.y_des)
    self.error_y.append(msg.error_y)
    self.ydot.append(msg.ydot)
    self.ydot_des.append(msg.ydot_des)
    self.error_ydot.append(msg.error_ydot)
    self.yddot_des.append(msg.yddot_des)
    self.yddot_command.append(msg.yddot_command)
    self.yddot_command_sol.append(msg.yddot_command_sol)

  def convertToNP(self):
    self.t = np.array(self.t)
    self.is_active = np.array(self.is_active)
    self.y = np.array(self.y)
    self.y_des = np.array(self.y_des)
    self.error_y = np.array(self.error_y)
    self.ydot = np.array(self.ydot)
    self.ydot_des = np.array(self.ydot_des)
    self.error_ydot = np.array(self.error_ydot)
    self.yddot_des = np.array(self.yddot_des)
    self.yddot_command = np.array(self.yddot_command)
    self.yddot_command_sol = np.array(self.yddot_command_sol)


def process_log(log, pos_map, vel_map):
  t_x = []
  t_u = []
  t_controller_switch = []
  t_contact_info = []
  fsm = []
  q = []
  v = []
  u = []
  kp = []
  kd = []
  t_pd = []
  estop_signal = []
  switch_signal = []
  osc_debug = dict()
  contact_info = [[], [], [], []]
  contact_info_locs = [[], [], [], []]
  cassie_out  = []

  # is_mujoco = False
  for event in log:
    if event.channel == "CASSIE_STATE_SIMULATION" or event.channel == "CASSIE_STATE_DISPATCHER":
      msg = dairlib.lcmt_robot_output.decode(event.data)
      q_temp = [[] for i in range(len(msg.position))]
      v_temp = [[] for i in range(len(msg.velocity))]
      for i in range(len(q_temp)):
        q_temp[pos_map[msg.position_names[i]]] = msg.position[i]
      for i in range(len(v_temp)):
        v_temp[vel_map[msg.velocity_names[i]]] = msg.velocity[i]
      q.append(q_temp)
      v.append(v_temp)
      t_x.append(msg.utime / 1e6)
    if event.channel == "CASSIE_INPUT" or event.channel == "PD_CONTROL":
      msg = dairlib.lcmt_robot_input.decode(event.data)
      u.append(msg.efforts)
      t_u.append(msg.utime / 1e6)
    if event.channel == "INPUT_SWITCH":
      msg = dairlib.lcmt_controller_switch.decode(event.data)
      switch_signal.append(msg.channel == "OSC_STANDING")
      t_controller_switch.append(msg.utime / 1e6)
    if event.channel == "PD_CONFIG":
      msg = dairlib.lcmt_pd_config.decode(event.data)
      kp.append(msg.kp)
      kd.append(msg.kd)
      t_pd.append(msg.timestamp / 1e6)
    if event.channel == "CASSIE_OUTPUT_ECHO":
      msg = dairlib.lcmt_cassie_out.decode(event.data)
      cassie_out.append(msg)
    if event.channel == "OSC_DEBUG":
      msg = dairlib.lcmt_osc_output.decode(event.data)
      num_osc_tracking_data = len(msg.tracking_data)
      for i in range(num_osc_tracking_data):
        if msg.tracking_data[i].name not in osc_debug:
          osc_debug[msg.tracking_data[i].name] = lcmt_osc_tracking_data_t()
        osc_debug[msg.tracking_data[i].name].append(msg.tracking_data[i], msg.utime / 1e6)
      fsm.append(msg.fsm_state)
    if event.channel == "CASSIE_CONTACT_RESULTS" or event.channel \
        == "CASSIE_CONTACT_DRAKE" or event.channel == "CASSIE_CONTACT_MUJOCO":
      # Need to distinguish between front and rear contact forces
      # Best way is to track the contact location and group by proximity
      msg = drake.lcmt_contact_results_for_viz.decode(event.data)
      t_contact_info.append(msg.timestamp / 1e6)
      num_left_contacts = 0
      num_right_contacts = 0
      for i in range(msg.num_point_pair_contacts):
        if "toe_left" in msg.point_pair_contact_info[i].body2_name:
          contact_info_locs[num_left_contacts].append(
            msg.point_pair_contact_info[i].contact_point)
          contact_info[num_left_contacts].append(
            msg.point_pair_contact_info[i].contact_force)
          num_left_contacts += 1
        elif "toe_right" in msg.point_pair_contact_info[i].body2_name:
          contact_info_locs[2 + num_right_contacts].append(
            msg.point_pair_contact_info[i].contact_point)
          contact_info[2 + num_right_contacts].append(
            msg.point_pair_contact_info[i].contact_force)
          num_right_contacts += 1
          # print("ERROR")
      while num_left_contacts != 2:
        contact_info[num_left_contacts].append((0.0, 0.0, 0.0))
        contact_info_locs[num_left_contacts].append((0.0, 0.0, 0.0))
        num_left_contacts += 1
      while num_right_contacts != 2:
        contact_info[2 + num_right_contacts].append((0.0, 0.0, 0.0))
        contact_info_locs[2 + num_right_contacts].append((0.0, 0.0,
                                                          0.0))
        num_right_contacts += 1
    # IMPORTANT: should not have two simulators in the same log
    # if event.channel == "CASSIE_CONTACT_MUJOCO":
    #   msg = dairlib.lcmt_cassie_mujoco_contact.decode(event.data)
    #   t_contact_info.append(msg.utime / 1e6)
    #   contact_info[0].append(msg.contact_forces[0:3])
    #   contact_info[1].append((0.0, 0.0, 0.0))
    #   contact_info[2].append(msg.contact_forces[6:9])
    #   contact_info[3].append((0.0, 0.0, 0.0))
    #   is_mujoco = True

  # Convert into numpy arrays
  t_x = np.array(t_x)
  t_u = np.array(t_u)
  t_controller_switch = np.array(t_controller_switch)
  t_contact_info = np.array(t_contact_info)
  t_pd = np.array(t_pd)
  fsm = np.array(fsm)
  q = np.array(q)
  v = np.array(v)
  u = np.array(u)
  kp = np.array(kp)
  kd = np.array(kd)
  estop_signal = np.array(estop_signal)
  switch_signal = np.array(switch_signal)
  contact_info = np.array(contact_info)
  contact_info_locs = np.array(contact_info_locs)

  for i in range(contact_info_locs.shape[1]):
    # Swap front and rear contacts if necessary
    # Order will be front in index 1
    if contact_info_locs[0, i, 0] > contact_info_locs[1, i, 0]:
      contact_info[[0, 1], i, :] = contact_info[[1, 0], i, :]
      contact_info_locs[[0, 1], i, :] = contact_info_locs[[1, 0], i, :]
    if contact_info_locs[2, i, 0] > contact_info_locs[3, i, 0]:
      contact_info[[2, 3], i, :] = contact_info[[3, 2], i, :]
      contact_info_locs[[2, 3], i, :] = contact_info_locs[[3, 2], i, :]

  # for i in range(len(osc_debug)):
  for key in osc_debug:
    osc_debug[key].convertToNP()

  x = np.hstack((q, v))

  return x, t_x, u, t_u, contact_info, contact_info_locs, t_contact_info, osc_debug, fsm, estop_signal, \
         switch_signal, t_controller_switch, t_pd, kp, kd, cassie_out


def generate_wo_spring_state_map():
  pos_map = dict()
  vel_map = dict()
  mot_map = dict()
  pos_map = {
    "q_w": 0,
    "q_x": 1,
    "q_y": 2,
    "q_z": 3,
    "base_x": 4,
    "base_y": 5,
    "base_z": 6,
    "hip_roll_left": 7,
    "hip_roll_right": 8,
    "hip_yaw_left": 9,
    "hip_yaw_right": 10,
    "hip_pitch_left": 11,
    "hip_pitch_right": 12,
    "knee_left": 13,
    "knee_right": 14,
    "ankle_left": 15,
    "ankle_right": 16,
    "toe_left": 17,
    "toe_right": 18
  }
  vel_map = {
    "q_x_dot": 0,
    "q_y_dot": 1,
    "q_z_dot": 2,
    "base_x_dot": 3,
    "base_y_dot": 4,
    "base_z_dot": 5,
    "hip_roll_left_dot": 6,
    "hip_roll_right_dot": 7,
    "hip_yaw_left_dot": 8,
    "hip_yaw_right_dot": 9,
    "hip_pitch_left_dot": 10,
    "hip_pitch_right_dot": 11,
    "knee_left_dot": 12,
    "knee_right_dot": 13,
    "ankle_left_dot": 14,
    "ankle_right_dot": 15,
    "toe_left_dot": 16,
    "toe_right_dot": 17
  }
  effort_map = {
    "hip_roll_left_motor": 0,
    "hip_roll_right_motor": 1,
    "hip_yaw_left_motor": 2,
    "hip_yaw_right_motor": 3,
    "hip_pitch_left_motor": 4,
    "hip_pitch_right_motor": 5,
    "knee_left_motor": 6,
    "knee_right_motor": 7,
    "toe_left_motor": 8,
    "toe_right_motor": 9
  }

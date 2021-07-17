import dairlib
import drake
import numpy as np


# Class to easily convert list of lcmt_osc_tracking_data_t to numpy arrays
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


def process_log(log, pos_map, vel_map, act_map, controller_channel):
  t_x = []
  t_u = []
  t_controller_switch = []
  t_contact_info = []
  fsm = []
  q = []
  v = []
  u_meas = []
  u = []
  kp = []
  kd = []
  t_pd = []
  estop_signal = []
  switch_signal = []
  osc_debug = dict()
  contact_forces = [[], [], [], []]  # Allocate space for all 4 point contacts
  contact_info_locs = [[], [], [], []]
  cassie_out = []  # Cassie out types
  osc_output = []
  u_pd = []
  t_u_pd = []

  full_log = dict()
  channel_to_type_map = dict()
  unknown_types = set()
  known_lcm_types = [dairlib.lcmt_robot_output, dairlib.lcmt_cassie_out, dairlib.lcmt_controller_switch,
                     dairlib.lcmt_osc_output, dairlib.lcmt_pd_config, dairlib.lcmt_robot_input,
                     drake.lcmt_contact_results_for_viz, dairlib.lcmt_contact]

  for event in log:
    if event.channel not in full_log and event.channel not in unknown_types:
      for lcmtype in known_lcm_types:
        try:
          lcmtype.decode(event.data)
          channel_to_type_map[event.channel] = lcmtype
        except ValueError:
          continue
      if event.channel in channel_to_type_map:
        full_log[event.channel] = []
      else:
        unknown_types.add(event.channel)
    if event.channel in full_log:
      full_log[event.channel].append(channel_to_type_map[event.channel].decode(event.data))
    if event.channel == "CASSIE_STATE_SIMULATION":
      msg = dairlib.lcmt_robot_output.decode(event.data)
      q_temp = [[] for i in range(len(msg.position))]
      v_temp = [[] for i in range(len(msg.velocity))]
      u_temp = [[] for i in range(len(msg.effort))]
      for i in range(len(q_temp)):
        q_temp[pos_map[msg.position_names[i]]] = msg.position[i]
      for i in range(len(v_temp)):
        v_temp[vel_map[msg.velocity_names[i]]] = msg.velocity[i]
      for i in range(len(u_temp)):
        u_temp[act_map[msg.effort_names[i]]] = msg.effort[i]
      q.append(q_temp)
      v.append(v_temp)
      u_meas.append(u_temp)
      t_x.append(msg.utime / 1e6)
    # if event.channel == "CASSIE_INPUT" or event.channel == "PD_CONTROL":
    if event.channel == "CASSIE_INPUT" or event.channel == controller_channel:
      msg = dairlib.lcmt_robot_input.decode(event.data)
      u.append(msg.efforts)
      t_u.append(msg.utime / 1e6)
    if event.channel == "PD_CONTROL":
      msg = dairlib.lcmt_robot_input.decode(event.data)
      u_pd.append(msg.efforts)
      t_u_pd.append(msg.utime / 1e6)
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
    if event.channel == "OSC_DEBUG_WALKING":
      msg = dairlib.lcmt_osc_output.decode(event.data)
      osc_output.append(msg)
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
          contact_forces[num_left_contacts].append(
            msg.point_pair_contact_info[i].contact_force)
          num_left_contacts += 1
        elif "toe_right" in msg.point_pair_contact_info[i].body2_name:
          contact_info_locs[2 + num_right_contacts].append(
            msg.point_pair_contact_info[i].contact_point)
          contact_forces[2 + num_right_contacts].append(
            msg.point_pair_contact_info[i].contact_force)
          num_right_contacts += 1
          # print("ERROR")
      while num_left_contacts != 2:
        contact_forces[num_left_contacts].append((0.0, 0.0, 0.0))
        contact_info_locs[num_left_contacts].append((0.0, 0.0, 0.0))
        num_left_contacts += 1
      while num_right_contacts != 2:
        contact_forces[2 + num_right_contacts].append((0.0, 0.0, 0.0))
        contact_info_locs[2 + num_right_contacts].append((0.0, 0.0,
                                                          0.0))
        num_right_contacts += 1

  # Convert into numpy arrays
  t_x = np.array(t_x)
  t_u = np.array(t_u)
  t_controller_switch = np.array(t_controller_switch)
  t_contact_info = np.array(t_contact_info)
  t_pd = np.array(t_pd)
  fsm = np.array(fsm)
  q = np.array(q)
  v = np.array(v)
  u_meas = np.array(u_meas)
  u = np.array(u)
  u_pd = np.array(u_pd)
  kp = np.array(kp)
  kd = np.array(kd)
  estop_signal = np.array(estop_signal)
  switch_signal = np.array(switch_signal)
  contact_forces = np.array(contact_forces)
  contact_info_locs = np.array(contact_info_locs)

  for i in range(contact_info_locs.shape[1]):
    # Swap front and rear contacts if necessary
    # Order will be front contact in index 1
    if contact_info_locs[0, i, 0] > contact_info_locs[1, i, 0]:
      contact_forces[[0, 1], i, :] = contact_forces[[1, 0], i, :]
      contact_info_locs[[0, 1], i, :] = contact_info_locs[[1, 0], i, :]
    if contact_info_locs[2, i, 0] > contact_info_locs[3, i, 0]:
      contact_forces[[2, 3], i, :] = contact_forces[[3, 2], i, :]
      contact_info_locs[[2, 3], i, :] = contact_info_locs[[3, 2], i, :]

  for key in osc_debug:
    osc_debug[key].convertToNP()

  x = np.hstack((q, v))  # combine into state vector

  return x, u_meas, t_x, u, t_u, contact_forces, contact_info_locs, \
         t_contact_info, osc_debug, fsm, estop_signal, \
         switch_signal, t_controller_switch, t_pd, kp, kd, cassie_out, u_pd, \
         t_u_pd, osc_output, full_log

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

def process_log(log, pos_map, vel_map):

  t_state = []
  t_lqr = []
  fsm = []
  q = []
  v = []
  u = []
  contact_info = [[], []]
  contact_info_locs = [[], []]
  osc_debug = dict()
  osc_output = []
  t_lcmlog_u = []

  for event in log:
    if event.channel == "RABBIT_STATE":
      msg = dairlib.lcmt_robot_output.decode(event.data)
      q_temp = [[] for i in range(len(msg.position))]
      v_temp = [[] for i in range(len(msg.velocity))]
      for i in range(len(q_temp)):
        q_temp[pos_map[msg.position_names[i]]] = msg.position[i]
      for i in range(len(v_temp)):
        v_temp[vel_map[msg.velocity_names[i]]] = msg.velocity[i]
      # import pdb; pdb.set_trace()
      q.append(q_temp)
      v.append(v_temp)
      t_state.append(msg.utime / 1e6)
    if event.channel == "RABBIT_INPUT":
      msg = dairlib.lcmt_robot_input.decode(event.data)
      u.append(msg.efforts)
      t_lqr.append(msg.utime / 1e6)
    if event.channel == "OSC_DEBUG_WALKING":
      msg = dairlib.lcmt_osc_output.decode(event.data)
      t_lcmlog_u.append(event.timestamp / 1e6)
      osc_output.append(msg)
      num_osc_tracking_data = len(msg.tracking_data)
      for i in range(num_osc_tracking_data):
        if msg.tracking_data[i].name not in osc_debug:
          osc_debug[msg.tracking_data[i].name] = lcmt_osc_tracking_data_t()
        osc_debug[msg.tracking_data[i].name].append(msg.tracking_data[i], msg.utime / 1e6)
      fsm.append(msg.fsm_state)
    if event.channel == "CONTACT_RESULTS":
      # Need to distinguish between front and rear contact forces
      # Best way is to track the contact location and group by proximity
      msg = drake.lcmt_contact_results_for_viz.decode(event.data)
      # t_contact_info.append(msg.timestamp / 1e6)
      num_left_contacts = 0
      num_right_contacts = 0
      for i in range(msg.num_point_pair_contacts):
        if msg.point_pair_contact_info[i].body1_name == \
            "left_lower_leg(2)":
          contact_info_locs[0].append(
            msg.point_pair_contact_info[i].contact_point)
          contact_info[0].append(
            msg.point_pair_contact_info[i].contact_force)
          num_left_contacts += 1
        elif msg.point_pair_contact_info[
          i].body1_name == "right_lower_leg(2)":
          contact_info_locs[1].append(
            msg.point_pair_contact_info[i].contact_point)
          contact_info[1].append(
            msg.point_pair_contact_info[i].contact_force)
          num_right_contacts += 1
        else:
          print("ERROR")
      if(num_left_contacts != 1):
        contact_info[0].append((0.0, 0.0, 0.0))
        contact_info_locs[0].append((0.0, 0.0, 0.0))
      if(num_right_contacts != 1):
        contact_info[1].append((0.0, 0.0, 0.0))
        contact_info_locs[1].append((0.0, 0.0, 0.0))

  # NOTE: t_fsm has the same timestamps as t_lqr
  # NOTE: t_contact has the same timestamps as t_state
  # ideally they should all be the same

  t_state = np.array(t_state)
  t_lqr = np.array(t_lqr)
  t_lcmlog_u = np.array(t_lcmlog_u)
  q = np.array(q)
  v = np.array(v)
  x = np.hstack((q, v))
  fsm = np.array(fsm)
  u = np.array(u)
  contact_info = -1 * np.array(contact_info)
  contact_info_locs = np.array(contact_info_locs)

  for key in osc_debug:
    osc_debug[key].convertToNP()

  return t_state, t_lqr, \
         x, u, fsm, contact_info, contact_info_locs, osc_debug, osc_output, t_lcmlog_u

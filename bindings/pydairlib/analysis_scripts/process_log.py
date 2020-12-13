import dairlib
import drake
import numpy as np

def process_log(log, pos_map, vel_map):

  t_state = []
  t_lqr = []
  fsm = []
  q = []
  v = []
  u = []
  contact_info = [[], []]
  contact_info_locs = [[], []]

  for event in log:
    if event.channel == "RABBIT_STATE_SIMULATION":
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
    # if event.channel == "FSM":
    #   msg = dairlib.lcmt_fsm_out.decode(event.data)
    #   fsm.append(msg.fsm_state)
    #   # t_fsm.append(msg.utime / 1e6)
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
  q = np.array(q)
  v = np.array(v)
  x = np.hstack((q, v))
  fsm = np.array(fsm)
  u = np.array(u)
  contact_info = -1 * np.array(contact_info)
  contact_info_locs = np.array(contact_info_locs)
  return t_state, t_lqr, \
         x, u, fsm, contact_info, contact_info_locs

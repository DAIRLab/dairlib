import dairlib
import drake
import numpy as np

class lcmt_osc_tracking_data_t:
    def __init__(self):
        self.y_dim = 0
        self.name = ""
        self.is_active = []
        self.y = []
        self.y_des = []
        self.error_y = []
        self.dy = []
        self.dy_des = []
        self.error_dy = []
        self.ddy_des = []
        self.ddy_command = []
        self.ddy_command_sol = []

    def append(self, msg):
        self.is_active.append(msg.is_active)
        self.y.append(msg.y)
        self.y_des.append(msg.y_des)
        self.error_y.append(msg.error_y)
        self.dy.append(msg.dy)
        self.dy_des.append(msg.dy_des)
        self.error_dy.append(msg.error_dy)
        self.ddy_des.append(msg.ddy_des)
        self.ddy_command.append(msg.ddy_command)
        self.ddy_command_sol.append(msg.ddy_command_sol)

    def convertToNP(self):
        self.is_active = np.array(self.is_active)
        self.y = np.array(self.y)
        self.y_des = np.array(self.y_des)
        self.error_y = np.array(self.error_y)
        self.dy = np.array(self.dy)
        self.dy_des = np.array(self.dy_des)
        self.error_dy = np.array(self.error_dy)
        self.ddy_des = np.array(self.ddy_des)
        self.ddy_command = np.array(self.ddy_command)
        self.ddy_command_sol = np.array(self.ddy_command_sol)

def process_log(log, pos_map, vel_map):

    t_state = []
    t_osc = []
    t_controller_switch = []
    t_osc_debug = []
    t_contact_info = []
    q = []
    v = []
    control_inputs = []
    estop_signal = []
    switch_signal = []
    osc_debug = [lcmt_osc_tracking_data_t() for i in range(4)]
    contact_info = [[], [], [], []]
    contact_info_locs = [[], [], [], []]

    is_mujoco = False
    for event in log:
        if event.channel == "CASSIE_STATE":
            msg = dairlib.lcmt_robot_output.decode(event.data)
            q_temp = [[] for i in range(len(msg.position))]
            v_temp = [[] for i in range(len(msg.velocity))]
            for i in range(len(q_temp)):
                q_temp[pos_map[msg.position_names[i]]] = msg.position[i]
            for i in range(len(v_temp)):
                v_temp[vel_map[msg.velocity_names[i]]] = msg.velocity[i]
            # import pdb; pdb.set_trace()
            # q.append(msg.position)
            # v.append(msg.velocity)
            q.append(q_temp)
            v.append(v_temp)
            t_state.append(msg.utime / 1e6)
        if event.channel == "CASSIE_INPUT":
            msg = dairlib.lcmt_robot_input.decode(event.data)
            control_inputs.append(msg.efforts)
            t_osc.append(msg.utime / 1e6)
        if event.channel == "INPUT_SWITCH":
            msg = dairlib.lcmt_controller_switch.decode(event.data)
            switch_signal.append(msg.channel == "OSC_STANDING")
            t_controller_switch.append(msg.utime / 1e6)
        if event.channel == "OSC_DEBUG":
            msg = dairlib.lcmt_osc_output.decode(event.data)
            num_osc_tracking_data = len(msg.tracking_data)
            for i in range(num_osc_tracking_data):
                osc_debug[i].append(msg.tracking_data[i])
            t_osc_debug.append(msg.utime / 1e6)
        if event.channel == "CASSIE_CONTACT_RESULTS" or event.channel \
                == "CASSIE_CONTACT_DRAKE":
            # Need to distinguish between front and rear contact forces
            # Best way is to track the contact location and group by proximity
            msg = drake.lcmt_contact_results_for_viz.decode(event.data)
            t_contact_info.append(msg.timestamp / 1e6)
            num_left_contacts = 0
            num_right_contacts = 0
            for i in range(msg.num_point_pair_contacts):
                if msg.point_pair_contact_info[i].body2_name == "toe_left(2)":
                    contact_info_locs[num_left_contacts].append(
                        msg.point_pair_contact_info[i].contact_point)
                    contact_info[num_left_contacts].append(
                        msg.point_pair_contact_info[i].contact_force)
                    num_left_contacts += 1
                elif msg.point_pair_contact_info[
                    i].body2_name == "toe_right(2)":
                    contact_info_locs[2 + num_right_contacts].append(
                        msg.point_pair_contact_info[i].contact_point)
                    contact_info[2 + num_right_contacts].append(
                        msg.point_pair_contact_info[i].contact_force)
                    num_right_contacts += 1
                else:
                    print("ERROR")
            while num_left_contacts != 2:
                contact_info[num_left_contacts].append((0.0, 0.0, 0.0))
                contact_info_locs[num_left_contacts].append((0.0, 0.0, 0.0))
                num_left_contacts += 1
            while num_right_contacts != 2:
                contact_info[2 + num_right_contacts].append((0.0, 0.0, 0.0))
                contact_info_locs[2 + num_right_contacts].append((0.0, 0.0,
                                                                  0.0))
                num_right_contacts += 1
        #IMPORTANT: should not have two simulators in the same log
        if event.channel == "CASSIE_CONTACT_MUJOCO":
            msg = dairlib.lcmt_cassie_mujoco_contact.decode(event.data)
            t_contact_info.append(msg.utime / 1e6)
            contact_info[0].append(msg.contact_forces[0:3])
            contact_info[1].append((0.0, 0.0, 0.0))
            contact_info[2].append(msg.contact_forces[6:9])
            contact_info[3].append((0.0, 0.0, 0.0))
            is_mujoco = True

    # Convert into numpy arrays
    t_state = np.array(t_state)
    t_osc = np.array(t_osc)
    t_controller_switch = np.array(t_controller_switch)
    t_contact_info = np.array(t_contact_info)
    t_osc_debug = np.array(t_osc_debug)
    q = np.array(q)
    v = np.array(v)
    control_inputs = np.array(control_inputs)
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

    for i in range(len(osc_debug)):
        osc_debug[i].convertToNP()

    # Need to completely rearrange pos and vel matrix from mujoco simulation
    # fb_quat = slice(0, 4)
    # fb_pos = slice(4, 7)
    # muj_fb_quat = slice(3, 7)
    # muj_fb_pos = slice(0, 3)
    # if is_mujoco:
    #     # q[:, fb_quat], q[:, fb_pos] = q[:, muj_fb_quat], q[:, muj_fb_pos]
    #     q[:, fb_pos], q[:, fb_quat] = q[:, muj_fb_pos], q[:, muj_fb_quat]
    # import pdb; pdb.set_trace()

    return contact_info, contact_info_locs, control_inputs, estop_signal, \
           osc_debug, q, switch_signal, t_contact_info, t_controller_switch, \
           t_osc, t_osc_debug, t_state, v


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

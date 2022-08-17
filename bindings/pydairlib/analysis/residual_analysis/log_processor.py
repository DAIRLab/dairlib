import numpy as np
import lcm
import dairlib
import drake
import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
from process_lcm_log import get_log_data

class LogProcessor():
    def __init__(self, start_time, duration):
        self.init_channel()
        self.start_time = start_time
        self.duration = duration

    def init_channel(self):
        self.state_channels_name = "CASSIE_STATE_DISPATCHER"
        self.contact_channels_name = 'CASSIE_CONTACT_DISPATCHER'
        self.channels = {self.state_channels_name:dairlib.lcmt_robot_output,
                        self.contact_channels_name:dairlib.lcmt_contact}

    def set_pos_map(self, pos_map, pos_map_inverse):
        self.pos_map = pos_map
        self.pos_map_inverse = pos_map_inverse
    
    def set_vel_map(self, vel_map, vel_map_inverse):
        self.vel_map = vel_map
        self.vel_map_inverse = vel_map_inverse

    def set_act_map(self, act_map, act_map_inverse):
        self.act_map = act_map
        self.act_map_inverse = act_map_inverse

    def set_log_path(self, path):
        self.path = path
        self.dir = os.path.split(path)[0]
        self.file_name = os.path.split(path)[1]

    def process_states_channel(self, state_data):
        t_x = []
        q = []
        u = []
        v = []

        for msg in state_data:
            q_temp = [[] for i in range(len(msg.position))]
            v_temp = [[] for i in range(len(msg.velocity))]
            u_temp = [[] for i in range(len(msg.effort))]
            for i in range(len(q_temp)):
                q_temp[self.pos_map[msg.position_names[i]]] = msg.position[i]
            for i in range(len(v_temp)):
                v_temp[self.vel_map[msg.velocity_names[i]]] = msg.velocity[i]
            for i in range(len(u_temp)):
                u_temp[self.act_map[msg.effort_names[i]]] = msg.effort[i]
            q.append(q_temp)
            v.append(v_temp)
            u.append(u_temp)
            t_x.append(msg.utime / 1e6)

        return {'t_x': np.array(t_x),
            'q': np.array(q),
            'v': np.array(v),
            'u': np.array(u)}

    def process_contact_channel(self, contact_data):
        t_contact = []
        is_contact = []
        for msg in contact_data:
            t_contact.append(msg.utime / 1e6)
            is_contact.append(msg.contact)
        t_contact = np.array(t_contact)
        is_contact = np.array(is_contact, dtype=bool)

        return {"t_contact": t_contact,
            "is_contact": is_contact}

    def process_all_channels(self, data_to_process):
        state_data = self.process_states_channel(data_to_process[self.state_channels_name])
        contact_data = self.process_contact_channel(data_to_process[self.contact_channels_name])

        return {"states_info":state_data, 
                "contact_info":contact_data}

    def process_log(self):
        
        self.log = lcm.EventLog(self.path, "r")
        processed_data = get_log_data(self.log, self.channels, self.start_time, self.duration, self.process_all_channels)
        
        return processed_data
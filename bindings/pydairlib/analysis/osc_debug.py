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